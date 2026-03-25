/**
 * @file display_motor_control.ino
 * @brief ESP32-S3 + Display RGB 800x480 + LinkerWX — Interface gráfica de controle de motor
 *
 * Fluxo geral:
 *  1. Inicializa o display RGB (Arduino_GFX) e o touch GT911.
 *  2. Conecta ao WiFi e ao servidor TCP LinkerWX.
 *  3. Recebe telemetria do inversor (dispositivo 1:1) e atualiza widgets na tela.
 *  4. Envia comandos ao motor (START, STOP, REVERSE, EMERGENCY, frequência) via LinkerWX.
 *  5. Executa rotinas programadas (Prog 1 / Prog 2) com temporização configurável.
 */

// =============================================================================
// IMAGENS EMBUTIDAS (pixels e máscaras dos botões de voltar das telas de prog)
// =============================================================================

#include "ipg1back.h"
#include "ipg2back.h"

// =============================================================================
// BIBLIOTECAS
// =============================================================================

#include "esp_log.h"
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <displayfk.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <LinkerWX.h>

// =============================================================================
// CONFIGURAÇÕES DO HARDWARE — Display e Touch
// =============================================================================

/**
 * MCU:     ESP32-S3 (embedded)
 * Display: ESP32-8048S070 (RGB, 800×480)
 * Touch:   GT911 (I2C)
 */

#define DISP_LED 2  // pino do backlight do display

// Resolução do display
const int DISPLAY_W = 800;
const int DISPLAY_H = 480;

// Mapeamento da área de toque (coordenadas brutas → pixels)
const int  TOUCH_MAP_X0  = 0;
const int  TOUCH_MAP_X1  = 800;
const int  TOUCH_MAP_Y0  = 0;
const int  TOUCH_MAP_Y1  = 480;

// Ajustes de orientação do touch
const bool TOUCH_SWAP_XY  = false;
const bool TOUCH_INVERT_X = false;
const bool TOUCH_INVERT_Y = false;

// Pinos I2C do touch GT911
const int TOUCH_SCL = 20;
const int TOUCH_SDA = 19;
const int TOUCH_INT = -1;  // não utilizado
const int TOUCH_RST = 38;

// Orientação e modo do display
const uint8_t rotationScreen = 0;    // 0 = paisagem padrão
const bool    isIPS          = false; // false = não usa bigEndian

// =============================================================================
// CONFIGURAÇÕES — WiFi e Servidor TCP LinkerWX
// =============================================================================

// Credenciais da rede WiFi
const char*    ssid       = "Labmaker";
const char*    password   = "Anglo2020pp";

// Endereço e porta do servidor LinkerWX
const char*    serverIP   = "168.231.99.137";
const uint16_t serverPort = 12345;

// Identificação deste dispositivo no protocolo LinkerWX
// Este display é o destino 1:2 (grupo 1, ID 2)
const uint16_t DEVICE_ID = 2;
const uint16_t GRUPO     = 1;
const String   TOKEN     = "987654321";

// Destino para envio de comandos ao motor (dispositivo 1:1)
const uint16_t TARGET_MOTOR_GROUP = 1;
const uint16_t TARGET_MOTOR_ID   = 1;

// =============================================================================
// OBJETOS GLOBAIS — Conexão e protocolo
// =============================================================================

WiFiClient client;  // socket TCP com o servidor LinkerWX
LinkerWX   linker;  // instância do protocolo LinkerWX

// Controle de estado da conexão e registro
bool     deviceRegistered           = false;
uint32_t lastReconnectCheck         = 0;
uint32_t lastRegisterTime           = 0;
const uint32_t RECONNECT_CHECK_INTERVAL = 30000; // 30 segundos

// =============================================================================
// ESTRUTURA DE TELEMETRIA DO MOTOR
// =============================================================================

/**
 * @brief Dados recebidos do inversor via LinkerWX (pacote chave=valor).
 *        O campo `fresh` sinaliza que há dados novos prontos para atualizar a UI.
 */
struct MotorData {
    float    fa  = 0.0f;  // frequência atual (Hz)
    float    fs  = 0.0f;  // frequência setada (Hz)
    float    v   = 0.0f;  // velocidade
    uint32_t rpm = 0;     // rotações por minuto
    float    tq  = 0.0f;  // torque (%)
    float    p   = 0.0f;  // potência (W)
    float    vdc = 0.0f;  // tensão do barramento DC (V)
    float    vs  = 0.0f;  // tensão de saída (V)
    float    i   = 0.0f;  // corrente de saída (A)
    float    tm  = 0.0f;  // temperatura do motor (°C)
    float    a1  = 0.0f;  // analógico 1
    float    a2  = 0.0f;  // analógico 2
    float    a3  = 0.0f;  // analógico 3
    uint16_t edo = 0;     // estado das saídas digitais
    uint16_t er  = 0;     // código de erro
    uint32_t tl  = 0;     // tempo ligado (s)
    float    tr  = 0.0f;  // tempo restante
    uint16_t cf  = 0;     // código de falha
    uint16_t inf = 0;     // informação adicional
    bool     fresh       = false; // true quando há dados novos não consumidos
    uint32_t lastUpdateMs = 0;   // timestamp da última atualização (ms)
};

MotorData motorData;

// =============================================================================
// MÁQUINA DE ESTADOS — Rotinas programadas (Prog 1 / Prog 2)
// =============================================================================

/**
 * @brief Estados possíveis da rotina programada.
 *  IDLE     → nenhuma rotina ativa.
 *  WAITING  → aguardando o tempo de início (startDelay).
 *  RUNNING  → motor ligado, aguardando runTime.
 *  STOPPED  → motor parado, aguardando stopTime para religar.
 */
enum ProgState {
    PROG_IDLE,
    PROG_WAITING,
    PROG_RUNNING,
    PROG_STOPPED
};

/**
 * @brief Parâmetros configuráveis de uma rotina programada.
 *        Preenchidos a partir dos NumberBoxes da tela correspondente.
 */
struct ProgConfig {
    float    freqMax;    // frequência máxima configurada (Hz)
    float    rpmMax;     // RPM máximo configurado
    uint32_t startDelay; // segundos até iniciar o motor
    uint32_t runTime;    // segundos com o motor ligado
    uint32_t stopTime;   // segundos com o motor parado (antes de religar)
};

static ProgState  progState   = PROG_IDLE;
static ProgConfig progCfg     = {};
static uint32_t   progStateMs = 0;   // timestamp da última transição de estado
static bool       progActive  = false;
static int        currentProg = 1;   // qual programa está ativo (1 ou 2)

// =============================================================================
// PROTÓTIPOS DE FUNÇÕES
// =============================================================================

// Telas
void screen0();
void screen1();
void screen2();
void loadWidgets();

// WiFi / TCP / LinkerWX
bool conectarWiFi();
bool conectarServidorTCP();
bool isWiFiConnected();
bool isServerConnected();
void sendRegisterCommand();
void checkReconnectServer();
void sendKeepAliveCommand();
void readReceivedData();
void parseTelemetry(String dados);
void sendCommandToMotor(const String& cmd);

// Callbacks LinkerWX
void onSRV(const SRVResponse& response);
void onRECV(const RECVResponse& response);
void onERR(const ERRResponse& response);
void onDEST(const DESTResponse& response);
void onError(ErrorCode errorCode, const char* command);
void onRECVOk();

// Callbacks de widgets
void slidergeral_cb();
void sliderprg1_cb();
void sliderprg2_cb();
void pg1starttime_cb();
void pg1runtime_cb();
void pg1endtime_cb();
void pg1freqmax_cb();
void pg1rpmmax_cb();
void pg2starttime_cb();
void pg2runtime_cb();
void pg2stoptime_cb();
void pg2freqmax_cb();
void pg2rpmmax_cb();
void ipg1back_cb();
void ipg2back_cb();
void startbtn_cb();
void reversebtn_cb();
void emergcybtn_cb();
void stopbtn_cb();
void prog1btn_cb();
void prog2btn_cb();
void prog1startbtn_cb();
void stop1btn_cb();
void prog2startbtn_cb();
void stop2btn_cb();

// Rotinas programadas
float calcFreqFromRpm(float rpmDesejado, float rpmMax, float freqMax);
void  progStart(int progNum);
void  progStop();
void  progRoutine();

// =============================================================================
// OBJETOS DO DISPLAY (Arduino_GFX + DisplayFK)
// =============================================================================

Arduino_ESP32RGBPanel *bus = nullptr; // barramento RGB do painel
Arduino_RGB_Display   *tft = nullptr; // driver do display
DisplayFK myDisplay;                  // camada de widgets e touch

// =============================================================================
// WIDGETS — Gauge (ponteiro analógico)
// =============================================================================

// Gauge de RPM posicionado na tela 0
GaugeSuper gaugerpm(220, 150, 0);

const uint8_t qtdGauge = 1;
GaugeSuper *arrayGauge[qtdGauge] = {&gaugerpm};

// Intervalos de cor do gauge (0–450–900–1350 RPM)
const uint8_t qtdIntervalG0 = 4;
int      range0[qtdIntervalG0]  = {0, 450, 900, 1350};
uint16_t colors0[qtdIntervalG0] = {CFK_COLOR025, CFK_COLOR019, CFK_COLOR013, CFK_COLOR007};

// =============================================================================
// WIDGETS — Sliders horizontais
// =============================================================================

HSlider slidergeral(60, 400, 0);  // controle manual de frequência (tela 0)
HSlider sliderprg1 (55, 385, 1);  // setpoint de RPM da Prog 1 (tela 1)
HSlider sliderprg2 (55, 385, 2);  // setpoint de RPM da Prog 2 (tela 2)

const uint8_t qtdHSlider = 3;
HSlider *arrayHslider[qtdHSlider] = {&slidergeral, &sliderprg1, &sliderprg2};

// Valores atuais dos sliders (atualizados pelos callbacks)
int hslider0_val = 0;  // slidergeral — frequência manual (0–100%)
int hslider1_val = 0;  // sliderprg1  — % do RPM máximo da Prog 1
int hslider2_val = 0;  // sliderprg2  — % do RPM máximo da Prog 2

// =============================================================================
// WIDGETS — Labels de texto
// =============================================================================

Label valuefact      (290, 325, 0); // frequência atual (tela 0)
Label valuefset      (290, 360, 0); // frequência setada (tela 0)
Label labelhslider15 (225, 425, 0); // subtítulo do slider geral (tela 0)
Label setrpmvalue    (270, 350, 1); // RPM alvo calculado (tela 1)
Label rpmvalue       (230, 420, 1); // % do slider da Prog 1 (tela 1)
Label text8copycopy  (270, 350, 2); // RPM alvo calculado (tela 2)
Label rpmvalue6      (230, 420, 2); // % do slider da Prog 2 (tela 2)

const uint8_t qtdLabel = 7;
Label *arrayLabel[qtdLabel] = {
    &valuefact, &valuefset, &labelhslider15,
    &setrpmvalue, &rpmvalue,
    &text8copycopy, &rpmvalue6
};

// =============================================================================
// WIDGETS — Gráficos de linha (histórico de RPM)
// =============================================================================

LineChart graphfreq              (414, 40, 0); // gráfico da tela 0
LineChart graphfreqcopycopy      (414, 40, 1); // gráfico da tela 1
LineChart graphfreqcopycopycopy  (414, 40, 2); // gráfico da tela 2

const uint8_t qtdLineChart = 3;
LineChart *arrayLinechart[qtdLineChart] = {
    &graphfreq, &graphfreqcopycopy, &graphfreqcopycopycopy
};

// Cores das séries de cada gráfico (2 séries por gráfico)
const uint8_t qtdLinesChart0 = 2;
uint16_t colorsChart0[qtdLinesChart0] = {CFK_COLOR031, CFK_COLOR055};
const uint8_t qtdLinesChart1 = 2;
uint16_t colorsChart1[qtdLinesChart1] = {CFK_COLOR031, CFK_COLOR055};
const uint8_t qtdLinesChart2 = 2;
uint16_t colorsChart2[qtdLinesChart2] = {CFK_COLOR031, CFK_COLOR055};

// =============================================================================
// WIDGETS — NumberBoxes (campos numéricos editáveis via teclado)
// =============================================================================

// Prog 1
NumberBox pg1starttime(275,  80, 1); // tempo de início (s)
NumberBox pg1runtime  (275, 130, 1); // tempo ligado (s)
NumberBox pg1endtime  (275, 175, 1); // tempo parado (s)
NumberBox pg1freqmax  (275, 220, 1); // frequência máxima (Hz)
NumberBox pg1rpmmax   (275, 270, 1); // RPM máximo

// Prog 2
NumberBox pg2starttime(275,  80, 2);
NumberBox pg2runtime  (275, 130, 2);
NumberBox pg2stoptime (275, 175, 2);
NumberBox pg2freqmax  (275, 220, 2);
NumberBox pg2rpmmax   (275, 270, 2);

const uint8_t qtdNumberbox = 10;
NumberBox *arrayNumberbox[qtdNumberbox] = {
    &pg1starttime, &pg1runtime, &pg1endtime, &pg1freqmax, &pg1rpmmax,
    &pg2starttime, &pg2runtime, &pg2stoptime, &pg2freqmax, &pg2rpmmax
};

// Valores padrão dos NumberBoxes (alterados pelos callbacks)
float nb0_val = 10;    // pg1starttime
float nb1_val = 20;    // pg1runtime
float nb2_val = 10;    // pg1endtime
float nb3_val = 60.0;  // pg1freqmax
float nb4_val = 1798;  // pg1rpmmax
float nb5_val = 10;    // pg2starttime
float nb6_val = 30;    // pg2runtime
float nb7_val = 30;    // pg2stoptime
float nb8_val = 60.0;  // pg2freqmax
float nb9_val = 1798;  // pg2rpmmax

// =============================================================================
// WIDGETS — Botões de texto (TextButton)
// =============================================================================

// Tela 0 — controle manual
TextButton startbtn   (415, 260, 0);
TextButton reversebtn (590, 260, 0);
TextButton emergcybtn (415, 325, 0);
TextButton stopbtn    (590, 325, 0);
TextButton prog1btn   (415, 390, 0); // navega para tela 1
TextButton prog2btn   (590, 390, 0); // navega para tela 2

// Tela 1 — controle da Prog 1
TextButton prog1startbtn(485, 260, 1);
TextButton stop1btn     (485, 360, 1);

// Tela 2 — controle da Prog 2
TextButton prog2startbtn(485, 260, 2);
TextButton stop2btn     (485, 360, 2);

const uint8_t qtdTextButton = 10;
TextButton *arrayTextButton[qtdTextButton] = {
    &startbtn, &reversebtn, &emergcybtn, &stopbtn, &prog1btn, &prog2btn,
    &prog1startbtn, &stop1btn,
    &prog2startbtn, &stop2btn
};

// =============================================================================
// WIDGETS — Analógicos verticais (barras de nível)
// =============================================================================

VAnalog analogvdc (75,  165, 0); // tensão DC
VAnalog analogtout(200, 165, 0); // tensão de saída
VAnalog analogcur (315, 165, 0); // corrente

const uint8_t qtdVAnalog = 3;
VAnalog *arrayVanalog[qtdVAnalog] = {&analogvdc, &analogtout, &analogcur};

// =============================================================================
// WIDGETS — Imagens (botões de voltar das telas de prog)
// =============================================================================

Image ipg1back(0, 0, 1); // botão "voltar" da tela 1
Image ipg2back(0, 0, 2); // botão "voltar" da tela 2

const uint8_t qtdImagem = 2;
Image *arrayImagem[qtdImagem] = {&ipg1back, &ipg2back};

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);

    // Inicializa o barramento RGB com os pinos e timings do painel ESP32-8048S070
    bus = new Arduino_ESP32RGBPanel(
        41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
        14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
         9 /* G0 */, 46 /* G1 */,  3 /* G2 */,  8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
        15 /* B0 */,  7 /* B1 */,  6 /* B2 */,  5 /* B3 */,  4 /* B4 */,
        0 /* hsync_polarity */, 180 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
        0 /* vsync_polarity */,  12 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
        1 /* pclk_active_neg */, 12000000 /* prefer_speed */, isIPS /* useBigEndian */,
        0 /* de_idle_high */, 0 /* pclk_idle_high */, 0 /* bounce_buffer_size_px */
    );

    tft = new Arduino_RGB_Display(800, 480, bus, rotationScreen, true);
    tft->begin();

    // Liga o backlight do display
#if defined(DISP_LED)
    pinMode(DISP_LED, OUTPUT);
    digitalWrite(DISP_LED, HIGH);
#endif

    // Configura o protocolo LinkerWX com ID, grupo e token
    if (!linker.setup(DEVICE_ID, GRUPO, TOKEN)) {
        Serial.println("Erro ao configurar LinkerWX");
        while (1) delay(1000); // trava em caso de falha crítica
    }

    // Define chave de criptografia dos pacotes
    const uint8_t cryptKey[] = {0xFF, 0xFE, 0x84};
    linker.setCryptKey(cryptKey, sizeof(cryptKey));

    // Registra todos os callbacks do protocolo LinkerWX
    linker.registerSRVCallback(onSRV);
    linker.registerRECVCallback(onRECV);
    linker.registerERRCallback(onERR);
    linker.registerDESTCallback(onDEST);
    linker.registerErrorCallback(onError);
    linker.registerRECVOkCallback(onRECVOk);

    // Configura o DisplayFK com referência ao driver gráfico
    myDisplay.setDrawObject(tft);

    // Configura o touch GT911
    myDisplay.setTouchCorners(TOUCH_MAP_X0, TOUCH_MAP_X1, TOUCH_MAP_Y0, TOUCH_MAP_Y1);
    myDisplay.setInvertAxis(TOUCH_INVERT_X, TOUCH_INVERT_Y);
    myDisplay.setSwapAxis(TOUCH_SWAP_XY);
    myDisplay.startTouchGT911(DISPLAY_W, DISPLAY_H, rotationScreen, TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST);
    myDisplay.enableTouchLog();

    // Inicializa todos os widgets e carrega a tela principal
    loadWidgets();
    myDisplay.loadScreen(screen0);

    // Cria a task FreeRTOS responsável por leitura de touch e redesenho
    myDisplay.createTask(false, 3);

    ESP_LOGI("setup", "passou setup");

    // Ajusta nível de log por módulo
    esp_log_level_set("LINKER",    ESP_LOG_INFO);
    esp_log_level_set("TELEMETRY", ESP_LOG_INFO);
}

// =============================================================================
// LOOP PRINCIPAL
// =============================================================================

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // intervalo de envio do slider manual (ms)

/**
 * @brief Loop principal — executado continuamente pelo ESP32-S3.
 *
 * Ordem de execução:
 *  1. Garante conexão WiFi.
 *  2. A cada 1 segundo, envia o valor do slider manual (se nenhuma rotina ativa).
 *  3. Executa a máquina de estados das rotinas programadas.
 *  4. Mantém a conexão TCP e o keep-alive LinkerWX.
 *  5. Processa dados recebidos do servidor.
 *  6. Atualiza os widgets com os dados de telemetria mais recentes.
 */
void loop() {
    if (!isWiFiConnected()) {
        conectarWiFi();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        static int lastSliderVal = -1;

        // Envia o valor do slider manual apenas se nenhuma rotina estiver ativa
        // e o valor tiver mudado desde o último envio
        if (!progActive) {
            if (hslider0_val != lastSliderVal) {
                lastSliderVal = hslider0_val;
                sendCommandToMotor(String(hslider0_val));
            }
        }
    }

    progRoutine();          // avança a máquina de estados das rotinas programadas
    checkReconnectServer(); // verifica e reconecta ao servidor TCP se necessário
    sendKeepAliveCommand(); // mantém a sessão LinkerWX ativa
    readReceivedData();     // processa bytes recebidos do servidor

    // Atualiza os widgets com os dados de telemetria (apenas se houver dados novos)
    myDisplay.startCustomDraw();

    if (motorData.fresh) {
        gaugerpm.setValue(motorData.rpm);

        valuefact.setText(String(motorData.fa));
        valuefset.setText(String(motorData.fs));

        // Calcula e exibe o RPM alvo com base no slider e no RPM máximo configurado
        setrpmvalue.setTextInt((int)((hslider1_val / 100.0) * nb4_val));
        text8copycopy.setTextInt((int)((hslider2_val / 100.0) * nb9_val));

        // Empurra o RPM atual para os três gráficos de linha
        graphfreq.push(0, motorData.rpm);
        graphfreqcopycopy.push(0, motorData.rpm);
        graphfreqcopycopycopy.push(0, motorData.rpm);

        // Atualiza as barras analógicas verticais
        analogvdc.setValue((int)motorData.vdc, true);
        analogtout.setValue((int)motorData.vs,  true);
        analogcur.setValue((int)motorData.i,   true);

        motorData.fresh = false; // consome o flag para evitar redesenho desnecessário
    }

    myDisplay.finishCustomDraw();

    // Variável disponível para uso futuro (ex: indicador visual de dados desatualizados)
    bool dadosValidos = (millis() - motorData.lastUpdateMs) < 1000;

    delay(50);
}

// =============================================================================
// TELAS — Desenho estático de cada tela (chamado ao navegar)
// =============================================================================

/**
 * @brief Tela 0 — Painel principal de monitoramento e controle manual.
 *        Contém: gauge de RPM, barras analógicas, labels de frequência,
 *        slider de frequência, gráfico e botões de controle.
 */
void screen0() {
    tft->fillScreen(CFK_GREY6);
    WidgetBase::backgroundColor = CFK_GREY6;
    tft->fillRoundRect(25, 25, 750, 430, 25, CFK_GREY5);
    tft->drawRoundRect(25, 25, 750, 430, 25, CFK_BLACK);

    myDisplay.printText("Frequencia Atual:",  60, 327, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular12pt7b);
    myDisplay.printText("Frequencia Setada:", 60, 362, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular12pt7b);
    myDisplay.printText("Tensao DC",   55, 306, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular8pt7b);
    myDisplay.printText("Tensao OUT", 175, 301, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular8pt7b);
    myDisplay.printText("Corrente",   305, 301, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular8pt7b);
    myDisplay.printText("RPM",        560, 227, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular11pt7b);

    myDisplay.drawWidgetsOnScreen(0);
}

/**
 * @brief Tela 1 — Configuração e controle da Prog 1 (ciclo temporizado).
 *        Contém: NumberBoxes de tempo/frequência/RPM, slider de setpoint,
 *        gráfico de RPM e botões de iniciar/parar.
 */
void screen1() {
    tft->fillScreen(CFK_GREY6);
    WidgetBase::backgroundColor = CFK_GREY6;
    tft->fillRoundRect(30, 30, 750, 430, 25, CFK_GREY5);
    tft->drawRoundRect(30, 30, 750, 430, 25, CFK_BLACK);

    myDisplay.printText("RPM",             560,  227, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular11pt7b);
    myDisplay.printText("Tempo de inicio:", 50,   82, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Tempo ligado:",    50,  127, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Tempo parado:",    50,  177, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Freq. maxima:",    55,  222, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("RPM maxima:",      55,  272, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("SET RPM:",        105,  352, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);

    myDisplay.drawWidgetsOnScreen(1);
}

/**
 * @brief Tela 2 — Configuração e controle da Prog 2 (idêntica à Prog 1 em layout).
 *        Os parâmetros são independentes (nb5–nb9).
 */
void screen2() {
    tft->fillScreen(CFK_GREY6);
    WidgetBase::backgroundColor = CFK_GREY6;
    tft->fillRoundRect(30, 30, 750, 430, 25, CFK_GREY5);
    tft->drawRoundRect(30, 30, 750, 430, 25, CFK_BLACK);

    myDisplay.printText("RPM",             560,  227, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular11pt7b);
    myDisplay.printText("Tempo de inicio:", 50,   82, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Tempo ligado:",    50,  127, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Tempo parado:",    50,  177, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Freq. maxima:",    55,  222, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("RPM maxima:",      55,  272, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("SET RPM:",        105,  352, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);

    myDisplay.drawWidgetsOnScreen(2);
}

// =============================================================================
// INICIALIZAÇÃO DOS WIDGETS (loadWidgets)
// =============================================================================

/**
 * @brief Configura individualmente cada widget e os registra no DisplayFK.
 *        Chamado uma única vez no setup(), antes de loadScreen().
 */
void loadWidgets() {

    // --- Gauge de RPM ---
    GaugeConfig configGauge0 = {
        .title           = "RPM",
        .intervals       = range0,
        .colors          = colors0,
        .fontFamily      = &RobotoRegular8pt7b,
        .minValue        = 0,
        .maxValue        = 1800,
        .width           = 289,
        .height          = 98,
        .borderColor     = CFK_COLOR007,
        .textColor       = CFK_BLACK,
        .backgroundColor = CFK_GREY6,
        .titleColor      = CFK_BLACK,
        .needleColor     = CFK_RED,
        .markersColor    = CFK_BLACK,
        .amountIntervals = qtdIntervalG0,
        .showLabels      = true
    };
    gaugerpm.setup(configGauge0);
    myDisplay.setGauge(arrayGauge, qtdGauge);

    // --- Sliders horizontais ---
    HSliderConfig configHSlider0 = {
        .callback        = slidergeral_cb,
        .subtitle        = &labelhslider15, // exibe % abaixo do slider
        .minValue        = 0,
        .maxValue        = 100,
        .radius          = 11,
        .width           = 299,
        .pressedColor    = CFK_GREY3,
        .backgroundColor = CFK_WHITE,
    };
    slidergeral.setup(configHSlider0);

    HSliderConfig configHSlider1 = {
        .callback        = sliderprg1_cb,
        .subtitle        = nullptr,
        .minValue        = 0,
        .maxValue        = 100,
        .radius          = 14,
        .width           = 342,
        .pressedColor    = CFK_GREY3,
        .backgroundColor = CFK_WHITE,
    };
    sliderprg1.setup(configHSlider1);

    HSliderConfig configHSlider2 = {
        .callback        = sliderprg2_cb,
        .subtitle        = nullptr,
        .minValue        = 0,
        .maxValue        = 100,
        .radius          = 14,
        .width           = 342,
        .pressedColor    = CFK_GREY3,
        .backgroundColor = CFK_WHITE,
    };
    sliderprg2.setup(configHSlider2);
    myDisplay.setHSlider(arrayHslider, qtdHSlider);

    // --- Labels ---
    LabelConfig configLabel0 = { .text = "", .prefix = "", .suffix = " Hz",
        .fontFamily = &RobotoRegular12pt7b, .datum = TL_DATUM,
        .fontColor = CFK_BLACK, .backgroundColor = CFK_GREY5 };
    valuefact.setup(configLabel0);

    LabelConfig configLabel1 = { .text = "", .prefix = "", .suffix = " Hz",
        .fontFamily = &RobotoRegular12pt7b, .datum = TL_DATUM,
        .fontColor = CFK_BLACK, .backgroundColor = CFK_GREY5 };
    valuefset.setup(configLabel1);

    LabelConfig configLabel2 = { .text = "", .prefix = "", .suffix = "%",
        .fontFamily = &RobotoRegular10pt7b, .datum = TL_DATUM,
        .fontColor = CFK_BLACK, .backgroundColor = CFK_GREY5 };
    labelhslider15.setup(configLabel2);

    LabelConfig configLabel3 = { .text = "900", .prefix = "", .suffix = "",
        .fontFamily = &RobotoRegular13pt7b, .datum = TL_DATUM,
        .fontColor = CFK_WHITE, .backgroundColor = CFK_GREY5 };
    setrpmvalue.setup(configLabel3);

    LabelConfig configLabel4 = { .text = "", .prefix = "", .suffix = "%",
        .fontFamily = &RobotoRegular13pt7b, .datum = TL_DATUM,
        .fontColor = CFK_WHITE, .backgroundColor = CFK_GREY5 };
    rpmvalue.setup(configLabel4);

    LabelConfig configLabel5 = { .text = "900", .prefix = "", .suffix = "",
        .fontFamily = &RobotoRegular13pt7b, .datum = TL_DATUM,
        .fontColor = CFK_WHITE, .backgroundColor = CFK_GREY5 };
    text8copycopy.setup(configLabel5);

    LabelConfig configLabel6 = { .text = "", .prefix = "", .suffix = "%",
        .fontFamily = &RobotoRegular13pt7b, .datum = TL_DATUM,
        .fontColor = CFK_WHITE, .backgroundColor = CFK_GREY5 };
    rpmvalue6.setup(configLabel6);
    myDisplay.setLabel(arrayLabel, qtdLabel);

    // --- Gráficos de linha ---
    LineChartConfig configLineChart0 = {
        .colorsSeries    = colorsChart0, .subtitles = nullptr,
        .font            = &RobotoRegular5pt7b,
        .minValue        = 0,   .maxValue = 2000,
        .width           = 335, .height   = 185,
        .gridColor       = CFK_BLACK, .borderColor = CFK_BLACK,
        .backgroundColor = CFK_GREY3, .textColor   = CFK_WHITE,
        .verticalDivision = 10,
        .maxPointsAmount  = LineChart::SHOW_ALL,
        .amountSeries     = qtdLinesChart0,
        .workInBackground = false,
        .showZeroLine     = true, .boldLine = false, .showDots = false
    };
    graphfreq.setup(configLineChart0);

    LineChartConfig configLineChart1 = {
        .colorsSeries    = colorsChart1, .subtitles = nullptr,
        .font            = &RobotoRegular5pt7b,
        .minValue        = 0,   .maxValue = 2000,
        .width           = 335, .height   = 185,
        .gridColor       = CFK_BLACK, .borderColor = CFK_BLACK,
        .backgroundColor = CFK_GREY3, .textColor   = CFK_WHITE,
        .verticalDivision = 10,
        .maxPointsAmount  = LineChart::SHOW_ALL,
        .amountSeries     = qtdLinesChart1,
        .workInBackground = false,
        .showZeroLine     = true, .boldLine = true, .showDots = false
    };
    graphfreqcopycopy.setup(configLineChart1);

    LineChartConfig configLineChart2 = {
        .colorsSeries    = colorsChart2, .subtitles = nullptr,
        .font            = &RobotoRegular5pt7b,
        .minValue        = 0,   .maxValue = 2000,
        .width           = 335, .height   = 185,
        .gridColor       = CFK_BLACK, .borderColor = CFK_BLACK,
        .backgroundColor = CFK_GREY3, .textColor   = CFK_WHITE,
        .verticalDivision = 10,
        .maxPointsAmount  = LineChart::SHOW_ALL,
        .amountSeries     = qtdLinesChart2,
        .workInBackground = false,
        .showZeroLine     = true, .boldLine = true, .showDots = false
    };
    graphfreqcopycopycopy.setup(configLineChart2);
    myDisplay.setLineChart(arrayLinechart, qtdLineChart);

    // --- Estilo global do teclado numérico (Numpad) ---
    Numpad::m_backgroundColor = CFK_GREY3;
    Numpad::m_letterColor     = CFK_BLACK;
    Numpad::m_keyColor        = CFK_GREY13;

    // --- NumberBoxes — Prog 1 ---
    NumberBoxConfig configNumberBox0 = { .funcPtr = screen1, .callback = pg1starttime_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb0_val,
        .width = 126, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg1starttime.setup(configNumberBox0);

    NumberBoxConfig configNumberBox1 = { .funcPtr = screen1, .callback = pg1runtime_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb1_val,
        .width = 127, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg1runtime.setup(configNumberBox1);

    NumberBoxConfig configNumberBox2 = { .funcPtr = screen1, .callback = pg1endtime_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb2_val,
        .width = 127, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg1endtime.setup(configNumberBox2);

    NumberBoxConfig configNumberBox3 = { .funcPtr = screen1, .callback = pg1freqmax_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb3_val,
        .width = 127, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 2 };
    pg1freqmax.setup(configNumberBox3);

    NumberBoxConfig configNumberBox4 = { .funcPtr = screen1, .callback = pg1rpmmax_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb4_val,
        .width = 129, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg1rpmmax.setup(configNumberBox4);

    // --- NumberBoxes — Prog 2 ---
    NumberBoxConfig configNumberBox5 = { .funcPtr = screen2, .callback = pg2starttime_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb5_val,
        .width = 125, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg2starttime.setup(configNumberBox5);

    NumberBoxConfig configNumberBox6 = { .funcPtr = screen2, .callback = pg2runtime_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb6_val,
        .width = 125, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg2runtime.setup(configNumberBox6);

    NumberBoxConfig configNumberBox7 = { .funcPtr = screen2, .callback = pg2stoptime_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb7_val,
        .width = 124, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg2stoptime.setup(configNumberBox7);

    NumberBoxConfig configNumberBox8 = { .funcPtr = screen2, .callback = pg2freqmax_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb8_val,
        .width = 125, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 2 };
    pg2freqmax.setup(configNumberBox8);

    NumberBoxConfig configNumberBox9 = { .funcPtr = screen2, .callback = pg2rpmmax_cb,
        .font = &RobotoRegular10pt7b, .startValue = nb9_val,
        .width = 126, .height = 25, .letterColor = CFK_WHITE,
        .backgroundColor = CFK_GREY6, .decimalPlaces = 0 };
    pg2rpmmax.setup(configNumberBox9);
    myDisplay.setNumberbox(arrayNumberbox, qtdNumberbox);

    // --- Botões de texto ---
    TextButtonConfig configTextButton0 = { .text = "START",     .callback = startbtn_cb,
        .fontFamily = &RobotoBold13pt7b, .width = 160, .height = 50, .radius = 10,
        .backgroundColor = CFK_COLOR025, .textColor = CFK_BLACK };
    startbtn.setup(configTextButton0);

    TextButtonConfig configTextButton1 = { .text = "REVERSE",   .callback = reversebtn_cb,
        .fontFamily = &RobotoBold13pt7b, .width = 160, .height = 50, .radius = 10,
        .backgroundColor = CFK_COLOR050, .textColor = CFK_BLACK };
    reversebtn.setup(configTextButton1);

    TextButtonConfig configTextButton2 = { .text = "EMERGENCY", .callback = emergcybtn_cb,
        .fontFamily = &RobotoBold13pt7b, .width = 160, .height = 50, .radius = 10,
        .backgroundColor = CFK_COLOR007, .textColor = CFK_BLACK };
    emergcybtn.setup(configTextButton2);

    TextButtonConfig configTextButton3 = { .text = "STOP",      .callback = stopbtn_cb,
        .fontFamily = &RobotoBold13pt7b, .width = 160, .height = 50, .radius = 10,
        .backgroundColor = CFK_COLOR019, .textColor = CFK_BLACK };
    stopbtn.setup(configTextButton3);

    TextButtonConfig configTextButton4 = { .text = "PROG 1",    .callback = prog1btn_cb,
        .fontFamily = &RobotoBold13pt7b, .width = 160, .height = 50, .radius = 10,
        .backgroundColor = CFK_COLOR013, .textColor = CFK_BLACK };
    prog1btn.setup(configTextButton4);

    TextButtonConfig configTextButton5 = { .text = "PROG 2",    .callback = prog2btn_cb,
        .fontFamily = &RobotoBold13pt7b, .width = 160, .height = 50, .radius = 10,
        .backgroundColor = CFK_COLOR013, .textColor = CFK_BLACK };
    prog2btn.setup(configTextButton5);

    TextButtonConfig configTextButton6 = { .text = "PROG 1",    .callback = prog1startbtn_cb,
        .fontFamily = &RobotoBold14pt7b, .width = 200, .height = 80, .radius = 10,
        .backgroundColor = CFK_COLOR013, .textColor = CFK_BLACK };
    prog1startbtn.setup(configTextButton6);

    TextButtonConfig configTextButton7 = { .text = "STOP",      .callback = stop1btn_cb,
        .fontFamily = &RobotoBold14pt7b, .width = 200, .height = 80, .radius = 10,
        .backgroundColor = CFK_COLOR019, .textColor = CFK_BLACK };
    stop1btn.setup(configTextButton7);

    TextButtonConfig configTextButton8 = { .text = "PROG 2",    .callback = prog2startbtn_cb,
        .fontFamily = &RobotoBold14pt7b, .width = 200, .height = 80, .radius = 10,
        .backgroundColor = CFK_COLOR013, .textColor = CFK_BLACK };
    prog2startbtn.setup(configTextButton8);

    TextButtonConfig configTextButton9 = { .text = "STOP",      .callback = stop2btn_cb,
        .fontFamily = &RobotoBold14pt7b, .width = 200, .height = 80, .radius = 10,
        .backgroundColor = CFK_COLOR019, .textColor = CFK_BLACK };
    stop2btn.setup(configTextButton9);
    myDisplay.setTextButton(arrayTextButton, qtdTextButton);

    // --- Analógicos verticais ---
    VerticalAnalogConfig configVAnalog0 = { .minValue = 0, .maxValue = 400,
        .width = 43, .height = 135, .arrowColor = CFK_COLOR007,
        .textColor = CFK_BLACK, .backgroundColor = CFK_WHITE,
        .borderColor = CFK_BLACK, .steps = 10 };
    analogvdc.setup(configVAnalog0);

    VerticalAnalogConfig configVAnalog1 = { .minValue = 0, .maxValue = 270,
        .width = 43, .height = 133, .arrowColor = CFK_COLOR007,
        .textColor = CFK_BLACK, .backgroundColor = CFK_WHITE,
        .borderColor = CFK_BLACK, .steps = 10 };
    analogtout.setup(configVAnalog1);

    VerticalAnalogConfig configVAnalog2 = { .minValue = 0, .maxValue = 10,
        .width = 43, .height = 127, .arrowColor = CFK_COLOR007,
        .textColor = CFK_BLACK, .backgroundColor = CFK_WHITE,
        .borderColor = CFK_BLACK, .steps = 10 };
    analogcur.setup(configVAnalog2);
    myDisplay.setVAnalog(arrayVanalog, qtdVAnalog);

    // --- Imagens (botões de voltar) ---
    ImageFromPixelsConfig configImage0 = {
        .pixels = ipg1backPixels, .maskAlpha = ipg1backMask,
        .cb = ipg1back_cb, .width = ipg1backW, .height = ipg1backH,
        .backgroundColor = CFK_GREY6
    };
    ipg1back.setupFromPixels(configImage0);

    ImageFromPixelsConfig configImage1 = {
        .pixels = ipg2backPixels, .maskAlpha = ipg2backMask,
        .cb = ipg2back_cb, .width = ipg2backW, .height = ipg2backH,
        .backgroundColor = CFK_GREY6
    };
    ipg2back.setupFromPixels(configImage1);
    myDisplay.setImage(arrayImagem, qtdImagem);
}

// =============================================================================
// CALLBACKS DOS WIDGETS — Sliders
// =============================================================================

/** @brief Atualiza hslider0_val com o valor atual do slider manual (tela 0). */
void slidergeral_cb() { hslider0_val = slidergeral.getValue(); }

/** @brief Atualiza hslider1_val e exibe a % no label da Prog 1. */
void sliderprg1_cb() {
    hslider1_val = sliderprg1.getValue();
    rpmvalue.setText(String(hslider1_val));
}

/** @brief Atualiza hslider2_val e exibe a % no label da Prog 2. */
void sliderprg2_cb() {
    hslider2_val = sliderprg2.getValue();
    rpmvalue6.setText(String(hslider2_val));
}

// =============================================================================
// CALLBACKS DOS WIDGETS — NumberBoxes (Prog 1)
// =============================================================================

void pg1starttime_cb() { nb0_val = pg1starttime.getValue(); }
void pg1runtime_cb()   { nb1_val = pg1runtime.getValue();   }
void pg1endtime_cb()   { nb2_val = pg1endtime.getValue();   }
void pg1freqmax_cb()   { nb3_val = pg1freqmax.getValue();   }
void pg1rpmmax_cb()    { nb4_val = pg1rpmmax.getValue();    }

// =============================================================================
// CALLBACKS DOS WIDGETS — NumberBoxes (Prog 2)
// =============================================================================

void pg2starttime_cb() { nb5_val = pg2starttime.getValue(); }
void pg2runtime_cb()   { nb6_val = pg2runtime.getValue();   }
void pg2stoptime_cb()  { nb7_val = pg2stoptime.getValue();  }
void pg2freqmax_cb()   { nb8_val = pg2freqmax.getValue();   }
void pg2rpmmax_cb()    { nb9_val = pg2rpmmax.getValue();    }

// =============================================================================
// CALLBACKS DOS WIDGETS — Botões de controle do motor (tela 0)
// =============================================================================

/** @brief Envia comando START ao motor via LinkerWX. */
void startbtn_cb()   { sendCommandToMotor("START");     Serial.println("Clicked on: startbtn_cb");   }

/** @brief Envia comando REVERSE ao motor via LinkerWX. */
void reversebtn_cb() { sendCommandToMotor("REVERSE");   Serial.println("Clicked on: reversebtn_cb"); }

/** @brief Envia comando EMERGENCY ao motor via LinkerWX. */
void emergcybtn_cb() { sendCommandToMotor("EMERGENCY"); Serial.println("Clicked on: emergcybtn_cb"); }

/** @brief Envia comando STOP ao motor via LinkerWX. */
void stopbtn_cb()    { sendCommandToMotor("STOP");      Serial.println("Clicked on: stopbtn_cb");    }

/** @brief Navega para a tela de configuração da Prog 1. */
void prog1btn_cb() {
    myDisplay.loadScreen(screen1);
    Serial.print("Clicked on: "); Serial.println("prog1btn_cb");
}

/** @brief Navega para a tela de configuração da Prog 2. */
void prog2btn_cb() {
    myDisplay.loadScreen(screen2);
    Serial.print("Clicked on: "); Serial.println("prog2btn_cb");
}

// =============================================================================
// CALLBACKS DOS WIDGETS — Botões de controle das rotinas programadas
// =============================================================================

/** @brief Inicia a rotina Prog 1 com os parâmetros configurados na tela 1. */
void prog1startbtn_cb() { progStart(1); Serial.println("Clicked on: prog1startbtn_cb"); }

/** @brief Para o motor e interrompe a rotina Prog 1. */
void stop1btn_cb() {
    sendCommandToMotor("STOP");
    progStop();
    Serial.println("Clicked on: stop1btn_cb");
}

/** @brief Inicia a rotina Prog 2 com os parâmetros configurados na tela 2. */
void prog2startbtn_cb() { progStart(2); Serial.println("Clicked on: prog2startbtn_cb"); }

/** @brief Para o motor e interrompe a rotina Prog 2. */
void stop2btn_cb() {
    sendCommandToMotor("STOP");
    progStop();
    Serial.println("Clicked on: stop2btn_cb");
}

// =============================================================================
// CALLBACKS DOS WIDGETS — Imagens (botões de voltar)
// =============================================================================

/** @brief Volta para a tela principal (screen0) a partir da tela da Prog 1. */
void ipg1back_cb() {
    myDisplay.loadScreen(screen0);
    Serial.println("Image pg1back clicked");
}

/** @brief Volta para a tela principal (screen0) a partir da tela da Prog 2. */
void ipg2back_cb() {
    myDisplay.loadScreen(screen0);
    Serial.println("Image pg2back clicked");
}

// =============================================================================
// WiFi / TCP / LinkerWX — conexão e manutenção
// =============================================================================

/**
 * @brief Tenta conectar ao WiFi com até 10 tentativas de 500 ms cada.
 * @return true se conectado com sucesso.
 */
bool conectarWiFi() {
    WiFi.begin(ssid, password);

    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 10) {
        delay(500);
        tentativas++;
    }

    return WiFi.status() == WL_CONNECTED;
}

/** @brief Verifica se o WiFi está conectado. */
bool isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief Abre conexão TCP com o servidor LinkerWX.
 *        Reseta deviceRegistered para forçar novo registro após reconexão.
 * @return true se a conexão foi estabelecida.
 */
bool conectarServidorTCP() {
    if (!client.connect(serverIP, serverPort)) return false;
    deviceRegistered = false;
    return true;
}

/** @brief Verifica se a conexão TCP com o servidor está ativa. */
bool isServerConnected() {
    return client.connected();
}

/**
 * @brief Envia o comando MYID (registro) ao servidor LinkerWX.
 *        Deve ser chamado logo após conectarServidorTCP().
 */
void sendRegisterCommand() {
    if (!isServerConnected()) return;

    uint8_t buffer[256];
    size_t packetSize = linker.getCommandRegisterBytes(buffer, sizeof(buffer));
    if (packetSize > 0) {
        client.write(buffer, packetSize);
    }
}

/**
 * @brief Verifica periodicamente se o servidor TCP ainda está acessível.
 *        Se desconectado, tenta reconectar e re-registrar o dispositivo.
 *        Intervalo controlado por RECONNECT_CHECK_INTERVAL.
 */
void checkReconnectServer() {
    if (!isServerConnected()) {
        deviceRegistered = false;
    }

    if (!isServerConnected() && millis() - lastReconnectCheck >= RECONNECT_CHECK_INTERVAL) {
        lastReconnectCheck = millis();
        if (conectarServidorTCP()) {
            delay(500);
            sendRegisterCommand();
        }
    }
}

/**
 * @brief Envia pacote keep-alive ao servidor para manter a sessão ativa.
 *        Só envia se o dispositivo estiver registrado e o intervalo mínimo
 *        desde o último registro tiver passado (300 ms).
 */
void sendKeepAliveCommand() {
    if (!deviceRegistered || !isServerConnected()) return;

    if (linker.needSendKeepAlive() && (millis() - lastRegisterTime > 300)) {
        linker.updateKeepAliveTime();
        uint8_t buffer[32];
        size_t packetSize = linker.getCommandAliveBytes(buffer, sizeof(buffer));
        if (packetSize > 0) {
            client.write(buffer, packetSize);
        }
    }
}

/**
 * @brief Lê bytes disponíveis no socket TCP e os repassa ao parser LinkerWX.
 *        Usa buffer de 512 bytes para acomodar pacotes maiores que o do transmissor.
 */
void readReceivedData() {
    while (client.available()) {
        uint8_t buffer[512];
        size_t bytesRead = client.readBytes(buffer, sizeof(buffer));
        if (bytesRead > 0) {
            linker.receiveData(buffer, bytesRead);
        }
    }
}

// =============================================================================
// CALLBACKS LinkerWX
// =============================================================================

/**
 * @brief Chamado quando o servidor confirma o registro do dispositivo.
 *        HI = registro novo; AGAIN = já estava registrado.
 */
void onSRV(const SRVResponse& response) {
    switch (response.type) {
        case SRVType::HI:
        case SRVType::AGAIN:
            deviceRegistered = true;
            lastRegisterTime = millis();
            ESP_LOGI("LINKER", "Display registrado no servidor");
            break;
        default:
            break;
    }
}

/**
 * @brief Chamado quando este dispositivo recebe uma mensagem.
 *        Repassa o payload para parseTelemetry() para atualizar motorData.
 */
void onRECV(const RECVResponse& response) {
    ESP_LOGI("LINKER", "RECV de %d:%d -> %s",
        response.sourceGroup, response.sourceId, response.data);

    String dados = String(response.data);
    dados.trim();

    if (dados.length() > 0) {
        parseTelemetry(dados);
    }
}

/**
 * @brief Chamado em caso de erro reportado pelo servidor LinkerWX.
 *        Códigos 1 e 10 forçam reconexão imediata.
 */
void onERR(const ERRResponse& response) {
    ESP_LOGE("LINKER", "ERR code=%d msg=%s", response.code, response.message);

    if (response.code == 1 || response.code == 10) {
        deviceRegistered   = false;
        client.stop();
        lastReconnectCheck = 0; // força reconexão imediata no próximo checkReconnectServer()
    }
}

/** @brief Chamado quando o servidor informa o IP/porta de um destino. */
void onDEST(const DESTResponse& response) {
    Serial.print("[DEST] ");
    Serial.print(response.ip);
    Serial.print(":");
    Serial.println(response.port);
}

/** @brief Chamado em erros internos da biblioteca LinkerWX. */
void onError(ErrorCode errorCode, const char* command) {
    Serial.print("[ERROR] ");
    Serial.print((int)errorCode);
    Serial.print(" cmd=");
    Serial.println(command);
}

/** @brief Chamado quando o servidor confirma o recebimento de um pacote enviado. */
void onRECVOk() {
    ESP_LOGI("LINKER", "RECV_OK confirmado pelo servidor");
}

// =============================================================================
// TELEMETRIA — parsing do pacote recebido
// =============================================================================

/**
 * @brief Faz o parse do payload de telemetria no formato "chave=valor;chave=valor;...".
 *        Atualiza os campos correspondentes em motorData e seta fresh=true.
 *
 * Chaves suportadas:
 *   fa, fs, v, rpm, tq, p, vdc, vs, i, tm, a1, a2, a3,
 *   edo, er, tl, tr, cf, if
 */
void parseTelemetry(String dados) {
    int start = 0;

    // Itera sobre cada par chave=valor separado por ';'
    while (start < dados.length()) {
        int sep  = dados.indexOf(';', start);
        if (sep == -1) sep = dados.length();

        String item = dados.substring(start, sep);
        int eq = item.indexOf('=');

        if (eq > 0) {
            String key   = item.substring(0, eq);
            String value = item.substring(eq + 1);

            // Mapeia cada chave para o campo correspondente em motorData
            if      (key == "fa")  motorData.fa  = value.toFloat();
            else if (key == "fs")  motorData.fs  = value.toFloat();
            else if (key == "v")   motorData.v   = value.toFloat();
            else if (key == "rpm") motorData.rpm = value.toInt();
            else if (key == "tq")  motorData.tq  = value.toFloat();
            else if (key == "p")   motorData.p   = value.toFloat();
            else if (key == "vdc") motorData.vdc = value.toFloat();
            else if (key == "vs")  motorData.vs  = value.toFloat();
            else if (key == "i")   motorData.i   = value.toFloat();
            else if (key == "tm")  motorData.tm  = value.toFloat();
            else if (key == "a1")  motorData.a1  = value.toFloat();
            else if (key == "a2")  motorData.a2  = value.toFloat();
            else if (key == "a3")  motorData.a3  = value.toFloat();
            else if (key == "edo") motorData.edo = value.toInt();
            else if (key == "er")  motorData.er  = value.toInt();
            else if (key == "tl")  motorData.tl  = value.toInt();
            else if (key == "tr")  motorData.tr  = value.toFloat();
            else if (key == "cf")  motorData.cf  = value.toInt();
            else if (key == "if")  motorData.inf = value.toInt();
        }

        start = sep + 1;
    }

    motorData.lastUpdateMs = millis();
    motorData.fresh        = true; // sinaliza que há dados novos para a UI

    // Log detalhado dos dados recebidos (visível via monitor serial com ESP_LOG)
    ESP_LOGI("TELEMETRY", "fa=%.1f fs=%.1f v=%.1f rpm=%lu tq=%.1f p=%.1f",
        motorData.fa, motorData.fs, motorData.v,
        (unsigned long)motorData.rpm, motorData.tq, motorData.p);
    ESP_LOGI("TELEMETRY", "vdc=%.1f vs=%.1f i=%.1f tm=%.1f",
        motorData.vdc, motorData.vs, motorData.i, motorData.tm);
    ESP_LOGI("TELEMETRY", "a1=%.1f a2=%.1f a3=%.1f edo=%u er=%u cf=%u inf=%u",
        motorData.a1, motorData.a2, motorData.a3,
        motorData.edo, motorData.er, motorData.cf, motorData.inf);
    ESP_LOGI("TELEMETRY", "tl=%lu tr=%.1f",
        (unsigned long)motorData.tl, motorData.tr);
}

// =============================================================================
// ENVIO DE COMANDOS AO MOTOR
// =============================================================================

/**
 * @brief Envia um comando de texto ao motor (dispositivo 1:1) via LinkerWX.
 *        O payload é prefixado com "cmd=" antes do envio.
 *        Comandos numéricos com valor ≤ 0 são bloqueados (evita enviar frequência zero).
 *
 * @param cmd  Comando a enviar: "START", "STOP", "REVERSE", "EMERGENCY" ou valor numérico (Hz).
 */
void sendCommandToMotor(const String& cmd) {
    if (!deviceRegistered || !client.connected()) {
        ESP_LOGW("LINKER", "Nao conectado. Comando ignorado: %s", cmd.c_str());
        return;
    }

    // Bloqueia envio de frequência zero ou negativa
    float val      = cmd.toFloat();
    bool isNumeric = (cmd.length() > 0 && (isDigit(cmd[0]) || cmd[0] == '-'));
    if (isNumeric && val <= 0.0f) return;

    String  payload    = "cmd=" + cmd;
    uint8_t buffer[256];
    size_t  packetSize = linker.getCommandSendToBytes(
        TARGET_MOTOR_GROUP, TARGET_MOTOR_ID,
        payload, SendType::REQ_ACK,
        buffer, sizeof(buffer)
    );

    if (packetSize > 0) {
        client.write(buffer, packetSize);
        ESP_LOGI("LINKER", "Enviado: %s", payload.c_str());
    } else {
        ESP_LOGE("LINKER", "Erro ao montar pacote para: %s", payload.c_str());
    }
}

// =============================================================================
// ROTINAS PROGRAMADAS (Prog 1 / Prog 2)
// =============================================================================

/**
 * @brief Calcula a frequência proporcional ao RPM desejado.
 *        Usa regra de três simples: freq = (rpmDesejado / rpmMax) × freqMax.
 *
 * @param rpmDesejado  RPM alvo.
 * @param rpmMax       RPM máximo configurado para a rotina.
 * @param freqMax      Frequência máxima correspondente ao rpmMax.
 * @return Frequência calculada em Hz, ou 0 se rpmMax for inválido.
 */
float calcFreqFromRpm(float rpmDesejado, float rpmMax, float freqMax) {
    if (rpmMax <= 0) return 0.0f;
    return (rpmDesejado / rpmMax) * freqMax;
}

/**
 * @brief Inicializa e ativa uma rotina programada.
 *        Carrega os parâmetros do programa selecionado (1 ou 2) a partir
 *        dos valores dos NumberBoxes e inicia a máquina de estados em PROG_WAITING.
 *
 * @param progNum  Número do programa a iniciar (1 ou 2).
 */
void progStart(int progNum) {
    currentProg = progNum;

    if (progNum == 1) {
        progCfg.freqMax    = nb3_val;
        progCfg.rpmMax     = nb4_val;
        progCfg.startDelay = (uint32_t)nb0_val;
        progCfg.runTime    = (uint32_t)nb1_val;
        progCfg.stopTime   = (uint32_t)nb2_val;
    } else {
        progCfg.freqMax    = nb8_val;
        progCfg.rpmMax     = nb9_val;
        progCfg.startDelay = (uint32_t)nb5_val;
        progCfg.runTime    = (uint32_t)nb6_val;
        progCfg.stopTime   = (uint32_t)nb7_val;
    }

    progState   = PROG_WAITING;
    progStateMs = millis();
    progActive  = true;

    Serial.printf("[PROG%d] Iniciado | delay=%lus run=%lus stop=%lus freqMax=%.2f rpmMax=%.0f\n",
        progNum,
        progCfg.startDelay, progCfg.runTime, progCfg.stopTime,
        progCfg.freqMax, progCfg.rpmMax);
}

/**
 * @brief Interrompe a rotina programada ativa e envia STOP ao motor.
 *        Retorna a máquina de estados para PROG_IDLE.
 */
void progStop() {
    if (!progActive) return;
    progActive = false;
    progState  = PROG_IDLE;
    sendCommandToMotor("STOP");
    Serial.println("[PROG] Rotina interrompida.");
}

/**
 * @brief Máquina de estados da rotina programada — chamada a cada iteração do loop.
 *
 * Transições:
 *   WAITING → RUNNING : após startDelay segundos, envia setpoint e START.
 *   RUNNING → STOPPED : após runTime segundos, envia STOP.
 *   STOPPED → RUNNING : após stopTime segundos, envia setpoint e START novamente.
 *
 * O setpoint enviado é o valor atual do slider do programa ativo (hslider1_val ou hslider2_val).
 */
void progRoutine() {
    if (!progActive) return;

    uint32_t elapsed = (millis() - progStateMs) / 1000; // tempo no estado atual (s)

    // Usa o slider do programa ativo como setpoint de velocidade
    int valorParaEnviar = (currentProg == 1) ? hslider1_val : hslider2_val;

    switch (progState) {

        case PROG_WAITING:
            if (elapsed >= progCfg.startDelay) {
                sendCommandToMotor(String(valorParaEnviar)); // envia setpoint
                delay(100);
                sendCommandToMotor("START");
                progState   = PROG_RUNNING;
                progStateMs = millis();
            }
            break;

        case PROG_RUNNING:
            if (elapsed >= progCfg.runTime) {
                Serial.println("[PROG] Parando motor (fim do tempo ligado)");
                sendCommandToMotor("STOP");
                progState   = PROG_STOPPED;
                progStateMs = millis();
            }
            break;

        case PROG_STOPPED:
            if (elapsed >= progCfg.stopTime) {
                sendCommandToMotor(String(valorParaEnviar)); // envia setpoint antes de religar
                delay(100);
                sendCommandToMotor("START");
                progState   = PROG_RUNNING;
                progStateMs = millis();
            }
            break;

        case PROG_IDLE:
        default:
            break;
    }
}