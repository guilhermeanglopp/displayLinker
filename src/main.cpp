#include "ipg1back.h"
#include "ipg2back.h"
#include "esp_log.h"
// Include for plugins of chip 0
// Include external libraries and files
#include <SPI.h>
#include <Arduino_GFX_Library.h>
#include <displayfk.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <LinkerWX.h>


    /* Project setup:
    * MCU: ESP32S3_EMBEDDED
    * Display: ESP32-8048S070
    * Touch: GT911
    */
// Defines for font and files
#define FORMAT_SPIFFS_IF_FAILED false
const int DISPLAY_W = 800;
const int DISPLAY_H = 480;
const int TOUCH_MAP_X0 = 0;
const int TOUCH_MAP_X1 = 800;
const int TOUCH_MAP_Y0 = 0;
const int TOUCH_MAP_Y1 = 480;
const bool TOUCH_SWAP_XY = false;
const bool TOUCH_INVERT_X = false;
const bool TOUCH_INVERT_Y = false;
const int TOUCH_SCL = 20;
const int TOUCH_SDA = 19;
const int TOUCH_INT = -1;
const int TOUCH_RST = 38;
const uint8_t rotationScreen = 0; // This value can be changed depending of orientation of your screen
const bool isIPS = false; // Come display can use this as bigEndian flag
// ============================================
// WIFI / LINKER
// ============================================
const char* ssid = "Labmaker";
const char* password = "Anglo2020pp";

const char* serverIP = "168.231.99.137";
const uint16_t serverPort = 12345;

const uint16_t DEVICE_ID = 2;   // este display é o destino 1:2
const uint16_t GRUPO = 1;
const String TOKEN = "987654321";

WiFiClient client;
LinkerWX linker;

bool deviceRegistered = false;
uint32_t lastReconnectCheck = 0;
uint32_t lastRegisterTime = 0;
const uint32_t RECONNECT_CHECK_INTERVAL = 30000;

struct MotorData {
    float fa = 0.0f;    // frequencia atual
    float fs = 0.0f;    // frequencia setada
    float v = 0.0f;     // velocidade
    uint32_t rpm = 0;   // rpm
    float tq = 0.0f;    // torque
    float p = 0.0f;     // potencia
    float vdc = 0.0f;   // tensao dc
    float vs = 0.0f;    // tensao saida
    float i = 0.0f;     // corrente
    float tm = 0.0f;    // temp motor
    float a1 = 0.0f;
    float a2 = 0.0f;
    float a3 = 0.0f;
    uint16_t edo = 0;
    uint16_t er = 0;
    uint32_t tl = 0;
    float tr = 0.0f;
    uint16_t cf = 0;
    uint16_t inf = 0;
    bool fresh = false;
    uint32_t lastUpdateMs = 0;
};

MotorData motorData;

enum ProgState {
    PROG_IDLE,
    PROG_WAITING,    // aguardando tempo de inicio
    PROG_RUNNING,    // motor ligado
    PROG_STOPPED     // motor parado (ciclo)
};

struct ProgConfig {
    float    freqMax;       // Hz máximo configurado
    float    rpmMax;        // RPM máximo configurado
    uint32_t startDelay;    // segundos até iniciar
    uint32_t runTime;       // segundos ligado
    uint32_t stopTime;      // segundos parado
};

static ProgState  progState    = PROG_IDLE;
static ProgConfig progCfg      = {};
static uint32_t   progStateMs  = 0;
static bool       progActive   = false;

// Prototypes for each screen
void screen0();
void screen1();
void screen2();
void loadWidgets();
bool conectarWiFi();
bool conectarServidorTCP();
bool isWiFiConnected();
bool isServerConnected();
void sendRegisterCommand();
void checkReconnectServer();
void sendKeepAliveCommand();
void readReceivedData();
void parseTelemetry(String dados);

void onSRV(const SRVResponse& response);
void onRECV(const RECVResponse& response);
void onERR(const ERRResponse& response);
void onDEST(const DESTResponse& response);
void onError(ErrorCode errorCode, const char* command);
void onRECVOk();
void sendCommandToMotor(const String& cmd);

// Prototypes for callback functions
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

float calcFreqFromRpm(float rpmDesejado, float rpmMax, float freqMax);
void progStart(int progNum);
void progStop();
void progRoutine();

#define DISP_LED 2
Arduino_ESP32RGBPanel *bus = nullptr;
Arduino_RGB_Display *tft = nullptr;
DisplayFK myDisplay;
// Create global objects. Constructor is: xPos, yPos and indexScreen
GaugeSuper gaugerpm(220, 150, 0);
const uint8_t qtdGauge = 1;
GaugeSuper *arrayGauge[qtdGauge] = {&gaugerpm};
const uint8_t qtdIntervalG0 = 4;
int range0[qtdIntervalG0] = {0,450,900,1350};
uint16_t colors0[qtdIntervalG0] = {CFK_COLOR025,CFK_COLOR019,CFK_COLOR013,CFK_COLOR007};
HSlider slidergeral(60, 400, 0);
HSlider sliderprg1(55, 385, 1);
HSlider sliderprg2(55, 385, 2);
const uint8_t qtdHSlider = 3;
HSlider *arrayHslider[qtdHSlider] = {&slidergeral, &sliderprg1, &sliderprg2};
int hslider0_val = 0; // Global variable that stores the value of the widget slidergeral
int hslider1_val = 0; // Global variable that stores the value of the widget sliderprg1
int hslider2_val = 0; // Global variable that stores the value of the widget sliderprg2
Label valuefact(290, 325, 0);
Label valuefset(290, 360, 0);
Label labelhslider15(225, 425, 0);
Label setrpmvalue(270, 350, 1);
Label rpmvalue(230, 420, 1);
Label text8copycopy(270, 350, 2);
Label rpmvalue6(230, 420, 2);
const uint8_t qtdLabel = 7;
Label *arrayLabel[qtdLabel] = {&valuefact, &valuefset, &labelhslider15, &setrpmvalue, &rpmvalue, &text8copycopy, &rpmvalue6};
LineChart graphfreq(414, 40, 0);
LineChart graphfreqcopycopy(414, 40, 1);
LineChart graphfreqcopycopycopy(414, 40, 2);
const uint8_t qtdLineChart = 3;
LineChart *arrayLinechart[qtdLineChart] = {&graphfreq, &graphfreqcopycopy, &graphfreqcopycopycopy};
const uint8_t qtdLinesChart0 = 2;
uint16_t colorsChart0[qtdLinesChart0] = {CFK_COLOR031,CFK_COLOR055};
const uint8_t qtdLinesChart1 = 2;
uint16_t colorsChart1[qtdLinesChart1] = {CFK_COLOR031,CFK_COLOR055};
const uint8_t qtdLinesChart2 = 2;
uint16_t colorsChart2[qtdLinesChart2] = {CFK_COLOR031,CFK_COLOR055};
NumberBox pg1starttime(275, 80, 1);
NumberBox pg1runtime(275, 130, 1);
NumberBox pg1endtime(275, 175, 1);
NumberBox pg1freqmax(275, 220, 1);
NumberBox pg1rpmmax(275, 270, 1);
NumberBox pg2starttime(275, 80, 2);
NumberBox pg2runtime(275, 130, 2);
NumberBox pg2stoptime(275, 175, 2);
NumberBox pg2freqmax(275, 220, 2);
NumberBox pg2rpmmax(275, 270, 2);
const uint8_t qtdNumberbox = 10;
NumberBox *arrayNumberbox[qtdNumberbox] = {&pg1starttime, &pg1runtime, &pg1endtime, &pg1freqmax, &pg1rpmmax, &pg2starttime, &pg2runtime, &pg2stoptime, &pg2freqmax, &pg2rpmmax};
float nb0_val = 10; // Global variable that stores the value of the widget pg1starttime
float nb1_val = 20; // Global variable that stores the value of the widget pg1runtime
float nb2_val = 10; // Global variable that stores the value of the widget pg1endtime
float nb3_val = 60.0; // Global variable that stores the value of the widget pg1freqmax
float nb4_val = 1798; // Global variable that stores the value of the widget pg1rpmmax
float nb5_val = 10; // Global variable that stores the value of the widget pg2starttime
float nb6_val = 30; // Global variable that stores the value of the widget pg2runtime
float nb7_val = 30; // Global variable that stores the value of the widget pg2stoptime
float nb8_val = 60.0; // Global variable that stores the value of the widget pg2freqmax
float nb9_val = 1798; // Global variable that stores the value of the widget pg2rpmmax
TextButton startbtn(415, 260, 0);
TextButton reversebtn(590, 260, 0);
TextButton emergcybtn(415, 325, 0);
TextButton stopbtn(590, 325, 0);
TextButton prog1btn(415, 390, 0);
TextButton prog2btn(590, 390, 0);
TextButton prog1startbtn(485, 260, 1);
TextButton stop1btn(485, 360, 1);
TextButton prog2startbtn(485, 260, 2);
TextButton stop2btn(485, 360, 2);
const uint8_t qtdTextButton = 10;
TextButton *arrayTextButton[qtdTextButton] = {&startbtn, &reversebtn, &emergcybtn, &stopbtn, &prog1btn, &prog2btn, &prog1startbtn, &stop1btn, &prog2startbtn, &stop2btn};
VAnalog analogvdc(75, 165, 0);
VAnalog analogtout(200, 165, 0);
VAnalog analogcur(315, 165, 0);
const uint8_t qtdVAnalog = 3;
VAnalog *arrayVanalog[qtdVAnalog] = {&analogvdc, &analogtout, &analogcur};
Image ipg1back(0, 0, 1);
Image ipg2back(0, 0, 2);
const uint8_t qtdImagem = 2;
Image *arrayImagem[qtdImagem] = {&ipg1back, &ipg2back};

void setup(){

    Serial.begin(115200);
    bus = new Arduino_ESP32RGBPanel(
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */,
    0 /* hsync_polarity */, 180 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    0 /* vsync_polarity */, 12 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 12000000 /* prefer_speed */, isIPS /* useBigEndian */,
    0 /* de_idle_high */, 0 /* pclk_idle_high */, 0 /* bounce_buffer_size_px */);
    tft = new Arduino_RGB_Display(
    800, 480, bus,
    rotationScreen, true);
    tft->begin();
    #if defined(DISP_LED)
    pinMode(DISP_LED, OUTPUT);
    digitalWrite(DISP_LED, HIGH);
    #endif

    if (!linker.setup(DEVICE_ID, GRUPO, TOKEN)) {
        Serial.println("Erro ao configurar LinkerWX");
        while (1) delay(1000);
    }

    const uint8_t cryptKey[] = {0xFF, 0xFE, 0x84};
    linker.setCryptKey(cryptKey, sizeof(cryptKey));

    linker.registerSRVCallback(onSRV);
    linker.registerRECVCallback(onRECV);
    linker.registerERRCallback(onERR);
    linker.registerDESTCallback(onDEST);
    linker.registerErrorCallback(onError);
    linker.registerRECVOkCallback(onRECVOk);
    myDisplay.setDrawObject(tft); // Reference to object to draw on screen
    // Setup touch
    myDisplay.setTouchCorners(TOUCH_MAP_X0, TOUCH_MAP_X1, TOUCH_MAP_Y0, TOUCH_MAP_Y1);
    myDisplay.setInvertAxis(TOUCH_INVERT_X, TOUCH_INVERT_Y);
    myDisplay.setSwapAxis(TOUCH_SWAP_XY);
    myDisplay.startTouchGT911(DISPLAY_W, DISPLAY_H, rotationScreen, TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST);
    myDisplay.enableTouchLog();
    loadWidgets(); // This function is used to setup with widget individualy
    myDisplay.loadScreen(screen0); // Use this line to change between screens
    myDisplay.createTask(false, 3); // Initialize the task to read touch and draw
    ESP_LOGI("setup","passou setup");
    esp_log_level_set("LINKER",    ESP_LOG_INFO);
    esp_log_level_set("TELEMETRY", ESP_LOG_INFO);
}

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // tempo em ms (1 segundo)
static int currentProg = 1; // Variável global ou estática para saber qual prog está ativo

void loop() {

    if (!isWiFiConnected()) {
        conectarWiFi();
    }

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        static int lastSliderVal = -1;

        // Só envia slider manual se nenhuma rotina estiver ativa
        if (!progActive) {
            if (hslider0_val != lastSliderVal) {
                lastSliderVal = hslider0_val;
                sendCommandToMotor(String(hslider0_val));
            }
        }
    }
    progRoutine(); // ← adicionar no loop
    checkReconnectServer();
    sendKeepAliveCommand();
    readReceivedData();

    myDisplay.startCustomDraw();

    if (motorData.fresh) {
        gaugerpm.setValue(motorData.rpm);

        valuefact.setText(String(motorData.fa));
        valuefset.setText(String(motorData.fs));

        setrpmvalue.setTextInt((int)((hslider1_val/100.0)*nb4_val));

        text8copycopy.setTextInt((int)((hslider2_val/100.0)*nb9_val));

        graphfreq.push(0, motorData.rpm);

        graphfreqcopycopy.push(0, motorData.rpm);

        graphfreqcopycopycopy.push(0, motorData.rpm);

        analogvdc.setValue((int)motorData.vdc, true);
        analogtout.setValue((int)motorData.vs, true);
        analogcur.setValue((int)motorData.i, true);

        motorData.fresh = false;
    }

    myDisplay.finishCustomDraw();
    bool dadosValidos = (millis() - motorData.lastUpdateMs) < 1000;

    delay(50);
}

void screen0(){

    tft->fillScreen(CFK_GREY6);
    WidgetBase::backgroundColor = CFK_GREY6;
    tft->fillRoundRect(25, 25, 750, 430, 25, CFK_GREY5);
    tft->drawRoundRect(25, 25, 750, 430, 25, CFK_BLACK);
    //This screen has a/an ponteiroAnalog
    //This screen has a/an verticalAnalog
    //This screen has a/an verticalAnalog
    //This screen has a/an verticalAnalog
    myDisplay.printText("Frequencia Atual:", 60, 327, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular12pt7b);
    myDisplay.printText("Frequencia Setada:", 60, 362, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular12pt7b);
    //This screen has a/an label
    //This screen has a/an label
    //This screen has a/an slider
    //This screen has a/an label
    myDisplay.printText("Tensao DC", 55, 306, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular8pt7b);
    myDisplay.printText("Tensao OUT", 175, 301, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular8pt7b);
    myDisplay.printText("Corrente", 305, 301, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular8pt7b);
    //This screen has a/an grafico
    //This screen has a/an textButton
    //This screen has a/an textButton
    //This screen has a/an textButton
    //This screen has a/an textButton
    myDisplay.printText("RPM", 560, 227, TL_DATUM, CFK_BLACK, CFK_GREY5, &RobotoRegular11pt7b);
    //This screen has a/an textButton
    //This screen has a/an textButton
    myDisplay.drawWidgetsOnScreen(0);
}

void screen1(){

    tft->fillScreen(CFK_GREY6);
    WidgetBase::backgroundColor = CFK_GREY6;
    tft->fillRoundRect(30, 30, 750, 430, 25, CFK_GREY5);
    tft->drawRoundRect(30, 30, 750, 430, 25, CFK_BLACK);
    //This screen has a/an grafico
    myDisplay.printText("RPM", 560, 227, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular11pt7b);
    //This screen has a/an textButton
    //This screen has a/an imagem
    myDisplay.printText("Tempo de inicio:", 50, 82, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an caixaNumero
    //This screen has a/an caixaNumero
    //This screen has a/an caixaNumero
    myDisplay.printText("Tempo ligado:", 50, 127, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Tempo parado:", 50, 177, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an slider
    myDisplay.printText("Freq. maxima:", 55, 222, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an caixaNumero
    myDisplay.printText("RPM maxima:", 55, 272, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an caixaNumero
    myDisplay.printText("SET RPM:", 105, 352, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an label
    //This screen has a/an label
    //This screen has a/an textButton
    myDisplay.drawWidgetsOnScreen(1);
}
void screen2(){

    tft->fillScreen(CFK_GREY6);
    WidgetBase::backgroundColor = CFK_GREY6;
    tft->fillRoundRect(30, 30, 750, 430, 25, CFK_GREY5);
    tft->drawRoundRect(30, 30, 750, 430, 25, CFK_BLACK);
    //This screen has a/an grafico
    myDisplay.printText("RPM", 560, 227, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular11pt7b);
    //This screen has a/an textButton
    //This screen has a/an imagem
    myDisplay.printText("Tempo de inicio:", 50, 82, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an caixaNumero
    //This screen has a/an caixaNumero
    //This screen has a/an caixaNumero
    myDisplay.printText("Tempo ligado:", 50, 127, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    myDisplay.printText("Tempo parado:", 50, 177, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an slider
    myDisplay.printText("Freq. maxima:", 55, 222, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an caixaNumero
    myDisplay.printText("RPM maxima:", 55, 272, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an caixaNumero
    myDisplay.printText("SET RPM:", 105, 352, TL_DATUM, CFK_WHITE, CFK_GREY5, &RobotoRegular13pt7b);
    //This screen has a/an label
    //This screen has a/an label
    //This screen has a/an textButton
    myDisplay.drawWidgetsOnScreen(2);
}

// Configure each widgtes to be used
void loadWidgets(){

    GaugeConfig configGauge0 = {
            .title = "RPM",
            .intervals = range0,
            .colors = colors0,
            .fontFamily = &RobotoRegular8pt7b,
            .minValue = 0,
            .maxValue = 1800,
            .width = 289,
            .height = 98,
            .borderColor = CFK_COLOR007,
            .textColor = CFK_BLACK,
            .backgroundColor = CFK_GREY6,
            .titleColor = CFK_BLACK,
            .needleColor = CFK_RED,
            .markersColor = CFK_BLACK,
            .amountIntervals = qtdIntervalG0,
            .showLabels = true
        };
    gaugerpm.setup(configGauge0);
    myDisplay.setGauge(arrayGauge,qtdGauge);


    HSliderConfig configHSlider0 = {
            .callback = slidergeral_cb,
            .subtitle = &labelhslider15,
            .minValue = 0,
            .maxValue = 100,
            .radius = 11,
            .width = 299,
            .pressedColor = CFK_GREY3,
            .backgroundColor = CFK_WHITE,
        };
    slidergeral.setup(configHSlider0);
    HSliderConfig configHSlider1 = {
            .callback = sliderprg1_cb,
            .subtitle = nullptr,
            .minValue = 0,
            .maxValue = 100,
            .radius = 14,
            .width = 342,
            .pressedColor = CFK_GREY3,
            .backgroundColor = CFK_WHITE,
        };
    sliderprg1.setup(configHSlider1);
    HSliderConfig configHSlider2 = {
            .callback = sliderprg2_cb,
            .subtitle = nullptr,
            .minValue = 0,
            .maxValue = 100,
            .radius = 14,
            .width = 342,
            .pressedColor = CFK_GREY3,
            .backgroundColor = CFK_WHITE,
        };
    sliderprg2.setup(configHSlider2);
    myDisplay.setHSlider(arrayHslider,qtdHSlider);


    LabelConfig configLabel0 = {
            .text = "",
            .prefix = "",
            .suffix = " Hz",
            .fontFamily = &RobotoRegular12pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_BLACK,
            .backgroundColor = CFK_GREY5,
        };
    valuefact.setup(configLabel0);
    LabelConfig configLabel1 = {
            .text = "",
            .prefix = "",
            .suffix = " Hz",
            .fontFamily = &RobotoRegular12pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_BLACK,
            .backgroundColor = CFK_GREY5,
        };
    valuefset.setup(configLabel1);
    LabelConfig configLabel2 = {
            .text = "",
            .prefix = "",
            .suffix = "%",
            .fontFamily = &RobotoRegular10pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_BLACK,
            .backgroundColor = CFK_GREY5,
        };
    labelhslider15.setup(configLabel2);
    LabelConfig configLabel3 = {
            .text = "900",
            .prefix = "",
            .suffix = "",
            .fontFamily = &RobotoRegular13pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_WHITE,
            .backgroundColor = CFK_GREY5,
        };
    setrpmvalue.setup(configLabel3);
    LabelConfig configLabel4 = {
            .text = "",
            .prefix = "",
            .suffix = "%",
            .fontFamily = &RobotoRegular13pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_WHITE,
            .backgroundColor = CFK_GREY5,
        };
    rpmvalue.setup(configLabel4);
    LabelConfig configLabel5 = {
            .text = "900",
            .prefix = "",
            .suffix = "",
            .fontFamily = &RobotoRegular13pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_WHITE,
            .backgroundColor = CFK_GREY5,
        };
    text8copycopy.setup(configLabel5);
    LabelConfig configLabel6 = {
            .text = "",
            .prefix = "",
            .suffix = "%",
            .fontFamily = &RobotoRegular13pt7b,
            .datum = TL_DATUM,
            .fontColor = CFK_WHITE,
            .backgroundColor = CFK_GREY5,
        };
    rpmvalue6.setup(configLabel6);
    myDisplay.setLabel(arrayLabel,qtdLabel);


    LineChartConfig configLineChart0 = {
            .colorsSeries = colorsChart0,
            .subtitles = nullptr,
            .font = &RobotoRegular5pt7b,
            .minValue = 0,
            .maxValue = 2000,
            .width = 335,
            .height = 185,
            .gridColor = CFK_BLACK,
            .borderColor = CFK_BLACK,
            .backgroundColor = CFK_GREY3,
            .textColor = CFK_WHITE,
            .verticalDivision = 10,
            .maxPointsAmount = LineChart::SHOW_ALL,
            .amountSeries = qtdLinesChart0,
            .workInBackground = false,
            .showZeroLine = true,
            .boldLine = false,
            .showDots = false
        };
    graphfreq.setup(configLineChart0);
    LineChartConfig configLineChart1 = {
            .colorsSeries = colorsChart1,
            .subtitles = nullptr,
            .font = &RobotoRegular5pt7b,
            .minValue = 0,
            .maxValue = 2000,
            .width = 335,
            .height = 185,
            .gridColor = CFK_BLACK,
            .borderColor = CFK_BLACK,
            .backgroundColor = CFK_GREY3,
            .textColor = CFK_WHITE,
            .verticalDivision = 10,
            .maxPointsAmount = LineChart::SHOW_ALL,
            .amountSeries = qtdLinesChart1,
            .workInBackground = false,
            .showZeroLine = true,
            .boldLine = true,
            .showDots = false
        };
    graphfreqcopycopy.setup(configLineChart1);
    LineChartConfig configLineChart2 = {
            .colorsSeries = colorsChart2,
            .subtitles = nullptr,
            .font = &RobotoRegular5pt7b,
            .minValue = 0,
            .maxValue = 2000,
            .width = 335,
            .height = 185,
            .gridColor = CFK_BLACK,
            .borderColor = CFK_BLACK,
            .backgroundColor = CFK_GREY3,
            .textColor = CFK_WHITE,
            .verticalDivision = 10,
            .maxPointsAmount = LineChart::SHOW_ALL,
            .amountSeries = qtdLinesChart2,
            .workInBackground = false,
            .showZeroLine = true,
            .boldLine = true,
            .showDots = false
        };
    graphfreqcopycopycopy.setup(configLineChart2);
    myDisplay.setLineChart(arrayLinechart,qtdLineChart);


    Numpad::m_backgroundColor = CFK_GREY3;
    Numpad::m_letterColor = CFK_BLACK;
    Numpad::m_keyColor = CFK_GREY13;


    NumberBoxConfig configNumberBox0 = {
            .funcPtr = screen1,
            .callback = pg1starttime_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb0_val,
            .width = 126,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg1starttime.setup(configNumberBox0);
    NumberBoxConfig configNumberBox1 = {
            .funcPtr = screen1,
            .callback = pg1runtime_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb1_val,
            .width = 127,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg1runtime.setup(configNumberBox1);
    NumberBoxConfig configNumberBox2 = {
            .funcPtr = screen1,
            .callback = pg1endtime_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb2_val,
            .width = 127,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg1endtime.setup(configNumberBox2);
    NumberBoxConfig configNumberBox3 = {
            .funcPtr = screen1,
            .callback = pg1freqmax_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb3_val,
            .width = 127,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 2
        };
    pg1freqmax.setup(configNumberBox3);
    NumberBoxConfig configNumberBox4 = {
            .funcPtr = screen1,
            .callback = pg1rpmmax_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb4_val,
            .width = 129,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg1rpmmax.setup(configNumberBox4);
    NumberBoxConfig configNumberBox5 = {
            .funcPtr = screen2,
            .callback = pg2starttime_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb5_val,
            .width = 125,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg2starttime.setup(configNumberBox5);
    NumberBoxConfig configNumberBox6 = {
            .funcPtr = screen2,
            .callback = pg2runtime_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb6_val,
            .width = 125,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg2runtime.setup(configNumberBox6);
    NumberBoxConfig configNumberBox7 = {
            .funcPtr = screen2,
            .callback = pg2stoptime_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb7_val,
            .width = 124,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg2stoptime.setup(configNumberBox7);
    NumberBoxConfig configNumberBox8 = {
            .funcPtr = screen2,
            .callback = pg2freqmax_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb8_val,
            .width = 125,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 2
        };
    pg2freqmax.setup(configNumberBox8);
    NumberBoxConfig configNumberBox9 = {
            .funcPtr = screen2,
            .callback = pg2rpmmax_cb,
            .font = &RobotoRegular10pt7b,
            .startValue = nb9_val,
            .width = 126,
            .height = 25,
            .letterColor = CFK_WHITE,
            .backgroundColor = CFK_GREY6,
            .decimalPlaces = 0
        };
    pg2rpmmax.setup(configNumberBox9);
    myDisplay.setNumberbox(arrayNumberbox,qtdNumberbox);


    TextButtonConfig configTextButton0 = {
            .text = "START",
            .callback = startbtn_cb,
            .fontFamily = &RobotoBold13pt7b,
            .width = 160,
            .height = 50,
            .radius = 10,
            .backgroundColor = CFK_COLOR025,
            .textColor = CFK_BLACK
        };

    startbtn.setup(configTextButton0);
    TextButtonConfig configTextButton1 = {
            .text = "REVERSE",
            .callback = reversebtn_cb,
            .fontFamily = &RobotoBold13pt7b,
            .width = 160,
            .height = 50,
            .radius = 10,
            .backgroundColor = CFK_COLOR050,
            .textColor = CFK_BLACK
        };

    reversebtn.setup(configTextButton1);
    TextButtonConfig configTextButton2 = {
            .text = "EMERGENCY",
            .callback = emergcybtn_cb,
            .fontFamily = &RobotoBold13pt7b,
            .width = 160,
            .height = 50,
            .radius = 10,
            .backgroundColor = CFK_COLOR007,
            .textColor = CFK_BLACK
        };

    emergcybtn.setup(configTextButton2);
    TextButtonConfig configTextButton3 = {
            .text = "STOP",
            .callback = stopbtn_cb,
            .fontFamily = &RobotoBold13pt7b,
            .width = 160,
            .height = 50,
            .radius = 10,
            .backgroundColor = CFK_COLOR019,
            .textColor = CFK_BLACK
        };

    stopbtn.setup(configTextButton3);
    TextButtonConfig configTextButton4 = {
            .text = "PROG 1",
            .callback = prog1btn_cb,
            .fontFamily = &RobotoBold13pt7b,
            .width = 160,
            .height = 50,
            .radius = 10,
            .backgroundColor = CFK_COLOR013,
            .textColor = CFK_BLACK
        };

    prog1btn.setup(configTextButton4);
    TextButtonConfig configTextButton5 = {
            .text = "PROG 2",
            .callback = prog2btn_cb,
            .fontFamily = &RobotoBold13pt7b,
            .width = 160,
            .height = 50,
            .radius = 10,
            .backgroundColor = CFK_COLOR013,
            .textColor = CFK_BLACK
        };

    prog2btn.setup(configTextButton5);
    TextButtonConfig configTextButton6 = {
            .text = "PROG 1",
            .callback = prog1startbtn_cb,
            .fontFamily = &RobotoBold14pt7b,
            .width = 200,
            .height = 80,
            .radius = 10,
            .backgroundColor = CFK_COLOR013,
            .textColor = CFK_BLACK
        };

    prog1startbtn.setup(configTextButton6);
    TextButtonConfig configTextButton7 = {
            .text = "STOP",
            .callback = stop1btn_cb,
            .fontFamily = &RobotoBold14pt7b,
            .width = 200,
            .height = 80,
            .radius = 10,
            .backgroundColor = CFK_COLOR019,
            .textColor = CFK_BLACK
        };

    stop1btn.setup(configTextButton7);
    TextButtonConfig configTextButton8 = {
            .text = "PROG 2",
            .callback = prog2startbtn_cb,
            .fontFamily = &RobotoBold14pt7b,
            .width = 200,
            .height = 80,
            .radius = 10,
            .backgroundColor = CFK_COLOR013,
            .textColor = CFK_BLACK
        };

    prog2startbtn.setup(configTextButton8);
    TextButtonConfig configTextButton9 = {
            .text = "STOP",
            .callback = stop2btn_cb,
            .fontFamily = &RobotoBold14pt7b,
            .width = 200,
            .height = 80,
            .radius = 10,
            .backgroundColor = CFK_COLOR019,
            .textColor = CFK_BLACK
        };

    stop2btn.setup(configTextButton9);
    myDisplay.setTextButton(arrayTextButton,qtdTextButton);


    VerticalAnalogConfig configVAnalog0 = {
            .minValue = 0,
            .maxValue = 400,
            .width = 43,
            .height = 135,
            .arrowColor = CFK_COLOR007,
            .textColor = CFK_BLACK,
            .backgroundColor = CFK_WHITE,
            .borderColor = CFK_BLACK,
            .steps = 10
        };
    analogvdc.setup(configVAnalog0);
    VerticalAnalogConfig configVAnalog1 = {
            .minValue = 0,
            .maxValue = 270,
            .width = 43,
            .height = 133,
            .arrowColor = CFK_COLOR007,
            .textColor = CFK_BLACK,
            .backgroundColor = CFK_WHITE,
            .borderColor = CFK_BLACK,
            .steps = 10
        };
    analogtout.setup(configVAnalog1);
    VerticalAnalogConfig configVAnalog2 = {
            .minValue = 0,
            .maxValue = 10,
            .width = 43,
            .height = 127,
            .arrowColor = CFK_COLOR007,
            .textColor = CFK_BLACK,
            .backgroundColor = CFK_WHITE,
            .borderColor = CFK_BLACK,
            .steps = 10
        };
    analogcur.setup(configVAnalog2);
    myDisplay.setVAnalog(arrayVanalog,qtdVAnalog);


    ImageFromPixelsConfig configImage0 = {
            .pixels = ipg1backPixels,
            .maskAlpha = ipg1backMask,
            .cb = ipg1back_cb,
            .width = ipg1backW,
            .height = ipg1backH,
            .backgroundColor = CFK_GREY6
        };
    ipg1back.setupFromPixels(configImage0);
    ImageFromPixelsConfig configImage1 = {
            .pixels = ipg2backPixels,
            .maskAlpha = ipg2backMask,
            .cb = ipg2back_cb,
            .width = ipg2backW,
            .height = ipg2backH,
            .backgroundColor = CFK_GREY6
        };
    ipg2back.setupFromPixels(configImage1);
    myDisplay.setImage(arrayImagem,qtdImagem);
}

// This function is a callback of this element slidergeral.
// You dont need call it. Make sure it is as short and quick as possible.
void slidergeral_cb(){
    hslider0_val = slidergeral.getValue();
}
// This function is a callback of this element sliderprg1.
// You dont need call it. Make sure it is as short and quick as possible.
void sliderprg1_cb(){
    hslider1_val = sliderprg1.getValue();
    rpmvalue.setText(String(hslider1_val));
}
// This function is a callback of this element sliderprg2.
// You dont need call it. Make sure it is as short and quick as possible.
void sliderprg2_cb(){
    hslider2_val = sliderprg2.getValue();
    rpmvalue6.setText(String(hslider2_val)); // Atualiza a porcentagem na tela da Prog 2
}
// This function is a callback of this element pg1starttime.
// You dont need call it. Make sure it is as short and quick as possible.
void pg1starttime_cb(){

    nb0_val = pg1starttime.getValue();
}
// This function is a callback of this element pg1runtime.
// You dont need call it. Make sure it is as short and quick as possible.
void pg1runtime_cb(){

    nb1_val = pg1runtime.getValue();
}
// This function is a callback of this element pg1endtime.
// You dont need call it. Make sure it is as short and quick as possible.
void pg1endtime_cb(){

    nb2_val = pg1endtime.getValue();
}
// This function is a callback of this element pg1freqmax.
// You dont need call it. Make sure it is as short and quick as possible.
void pg1freqmax_cb(){

    nb3_val = pg1freqmax.getValue();
}
// This function is a callback of this element pg1rpmmax.
// You dont need call it. Make sure it is as short and quick as possible.
void pg1rpmmax_cb(){

    nb4_val = pg1rpmmax.getValue();
}
// This function is a callback of this element pg2starttime.
// You dont need call it. Make sure it is as short and quick as possible.
void pg2starttime_cb(){

    nb5_val = pg2starttime.getValue();
}
// This function is a callback of this element pg2runtime.
// You dont need call it. Make sure it is as short and quick as possible.
void pg2runtime_cb(){

    nb6_val = pg2runtime.getValue();
}
// This function is a callback of this element pg2stoptime.
// You dont need call it. Make sure it is as short and quick as possible.
void pg2stoptime_cb(){

    nb7_val = pg2stoptime.getValue();
}
// This function is a callback of this element pg2freqmax.
// You dont need call it. Make sure it is as short and quick as possible.
void pg2freqmax_cb(){

    nb8_val = pg2freqmax.getValue();
}
// This function is a callback of this element pg2rpmmax.
// You dont need call it. Make sure it is as short and quick as possible.
void pg2rpmmax_cb(){

    nb9_val = pg2rpmmax.getValue();
}
// This function is a callback of this element startbtn.
// You dont need call it. Make sure it is as short and quick as possible.
void startbtn_cb(){
    sendCommandToMotor("START");
    Serial.println("Clicked on: startbtn_cb");
}
// This function is a callback of this element reversebtn.
// You dont need call it. Make sure it is as short and quick as possible.
void reversebtn_cb(){
    sendCommandToMotor("REVERSE");
    Serial.println("Clicked on: reversebtn_cb");
}
// This function is a callback of this element emergcybtn.
// You dont need call it. Make sure it is as short and quick as possible.
void emergcybtn_cb(){
    sendCommandToMotor("EMERGENCY");
    Serial.println("Clicked on: emergcybtn_cb");
}
// This function is a callback of this element stopbtn.
// You dont need call it. Make sure it is as short and quick as possible.
void stopbtn_cb(){
    sendCommandToMotor("STOP");
    Serial.println("Clicked on: stopbtn_cb");
}
// This function is a callback of this element prog1btn.
// You dont need call it. Make sure it is as short and quick as possible.
void prog1btn_cb(){
    myDisplay.loadScreen(screen1); // Use this line to change between screens
    Serial.print("Clicked on: ");Serial.println("prog1btn_cb");
}
// This function is a callback of this element prog2btn.
// You dont need call it. Make sure it is as short and quick as possible.
void prog2btn_cb(){
    myDisplay.loadScreen(screen2); // Use this line to change between screens
    Serial.print("Clicked on: ");Serial.println("prog2btn_cb");
}
// This function is a callback of this element prog1startbtn.
// You dont need call it. Make sure it is as short and quick as possible.
void prog1startbtn_cb() {
    progStart(1);
    Serial.println("Clicked on: prog1startbtn_cb");
}
// This function is a callback of this element stop1btn.
// You dont need call it. Make sure it is as short and quick as possible.
void stop1btn_cb() {
    sendCommandToMotor("STOP");
    progStop();
    Serial.println("Clicked on: stop1btn_cb");
}
// This function is a callback of this element prog2startbtn.
// You dont need call it. Make sure it is as short and quick as possible.
void prog2startbtn_cb(){
    progStart(2);
    Serial.println("Clicked on: prog2startbtn_cb");
}
// This function is a callback of this element stop2btn.
// You dont need call it. Make sure it is as short and quick as possible.
void stop2btn_cb(){
    sendCommandToMotor("STOP");
    progStop();
    Serial.println("Clicked on: stop2btn_cb");
}
// This function is a callback of this element ipg1back.
// You dont need call it. Make sure it is as short and quick as possible.
void ipg1back_cb(){
    myDisplay.loadScreen(screen0); // Use this line to change between screens
    // Image pg1back clicked
    Serial.println("Image pg1back clicked");
}
// This function is a callback of this element ipg2back.
// You dont need call it. Make sure it is as short and quick as possible.
void ipg2back_cb(){
    myDisplay.loadScreen(screen0); // Use this line to change between screens
    // Image pg2back clicked
    Serial.println("Image pg2back clicked");
}

bool conectarWiFi() {
    WiFi.begin(ssid, password);

    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 10) {
        delay(500);
        tentativas++;
    }

    return WiFi.status() == WL_CONNECTED;
}

bool isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool conectarServidorTCP() {
    if (!client.connect(serverIP, serverPort)) {
        return false;
    }

    deviceRegistered = false;
    return true;
}

bool isServerConnected() {
    return client.connected();
}

void sendRegisterCommand() {
    if (!isServerConnected()) return;

    uint8_t buffer[256];
    size_t packetSize = linker.getCommandRegisterBytes(buffer, sizeof(buffer));
    if (packetSize > 0) {
        client.write(buffer, packetSize);
    }
}

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

void readReceivedData() {
    while (client.available()) {
        uint8_t buffer[512];
        size_t bytesRead = client.readBytes(buffer, sizeof(buffer));
        if (bytesRead > 0) {
            linker.receiveData(buffer, bytesRead);
        }
    }
}

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

void onRECV(const RECVResponse& response) {
    ESP_LOGI("LINKER", "RECV de %d:%d -> %s",
        response.sourceGroup,
        response.sourceId,
        response.data
    );

    String dados = String(response.data);
    dados.trim();

    if (dados.length() > 0) {
        parseTelemetry(dados);
    }
}

void onERR(const ERRResponse& response) {
    ESP_LOGE("LINKER", "ERR code=%d msg=%s", response.code, response.message);

    if (response.code == 1 || response.code == 10) {
        deviceRegistered = false;
        client.stop();
        lastReconnectCheck = 0; // força reconexão imediata no próximo checkReconnectServer()
    }
}

void onDEST(const DESTResponse& response) {
    Serial.print("[DEST] ");
    Serial.print(response.ip);
    Serial.print(":");
    Serial.println(response.port);
}

void onError(ErrorCode errorCode, const char* command) {
    Serial.print("[ERROR] ");
    Serial.print((int)errorCode);
    Serial.print(" cmd=");
    Serial.println(command);
}

void onRECVOk() {
    ESP_LOGI("LINKER", "RECV_OK confirmado pelo servidor");
}

void parseTelemetry(String dados) {
    int start = 0;

    while (start < dados.length()) {
        int sep = dados.indexOf(';', start);
        if (sep == -1) sep = dados.length();

        String item = dados.substring(start, sep);
        int eq = item.indexOf('=');

        if (eq > 0) {
            String key = item.substring(0, eq);
            String value = item.substring(eq + 1);

            if (key == "fa") motorData.fa = value.toFloat();
            else if (key == "fs") motorData.fs = value.toFloat();
            else if (key == "v") motorData.v = value.toFloat();
            else if (key == "rpm") motorData.rpm = value.toInt();
            else if (key == "tq") motorData.tq = value.toFloat();
            else if (key == "p") motorData.p = value.toFloat();
            else if (key == "vdc") motorData.vdc = value.toFloat();
            else if (key == "vs") motorData.vs = value.toFloat();
            else if (key == "i") motorData.i = value.toFloat();
            else if (key == "tm") motorData.tm = value.toFloat();
            else if (key == "a1") motorData.a1 = value.toFloat();
            else if (key == "a2") motorData.a2 = value.toFloat();
            else if (key == "a3") motorData.a3 = value.toFloat();
            else if (key == "edo") motorData.edo = value.toInt();
            else if (key == "er") motorData.er = value.toInt();
            else if (key == "tl") motorData.tl = value.toInt();
            else if (key == "tr") motorData.tr = value.toFloat();
            else if (key == "cf") motorData.cf = value.toInt();
            else if (key == "if") motorData.inf = value.toInt();
        }

        start = sep + 1;
    }

    motorData.lastUpdateMs = millis();
    motorData.fresh = true;

    ESP_LOGI("TELEMETRY",
        "fa=%.1f fs=%.1f v=%.1f rpm=%lu tq=%.1f p=%.1f",
        motorData.fa, motorData.fs, motorData.v,
        (unsigned long)motorData.rpm, motorData.tq, motorData.p
    );

    ESP_LOGI("TELEMETRY",
        "vdc=%.1f vs=%.1f i=%.1f tm=%.1f",
        motorData.vdc, motorData.vs, motorData.i, motorData.tm
    );

    ESP_LOGI("TELEMETRY",
        "a1=%.1f a2=%.1f a3=%.1f edo=%u er=%u cf=%u inf=%u",
        motorData.a1, motorData.a2, motorData.a3,
        motorData.edo, motorData.er, motorData.cf, motorData.inf
    );

    ESP_LOGI("TELEMETRY",
        "tl=%lu tr=%.1f",
        (unsigned long)motorData.tl, motorData.tr
    );
}

const uint16_t TARGET_MOTOR_GROUP = 1;
const uint16_t TARGET_MOTOR_ID = 1;

void sendCommandToMotor(const String& cmd) {
    if (!deviceRegistered || !client.connected()) {
        ESP_LOGW("LINKER", "Nao conectado. Comando ignorado: %s", cmd.c_str());
        return;
    }

    // Bloqueia envio de frequência zero
    float val = cmd.toFloat();
    bool isNumeric = (cmd.length() > 0 && (isDigit(cmd[0]) || cmd[0] == '-'));
    if (isNumeric && val <= 0.0f) {
        return;
    }

    String payload = "cmd=" + cmd;
    uint8_t buffer[256];
    size_t packetSize = linker.getCommandSendToBytes(
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

// ============================================
// PROG ROUTINE
// ============================================



// Calcula frequência proporcional ao RPM desejado
float calcFreqFromRpm(float rpmDesejado, float rpmMax, float freqMax) {
    if (rpmMax <= 0) return 0.0f;
    return (rpmDesejado / rpmMax) * freqMax;
}

void progStart(int progNum) {
    currentProg = progNum; // Salva qual programa iniciou
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

void progStop() {
    if (!progActive) return;
    progActive = false;
    progState  = PROG_IDLE;
    sendCommandToMotor("STOP");
    Serial.println("[PROG] Rotina interrompida.");
}

void progRoutine() {
    if (!progActive) return;

    uint32_t elapsed = (millis() - progStateMs) / 1000;

    int valorParaEnviar = (currentProg == 1) ? hslider1_val : hslider2_val;

    switch (progState) {

        case PROG_WAITING:
            if (elapsed >= progCfg.startDelay) {
                sendCommandToMotor(String(valorParaEnviar));
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
                sendCommandToMotor(String(valorParaEnviar));
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