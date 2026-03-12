#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// Adafruit HUZZAH32 ESP32 Feather - D16 (RX), D17 (TX)
// STM32 TX -> Feather D16 (RX, GPIO 16)  |  STM32 RX -> Feather D17 (TX, GPIO 17)  |  GND ortak
// Docklight ile ayni: 115200, 8N1.
#define UART_RX 16
#define UART_TX 17
#define UART_BAUD 115200

// --- ESP32 tarafi gecikme suresi (hesaplanan) ---
// Sorgu araligi: READ_INTERVAL_MS (her bu kadar ms'de bir $A gonderilir)
// readSTM32Data icinde: READ_DELAY_MS (komut sonrasi bekleme) + cevap gelene kadar (max READ_TIMEOUT_MS)
// Loop her tur: LOOP_DELAY_MS
//
// Toplam ESP32 gecikmesi (ortalama):
//   ~ (READ_INTERVAL_MS/2) + READ_DELAY_MS + (cevap byte suresi ~3-5ms)  =>  yaklasik 25+10+4 = ~40ms
// En kotu (bir onceki okumadan hemen sonra veri uretildiyse):  ~ READ_INTERVAL_MS + READ_DELAY_MS + 50 = ~110ms
#define READ_INTERVAL_MS   50   // Kac ms'de bir sensör verisi istenir (saniyede 20 istek)
#define GESTURE_READ_MS    20   // Gesture ekranindayken daha sik oku (saniyede ~50 istek)
#define READ_DELAY_MS      12   // $A gonderdikten sonra STM32 cevabi icin bekleme (ms)
#define READ_TIMEOUT_MS    150  // Cevap gelmezse en fazla bu kadar ms bekle (timeout)
#define LOOP_DELAY_MS      5    // Her loop sonu bekleme (ms)
#define BUTTON_DEBOUNCE_MS 450  // Buton basimlari arasi min sure (ms)
// Z ekseni icin 1 tur mikrostep sayisi (STM32 Z mapping farkli oldugu icin ayrica kalibre edilir)
#define Z_MOTOR_TURN_STEPS 2000
#define SENSOR_STATUS_REFRESH_MS 100   // NTC/IR baglanti durumunu periyodik yenileme (ms)
#define GESTURE_LOOP_DELAY_MS 2 // Gesture ekranindayken daha hizli dongu
#define SCREEN_UPDATE_MS   50   // OLED yenileme araligi (ms)
#define GESTURE_SCREEN_MS  30   // Gesture ekraninda daha sik yenile
// NTC test ozel parametreleri
#define NTC_SAMPLE_COUNT        20   // NTC testi icin alinacak olcum sayisi
#define NTC_SAMPLE_INTERVAL_MS  100  // NTC testi sirasinda olcumler arasi bekleme (ms)
#define NTC_TEST_TIMEOUT_MS    5000  // NTC testi max sure (ms), asilirsa FAIL
#define NTC_STABILITY_DELTA_C   3.0f // NTC testi icin max izin verilen genel sapma (min-max farki, C)
#define NTC_STEP_DELTA_C        0.7f // Iki ardil olcum arasinda izin verilen max fark (C)
// IR Temp test ozel parametreleri (NTC ile ayni mantik)
#define IR_SAMPLE_COUNT         20   // IR testi icin alinacak olcum sayisi
#define IR_SAMPLE_INTERVAL_MS   110  // IR testi sirasinda olcumler arasi bekleme (ms) - timing stabilitesi
#define IR_TEST_TIMEOUT_MS       5000  // IR testi max sure (ms), asilirsa FAIL
#define IR_STABILITY_DELTA_C       4.0f // IR testi icin max sapma (IR gurultulu olabilir, NTC'den gevsek)
#define IR_STEP_DELTA_C            1.0f // Iki ardil olcum arasi max fark (C)
#define INTAKE_FAN_SPINUP_MS     5000  // Intake fan komutu sonrasi hata kontrolu icin bekleme
#define EXHAUST_FAN_SPINUP_MS    5000  // Exhaust fan komutu sonrasi hata kontrolu icin bekleme
#define FAN_TEST_STEP_MS          400  // Fan testinde hiz kademeleri arasi bekleme
#define FAN_TEST_SETTLE_MS       1500  // %100'de RPM olcumu oncesi bekleme
#define FAN_TEST_MIN_RPM       2500.0f // Test gecmek icin minimum RPM
#define LOADCELL_UPDATE_MS         500  // Loadcell sonuc ekraninda yenileme araligi (ms)
#define LOADCELL_TARE_WAIT_MS    7000  // $WT gonderildikten sonra makul degerler icin max bekleme suresi (ms)
#define LOADCELL_POST_TARE_READY_G 15.0f // Sonuc ekranina gecmeden once kabul edilen max mutlak deger
#define LOADCELL_POST_TARE_RETRY_DELAY_MS 300 // TARE sonrasi tekrar okumalar arasi bekleme
#define LOADCELL_VALIDATE_ROUNDS     5   // Sonuc ekrani oncesi ek dogrulama turu

// OLED Ekran - 128x64, I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Encoder Pinleri - Rotary Encoder KY-040 veya benzeri
// Yeni bağlantı (I2C hatlarından UZAK):
//   Encoder CLK -> Feather D32 (GPIO 32)
//   Encoder DT  -> Feather D33 (GPIO 33)
//   Encoder SW  -> Feather D27 (GPIO 27)
//   Encoder VCC -> 3.3V
//   Encoder GND -> GND
#define ENCODER_CLK 32  // CLK pini (GPIO 32)
#define ENCODER_DT 33   // DT pini  (GPIO 33)
#define ENCODER_SW 27   // Buton pini (GPIO 27)

// Gesture sensor degerleri
#define GESTURE_NONE  0
#define GESTURE_UP    1
#define GESTURE_DOWN  2
#define GESTURE_LEFT  3
#define GESTURE_RIGHT 4

// STM32'den okunan veriler (raw degerler, /10 yapilmis)
float mcu_load_raw = 0.0;
float pcb_temp_raw = 0.0;
float plate_temp_raw = 0.0;
float resin_temp_raw = 0.0;
float intake1_fan_raw = 0.0;
float intake2_fan_raw = 0.0;
float exhaust_fan_raw = 0.0;
// $X ile gelen hata/status degerleri (0: OK, 1: HATA)
int intake1_fan_error     = 0;
int intake2_fan_error     = 0;
int exhaust_fan_error     = 0;
int gesture_sensor_status = 0; // 0: OK, 1: HATA
int projector_sensor_status = 0; // 0: OK, 1: HATA
int force_sensor_status = 0;     // 0: OK, 1: HATA
int gesture_type          = GESTURE_NONE;      // STM32'den gelen anlik deger (0-4)
int last_gesture_type     = GESTURE_NONE;      // Ekranda gosterilecek son valid deger

// TMC Status degerleri (1 veya 0)
int z_tmc_status_stop_r = 0;
int y_tmc_status_stop_r = 0;
int y_tmc_status_stop_l = 0;
int cvr1_tmc_status_stop_r = 0;
int cvr1_tmc_status_stop_l = 0;
int cvr2_tmc_status_stop_r = 0;
int cvr2_tmc_status_stop_l = 0;

// Encoder degiskenleri
volatile int encoderPos = 0;
int lastEncoderPos = 0;
int lastCLK = 0;

// Menu sistemi
enum MenuState {
  MENU_MAIN,
  MENU_IR_TEMP,
  MENU_NTC,
  MENU_INTAKE_FAN,
  MENU_EXHAUST_FAN,
  MENU_RGB_LED,
  MENU_GESTURE,
  MENU_Z_REF,
  MENU_Y_REF,
  MENU_CVR1_REF,
  MENU_CVR2_REF,
  MENU_BRAKE_MOTOR,
  MENU_Z_MOTOR,
  MENU_Y_MOTOR,
  MENU_CVR_MOTOR,
  MENU_LOADCELL,
  MENU_PROJEKSIYON
};

MenuState currentMenu = MENU_MAIN;
int menuSelection = 0; // menü seçimi
enum FanTestPhase {
  FAN_TEST_IDLE = 0,
  FAN_TEST_RAMP_UP,
  FAN_TEST_MEASURE,
  FAN_TEST_RAMP_DOWN
};
const char* menuItems[] = {
  "IR Temp. Sensor",
  "NTC",
  "Intake Fan",
  "Exhaust Fan",
  "RGB LED",
  "Gesture Sensor",
  "Z Ref.",
  "Y Ref.",
  "CVR1 Ref.",
  "CVR2 Ref.",
  "Motor Freni",
  "Z Motor",
  "Y Motor",
  "CVR 1-2 Motor",
  "Loadcell",
  "Projection"
};
const int menuItemCount = 16;
bool screenNeedsUpdate = true;

// Intake Fan ayarlama degiskenleri
int fanSpeedPercent = 0; // 0-100 arasi, %10'luk adimlarla (0, 10, 20, ..., 100)
bool fanSpeedSent = false; // Komut gonderildi mi?
unsigned long lastIntakeFanCommandMs = 0; // Son intake fan komut zamani
int intakeFanSelection = 0; // 0: Test Et, 1: Cikis
bool intakeFanTestRunning = false;
bool intakeFanHasResult = false;
bool intakeFanStatusSuccess = false;
FanTestPhase intakeFanTestPhase = FAN_TEST_IDLE;
unsigned long intakeFanPhaseStartMs = 0;
unsigned long intakeFanLastStepMs = 0;
char intakeFanFailLabel[24] = "";

// Exhaust Fan ayarlama degiskenleri
int exhaustFanSpeedPercent = 0; // 0-100 arasi, %10'luk adimlarla (0, 10, 20, ..., 100)
bool exhaustFanSpeedSent = false; // Komut gonderildi mi?
unsigned long lastExhaustFanCommandMs = 0; // Son exhaust fan komut zamani
int exhaustFanSelection = 0; // 0: Test Et, 1: Cikis
bool exhaustFanTestRunning = false;
bool exhaustFanHasResult = false;
bool exhaustFanStatusSuccess = false;
FanTestPhase exhaustFanTestPhase = FAN_TEST_IDLE;
unsigned long exhaustFanPhaseStartMs = 0;
unsigned long exhaustFanLastStepMs = 0;
char exhaustFanFailLabel[24] = "";

// RGB LED ayarlama degiskenleri
int rgbHue = 0;        // Hue: 0-360 arasi
int rgbSaturation = 100; // Saturation: 0-100 arasi
int rgbValue = 100;     // Value/Brightness: 0-100 arasi
int rgbSelectedParam = 0; // 0: Hue, 1: Saturation, 2: Value
int rgbParamSelection = 0; // 0: Hue, 1: Saturation, 2: Value, 3: Cikis
bool rgbMode = false; // false: parametre secimi, true: deger ayarlama
bool rgbCommandSent = false; // Komut gonderildi mi?
int  rgbMenuSelection = 0;   // RGB menusu: 0 = LED Test, 1 = Cikis

// Motor freni (Brake Motor) ayarlama degiskenleri
bool brakeMotorActive = false; // false: pasif ($B0), true: aktif ($B1)
int  brakeMotorSelection = 0;  // 0: Test icin tikla, 1: Cikis

// Z Motor ayarlama degiskenleri (mikrostep tabanli)
bool zMotorEnabled = false;        // $SZE / $SZD
int  zMotorDir = 1;                // 0: geri, 1: ileri
long zMotorDistanceSteps = Z_MOTOR_TURN_STEPS;   // mikrostep cinsinden mesafe (1 tur - Z icin ayrica kalibre)
long zMotorSpeedStepsPerS = 1600;  // mikrostep/s cinsinden hiz (örn. 1 tur/s)
int  zMotorParamSelection = 0;     // 0: Durum, 1: Yon, 2: Mesafe, 3: Hiz, 4: Hareket, 5: Geri
bool zMotorEditMode = false;       // false: satir secimi, true: deger ayarlama (Yon/Mesafe/Hiz)

// Y Motor ayarlama degiskenleri (mikrostep tabanli)
bool yMotorEnabled = false;      // $SYE / $SYD
int  yMotorDir = 1;              // 0: geri, 1: ileri
long yMotorDistanceSteps = 1600; // mikrostep cinsinden mesafe (örn. 1 tur = 1600)
long yMotorSpeedStepsPerS = 1600;// mikrostep/s cinsinden hiz (örn. 1 tur/s)
int  yMotorParamSelection = 0;   // 0: Durum, 1: Yon, 2: Mesafe, 3: Hiz, 4: Hareket, 5: Geri
bool yMotorEditMode = false;     // false: satir secimi, true: deger ayarlama

// CVR Motor ayarlama degiskenleri (mikrostep tabanli, 2 motor: CVR-1 ve CVR-2)
int  cvrMotorSelected = 1;           // 1: CVR-1, 2: CVR-2
bool cvrMotorEnabled[3] = {false};   // index 1 ve 2 kullaniliyor ($S1E/$S2E, $S1D/$S2D)
int  cvrMotorDir[3] = {0, 1, 1};     // 0: geri, 1: ileri
long cvrMotorDistanceSteps[3] = {0, 1600, 1600};    // mikrostep mesafe
long cvrMotorSpeedStepsPerS[3] = {0, 1600, 1600};   // mikrostep/s hiz
int  cvrMotorParamSelection = 0;     // 0: Motor, 1: Durum, 2: Yon, 3: Mesafe, 4: Hiz, 5: Hareket, 6: Geri
bool cvrMotorEditMode = false;       // false: satir secimi, true: deger ayarlama

// Projeksiyon (LED) ayarlama / test degiskenleri
bool projeksiyonLedOn = false;       // $P1 ON / $P0 OFF
int  projeksiyonAkim = 512;          // 91-1023, $PC<deger>
// Projeksiyon test menusu durum degiskenleri
bool projectorHasResult     = false; // test yapildi mi
bool projectorStatusSuccess = false; // true: SUCCESS, false: FAIL
int  projectorSelection     = 0;     // 0: LED, 1: Akim, 2: Test, 3: Cikis
bool projectorEditMode      = false; // true: Akim ayarlama modu

// NTC test menusu durum degiskenleri
bool  ntcTestRunning   = false;  // true iken 100 olcum toplanir
int   ntcSampleCount   = 0;      // kac olcum alindi
float ntcSampleSum     = 0.0f;   // olcumlerin toplami
float ntcAverageTemp   = 0.0f;   // hesaplanan ortalama sicaklik
float ntcMinTemp       = 0.0f;   // olcumler icindeki minimum sicaklik
float ntcMaxTemp       = 0.0f;   // olcumler icindeki maksimum sicaklik
float ntcLastTemp      = 0.0f;   // bir onceki olcum
bool  ntcHasLastTemp   = false;  // onceki olcum var mi
bool  ntcHasResult     = false;  // test tamamlandi mi
bool  ntcStatusSuccess = false;  // true: SUCCESS, false: FAIL
unsigned long ntcTestStartTime = 0; // testi baslatma zamani (ms)
int   ntcSelection     = 0;      // 0: Test, 1: Cikis
int   ntcSensorStatus  = -1;     // $X komutundan gelen ham NTC status degeri
bool  ntcSensorStatusValid = false; // $X cevabi alindiysa true
bool  ntcSensorDisconnected = false; // status=1 iken true, ekranda "FAIL"

// IR Temp menusu durum degiskenleri
bool  irHasResult      = false;   // son test yapildi mi
int   irSelection      = 0;       // 0: Test, 1: Cikis
bool  irStatusSuccess  = false;  // true: SUCCESS, false: FAIL
bool  irTestRunning    = false;  // true iken 20 olcum toplanir
int   irSampleCount    = 0;      // kac olcum alindi
float irSampleSum      = 0.0f;   // olcumlerin toplami
float irAverageTemp    = 0.0f;   // hesaplanan ortalama sicaklik
float irMinTemp        = 0.0f;   // min sicaklik
float irMaxTemp        = 0.0f;   // max sicaklik
float irLastTempStep   = 0.0f;   // bir onceki olcum (ardil fark kontrolu icin)
bool  irHasLastTemp    = false;   // onceki olcum var mi
unsigned long irTestStartTime = 0;      // testi baslatma zamani (ms)
int   irSensorStatus   = -1;     // $X komutundan gelen ham IR status degeri
bool  irSensorStatusValid = false; // $X cevabi alindiysa true
bool  irSensorDisconnected = false; // status=1 iken true, ekranda "FAIL"

// Gesture menusu durum degiskenleri
bool  gestureHasResult     = false;  // test yapildi mi
bool  gestureStatusSuccess = false;  // true: SUCCESS, false: FAIL
int   gestureSelection     = 0;      // 0: Test, 1: Cikis

// Z / Y / CVR motor test menuleri icin secim degiskenleri
int   zMotorTestSelection   = 0;      // 0: Test, 1: Cikis
int   yMotorTestSelection   = 0;      // 0: Test, 1: Cikis
int   cvrMotorTestSelection = 0;      // 0: Test, 1: Cikis

// Loadcell menusu icin secim / ekran durumu
int   loadcellSelection     = 0;      // 0: Test Et, 1: Cikis (sadece menu modunda)
int   loadcellScreenMode    = 0;      // 0: menu, 1: Test sonucu (4 deger), 2: HATA ekrani
int   loadcellErrorType     = 0;      // 0: genel hata, 1: amplifier kart hatasi
int   loadcellFaultMask     = 0;      // Bit0:L1 Bit1:L2 Bit2:L3 Bit3:L4
float loadcell1_g = 0.0f, loadcell2_g = 0.0f, loadcell3_g = 0.0f, loadcell4_g = 0.0f;  // gram


static unsigned long lastRead = 0;
static unsigned long lastButtonPress = 0;
static unsigned long lastSensorStatusCheck = 0;
static unsigned long lastLoadcellUpdate = 0;

// Forward declaration
void readSTM32Data();
void IRAM_ATTR encoderISR();
void drawMenu();
void drawIRTempScreen();
void drawNTCScreen();
void drawIntakeFanScreen();
void drawExhaustFanScreen();
void drawRGBLedScreen();
void drawGestureScreen();
void drawZRefScreen();
void drawYRefScreen();
void drawCVR1RefScreen();
void drawCVR2RefScreen();
void drawBrakeMotorScreen();
void drawZMotorScreen();
void drawYMotorScreen();
void drawCVRMotorScreen();
void drawLoadcellScreen();
void drawProjeksiyonScreen();
void sendBrakeMotorCommand(bool active);
void sendProjeksiyonOn();
void sendProjeksiyonOff();
void sendProjeksiyonCurrent();
void runRGBLedTest();
void runZMotorTest();
void runYMotorTest();
void runLoadcellTest();
void sendZMotorEnable(bool enable);
void sendZMotorStop();
void sendZMotorMove();
void sendYMotorEnable(bool enable);
void sendYMotorStop();
void sendYMotorMove();
void sendCVRMotorEnable(int motor, bool enable);
void sendCVRMotorStop(int motor);
void sendCVRMotorMove(int motor);
void sendIntakeFanCommand();
void sendExhaustFanCommand();
void startIntakeFanTest();
void startExhaustFanTest();
void updateIntakeFanTest();
void updateExhaustFanTest();
void sendRGBLedCommand();
void sendGestureInit();
void updateMenu();

// Sensor durum sorgu fonksiyonlari ($X komutu)
bool getSensorStatus(int &ntcStatus, int &irStatus);
bool isNTCSensorOk();
bool isIRSensorOk();

// Helper fonksiyonlar - UI iyilestirmeleri
void drawHeader(const char* title);
void drawProgressBar(int x, int y, int width, int percent);
void drawCenteredText(int y, const char* text, int textSize = 2);
void drawStatusScreen(const char* title, const char* statusText, bool isActive);
void showStartupScreen();

void setup() {
  // OLED Ekran baslat (Serial'den once, hizli baslat)
  Wire.begin();
  delay(10); // I2C stabilizasyonu icin kucuk bekleme
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    // OLED bulunamadi, devam et
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display(); // Ekrani hemen aktif et (bos ekran)

    // Ilk olarak acilis ekranini goster (5 sn)
    showStartupScreen();
  }

  // Serial baslat (non-blocking)
  Serial.begin(115200);
  delay(50);

  // UART ve Encoder hizli baslat
  Serial1.setPins(UART_RX, UART_TX);
  Serial1.begin(UART_BAUD);
  delay(50);
  Serial1.flush();
  while (Serial1.available()) Serial1.read();
  
  // Encoder pinlerini ayarla
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  lastCLK = digitalRead(ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  
  // Ilk veriyi al
  delay(200);
  readSTM32Data();
  lastRead = millis();

  // Kurulum tamamlandiktan sonra ana menuyu hazirla ve goster
  currentMenu = MENU_MAIN;
  menuSelection = 0;
  drawMenu();
  display.display();
}

// STM32'den veri oku ve parse et
void readSTM32Data() {
  // Once buffer'daki eski/karisik veriyi temizle (getSensorStatus veya onceki okumadan kalma)
  while (Serial1.available()) {
    Serial1.read();
  }

  // $A\r\n gonder
  Serial1.print("$A\r\n");
  Serial1.flush();
  delay(READ_DELAY_MS);
  
  // Bir satir oku (\r\n gelene kadar, timeout: READ_TIMEOUT_MS)
  char buffer[96];
  int index = 0;
  unsigned long startTime = millis();
  bool lineComplete = false;
  
  while (millis() - startTime < READ_TIMEOUT_MS && index < (int)sizeof(buffer) - 1) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\r' || c == '\n') {
        buffer[index] = '\0';
        lineComplete = true;
        break;
      }
      if (c >= 32 && c < 127) {
        buffer[index++] = c;
      }
    }
    // delay yok: veri gelir gelmez okumak icin
  }
  
  if (!lineComplete || index == 0) {
    return; // Timeout veya satir tamamlanmadi
  }
  
  // Parse: $ ile baslamali, yoksa kabul etme
  if (buffer[0] != '$') {
    return; // $ ile baslamiyorsa kabul etme
  }
  
  // Sayilari parse et (15 sayi bekleniyor: 7 sensor + gesture + 7 TMC status)
  int values[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int valueIndex = 0;
  int numValue = 0;
  bool inNumber = false;
  
  for (int i = 1; i <= index; i++) {
    char c = (i < index) ? buffer[i] : '\0';
    if (c >= '0' && c <= '9') {
      numValue = numValue * 10 + (c - '0');
      inNumber = true;
    } else if (c == ',' || c == '\0' || c == '\r' || c == '\n') {
      if (inNumber && valueIndex < 15) {
        values[valueIndex++] = numValue;
        numValue = 0;
        inNumber = false;
      }
      if (c == '\0' || c == '\r' || c == '\n') break;
    }
  }
  
  // Son sayiyi ekle (eğer varsa)
  if (inNumber && valueIndex < 15) {
    values[valueIndex++] = numValue;
  }
  
  // En az 4 sayi varsa tum sensörleri guncelle
  if (valueIndex >= 4) {
    mcu_load_raw   = values[0] / 10.0;
    pcb_temp_raw   = values[1] / 10.0;
    plate_temp_raw = values[2] / 10.0;
    resin_temp_raw = values[3] / 10.0;

    // NTC/IR baglanti durumu: $A verisinden aninda tespit (50ms'de bir - ekran guncellemesi icin)
    if (currentMenu == MENU_NTC && !ntcTestRunning) {
      if (plate_temp_raw < -20.0f || plate_temp_raw > 150.0f || plate_temp_raw == 255.0f) {
        ntcSensorStatus = 1;
        ntcSensorStatusValid = true;
      } else if (plate_temp_raw >= 0.1f && plate_temp_raw <= 99.9f) {
        ntcSensorStatus = 0;
        ntcSensorStatusValid = true;
      }
      screenNeedsUpdate = true;
    }
    if (currentMenu == MENU_IR_TEMP && !irTestRunning) {
      // Sadece BAGLI guncelle; YOK $A'dan set etme (IR gurultulu olabilir, yanlis FAIL onleme)
      if (resin_temp_raw >= 0.0f && resin_temp_raw <= 99.9f) {
        irSensorStatus = 0;
        irSensorStatusValid = true;
      }
      screenNeedsUpdate = true;
    }

    // NTC menusu icin 20 olcumluk test toplama (yalnizca plate_temp_raw kullanilir)
    if (currentMenu == MENU_NTC && ntcTestRunning) {
      // Aralik disi deger gorursek direkt FAIL
      if (plate_temp_raw < 0.0f || plate_temp_raw > 100.0f) {
        ntcTestRunning   = false;
        ntcHasResult     = true;
        ntcStatusSuccess = false;
      } else {
        // Iki ardil olcum arasindaki farki kontrol et
        if (ntcHasLastTemp) {
          float stepDiff = plate_temp_raw - ntcLastTemp;
          if (stepDiff < 0.0f) stepDiff = -stepDiff;
          if (stepDiff > NTC_STEP_DELTA_C) {
            // Bir onceki olcumden 0.7 C'den fazla sapma: hemen FAIL
            ntcTestRunning   = false;
            ntcHasResult     = true;
            ntcStatusSuccess = false;
            drawNTCScreen();
            return;
          }
        }

        ntcLastTemp    = plate_temp_raw;
        ntcHasLastTemp = true;

        // Istatistikleri guncelle (min, max, ortalama icin)
        ntcSampleSum   += plate_temp_raw;
        ntcSampleCount += 1;
        if (ntcSampleCount == 1) {
          ntcMinTemp = plate_temp_raw;
          ntcMaxTemp = plate_temp_raw;
        } else {
          if (plate_temp_raw < ntcMinTemp) ntcMinTemp = plate_temp_raw;
          if (plate_temp_raw > ntcMaxTemp) ntcMaxTemp = plate_temp_raw;
        }
        if (ntcSampleCount >= NTC_SAMPLE_COUNT) {
          ntcAverageTemp   = ntcSampleSum / ntcSampleCount;
          float delta      = ntcMaxTemp - ntcMinTemp;
          ntcStatusSuccess = (ntcAverageTemp >= 0.0f && ntcAverageTemp <= 100.0f &&
                              delta <= NTC_STABILITY_DELTA_C);
          ntcHasResult     = true;
          ntcTestRunning   = false;
        }
      }
      // Test surecinde/bitince ekrani guncelle
      drawNTCScreen();
    }

    // IR Temp menusu icin 20 olcumluk test toplama (resin_temp_raw - NTC ile ayni mantik)
    if (currentMenu == MENU_IR_TEMP && irTestRunning) {
      if (resin_temp_raw < 0.0f || resin_temp_raw > 100.0f) {
        irTestRunning   = false;
        irHasResult     = true;
        irStatusSuccess = false;
      } else {
        if (irHasLastTemp) {
          float stepDiff = resin_temp_raw - irLastTempStep;
          if (stepDiff < 0.0f) stepDiff = -stepDiff;
          if (stepDiff > IR_STEP_DELTA_C) {
            irTestRunning   = false;
            irHasResult     = true;
            irStatusSuccess = false;
            drawIRTempScreen();
            return;
          }
        }
        irLastTempStep = resin_temp_raw;
        irHasLastTemp  = true;
        irSampleSum   += resin_temp_raw;
        irSampleCount += 1;
        if (irSampleCount == 1) {
          irMinTemp = resin_temp_raw;
          irMaxTemp = resin_temp_raw;
        } else {
          if (resin_temp_raw < irMinTemp) irMinTemp = resin_temp_raw;
          if (resin_temp_raw > irMaxTemp) irMaxTemp = resin_temp_raw;
        }
        if (irSampleCount >= IR_SAMPLE_COUNT) {
          irAverageTemp   = irSampleSum / irSampleCount;
          float delta     = irMaxTemp - irMinTemp;
          irStatusSuccess = (irAverageTemp >= 0.0f && irAverageTemp <= 100.0f &&
                             delta <= IR_STABILITY_DELTA_C);
          irHasResult     = true;
          irTestRunning   = false;
        }
      }
      drawIRTempScreen();
    }

    if (valueIndex >= 5) intake1_fan_raw = values[4] / 10.0;
    if (valueIndex >= 6) intake2_fan_raw = values[5] / 10.0;
    if (valueIndex >= 7) exhaust_fan_raw = values[6] / 10.0;
    
    if (valueIndex >= 8) {
      gesture_type = values[7];
      if (gesture_type < GESTURE_NONE || gesture_type > GESTURE_RIGHT) {
        gesture_type = GESTURE_NONE;
      }
      // NONE disindaki son valid degeri ekranda tut
      if (gesture_type != GESTURE_NONE) {
        last_gesture_type = gesture_type;
      }
      // Ekranin ne zaman cizilecegini loop() belirlesin
      if (currentMenu == MENU_GESTURE) {
        screenNeedsUpdate = true;
      }
    }
    
    // TMC Status degerleri (9-15 arasi: gesture sonrasi 7 TMC status)
    // values[8] = z_tmc_status_stop_r (9. deger)
    // values[9] = y_tmc_status_stop_r (10. deger)
    // values[10] = y_tmc_status_stop_l (11. deger)
    // values[11] = cvr1_tmc_status_stop_r (12. deger)
    // values[12] = cvr1_tmc_status_stop_l (13. deger)
    // values[13] = cvr2_tmc_status_stop_r (14. deger)
    // values[14] = cvr2_tmc_status_stop_l (15. deger)
    if (valueIndex >= 9) {
      z_tmc_status_stop_r = values[8];
      if (valueIndex >= 10) y_tmc_status_stop_r = values[9];
      if (valueIndex >= 11) y_tmc_status_stop_l = values[10];
      if (valueIndex >= 12) cvr1_tmc_status_stop_r = values[11];
      if (valueIndex >= 13) cvr1_tmc_status_stop_l = values[12];
      if (valueIndex >= 14) cvr2_tmc_status_stop_r = values[13];
      if (valueIndex >= 15) cvr2_tmc_status_stop_l = values[14];
      
      // Ekran guncellemesi gerekli
      screenNeedsUpdate = true;
      
      // TMC Ref ekranlarindaysak hemen ciz (gecikmesiz)
      if (currentMenu == MENU_Z_REF) drawZRefScreen();
      else if (currentMenu == MENU_Y_REF) drawYRefScreen();
      else if (currentMenu == MENU_CVR1_REF) drawCVR1RefScreen();
      else if (currentMenu == MENU_CVR2_REF) drawCVR2RefScreen();
    }
    
    // Tek satirda hizli yazdir (cok sayida Serial.print yerine tek println)
    char line[128];
    int n = snprintf(line, sizeof(line), "%s | %.1f %.1f %.1f %.1f",
                     buffer, mcu_load_raw, pcb_temp_raw, plate_temp_raw, resin_temp_raw);
    if (valueIndex >= 5) n += snprintf(line + n, sizeof(line) - n, " %.1f", intake1_fan_raw);
    if (valueIndex >= 6) n += snprintf(line + n, sizeof(line) - n, " %.1f", intake2_fan_raw);
    if (valueIndex >= 7) n += snprintf(line + n, sizeof(line) - n, " %.1f", exhaust_fan_raw);
    if (valueIndex >= 8)
      n += snprintf(line + n, sizeof(line) - n, " g%d", gesture_type);
    if (valueIndex >= 9)
      n += snprintf(line + n, sizeof(line) - n, " z%d", z_tmc_status_stop_r);
    if (valueIndex >= 10)
      n += snprintf(line + n, sizeof(line) - n, " y%d,%d", y_tmc_status_stop_r, y_tmc_status_stop_l);
    if (valueIndex >= 12)
      n += snprintf(line + n, sizeof(line) - n, " c1%d,%d", cvr1_tmc_status_stop_r, cvr1_tmc_status_stop_l);
    if (valueIndex >= 14)
      n += snprintf(line + n, sizeof(line) - n, " c2%d,%d", cvr2_tmc_status_stop_r, cvr2_tmc_status_stop_l);
    Serial.println(line);
    
    // Ekran guncellemesi gerekli
    screenNeedsUpdate = true;
  }
}

// Encoder interrupt handler
void IRAM_ATTR encoderISR() {
  int CLK = digitalRead(ENCODER_CLK);
  int DT = digitalRead(ENCODER_DT);
  
  if (CLK != lastCLK) {
    if (DT != CLK) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    lastCLK = CLK;
  }
}

// Helper fonksiyonlar - UI iyilestirmeleri
void drawHeader(const char* title) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
}

void drawProgressBar(int x, int y, int width, int percent) {
  // Progress bar çerçevesi
  display.drawRect(x, y, width, 6, SSD1306_WHITE);
  // Dolu kısım
  int filled = (width - 2) * percent / 100;
  if (filled > 0) {
    display.fillRect(x + 1, y + 1, filled, 4, SSD1306_WHITE);
  }
}

void drawCenteredText(int y, const char* text, int textSize) {
  display.setTextSize(textSize);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.print(text);
}

void drawStatusScreen(const char* title, const char* statusText, bool isActive) {
  display.clearDisplay();
  drawHeader(title);
  
  // Durum merkezde büyük fontla
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print(statusText);
  
  // Alt bilgi
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("Durum: ");
  display.print(isActive ? "AKTIF" : "PASIF");
  
  display.display();
}

void showStartupScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Ust satir: "IQC Giris Kalite"
  drawCenteredText(24, "IQC Giris Kalite", 1);
  // Alt satir: "(Test Kiti)"
  drawCenteredText(36, "Test Kiti", 1);

  display.display();
  delay(5000); // Ilk acilista 5 saniye bekle
}

// Menu cizme fonksiyonlari
void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("MENU:");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  
  // Ekranda maksimum 6 satır gosterilebilir (64-10=54 pixel, her satir ~9 pixel)
  // Secili ogeyi ekranin ortasinda tutmak icin kaydirma
  int visibleItems = 6; // Ekranda gosterilecek maksimum oge sayisi
  int startIndex = 0;
  
  // Secili ogeyi ekranin ortasina getir
  if (menuSelection >= visibleItems / 2) {
    startIndex = menuSelection - (visibleItems / 2) + 1;
  }
  if (startIndex + visibleItems > menuItemCount) {
    startIndex = menuItemCount - visibleItems;
  }
  if (startIndex < 0) startIndex = 0;
  
  // Menü ogelerini ciz (sadece gorunur olanlar)
  int yPos = 12; // Baslik altindan basla
  for (int i = startIndex; i < startIndex + visibleItems && i < menuItemCount; i++) {
    display.setCursor(0, yPos);
    if (i == menuSelection) {
      display.print(">");
    } else {
      display.print(" ");
    }
    display.print(" ");
    display.println(menuItems[i]);
    yPos += 9; // Her satir icin yukari kaydir
  }
  
  display.display();
}

void drawIRTempScreen() {
  display.clearDisplay();
  drawCenteredText(0, "IR Temp", 1);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  if (irTestRunning) {
    display.print("TESTING");
  } else if (irSensorDisconnected) {
    display.print("FAIL");
  } else if (irHasResult) {
    display.print(irStatusSuccess ? "SUCCESS" : "FAIL");
  } else {
    display.print("BEKLEME");
  }

  display.setCursor(0, 26);
  display.print("Deger: ");
  if (irSensorDisconnected) {
    display.print("0.0 C");
  } else if (irHasResult) {
    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "%.1f C", irAverageTemp);
    display.print(tempStr);
  } else {
    display.print("--.- C");
  }

  if (irTestRunning) {
    drawCenteredText(38, "Olcum yapiliyor...", 1);
  } else {
    int y1 = 38;
    display.setCursor(0, y1);
    display.print(irSelection == 0 ? ">" : " ");
    display.print(" Test icin tikla");

    int y2 = 48;
    display.setCursor(0, y2);
    display.print(irSelection == 1 ? ">" : " ");
    display.print(" Cikis");
  }

  display.display();
}

void drawNTCScreen() {
  display.clearDisplay();
  drawCenteredText(0, "NTC Test", 1);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  if (ntcTestRunning) {
    display.print("TESTING");
  } else if (ntcSensorDisconnected) {
    display.print("FAIL");
  } else if (ntcHasResult) {
    display.print(ntcStatusSuccess ? "SUCCESS" : "FAIL");
  } else {
    display.print("BEKLEME");
  }

  display.setCursor(0, 26);
  display.print("Deger: ");
  if (ntcSensorDisconnected) {
    display.print("0.0 C");
  } else if (ntcHasResult) {
    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "%.1f C", ntcAverageTemp);
    display.print(tempStr);
  } else {
    display.print("--.- C");
  }

  if (ntcTestRunning) {
    drawCenteredText(38, "Olcum yapiliyor...", 1);
  } else {
    display.setTextSize(1);
    int y1 = 38;
    display.setCursor(0, y1);
    display.print(ntcSelection == 0 ? ">" : " ");
    display.print(" Test icin tikla");

    int y2 = 48;
    display.setCursor(0, y2);
    display.print(ntcSelection == 1 ? ">" : " ");
    display.print(" Cikis");
  }

  display.display();
}

void drawGestureScreen() {
  display.clearDisplay();
  drawHeader("Gesture Sensor");
  
  // Gesture ismi ve degeri
  const char* gestureNames[] = {"NONE", "UP", "DOWN", "LEFT", "RIGHT"};
  int g = last_gesture_type;
  if (g < 0 || g > 4) g = 0;
  
  // Durum satiri
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  if (gestureHasResult) {
    display.print(gestureStatusSuccess ? "SUCCESS" : "FAIL");
  } else {
    display.print("BEKLEME");
  }

  // Gesture adı merkezde büyük fontla (son gelen deger ekranda kalir)
  display.setTextSize(2);
  display.setCursor(0, 28);
  display.print(gestureNames[g]);

  // Alt bilgi: Test / Cikis secenekleri
  display.setTextSize(1);
  int y1 = 50;
  display.setCursor(0, y1);
  display.print(gestureSelection == 0 ? ">" : " ");
  display.print(" Test icin tikla");

  int y2 = 58;
  display.setCursor(0, y2);
  display.print(gestureSelection == 1 ? ">" : " ");
  display.print(" Cikis");
  
  display.display();
}

void drawZRefScreen() {
  display.clearDisplay();
  drawHeader("Z Optik Limit");

  // Yalnizca 0 / 1 degerini buyuk ve ortali goster
  display.setTextSize(3);
  char buf[4];
  snprintf(buf, sizeof(buf), "%d", z_tmc_status_stop_r);
  // 32 satirini kullanarak hem dikey hem yatay ortalama
  drawCenteredText(32, buf, 3);

  display.display();
}

void drawYRefScreen() {
  display.clearDisplay();
  drawHeader("Y Optik Limit");

  display.setTextSize(2);
  char lineBuf[16];

  // Right satiri
  snprintf(lineBuf, sizeof(lineBuf), "Right: %d", y_tmc_status_stop_r);
  drawCenteredText(22, lineBuf, 2);

  // Left satiri
  snprintf(lineBuf, sizeof(lineBuf), "Left : %d", y_tmc_status_stop_l);
  drawCenteredText(42, lineBuf, 2);

  display.display();
}

void drawCVR1RefScreen() {
  display.clearDisplay();
  drawHeader("CVR1 Optik Limit");

  display.setTextSize(2);
  char lineBuf[16];

  // Up satiri (cvr1_tmc_status_stop_l)
  snprintf(lineBuf, sizeof(lineBuf), "Up   : %d", cvr1_tmc_status_stop_l);
  drawCenteredText(22, lineBuf, 2);

  // Down satiri (cvr1_tmc_status_stop_r)
  snprintf(lineBuf, sizeof(lineBuf), "Down : %d", cvr1_tmc_status_stop_r);
  drawCenteredText(42, lineBuf, 2);

  display.display();
}

void drawCVR2RefScreen() {
  display.clearDisplay();
  drawHeader("CVR2 Optik Limit");

  display.setTextSize(2);
  char lineBuf[16];

  // Up satiri (cvr2_tmc_status_stop_l)
  snprintf(lineBuf, sizeof(lineBuf), "Up   : %d", cvr2_tmc_status_stop_l);
  drawCenteredText(22, lineBuf, 2);

  // Down satiri (cvr2_tmc_status_stop_r)
  snprintf(lineBuf, sizeof(lineBuf), "Down : %d", cvr2_tmc_status_stop_r);
  drawCenteredText(42, lineBuf, 2);

  display.display();
}

// Loadcell menusu: Test Et / Cikis
void drawLoadcellScreen() {
  display.clearDisplay();
  if (loadcellScreenMode == 0) {
    // Normal menu modu: Test Et / Cikis
    drawHeader("Loadcell");

    display.setTextSize(1);
    int y1 = 26;
    display.setCursor(0, y1);
    display.print(loadcellSelection == 0 ? ">" : " ");
    display.print(" Test Et");

    int y2 = 38;
    display.setCursor(0, y2);
    display.print(loadcellSelection == 1 ? ">" : " ");
    display.print(" Cikis");
  } else if (loadcellScreenMode == 1) {
    // Test sonucu: 4 loadcell degeri (gram)
    drawHeader("Loadcell Test");
    display.setTextSize(1);
    char buf[24];
    snprintf(buf, sizeof(buf), "L1: %.2f g", loadcell1_g);
    display.setCursor(0, 16);
    display.print(buf);
    snprintf(buf, sizeof(buf), "L2: %.2f g", loadcell2_g);
    display.setCursor(0, 26);
    display.print(buf);
    snprintf(buf, sizeof(buf), "L3: %.2f g", loadcell3_g);
    display.setCursor(0, 36);
    display.print(buf);
    snprintf(buf, sizeof(buf), "L4: %.2f g", loadcell4_g);
    display.setCursor(0, 46);
    display.print(buf);
    display.setCursor(0, 56);
    display.print("Buton: Cikis");
  } else if (loadcellScreenMode == 2) {
    // HATA ekrani (force_sensor_status == 1 veya $X hatasi)
    drawHeader("Loadcell");
    if (loadcellErrorType == 1) {
      display.setTextSize(1);
      drawCenteredText(24, "Amplifier Kart", 1);
      drawCenteredText(38, "Hatasi", 1);
    } else if (loadcellFaultMask != 0) {
      char line1[24] = "";
      char line2[24] = "";
      bool firstLine = true;

      for (int i = 0; i < 4; i++) {
        if (loadcellFaultMask & (1 << i)) {
          char part[8];
          snprintf(part, sizeof(part), "L%d", i + 1);
          if (firstLine) {
            if (strlen(line1) > 0) strncat(line1, " ", sizeof(line1) - strlen(line1) - 1);
            strncat(line1, part, sizeof(line1) - strlen(line1) - 1);
            if (strlen(line1) >= 5) firstLine = false;
          } else {
            if (strlen(line2) > 0) strncat(line2, " ", sizeof(line2) - strlen(line2) - 1);
            strncat(line2, part, sizeof(line2) - strlen(line2) - 1);
          }
        }
      }

      display.setTextSize(1);
      drawCenteredText(18, "Arizali Loadcell", 1);
      drawCenteredText(32, line1[0] ? line1 : "L?", 1);
      if (line2[0]) {
        drawCenteredText(44, line2, 1);
      }
    } else {
      display.setTextSize(2);
      // HATA yazisini ekranda ortala
      drawCenteredText(32, "HATA", 2);
    }
    display.setTextSize(1);
    display.setCursor(0, 56);
    display.print("Buton: Cikis");
  }

  display.display();
}

void drawBrakeMotorScreen() {
  display.clearDisplay();
  drawHeader("Motor Freni");

  // Durum satiri
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  display.print(brakeMotorActive ? "AKTIF" : "PASIF");

  // Alt menü: Test / Cikis
  int y1 = 32;
  display.setCursor(0, y1);
  display.print(brakeMotorSelection == 0 ? ">" : " ");
  display.print(" Test icin tikla");

  int y2 = 42;
  display.setCursor(0, y2);
  display.print(brakeMotorSelection == 1 ? ">" : " ");
  display.print(" Cikis");

  display.display();
}

void sendBrakeMotorCommand(bool active) {
  Serial1.print("$B");
  Serial1.print(active ? "1" : "0");
  Serial1.print("\r\n");
  Serial1.flush();
  
  // Debug
  Serial.print("Brake Motor: ");
  Serial.println(active ? "$B1" : "$B0");
}

// Motor freni testi: 5 kez ac/kapa, ekranda ilerleme 1/5, 2/5 ... goster
void runBrakeMotorTest() {
  for (int i = 1; i <= 5; ++i) {
    // Ekranda ilerlemeyi goster
    display.clearDisplay();
    drawHeader("Motor Freni");
    display.setTextSize(2);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d/5", i);
    drawCenteredText(28, buf, 2);
    display.display();

    // Motor frenini AC
    brakeMotorActive = true;
    sendBrakeMotorCommand(true);
    delay(1000);

    // Motor frenini KAPAT
    brakeMotorActive = false;
    sendBrakeMotorCommand(false);
    delay(1000);
  }

  // Test bittikten sonra normal ekrana don
  drawBrakeMotorScreen();
}

// Z Motor test ekrani: Test / Cikis
void drawZMotorScreen() {
  display.clearDisplay();
  drawHeader("Z Motor Test");

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("1 tur sol/sag, 3 hiz");

  int y1 = 32;
  display.setCursor(0, y1);
  display.print(zMotorTestSelection == 0 ? ">" : " ");
  display.print(" Test icin tikla");

  int y2 = 42;
  display.setCursor(0, y2);
  display.print(zMotorTestSelection == 1 ? ">" : " ");
  display.print(" Cikis");

  display.display();
}

void sendZMotorEnable(bool enable) {
  // Z motoru icin S on ekli protokol: $SZE / $SZD
  Serial1.print(enable ? "$SZE" : "$SZD");
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("Z Motor Enable: ");
  Serial.println(enable ? "$SZE\\r\\n" : "$SZD\\r\\n");
}

void sendZMotorStop() {
  // Z motor durdurma: $SZP
  Serial1.print("$SZP\r\n");
  Serial1.flush();
  Serial.println("Z Motor Stop: $SZP");
}

void sendZMotorMove() {
  // Format: $SZ<yon>,<mesafe>,<hiz>\r\n  (mesafe: mikrostep, hiz: mikrostep/s)
  Serial1.print("$SZ");
  Serial1.print(zMotorDir);
  Serial1.print(",");
  Serial1.print(zMotorDistanceSteps);
  Serial1.print(",");
  Serial1.print(zMotorSpeedStepsPerS);
  Serial1.print("\r\n");
  Serial1.flush();

  Serial.print("Z Motor Move: $SZ");
  Serial.print(zMotorDir);
  Serial.print(",");
  Serial.print(zMotorDistanceSteps);
  Serial.print(",");
  Serial.print(zMotorSpeedStepsPerS);
  Serial.println("\\r\\n");
}

// Y Motor test ekrani: Test / Cikis
void drawYMotorScreen() {
  display.clearDisplay();
  drawHeader("Y Motor Test");

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("1 tur sol/sag, 3 hiz");

  int y1 = 32;
  display.setCursor(0, y1);
  display.print(yMotorTestSelection == 0 ? ">" : " ");
  display.print(" Test icin tikla");

  int y2 = 42;
  display.setCursor(0, y2);
  display.print(yMotorTestSelection == 1 ? ">" : " ");
  display.print(" Cikis");

  display.display();
}

void sendYMotorEnable(bool enable) {
  Serial1.print(enable ? "$SYE" : "$SYD");
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("Y Motor Enable: ");
  Serial.println(enable ? "$SYE\\r\\n" : "$SYD\\r\\n");
}

void sendYMotorStop() {
  Serial1.print("$SYP\r\n");
  Serial1.flush();
  Serial.println("Y Motor Stop: $SYP");
}

void sendYMotorMove() {
  // Format: $SY<yon>,<mesafe>,<hiz>\r\n  (mesafe: mikrostep, hiz: mikrostep/s)
  Serial1.print("$SY");
  Serial1.print(yMotorDir);
  Serial1.print(",");
  Serial1.print(yMotorDistanceSteps);
  Serial1.print(",");
  Serial1.print(yMotorSpeedStepsPerS);
  Serial1.print("\r\n");
  Serial1.flush();

  Serial.print("Y Motor Move: $SY");
  Serial.print(yMotorDir);
  Serial.print(",");
  Serial.print(yMotorDistanceSteps);
  Serial.print(",");
  Serial.print(yMotorSpeedStepsPerS);
  Serial.println("\\r\\n");
}

// CVR 1-2 Motor test ekrani: Test / Cikis
void drawCVRMotorScreen() {
  display.clearDisplay();
  drawHeader("CVR 1-2 Motor Test");

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Iki motor: 0-360-0, 3 hiz");

  int y1 = 32;
  display.setCursor(0, y1);
  display.print(cvrMotorTestSelection == 0 ? ">" : " ");
  display.print(" Test icin tikla");

  int y2 = 42;
  display.setCursor(0, y2);
  display.print(cvrMotorTestSelection == 1 ? ">" : " ");
  display.print(" Cikis");

  display.display();
}

void sendCVRMotorEnable(int motor, bool enable) {
  Serial1.print("$S");
  Serial1.print(motor);
  Serial1.print(enable ? "E" : "D");
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("CVR");
  Serial.print(motor);
  Serial.print(" Enable: $S");
  Serial.print(motor);
  Serial.println(enable ? "E\\r\\n" : "D\\r\\n");
}

void sendCVRMotorStop(int motor) {
  Serial1.print("$S");
  Serial1.print(motor);
  Serial1.print("P\r\n");
  Serial1.flush();
  Serial.print("CVR");
  Serial.print(motor);
  Serial.println(" Stop");
}

void sendCVRMotorMove(int motor) {
  // Format: $S<MotorNo><yon>,<mesafe>,<hiz>\r\n
  Serial1.print("$S");
  Serial1.print(motor);
  Serial1.print(cvrMotorDir[motor]);
  Serial1.print(",");
  Serial1.print(cvrMotorDistanceSteps[motor]);
  Serial1.print(",");
  Serial1.print(cvrMotorSpeedStepsPerS[motor]);
  Serial1.print("\r\n");
  Serial1.flush();

  Serial.print("CVR");
  Serial.print(motor);
  Serial.print(" Move: $S");
  Serial.print(motor);
  Serial.print(cvrMotorDir[motor]);
  Serial.print(",");
  Serial.print(cvrMotorDistanceSteps[motor]);
  Serial.print(",");
  Serial.print(cvrMotorSpeedStepsPerS[motor]);
  Serial.println("\\r\\n");
}

// --- Projeksiyon (LED) ---
void sendProjeksiyonOn() {
  Serial1.print("$P1\r\n");
  Serial1.flush();
  Serial.println("Projeksiyon: $P1\\r\\n (LED ON)");
}

void sendProjeksiyonOff() {
  Serial1.print("$P0\r\n");
  Serial1.flush();
  Serial.println("Projeksiyon: $P0\\r\\n (LED OFF)");
}

void sendProjeksiyonCurrent() {
  Serial1.print("$PC");
  Serial1.print(projeksiyonAkim);
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("Projeksiyon Akim: $PC");
  Serial.print(projeksiyonAkim);
  Serial.println("\\r\\n");
}

void drawProjeksiyonScreen() {
  display.clearDisplay();
  drawHeader("Projeksiyon (LED)");

  // Durum satiri
  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  if (projectorHasResult) {
    display.print(projectorStatusSuccess ? "SUCCESS" : "FAIL");
  } else {
    display.print("BEKLEME");
  }

  // Menu satirlari: LED / Akim / Test / Cikis
  int y = 28;
  display.setCursor(0, y);
  display.print(projectorSelection == 0 ? ">" : " ");
  display.print("LED : ");
  display.print(projeksiyonLedOn ? "ACIK" : "KAPALI");

  y += 10;
  display.setCursor(0, y);
  display.print(projectorSelection == 1 ? ">" : " ");
  display.print("Akim: ");
  display.print(projeksiyonAkim);
  if (projectorEditMode && projectorSelection == 1) {
    display.print(" *");
  }

  y += 10;
  display.setCursor(0, y);
  display.print(projectorSelection == 2 ? ">" : " ");
  display.print("Test icin tikla");

  y += 10;
  display.setCursor(0, y);
  display.print(projectorSelection == 3 ? ">" : " ");
  display.print("Cikis");

  display.display();
}

static const char* getFanTestPhaseLabel(FanTestPhase phase) {
  if (phase == FAN_TEST_RAMP_UP) return "YUKSEL";
  if (phase == FAN_TEST_MEASURE) return "OLCUM";
  if (phase == FAN_TEST_RAMP_DOWN) return "DUSUS";
  return "BEKLEME";
}

static void setIntakeFanFailLabel(bool f1Fail, bool f2Fail) {
  if (f1Fail && f2Fail) {
    snprintf(intakeFanFailLabel, sizeof(intakeFanFailLabel), "F1 + F2");
  } else if (f1Fail) {
    snprintf(intakeFanFailLabel, sizeof(intakeFanFailLabel), "F1");
  } else if (f2Fail) {
    snprintf(intakeFanFailLabel, sizeof(intakeFanFailLabel), "F2");
  } else {
    intakeFanFailLabel[0] = '\0';
  }
}

static void resetIntakeFanState() {
  intakeFanSelection = 0;
  intakeFanTestRunning = false;
  intakeFanHasResult = false;
  intakeFanStatusSuccess = false;
  intakeFanTestPhase = FAN_TEST_IDLE;
  intakeFanPhaseStartMs = 0;
  intakeFanLastStepMs = 0;
  intakeFanFailLabel[0] = '\0';
  fanSpeedPercent = 0;
  fanSpeedSent = false;
  intake1_fan_error = 0;
  intake2_fan_error = 0;
  intake1_fan_raw = 0.0f;
  intake2_fan_raw = 0.0f;
}

static void failIntakeFanTest(bool f1Fail, bool f2Fail) {
  intakeFanTestRunning = false;
  intakeFanHasResult = true;
  intakeFanStatusSuccess = false;
  intakeFanTestPhase = FAN_TEST_IDLE;
  setIntakeFanFailLabel(f1Fail, f2Fail);
  fanSpeedPercent = 0;
  sendIntakeFanCommand();
  drawIntakeFanScreen();
}

static void resetExhaustFanState() {
  exhaustFanSelection = 0;
  exhaustFanTestRunning = false;
  exhaustFanHasResult = false;
  exhaustFanStatusSuccess = false;
  exhaustFanTestPhase = FAN_TEST_IDLE;
  exhaustFanPhaseStartMs = 0;
  exhaustFanLastStepMs = 0;
  exhaustFanFailLabel[0] = '\0';
  exhaustFanSpeedPercent = 0;
  exhaustFanSpeedSent = false;
  exhaust_fan_error = 0;
  exhaust_fan_raw = 0.0f;
}

static void failExhaustFanTest(const char* label) {
  exhaustFanTestRunning = false;
  exhaustFanHasResult = true;
  exhaustFanStatusSuccess = false;
  exhaustFanTestPhase = FAN_TEST_IDLE;
  snprintf(exhaustFanFailLabel, sizeof(exhaustFanFailLabel), "%s", label);
  exhaustFanSpeedPercent = 0;
  sendExhaustFanCommand();
  drawExhaustFanScreen();
}

void drawIntakeFanScreen() {
  display.clearDisplay();
  if (intakeFanTestRunning) {
    drawHeader("INTAKE TEST");
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.print(fanSpeedPercent);
    display.setTextSize(1);
    display.print("%");
    display.setCursor(78, 16);
    display.print(getFanTestPhaseLabel(intakeFanTestPhase));

    drawProgressBar(0, 32, 128, fanSpeedPercent);

    display.setCursor(0, 42);
    display.print("F1:");
    display.print(intake1_fan_raw, 0);
    display.print("  F2:");
    display.print(intake2_fan_raw, 0);

    display.setCursor(0, 52);
    display.print("RPM min ");
    display.print((int)FAN_TEST_MIN_RPM);
  } else if (intakeFanHasResult) {
    drawHeader("INTAKE TEST");
    display.setTextSize(2);
    drawCenteredText(intakeFanStatusSuccess ? 24 : 16, intakeFanStatusSuccess ? "SUCCESS" : "FAIL", 2);
    display.setTextSize(1);
    if (!intakeFanStatusSuccess && intakeFanFailLabel[0]) {
      drawCenteredText(38, intakeFanFailLabel, 1);
    }
    display.setCursor(0, 56);
    display.print("Buton: Menu");
  } else {
    drawHeader("INTAKE FAN");
    display.setTextSize(1);
    display.setCursor(0, 26);
    display.print(intakeFanSelection == 0 ? ">" : " ");
    display.print(" Test Et");
    display.setCursor(0, 38);
    display.print(intakeFanSelection == 1 ? ">" : " ");
    display.print(" Cikis");
  }
  display.display();
}

void drawExhaustFanScreen() {
  display.clearDisplay();
  if (exhaustFanTestRunning) {
    drawHeader("EXHAUST TEST");
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.print(exhaustFanSpeedPercent);
    display.setTextSize(1);
    display.print("%");
    display.setCursor(78, 16);
    display.print(getFanTestPhaseLabel(exhaustFanTestPhase));

    drawProgressBar(0, 32, 128, exhaustFanSpeedPercent);

    display.setCursor(0, 42);
    display.print("RPM: ");
    display.print(exhaust_fan_raw, 0);
    display.setCursor(0, 52);
    display.print("RPM min ");
    display.print((int)FAN_TEST_MIN_RPM);
  } else if (exhaustFanHasResult) {
    drawHeader("EXHAUST TEST");
    display.setTextSize(2);
    drawCenteredText(exhaustFanStatusSuccess ? 24 : 16, exhaustFanStatusSuccess ? "SUCCESS" : "FAIL", 2);
    display.setTextSize(1);
    if (!exhaustFanStatusSuccess && exhaustFanFailLabel[0]) {
      drawCenteredText(38, exhaustFanFailLabel, 1);
    }
    display.setCursor(0, 56);
    display.print("Buton: Menu");
  } else {
    drawHeader("EXHAUST FAN");
    display.setTextSize(1);
    display.setCursor(0, 26);
    display.print(exhaustFanSelection == 0 ? ">" : " ");
    display.print(" Test Et");
    display.setCursor(0, 38);
    display.print(exhaustFanSelection == 1 ? ">" : " ");
    display.print(" Cikis");
  }
  display.display();
}

void drawRGBLedScreen() {
  display.clearDisplay();
  drawHeader("RGB LED");

  // Basit RGB test menusu: LED Test / Cikis
  int y1 = 26;
  display.setCursor(0, y1);
  display.print(rgbMenuSelection == 0 ? ">" : " ");
  display.print(" LED Test");

  int y2 = 38;
  display.setCursor(0, y2);
  display.print(rgbMenuSelection == 1 ? ">" : " ");
  display.print(" Cikis");

  display.display();
}

// RGB LED komutunu gonder
void sendRGBLedCommand() {
  // Format: $LA320,100,100\r\n ($ + LA + Hue,Saturation,Value)
  Serial1.print("$LA");
  Serial1.print(rgbHue);
  Serial1.print(",");
  Serial1.print(rgbSaturation);
  Serial1.print(",");
  Serial1.print(rgbValue);
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("Gonderildi: $LA");
  Serial.print(rgbHue);
  Serial.print(",");
  Serial.print(rgbSaturation);
  Serial.print(",");
  Serial.print(rgbValue);
  Serial.println("\\r\\n");
  
  rgbCommandSent = true;
}

// RGB LED test sekansi: Kirmizi, Yesil, Mavi (2 tur) + Rainbow
void runRGBLedTest() {
  // Yardimci lambda: belirli bir rengi yak ve ekranda ortali yaz
  auto showColor = [](int hue, int sat, int val, const char* label) {
    rgbHue        = hue;
    rgbSaturation = sat;
    rgbValue      = val;
    sendRGBLedCommand();

    display.clearDisplay();
    drawHeader("RGB LED TEST");
    drawCenteredText(32, label, 2);
    display.display();
  };

  // 2 tur: KIRMIZI -> YESIL -> MAVI
  for (int cycle = 0; cycle < 2; ++cycle) {
    showColor(0,   100, 100, "KIRMIZI");
    delay(1000);
    showColor(120, 100, 100, "YESIL");
    delay(1000);
    showColor(240, 100, 100, "MAVI");
    delay(1000);
  }

  // Rainbow gecisi: 5 saniye boyunca Hue 0-360 arasi don
  unsigned long start = millis();
  while (millis() - start < 5000) {
    unsigned long t = millis() - start;
    float progress = (float)t / 5000.0f;
    int hue = (int)(progress * 360.0f);
    if (hue > 360) hue = 360;

    rgbHue        = hue;
    rgbSaturation = 100;
    rgbValue      = 100;
    sendRGBLedCommand();

    display.clearDisplay();
    drawHeader("RGB LED TEST");
    drawCenteredText(32, "RAINBOW", 2);
    display.display();

    delay(80);
  }

  // Test sonunda RGB LED'i sondur
  rgbHue        = 0;
  rgbSaturation = 0;
  rgbValue      = 0;
  sendRGBLedCommand();

  // Test bittikten sonra menü ekranina geri don
  drawRGBLedScreen();
}

// Ortak yardimci: verilen parametrelerle Z motoru bir tur dondur
// NOT: Enable/disable disaridan kontrol edilir; burada sadece move ve bekleme yapilir.
static void zMotorMoveOneTurn(int dir, int speedStepsPerS) {
  zMotorDir            = dir;
  zMotorDistanceSteps  = Z_MOTOR_TURN_STEPS; // 1 tur (Z icin)
  zMotorSpeedStepsPerS = speedStepsPerS;
  sendZMotorMove();
  // Tahmini sure: mesafe / hiz (s) + pay
  unsigned long moveMs = (unsigned long)(((unsigned long)Z_MOTOR_TURN_STEPS * 1000UL) / (unsigned long)speedStepsPerS) + 300;
  delay(moveMs);
}

void runZMotorTest() {
  display.clearDisplay();
  drawHeader("Z Motor Test");
  drawCenteredText(32, "TESTING...", 2);
  display.display();

  // Test baslangicinda once durdur, sonra enable et
  sendZMotorStop();
  delay(100);
  sendZMotorEnable(true);
  delay(150);

  // Her fazda: bir tur bir yönde, sonra dur/boşluk, sonra bir tur diger yönde

  // Düsük hiz
  zMotorMoveOneTurn(1, 400);   // saga 1 tur
  sendZMotorStop();            // harekete net bir stop
  delay(800);                  // bosluk
  zMotorMoveOneTurn(0, 400);   // sola 1 tur
  sendZMotorStop();
  delay(1200);

  // Orta hiz
  zMotorMoveOneTurn(1, 800);
  sendZMotorStop();
  delay(800);
  zMotorMoveOneTurn(0, 800);
  sendZMotorStop();
  delay(1200);

  // Yüksek hiz
  zMotorMoveOneTurn(1, 1600);
  sendZMotorStop();
  delay(800);
  zMotorMoveOneTurn(0, 1600);

  sendZMotorStop();
  sendZMotorEnable(false);

  // Test bitti: ekrani guncelle
  drawZMotorScreen();
}

// Ortak yardimci: verilen parametrelerle Y motoru bir tur dondur
// NOT: Enable/disable disaridan kontrol edilir; burada sadece move ve bekleme yapilir.
static void yMotorMoveOneTurn(int dir, int speedStepsPerS) {
  yMotorDir            = dir;
  yMotorDistanceSteps  = 1600; // 1 tur
  yMotorSpeedStepsPerS = speedStepsPerS;
  sendYMotorMove();
  unsigned long moveMs = (unsigned long)((1600UL * 1000UL) / (unsigned long)speedStepsPerS) + 300;
  delay(moveMs);
}

void runYMotorTest() {
  display.clearDisplay();
  drawHeader("Y Motor Test");
  drawCenteredText(32, "TESTING...", 2);
  display.display();

  // Test baslangicinda once durdur, sonra enable et
  sendYMotorStop();
  delay(100);
  sendYMotorEnable(true);
  delay(150);

  // Her fazda: bir tur bir yönde, sonra bir tur diger yönde

  // Düsük hiz
  yMotorMoveOneTurn(1, 400);   // saga 1 tur
  delay(1500);
  yMotorMoveOneTurn(0, 400);   // sola 1 tur
  delay(2000);

  // Orta hiz
  yMotorMoveOneTurn(1, 800);
  delay(1500);
  yMotorMoveOneTurn(0, 800);
  delay(2000);

  // Yüksek hiz
  yMotorMoveOneTurn(1, 1600);
  delay(1500);
  yMotorMoveOneTurn(0, 1600);

  sendYMotorStop();
  sendYMotorEnable(false);

  // Test bitti: ekrani guncelle
  drawYMotorScreen();
}

// CVR 1-2 motorlari icin ortak: verilen hizla N tur saga veya sola
static void cvrMotorsMoveTurnsBoth(int dir, int turns, int speedStepsPerS) {
  long totalSteps = 1600L * (long)turns;
  for (int m = 1; m <= 2; ++m) {
    cvrMotorDir[m]            = dir;
    cvrMotorDistanceSteps[m]  = totalSteps;
    cvrMotorSpeedStepsPerS[m] = speedStepsPerS;
  }
  // Hareket komutlari ard arda gonderilir (neredeyse eszamanli)
  sendCVRMotorMove(1);
  sendCVRMotorMove(2);
  unsigned long moveMs = (unsigned long)((totalSteps * 1000L) / (long)speedStepsPerS) + 500;
  delay(moveMs);
}

void runCVRMotorTest() {
  display.clearDisplay();
  drawHeader("CVR 1-2 Motor Test");
  drawCenteredText(32, "TESTING...", 2);
  display.display();

  // Baslangicta stop + enable
  sendCVRMotorStop(1);
  sendCVRMotorStop(2);
  delay(100);
  sendCVRMotorEnable(1, true);
  sendCVRMotorEnable(2, true);
  delay(150);

  // HIZLI MOD: 40 tur saga, sonra 40 tur sola (her iki motor birlikte)
  int fastSpeed = 4000; // biraz daha hizli

  // 40 tur saga
  cvrMotorsMoveTurnsBoth(1, 40, fastSpeed);
  sendCVRMotorStop(1);
  sendCVRMotorStop(2);
  delay(500);

  // 40 tur sola
  cvrMotorsMoveTurnsBoth(0, 40, fastSpeed);
  sendCVRMotorStop(1);
  sendCVRMotorStop(2);

  // Test sonunda motorlari disable et
  sendCVRMotorEnable(1, false);
  sendCVRMotorEnable(2, false);

  // Test bitti: ekrani guncelle
  drawCVRMotorScreen();
}

// Gesture sensör konfigürasyon komutu ($I)
void sendGestureInit() {
  Serial1.print("$I\r\n");
  Serial1.flush();
  Serial.println("Gesture init: $I\\r\\n");
}

static void sendLoadcellConfig() {
  sendGestureInit();
  delay(500);
}

static int getLoadcellFaultMask(float v1, float v2, float v3, float v4) {
  int faultMask = 0;
  if (v1 < -LOADCELL_POST_TARE_READY_G || v1 > LOADCELL_POST_TARE_READY_G) faultMask |= 1 << 0;
  if (v2 < -LOADCELL_POST_TARE_READY_G || v2 > LOADCELL_POST_TARE_READY_G) faultMask |= 1 << 1;
  if (v3 < -LOADCELL_POST_TARE_READY_G || v3 > LOADCELL_POST_TARE_READY_G) faultMask |= 1 << 2;
  if (v4 < -LOADCELL_POST_TARE_READY_G || v4 > LOADCELL_POST_TARE_READY_G) faultMask |= 1 << 3;
  return faultMask;
}

// $Wn komutu ile n. loadcell degerini oku (gram, ornek: $-152.28)
static bool readLoadcellValue(int n, float &out) {
  while (Serial1.available()) Serial1.read();
  Serial1.print("$W");
  Serial1.print(n);
  Serial1.print("\r\n");
  Serial1.flush();
  delay(15);

  char buffer[24];
  int index = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < READ_TIMEOUT_MS && index < (int)sizeof(buffer) - 1) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\r' || c == '\n') {
        if (index > 0) {
          buffer[index] = '\0';
          if (buffer[0] == '$') {
            out = atof(buffer + 1);
            return true;
          }
          return false;
        }
      } else if (c >= 32 && c < 127) {
        buffer[index++] = c;
      }
    }
  }
  return false;
}

static bool readAllLoadcellValues(float &v1, float &v2, float &v3, float &v4, int *readFaultMask = nullptr) {
  int faultMask = 0;
  bool allReadOk = true;
  float tmp = 0.0f;

  if (readLoadcellValue(1, tmp)) {
    v1 = tmp;
  } else {
    faultMask |= 1 << 0;
    allReadOk = false;
  }
  delay(100);

  if (readLoadcellValue(2, tmp)) {
    v2 = tmp;
  } else {
    faultMask |= 1 << 1;
    allReadOk = false;
  }
  delay(100);

  if (readLoadcellValue(3, tmp)) {
    v3 = tmp;
  } else {
    faultMask |= 1 << 2;
    allReadOk = false;
  }
  delay(100);

  if (readLoadcellValue(4, tmp)) {
    v4 = tmp;
  } else {
    faultMask |= 1 << 3;
    allReadOk = false;
  }

  if (readFaultMask != nullptr) {
    *readFaultMask = faultMask;
  }

  return allReadOk;
}

static void resetLoadcellTestState() {
  loadcellErrorType = 0;
  loadcellFaultMask = 0;
  loadcell1_g = 0.0f;
  loadcell2_g = 0.0f;
  loadcell3_g = 0.0f;
  loadcell4_g = 0.0f;
  lastLoadcellUpdate = 0;
}

void runLoadcellTest() {
  // Her yeni test sifirdan baslasin
  resetLoadcellTestState();

  // Butona basar basmaz ekranda TARE goster (tare suresi boyunca ekranda kalacak)
  display.clearDisplay();
  drawHeader("Loadcell");
  display.setTextSize(2);
  drawCenteredText(32, "TARE...", 2);
  display.display();

  for (int attempt = 0; attempt < 2; attempt++) {
    // 1) Her test baslangicinda konfig gonder
    sendLoadcellConfig();

    // 2) $X ile force_sensor_status kontrolu
    int ntcDummy = 0, irDummy = 0;
    if (!getSensorStatus(ntcDummy, irDummy) || force_sensor_status == 1) {
      if (attempt == 0) {
        delay(300);
        continue;
      }
      loadcellErrorType = (force_sensor_status == 1) ? 1 : 0;
      loadcellScreenMode = 2;
      drawLoadcellScreen();
      return;
    }

    // TARE komutunu gonder
    Serial1.print("$WT\r\n");
    Serial1.flush();

    float v1 = 0.0f, v2 = 0.0f, v3 = 0.0f, v4 = 0.0f;
    bool  hasValidRead = false;
    int   observedFaultMask = 0;

    // Ilk gecici/yuksek degerler ekrana dusmesin diye, degerler makul seviyeye
    // gelene kadar TARE... ekraninda beklemeye devam et.
    unsigned long tareWaitStart = millis();
    bool valuesReady = false;

    while ((millis() - tareWaitStart) < LOADCELL_TARE_WAIT_MS) {
      int readFaultMask = 0;
      bool readOk = readAllLoadcellValues(v1, v2, v3, v4, &readFaultMask);
      observedFaultMask |= readFaultMask;
      if (!readOk && readFaultMask == 0) {
        break;
      }
      hasValidRead = true;

      valuesReady =
        (v1 >= -LOADCELL_POST_TARE_READY_G && v1 <= LOADCELL_POST_TARE_READY_G) &&
        (v2 >= -LOADCELL_POST_TARE_READY_G && v2 <= LOADCELL_POST_TARE_READY_G) &&
        (v3 >= -LOADCELL_POST_TARE_READY_G && v3 <= LOADCELL_POST_TARE_READY_G) &&
        (v4 >= -LOADCELL_POST_TARE_READY_G && v4 <= LOADCELL_POST_TARE_READY_G);

      if (valuesReady) {
        int faultMask = 0;

        // Sonucu gostermeden once 5 tur daha kontrol et; TARE... ekranda kalmaya devam eder.
        for (int round = 0; round < LOADCELL_VALIDATE_ROUNDS; round++) {
          int readFaultMask = 0;
          bool readOk = readAllLoadcellValues(v1, v2, v3, v4, &readFaultMask);
          faultMask |= readFaultMask;
          if (!readOk && readFaultMask == 0) {
            break;
          }
          faultMask |= getLoadcellFaultMask(v1, v2, v3, v4);

          if (round < (LOADCELL_VALIDATE_ROUNDS - 1)) {
            delay(LOADCELL_POST_TARE_RETRY_DELAY_MS);
          }
        }

        if (faultMask != 0) {
          loadcellErrorType = 0;
          loadcellFaultMask = faultMask;
          loadcellScreenMode = 2;
          drawLoadcellScreen();
          return;
        }

        // Son okunan degerleri global degiskenlere yaz
        loadcellFaultMask = 0;
        loadcell1_g = v1;
        loadcell2_g = v2;
        loadcell3_g = v3;
        loadcell4_g = v4;

        loadcellScreenMode = 1;
        lastLoadcellUpdate = millis();
        drawLoadcellScreen();
        return;
      }

      delay(LOADCELL_POST_TARE_RETRY_DELAY_MS);
    }

    if (hasValidRead) {
      int faultMask = observedFaultMask | getLoadcellFaultMask(v1, v2, v3, v4);
      if (faultMask != 0) {
        loadcellErrorType = 0;
        loadcellFaultMask = faultMask;
        loadcellScreenMode = 2;
        drawLoadcellScreen();
        return;
      }
    }

    if (attempt == 0) {
      delay(300);
    }
  }

  loadcellErrorType = 0;
  loadcellScreenMode = 2;
  drawLoadcellScreen();
}

// Intake fan komutlarini gonder
void sendIntakeFanCommand() {
  // Format: $F1550\r\n ($ + F + fan_no + hiz)
  // Hiz: yuzde * 1999 / 100 (0-1999 arasi)
  int fanSpeedValue = (fanSpeedPercent * 1999) / 100;
  
  // Fan 1 icin komut gonder
  Serial1.print("$F1");
  Serial1.print(fanSpeedValue);
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("Gonderildi: $F1");
  Serial.print(fanSpeedValue);
  Serial.println("\\r\\n");
  
  delay(25); // F1/F2 arasi kisa gecikme
  
  // Fan 2 icin komut gonder
  Serial1.print("$F2");
  Serial1.print(fanSpeedValue);
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print(" $F2");
  Serial.print(fanSpeedValue);
  Serial.println("\\r\\n");
  
  fanSpeedSent = true;
  lastIntakeFanCommandMs = millis();
}

// Exhaust fan komutunu gonder
void sendExhaustFanCommand() {
  // Format: $F31000\r\n ($ + F + 3 + hiz)
  // Hiz: yuzde * 1999 / 100 (0-1999 arasi)
  int exhaustFanSpeedValue = (exhaustFanSpeedPercent * 1999) / 100;
  
  // Exhaust fan icin komut gonder
  Serial1.print("$F3");
  Serial1.print(exhaustFanSpeedValue);
  Serial1.print("\r\n");
  Serial1.flush();
  Serial.print("Gonderildi: $F3");
  Serial.print(exhaustFanSpeedValue);
  Serial.println("\\r\\n");
  
  exhaustFanSpeedSent = true;
  lastExhaustFanCommandMs = millis();
}

void startIntakeFanTest() {
  int ntcDummy = 0;
  int irDummy  = 0;

  intakeFanTestRunning = false;
  intakeFanHasResult = false;
  intakeFanStatusSuccess = false;
  intakeFanFailLabel[0] = '\0';
  intake1_fan_error = 0;
  intake2_fan_error = 0;
  intake1_fan_raw = 0.0f;
  intake2_fan_raw = 0.0f;
  fanSpeedPercent = 0;
  sendIntakeFanCommand();
  delay(40);

  if (!getSensorStatus(ntcDummy, irDummy)) {
    failIntakeFanTest(false, false);
    snprintf(intakeFanFailLabel, sizeof(intakeFanFailLabel), "STATUS");
    drawIntakeFanScreen();
    return;
  }

  intakeFanTestRunning = true;
  intakeFanHasResult = false;
  intakeFanTestPhase = FAN_TEST_RAMP_UP;
  intakeFanPhaseStartMs = millis();
  intakeFanLastStepMs = intakeFanPhaseStartMs;
  drawIntakeFanScreen();
}

void startExhaustFanTest() {
  int ntcDummy = 0;
  int irDummy  = 0;

  exhaustFanTestRunning = false;
  exhaustFanHasResult = false;
  exhaustFanStatusSuccess = false;
  exhaustFanFailLabel[0] = '\0';
  exhaust_fan_error = 0;
  exhaust_fan_raw = 0.0f;
  exhaustFanSpeedPercent = 0;
  sendExhaustFanCommand();
  delay(40);

  if (!getSensorStatus(ntcDummy, irDummy)) {
    failExhaustFanTest("STATUS");
    return;
  }

  exhaustFanTestRunning = true;
  exhaustFanHasResult = false;
  exhaustFanTestPhase = FAN_TEST_RAMP_UP;
  exhaustFanPhaseStartMs = millis();
  exhaustFanLastStepMs = exhaustFanPhaseStartMs;
  drawExhaustFanScreen();
}

void updateIntakeFanTest() {
  if (!intakeFanTestRunning) return;

  unsigned long now = millis();
  if (intakeFanTestPhase == FAN_TEST_RAMP_UP) {
    if (now - intakeFanLastStepMs >= FAN_TEST_STEP_MS) {
      intakeFanLastStepMs = now;
      if (fanSpeedPercent < 100) {
        fanSpeedPercent += 10;
        if (fanSpeedPercent > 100) fanSpeedPercent = 100;
        sendIntakeFanCommand();
      }
      if (fanSpeedPercent >= 100) {
        intakeFanTestPhase = FAN_TEST_MEASURE;
        intakeFanPhaseStartMs = now;
      }
      drawIntakeFanScreen();
    }
  } else if (intakeFanTestPhase == FAN_TEST_MEASURE) {
    if (now - intakeFanPhaseStartMs >= FAN_TEST_SETTLE_MS) {
      int ntcDummy = 0;
      int irDummy  = 0;
      bool gotStatus = getSensorStatus(ntcDummy, irDummy);
      bool f1Fail = !gotStatus || (intake1_fan_error == 1) || (intake1_fan_raw < FAN_TEST_MIN_RPM);
      bool f2Fail = !gotStatus || (intake2_fan_error == 1) || (intake2_fan_raw < FAN_TEST_MIN_RPM);
      if (f1Fail || f2Fail) {
        failIntakeFanTest(f1Fail, f2Fail);
        if (!gotStatus) snprintf(intakeFanFailLabel, sizeof(intakeFanFailLabel), "STATUS");
      } else {
        intakeFanTestPhase = FAN_TEST_RAMP_DOWN;
        intakeFanLastStepMs = now;
      }
      drawIntakeFanScreen();
    }
  } else if (intakeFanTestPhase == FAN_TEST_RAMP_DOWN) {
    if (now - intakeFanLastStepMs >= FAN_TEST_STEP_MS) {
      intakeFanLastStepMs = now;
      if (fanSpeedPercent > 0) {
        fanSpeedPercent -= 10;
        if (fanSpeedPercent < 0) fanSpeedPercent = 0;
        sendIntakeFanCommand();
      }
      if (fanSpeedPercent <= 0) {
        intakeFanTestRunning = false;
        intakeFanHasResult = true;
        intakeFanStatusSuccess = true;
        intakeFanTestPhase = FAN_TEST_IDLE;
      }
      drawIntakeFanScreen();
    }
  }
}

void updateExhaustFanTest() {
  if (!exhaustFanTestRunning) return;

  unsigned long now = millis();
  if (exhaustFanTestPhase == FAN_TEST_RAMP_UP) {
    if (now - exhaustFanLastStepMs >= FAN_TEST_STEP_MS) {
      exhaustFanLastStepMs = now;
      if (exhaustFanSpeedPercent < 100) {
        exhaustFanSpeedPercent += 10;
        if (exhaustFanSpeedPercent > 100) exhaustFanSpeedPercent = 100;
        sendExhaustFanCommand();
      }
      if (exhaustFanSpeedPercent >= 100) {
        exhaustFanTestPhase = FAN_TEST_MEASURE;
        exhaustFanPhaseStartMs = now;
      }
      drawExhaustFanScreen();
    }
  } else if (exhaustFanTestPhase == FAN_TEST_MEASURE) {
    if (now - exhaustFanPhaseStartMs >= FAN_TEST_SETTLE_MS) {
      int ntcDummy = 0;
      int irDummy  = 0;
      bool gotStatus = getSensorStatus(ntcDummy, irDummy);
      bool exhFail = !gotStatus || (exhaust_fan_error == 1) || (exhaust_fan_raw < FAN_TEST_MIN_RPM);
      if (exhFail) {
        failExhaustFanTest(!gotStatus ? "STATUS" : "EXHAUST");
      } else {
        exhaustFanTestPhase = FAN_TEST_RAMP_DOWN;
        exhaustFanLastStepMs = now;
      }
      drawExhaustFanScreen();
    }
  } else if (exhaustFanTestPhase == FAN_TEST_RAMP_DOWN) {
    if (now - exhaustFanLastStepMs >= FAN_TEST_STEP_MS) {
      exhaustFanLastStepMs = now;
      if (exhaustFanSpeedPercent > 0) {
        exhaustFanSpeedPercent -= 10;
        if (exhaustFanSpeedPercent < 0) exhaustFanSpeedPercent = 0;
        sendExhaustFanCommand();
      }
      if (exhaustFanSpeedPercent <= 0) {
        exhaustFanTestRunning = false;
        exhaustFanHasResult = true;
        exhaustFanStatusSuccess = true;
        exhaustFanTestPhase = FAN_TEST_IDLE;
      }
      drawExhaustFanScreen();
    }
  }
}

void updateMenu() {
  // Encoder ile menü seçimi veya hız ayarlama
  if (encoderPos != lastEncoderPos) {
    int rawDiff = encoderPos - lastEncoderPos;
    if (rawDiff > -2 && rawDiff < 2) {
      return;
    }
    int diff = rawDiff / 2;
    lastEncoderPos += diff * 2;
    if (currentMenu == MENU_MAIN) {
      menuSelection += diff;
      if (menuSelection < 0) menuSelection = menuItemCount - 1;
      if (menuSelection >= menuItemCount) menuSelection = 0;
      drawMenu();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_INTAKE_FAN) {
      if (!intakeFanTestRunning && !intakeFanHasResult) {
        intakeFanSelection += diff;
        if (intakeFanSelection < 0) intakeFanSelection = 1;
        if (intakeFanSelection > 1) intakeFanSelection = 0;
        drawIntakeFanScreen();
      }
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_EXHAUST_FAN) {
      if (!exhaustFanTestRunning && !exhaustFanHasResult) {
        exhaustFanSelection += diff;
        if (exhaustFanSelection < 0) exhaustFanSelection = 1;
        if (exhaustFanSelection > 1) exhaustFanSelection = 0;
        drawExhaustFanScreen();
      }
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_RGB_LED) {
      // RGB LED menusu: LED Test / Cikis
      rgbMenuSelection += diff;
      if (rgbMenuSelection < 0) rgbMenuSelection = 1;
      if (rgbMenuSelection > 1) rgbMenuSelection = 0;
      drawRGBLedScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_BRAKE_MOTOR) {
      // Motor freni ekraninda: Test / Cikis secimi
      brakeMotorSelection += diff;
      if (brakeMotorSelection < 0) brakeMotorSelection = 1;
      if (brakeMotorSelection > 1) brakeMotorSelection = 0;
      drawBrakeMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_Z_MOTOR) {
      // Z MOTOR test ekraninda: Test / Cikis secimi
      zMotorTestSelection += diff;
      if (zMotorTestSelection < 0) zMotorTestSelection = 1;
      if (zMotorTestSelection > 1) zMotorTestSelection = 0;
      drawZMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_Y_MOTOR) {
      // Y MOTOR test ekraninda: Test / Cikis secimi
      yMotorTestSelection += diff;
      if (yMotorTestSelection < 0) yMotorTestSelection = 1;
      if (yMotorTestSelection > 1) yMotorTestSelection = 0;
      drawYMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_CVR_MOTOR) {
      // CVR 1-2 MOTOR test ekraninda: Test / Cikis secimi
      cvrMotorTestSelection += diff;
      if (cvrMotorTestSelection < 0) cvrMotorTestSelection = 1;
      if (cvrMotorTestSelection > 1) cvrMotorTestSelection = 0;
      drawCVRMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_LOADCELL) {
      // Loadcell ekraninda: sadece menu modunda encoder ile secim
      if (loadcellScreenMode == 0) {
        loadcellSelection += diff;
        if (loadcellSelection < 0) loadcellSelection = 1;
        if (loadcellSelection > 1) loadcellSelection = 0;
        drawLoadcellScreen();
        screenNeedsUpdate = false;
      }
    } else if (currentMenu == MENU_PROJEKSIYON) {
      // Projeksiyon ekraninda: menu secimi veya akim ayarlama
      if (projectorEditMode && projectorSelection == 1) {
        // Akim ayarlama modu
        projeksiyonAkim += diff * 10;
        if (projeksiyonAkim < 91) projeksiyonAkim = 91;
        if (projeksiyonAkim > 1023) projeksiyonAkim = 1023;
        sendProjeksiyonCurrent();
      } else {
        // Menu secimi: LED / Akim / Test / Cikis
        projectorSelection += diff;
        if (projectorSelection < 0) projectorSelection = 3;
        if (projectorSelection > 3) projectorSelection = 0;
      }
      drawProjeksiyonScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_NTC) {
      // NTC ekraninda encoder ile alt secenekler (Test / Cikis) arasında gez
      if (!ntcTestRunning) {
        ntcSelection += diff;
        if (ntcSelection < 0) ntcSelection = 1;
        if (ntcSelection > 1) ntcSelection = 0;
        drawNTCScreen();
        screenNeedsUpdate = false;
      }
    } else if (currentMenu == MENU_IR_TEMP) {
      if (!irTestRunning) {
        irSelection += diff;
        if (irSelection < 0) irSelection = 1;
        if (irSelection > 1) irSelection = 0;
        drawIRTempScreen();
      }
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_GESTURE) {
      // Gesture ekraninda encoder ile alt secenekler (Test / Cikis) arasında gez
      gestureSelection += diff;
      if (gestureSelection < 0) gestureSelection = 1;
      if (gestureSelection > 1) gestureSelection = 0;
      drawGestureScreen();
      screenNeedsUpdate = false;
    }
  }
  
  // Buton ile seçim/geri dön
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(ENCODER_SW);
  
  // Buton basıldı (HIGH -> LOW geçişi, pull-up olduğu için LOW = basılı)
  if (lastButtonState == HIGH && currentButtonState == LOW && millis() - lastButtonPress > BUTTON_DEBOUNCE_MS) {
    lastButtonPress = millis();
    delay(25); // Donanim debounce
    
    if (currentMenu == MENU_MAIN) {
      // Menüden seçim yap
      if (menuSelection == 0) {
        currentMenu = MENU_IR_TEMP;
        irHasResult      = false;
        irTestRunning    = false;
        irSampleCount    = 0;
        irSampleSum      = 0.0f;
        irAverageTemp    = 0.0f;
        irSelection      = 0;
        irSensorStatus   = -1;
        irSensorStatusValid = false;
        int ntcStatusTemp = 0;
        if (getSensorStatus(ntcStatusTemp, irSensorStatus)) {
          irSensorStatusValid = true;
          irSensorDisconnected = (irSensorStatus == 1);
        }
        lastSensorStatusCheck = millis();
        delay(40);
        drawIRTempScreen();
      } else if (menuSelection == 1) {
        currentMenu = MENU_NTC;
        // NTC test durumunu sifirla
        ntcTestRunning   = false;
        ntcSampleCount   = 0;
        ntcSampleSum     = 0.0f;
        ntcAverageTemp   = 0.0f;
        ntcHasResult     = false;
        ntcStatusSuccess = false;
        ntcTestStartTime = 0;
        ntcSelection     = 0; // varsayilan secim: Test
        // Menüye girerken $X komutunu gonder ve NTC sensor durumunu oku
        ntcSensorStatus      = -1;
        ntcSensorStatusValid = false;
        int irStatusTemp     = 0;
        if (getSensorStatus(ntcSensorStatus, irStatusTemp)) {
          ntcSensorStatusValid = true;
          ntcSensorDisconnected = (ntcSensorStatus == 1);
        }
        lastSensorStatusCheck = millis();
        delay(40);
        drawNTCScreen();
      } else if (menuSelection == 2) {
        currentMenu = MENU_INTAKE_FAN;
        resetIntakeFanState();
        lastEncoderPos = encoderPos;
        drawIntakeFanScreen();
      } else if (menuSelection == 3) {
        currentMenu = MENU_EXHAUST_FAN;
        resetExhaustFanState();
        lastEncoderPos = encoderPos;
        drawExhaustFanScreen();
      } else if (menuSelection == 4) {
        currentMenu = MENU_RGB_LED;
        rgbHue = 0; // Hue 0 ile baslat
        rgbSaturation = 100; // Saturation 100 ile baslat
        rgbValue = 100; // Value 100 ile baslat
        rgbSelectedParam = 0; // Hue secili
        rgbParamSelection = 0; // Parametre secimi: Hue
        rgbMode = false; // Parametre secimi modu
        rgbCommandSent = false; // Komut gonderilmedi
        encoderPos = 0; // Encoder pozisyonunu sifirla
        lastEncoderPos = 0; // Encoder pozisyonunu sifirla
        drawRGBLedScreen();
      } else if (menuSelection == 5) {
        currentMenu = MENU_GESTURE;
        // Gesture menüsüne girerken sensörü konfigüre et ($I)
        gesture_sensor_status = 0;
        gestureHasResult      = false;
        gestureStatusSuccess  = false;
        gestureSelection      = 0; // Varsayilan: Test
        last_gesture_type     = GESTURE_NONE;
        sendGestureInit();
        delay(40); // STM32'nin $I sonrasi hazir olmasi icin kisa bekleme
        drawGestureScreen();
      } else if (menuSelection == 6) {
        currentMenu = MENU_Z_REF;
        drawZRefScreen();
      } else if (menuSelection == 7) {
        currentMenu = MENU_Y_REF;
        drawYRefScreen();
      } else if (menuSelection == 8) {
        currentMenu = MENU_CVR1_REF;
        drawCVR1RefScreen();
      } else if (menuSelection == 9) {
        currentMenu = MENU_CVR2_REF;
        drawCVR2RefScreen();
      } else if (menuSelection == 10) {
        currentMenu = MENU_BRAKE_MOTOR;
        brakeMotorActive = false; // Pasif ile baslat
        lastEncoderPos = encoderPos; // Encoder pozisyonunu koru
        drawBrakeMotorScreen();
      } else if (menuSelection == 11) {
        currentMenu = MENU_Z_MOTOR;
        // Varsayilan Z motor parametreleri
        zMotorEnabled = false;
        zMotorDir = 1;
        zMotorDistanceSteps = Z_MOTOR_TURN_STEPS;   // 1 tur (Z icin kalibre)
        zMotorSpeedStepsPerS = 1600;  // 1 tur/s
        zMotorParamSelection = 0;
        zMotorEditMode = false;
        encoderPos = 0;
        lastEncoderPos = 0;
        Serial.println("Z Menu: enter");
        drawZMotorScreen();
      } else if (menuSelection == 12) {
        currentMenu = MENU_Y_MOTOR;
        // Varsayilan Y motor parametreleri
        yMotorEnabled = false;
        yMotorDir = 1;
        yMotorDistanceSteps = 1600;   // 1 tur
        yMotorSpeedStepsPerS = 1600;  // 1 tur/s
        yMotorParamSelection = 0;
        yMotorEditMode = false;
        encoderPos = 0;
        lastEncoderPos = 0;
        drawYMotorScreen();
      } else if (menuSelection == 13) {
        currentMenu = MENU_CVR_MOTOR;
        // Varsayilan CVR motor parametreleri
        cvrMotorSelected = 1;
        cvrMotorEnabled[1] = false;
        cvrMotorEnabled[2] = false;
        cvrMotorDir[1] = 1;
        cvrMotorDir[2] = 1;
        cvrMotorDistanceSteps[1] = 1600;
        cvrMotorDistanceSteps[2] = 1600;
        cvrMotorSpeedStepsPerS[1] = 1600;
        cvrMotorSpeedStepsPerS[2] = 1600;
        cvrMotorParamSelection = 0;
        cvrMotorEditMode = false;
        encoderPos = 0;
        lastEncoderPos = 0;
        drawCVRMotorScreen();
      } else if (menuSelection == 14) {
        currentMenu = MENU_LOADCELL;
        loadcellSelection = 0;
        encoderPos = 0;
        lastEncoderPos = 0;
        drawLoadcellScreen();
      } else if (menuSelection == 15) {
        currentMenu = MENU_PROJEKSIYON;
        projeksiyonLedOn = false;
        projeksiyonAkim = 512;
        projectorHasResult     = false;
        projectorStatusSuccess = false;
        projectorSelection     = 0; // Varsayilan: LED satiri
        projectorEditMode      = false;
        // Baslangic sirasi: $PF -> 500ms -> $I -> $X
        Serial1.print("$PF\r\n");
        Serial1.flush();
        Serial.println("Projeksiyon: $PF\\r\\n (OFF)");
        delay(500);
        // $I komutu projektoru de ayarlar
        sendGestureInit();
        delay(40);
        int ntcDummy = 0;
        int irDummy  = 0;
        if (getSensorStatus(ntcDummy, irDummy)) {
          projectorHasResult     = true;
          projectorStatusSuccess = (projector_sensor_status == 0);
        }
        encoderPos = 0;
        lastEncoderPos = 0;
        drawProjeksiyonScreen();
      }
    } else if (currentMenu == MENU_NTC) {
      // NTC menusu: buton islemleri
      if (!ntcTestRunning) {
        if (ntcSelection == 0) {
          // Test icin tikla: once sensor durumunu kontrol et
          bool sensorOk = false;

          if (ntcSensorStatusValid) {
            // Menüye girerken okunmus $X sonucunu kullan
            sensorOk = (ntcSensorStatus == 0);
          } else {
            // Henuz okunmadiysa bir kez $X ile dene ve sonucu cache'le
            sensorOk = isNTCSensorOk();
            ntcSensorStatus      = sensorOk ? 0 : 1;
            ntcSensorStatusValid = true;
          }

          if (!sensorOk) {
            ntcTestRunning   = false;
            ntcHasResult     = true;
            ntcStatusSuccess = false;
          } else {
            // Sensor saglam ise NTC testini bastan baslat
            ntcTestRunning   = true;
            ntcSampleCount   = 0;
            ntcSampleSum     = 0.0f;
            ntcAverageTemp   = 0.0f;
            ntcMinTemp       = 0.0f;
            ntcMaxTemp       = 0.0f;
            ntcLastTemp      = 0.0f;
            ntcHasLastTemp   = false;
            ntcHasResult     = false;
            ntcStatusSuccess = true; // baslangicta OK, olcumler bozar ise FAIL olur
            ntcTestStartTime = millis();
          }
          // Sensor durum/NTC test bilgilerini ekrana yansıt
          drawNTCScreen();
        } else if (ntcSelection == 1) {
          // Cikis: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      }
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_INTAKE_FAN) {
      if (intakeFanTestRunning) {
        drawIntakeFanScreen();
      } else if (intakeFanHasResult) {
        resetIntakeFanState();
        drawIntakeFanScreen();
      } else if (intakeFanSelection == 0) {
        startIntakeFanTest();
      } else {
        fanSpeedPercent = 0;
        sendIntakeFanCommand();
        resetIntakeFanState();
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_EXHAUST_FAN) {
      if (exhaustFanTestRunning) {
        drawExhaustFanScreen();
      } else if (exhaustFanHasResult) {
        resetExhaustFanState();
        drawExhaustFanScreen();
      } else if (exhaustFanSelection == 0) {
        startExhaustFanTest();
      } else {
        exhaustFanSpeedPercent = 0;
        sendExhaustFanCommand();
        resetExhaustFanState();
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_RGB_LED) {
      // RGB LED menusu: LED Test veya Cikis
      if (rgbMenuSelection == 0) {
        // LED test akisi
        runRGBLedTest();
      } else if (rgbMenuSelection == 1) {
        // Cikis: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_GESTURE) {
      // Gesture menusu: Test / Cikis
      if (gestureSelection == 0) {
        // TEST akisi
        // 1) Sensörü yeniden konfigure et ($I)
        sendGestureInit();
        delay(40); // $I sonrasi kisa bekleme
        // 2) $X ile durum kontrolu
        int ntcDummy = 0;
        int irDummy  = 0;
        if (getSensorStatus(ntcDummy, irDummy)) {
          // gesture_sensor_status global olarak guncellendi
          gestureHasResult     = true;
          gestureStatusSuccess = (gesture_sensor_status == 0);
        } else {
          // $X cevabi alinmazsa FAIL kabul et
          gestureHasResult     = true;
          gestureStatusSuccess = false;
        }
        drawGestureScreen();
      } else if (gestureSelection == 1) {
        // CIKIS: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_IR_TEMP) {
      if (!irTestRunning) {
        if (irSelection == 0) {
          // Test icin tikla: $A cache kullan ($X cagirmak $A ile cakisma yapiyor, arka arkaya test bozuluyor)
          bool sensorOk = false;
          if (irSensorStatusValid) {
            sensorOk = (irSensorStatus == 0);
          } else {
            sensorOk = isIRSensorOk();
            irSensorStatus      = sensorOk ? 0 : 1;
            irSensorStatusValid = true;
          }
          if (!sensorOk) {
            irHasResult     = true;
            irStatusSuccess = false;
          } else {
            irTestRunning    = true;
            irSampleCount    = 0;
            irSampleSum      = 0.0f;
            irAverageTemp    = 0.0f;
            irMinTemp        = 0.0f;
            irMaxTemp        = 0.0f;
            irLastTempStep   = 0.0f;
            irHasLastTemp    = false;
            irHasResult      = false;
            irStatusSuccess  = true;
            irTestStartTime  = millis();
          }
          drawIRTempScreen();
        } else if (irSelection == 1) {
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      }
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_Z_REF || currentMenu == MENU_Y_REF || 
               currentMenu == MENU_CVR1_REF || currentMenu == MENU_CVR2_REF) {
      // TMC Ref ekranlarindan butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_BRAKE_MOTOR) {
      // Motor freni ekraninda: Test veya Cikis
      if (brakeMotorSelection == 0) {
        runBrakeMotorTest();
      } else {
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_Z_MOTOR) {
      // Z Motor test ekraninda: Test / Cikis
      if (zMotorTestSelection == 0) {
        // TEST akisi
        runZMotorTest();
      } else if (zMotorTestSelection == 1) {
        // Geri: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_Y_MOTOR) {
      // Y Motor test ekraninda: Test / Cikis
      if (yMotorTestSelection == 0) {
        // TEST akisi
        runYMotorTest();
      } else if (yMotorTestSelection == 1) {
        // Geri: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_CVR_MOTOR) {
      // CVR 1-2 Motor test ekraninda: Test / Cikis
      if (cvrMotorTestSelection == 0) {
        // TEST akisi (iki motor ayni anda)
        runCVRMotorTest();
      } else if (cvrMotorTestSelection == 1) {
        // CIKIS: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_PROJEKSIYON) {
      // Projeksiyon ekraninda: LED / Akim / Test / Cikis
      if (projectorSelection == 0) {
        // LED ac/kapa (sadece test SUCCESS ise izin ver)
        if (projectorStatusSuccess) {
          projeksiyonLedOn = !projeksiyonLedOn;
          if (projeksiyonLedOn) {
            sendProjeksiyonOn();
          } else {
            sendProjeksiyonOff();
          }
        }
        drawProjeksiyonScreen();
      } else if (projectorSelection == 1) {
        // Akim: edit modunu ac/kapat
        projectorEditMode = !projectorEditMode;
        drawProjeksiyonScreen();
      } else if (projectorSelection == 2) {
        // TEST akisi:
        // 1) Projektoru kapat: $PF
        Serial1.print("$PF\r\n");
        Serial1.flush();
        Serial.println("Projeksiyon TEST: $PF\\r\\n (OFF)");
        delay(500);
        // 2) Config gonder: $I
        sendGestureInit();
        delay(40);
        // 3) $X ile durum kontrolu
        int ntcDummy = 0;
        int irDummy  = 0;
        if (getSensorStatus(ntcDummy, irDummy)) {
          projectorHasResult     = true;
          projectorStatusSuccess = (projector_sensor_status == 0);
        } else {
          projectorHasResult     = true;
          projectorStatusSuccess = false;
        }
        drawProjeksiyonScreen();
      } else if (projectorSelection == 3) {
        // CIKIS: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_LOADCELL) {
      // Loadcell menusu: Test Et / Cikis veya sonuc ekranlari
      if (loadcellScreenMode == 0) {
        // Menu modu
        if (loadcellSelection == 0) {
          // Test Et: $I -> 500ms -> $X -> (hata varsa HATA, yoksa tare + 4 okuma)
          runLoadcellTest();
        } else {
          // Cikis: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      } else {
        // SUCCESS veya HATA ekranindayken butona basinca loadcell menüsüne don
        sendLoadcellConfig();
        resetLoadcellTestState();
        loadcellScreenMode = 0;
        loadcellSelection  = 0;
        drawLoadcellScreen();
      }
    } else {
      // Diger detay ekranlarindan geri don
      currentMenu = MENU_MAIN;
      drawMenu();
    }
    screenNeedsUpdate = false;
  }
  
  lastButtonState = currentButtonState;
}

static unsigned long lastScreenUpdate = 0;

// Function pointer array - ekran çizim fonksiyonları için optimize edilmiş
typedef void (*DrawScreenFunc)();
DrawScreenFunc drawScreenFunctions[] = {
  drawMenu,              // MENU_MAIN
  drawIRTempScreen,      // MENU_IR_TEMP
  drawNTCScreen,         // MENU_NTC
  drawIntakeFanScreen,   // MENU_INTAKE_FAN
  drawExhaustFanScreen,  // MENU_EXHAUST_FAN
  drawRGBLedScreen,      // MENU_RGB_LED
  drawGestureScreen,     // MENU_GESTURE
  drawZRefScreen,        // MENU_Z_REF
  drawYRefScreen,        // MENU_Y_REF
  drawCVR1RefScreen,     // MENU_CVR1_REF
  drawCVR2RefScreen,     // MENU_CVR2_REF
  drawBrakeMotorScreen,  // MENU_BRAKE_MOTOR
  drawZMotorScreen,      // MENU_Z_MOTOR
  drawYMotorScreen,      // MENU_Y_MOTOR
  drawCVRMotorScreen,    // MENU_CVR_MOTOR
  drawLoadcellScreen,    // MENU_LOADCELL
  drawProjeksiyonScreen  // MENU_PROJEKSIYON
};

// Optimize edilmiş ekran çizim fonksiyonu
void drawCurrentScreen() {
  if (currentMenu >= 0 && currentMenu < sizeof(drawScreenFunctions) / sizeof(drawScreenFunctions[0])) {
    drawScreenFunctions[currentMenu]();
  }
}

// $X komutu ile NTC, IR, fan, gesture, projeksiyon ve force sensor durumlarini oku
bool getSensorStatus(int &ntcStatus, int &irStatus) {
  ntcStatus = 1;
  irStatus  = 1;
  exhaust_fan_error     = 0;
  intake1_fan_error     = 0;
  intake2_fan_error     = 0;
  gesture_sensor_status = 0;
  projector_sensor_status = 0;
  force_sensor_status = 0;

  // Once eski veriyi temizle ki sadece taze $X cevabini okuyalim
  while (Serial1.available()) {
    Serial1.read();
  }

  Serial1.print("$X\r\n");
  Serial1.flush();

  char buffer[32];
  int index = 0;
  unsigned long startTime = millis();
  bool lineComplete = false;

  while (millis() - startTime < READ_TIMEOUT_MS && index < (int)sizeof(buffer) - 1) {
    if (Serial1.available()) {
      char c = Serial1.read();
      if (c == '\r' || c == '\n') {
        if (index > 0) {
          buffer[index] = '\0';
          lineComplete = true;
          break;
        }
      } else if (c >= 32 && c < 127) {
        buffer[index++] = c;
      }
    }
  }

  if (!lineComplete) return false;
  if (buffer[0] != '$') return false;

  // Beklenen format:
  // $ntc_sensor_status,
  //  ir_sensor_status,
  //  exhaust_fan_err,
  //  intake1_fan_err,
  //  intake2_fan_err,
  //  gesture_sensor_status,
  //  projector_sensor_status,
  //  force_sensor_status
  // Su an STM32 tarafindan 8 deger gonderiliyor: $0,1,0,0,0,0,0,1
  //  0: ntc, 1: ir, 2: exhaust, 3: intake1, 4: intake2, 5: gesture, 6: projector, 7: force
  int values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int valueIndex = 0;
  int numValue = 0;
  bool inNumber = false;

  for (int i = 1; i <= index; i++) {
    char c = (i < index) ? buffer[i] : '\0';
    if (c >= '0' && c <= '9') {
      numValue = numValue * 10 + (c - '0');
      inNumber = true;
    } else if (c == ',' || c == '\0' || c == '\r' || c == '\n') {
      if (inNumber && valueIndex < 8) {
        values[valueIndex++] = numValue;
        numValue = 0;
        inNumber = false;
      }
      if (c == '\0' || c == '\r' || c == '\n') break;
    }
  }

  if (inNumber && valueIndex < 8) {
    values[valueIndex++] = numValue;
  }

  if (valueIndex < 2) return false;

  ntcStatus = values[0];
  irStatus  = values[1];
  if (valueIndex >= 3) exhaust_fan_error       = values[2];
  if (valueIndex >= 4) intake1_fan_error       = values[3];
  if (valueIndex >= 5) intake2_fan_error       = values[4];
  if (valueIndex >= 6) gesture_sensor_status     = values[5];
  if (valueIndex >= 7) projector_sensor_status   = values[6];
  if (valueIndex >= 8) force_sensor_status       = values[7];

  // Debug: $X cevabini ve parse edilen status degerlerini goster
  Serial.print("X cevabi: ");
  Serial.println(buffer);
  Serial.print("NTC status = ");
  Serial.print(ntcStatus);
  Serial.print(" , IR status = ");
  Serial.print(irStatus);
  Serial.print(" , EXH err = ");
  Serial.print(exhaust_fan_error);
  Serial.print(" , IN1 err = ");
  Serial.print(intake1_fan_error);
  Serial.print(" , IN2 err = ");
  Serial.print(intake2_fan_error);
  Serial.print(" , GESTURE status = ");
  Serial.print(gesture_sensor_status);
  Serial.print(" , PROJ status = ");
  Serial.print(projector_sensor_status);
  Serial.print(" , FORCE status = ");
  Serial.println(force_sensor_status);

  // $X cevabindan arta kalan byte'lari temizle (sonraki $A okumasini bozmasin)
  while (Serial1.available()) {
    Serial1.read();
  }

  return true;
}

bool isNTCSensorOk() {
  int ntcStatus = 1, irStatus = 1;
  if (!getSensorStatus(ntcStatus, irStatus)) {
    return false;
  }
  return ntcStatus == 0;
}

bool isIRSensorOk() {
  int ntcStatus = 1, irStatus = 1;
  if (!getSensorStatus(ntcStatus, irStatus)) {
    return false;
  }
  return irStatus == 0;
}

void loop() {
  // Menu guncelle
  updateMenu();
  
  unsigned long now = millis();

  updateIntakeFanTest();
  updateExhaustFanTest();
  
  // Sensör verisi: Gesture ekranindayken daha sik istek,
  // NTC/IR testi sirasinda 100ms aralikla olcum
  unsigned long readInterval;
  if (currentMenu == MENU_GESTURE) {
    readInterval = GESTURE_READ_MS;
  } else if (currentMenu == MENU_NTC && ntcTestRunning) {
    readInterval = NTC_SAMPLE_INTERVAL_MS;
  } else if (currentMenu == MENU_IR_TEMP && irTestRunning) {
    readInterval = NTC_SAMPLE_INTERVAL_MS;  // NTC ile ayni: 100ms
  } else {
    readInterval = READ_INTERVAL_MS;
  }
  if (currentMenu != MENU_LOADCELL && now - lastRead >= readInterval) {
    lastRead = now;
    readSTM32Data();
    // Veri gelince ekrani hemen guncelle (gecikmesiz yazdir)
    if (screenNeedsUpdate) {
      screenNeedsUpdate = false;
      drawCurrentScreen();
    }
  }
  
  // Periyodik ekran yenileme (veri gelmese bile)
  unsigned long screenUpdateInterval = (currentMenu == MENU_GESTURE) ? GESTURE_SCREEN_MS : SCREEN_UPDATE_MS;
  if (now - lastScreenUpdate >= screenUpdateInterval) {
    lastScreenUpdate = now;
    drawCurrentScreen();
  }

  // NTC/IR sensör, fan, gesture ve projeksiyon hata durumunu periyodik yenile (test calisirken degil)
  if (now - lastSensorStatusCheck >= SENSOR_STATUS_REFRESH_MS) {
    if (currentMenu == MENU_NTC && !ntcTestRunning) {
      int irStatusTemp = 0;
      if (getSensorStatus(ntcSensorStatus, irStatusTemp)) {
        ntcSensorStatusValid = true;
        ntcSensorDisconnected = (ntcSensorStatus == 1);
      }
      lastSensorStatusCheck = now;
      delay(40);
      drawNTCScreen();
    } else if (currentMenu == MENU_IR_TEMP && !irTestRunning) {
      int ntcStatusTemp = 0;
      if (getSensorStatus(ntcStatusTemp, irSensorStatus)) {
        irSensorStatusValid = true;
        irSensorDisconnected = (irSensorStatus == 1);
      }
      lastSensorStatusCheck = now;
      delay(40);
      drawIRTempScreen();
    } else if (currentMenu == MENU_INTAKE_FAN) {
      // Test kosarken UART karmasasi olusmasin diye periyodik $X sorgusunu durdur.
      if (!intakeFanTestRunning) {
        int ntcDummy = 0;
        int irDummy  = 0;
        getSensorStatus(ntcDummy, irDummy);
      }
      lastSensorStatusCheck = now;
      delay(40);
      drawIntakeFanScreen();
    } else if (currentMenu == MENU_EXHAUST_FAN) {
      // Test kosarken UART karmasasi olusmasin diye periyodik $X sorgusunu durdur.
      if (!exhaustFanTestRunning) {
        int ntcDummy = 0;
        int irDummy  = 0;
        getSensorStatus(ntcDummy, irDummy);
      }
      lastSensorStatusCheck = now;
      delay(40);
      drawExhaustFanScreen();
    } else if (currentMenu == MENU_PROJEKSIYON) {
      // Projeksiyon menüsünde $X ile projeksiyon sensor status de guncelle
      int ntcDummy = 0;
      int irDummy  = 0;
      if (getSensorStatus(ntcDummy, irDummy)) {
        // projector_sensor_status global olarak guncellendi
        screenNeedsUpdate = true;
      }
      lastSensorStatusCheck = now;
      delay(40);
    } else {
      lastSensorStatusCheck = now;
    }
  }

  // Loadcell sonuc ekraninda 4 degeri periyodik guncelle
  if (currentMenu == MENU_LOADCELL && loadcellScreenMode == 1) {
    if (now - lastLoadcellUpdate >= LOADCELL_UPDATE_MS) {
      lastLoadcellUpdate = now;

      // Her yenilemede once force_sensor_status'u kontrol et
      int ntcDummy2 = 0, irDummy2 = 0;
      if (!getSensorStatus(ntcDummy2, irDummy2) || force_sensor_status == 1) {
        // Loadcell sensorde hata olursa hemen HATA ekranina gec
        loadcellErrorType = (force_sensor_status == 1) ? 1 : 0;
        loadcellScreenMode = 2;
        drawLoadcellScreen();
      } else {
        // Hata yoksa 4 loadcell degerini guncelle
        float v1 = loadcell1_g;
        float v2 = loadcell2_g;
        float v3 = loadcell3_g;
        float v4 = loadcell4_g;
        int readFaultMask = 0;

        if (readAllLoadcellValues(v1, v2, v3, v4, &readFaultMask)) {
          loadcell1_g = v1;
          loadcell2_g = v2;
          loadcell3_g = v3;
          loadcell4_g = v4;
          screenNeedsUpdate = true;
        } else {
          loadcellErrorType = 0;
          loadcellFaultMask = readFaultMask | getLoadcellFaultMask(v1, v2, v3, v4);
          loadcellScreenMode = 2;
          drawLoadcellScreen();
        }
      }
    }
  }

  // NTC testi icin timeout kontrolu
  if (currentMenu == MENU_NTC && ntcTestRunning && ntcTestStartTime > 0) {
    if (now - ntcTestStartTime > NTC_TEST_TIMEOUT_MS) {
      ntcTestRunning = false;
      ntcHasResult   = true;
      if (ntcSampleCount > 0) {
        ntcAverageTemp = ntcSampleSum / ntcSampleCount;
        ntcStatusSuccess = (ntcAverageTemp >= 0.0f && ntcAverageTemp <= 100.0f);
      } else {
        ntcAverageTemp   = 0.0f;
        ntcStatusSuccess = false;
      }
      drawNTCScreen();
    }
  }

  // IR Temp testi icin timeout kontrolu (NTC ile ayni)
  if (currentMenu == MENU_IR_TEMP && irTestRunning && irTestStartTime > 0) {
    if (now - irTestStartTime > IR_TEST_TIMEOUT_MS) {
      irTestRunning = false;
      irHasResult   = true;
      if (irSampleCount > 0) {
        irAverageTemp = irSampleSum / irSampleCount;
        irStatusSuccess = (irAverageTemp >= 0.0f && irAverageTemp <= 100.0f);
      } else {
        irAverageTemp   = 0.0f;
        irStatusSuccess = false;
      }
      drawIRTempScreen();
    }
  }
  
  delay(LOOP_DELAY_MS);
}
