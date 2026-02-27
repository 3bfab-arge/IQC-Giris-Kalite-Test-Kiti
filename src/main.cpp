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
#define READ_DELAY_MS      10   // $A gonderdikten sonra STM32 cevabi icin bekleme (ms)
#define READ_TIMEOUT_MS    150  // Cevap gelmezse en fazla bu kadar ms bekle (timeout)
#define LOOP_DELAY_MS      5    // Her loop sonu bekleme (ms)
#define GESTURE_LOOP_DELAY_MS 2 // Gesture ekranindayken daha hizli dongu
#define SCREEN_UPDATE_MS   50   // OLED yenileme araligi (ms)
#define GESTURE_SCREEN_MS  30   // Gesture ekraninda daha sik yenile
// NTC test ozel parametreleri
#define NTC_SAMPLE_COUNT        20   // NTC testi icin alinacak olcum sayisi
#define NTC_SAMPLE_INTERVAL_MS  100  // NTC testi sirasinda olcumler arasi bekleme (ms)
#define NTC_TEST_TIMEOUT_MS    5000  // NTC testi max sure (ms), asilirsa FAIL
#define NTC_STABILITY_DELTA_C   3.0f // NTC testi icin max izin verilen genel sapma (min-max farki, C)
#define NTC_STEP_DELTA_C        0.7f // Iki ardil olcum arasinda izin verilen max fark (C)

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
float exthaust_fan_raw = 0.0;
int gesture_type = GESTURE_NONE; // 0-4 arasi

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
  MENU_PROJEKSIYON
};

MenuState currentMenu = MENU_MAIN;
int menuSelection = 0; // 0-14 arasi menü seçimi
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
  "Brake Motor",
  "Z Motor",
  "Y Motor",
  "CVR 1-2 Motor",
  "Projection"
};
const int menuItemCount = 15;
bool screenNeedsUpdate = true;

// Intake Fan ayarlama degiskenleri
int fanSpeedPercent = 0; // 0-100 arasi, %10'luk adimlarla (0, 10, 20, ..., 100)
bool fanSpeedSent = false; // Komut gonderildi mi?

// Exhaust Fan ayarlama degiskenleri
int exhaustFanSpeedPercent = 0; // 0-100 arasi, %10'luk adimlarla (0, 10, 20, ..., 100)
bool exhaustFanSpeedSent = false; // Komut gonderildi mi?

// RGB LED ayarlama degiskenleri
int rgbHue = 0;        // Hue: 0-360 arasi
int rgbSaturation = 100; // Saturation: 0-100 arasi
int rgbValue = 100;     // Value/Brightness: 0-100 arasi
int rgbSelectedParam = 0; // 0: Hue, 1: Saturation, 2: Value
int rgbParamSelection = 0; // 0: Hue, 1: Saturation, 2: Value, 3: Cikis
bool rgbMode = false; // false: parametre secimi, true: deger ayarlama
bool rgbCommandSent = false; // Komut gonderildi mi?

// Brake Motor ayarlama degiskenleri
bool brakeMotorActive = false; // false: pasif ($B0), true: aktif ($B1)

// Z Motor ayarlama degiskenleri (mikrostep tabanli)
bool zMotorEnabled = false;        // $SZE / $SZD
int  zMotorDir = 1;                // 0: geri, 1: ileri
long zMotorDistanceSteps = 1600;   // mikrostep cinsinden mesafe (örn. 1 tur = 1600)
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

// Projeksiyon (LED) ayarlama degiskenleri
bool projeksiyonLedOn = false;       // $P1 ON / $P0 OFF
int  projeksiyonAkim = 512;          // 91-1023, $PC<deger>
int  projeksiyonParamSelection = 0;  // 0: Durum, 1: Akim, 2: Geri
bool projeksiyonEditMode = false;    // false: satir secimi, true: akim ayarlama

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

// IR Temp menusu durum degiskenleri
bool  irHasResult   = false;   // son test yapildi mi
float irLastTemp    = 0.0f;    // son testte okunan sicaklik
int   irSelection   = 0;       // 0: Test, 1: Cikis
bool  irStatusSuccess = false; // true: SUCCESS, false: FAIL

static unsigned long lastRead = 0;
static unsigned long lastButtonPress = 0;

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
void drawProjeksiyonScreen();
void sendBrakeMotorCommand(bool active);
void sendProjeksiyonOn();
void sendProjeksiyonOff();
void sendProjeksiyonCurrent();
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
void sendRGBLedCommand();
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
  // $A\r\n gonder
  Serial1.print("$A\r\n");
  Serial1.flush();
  delay(READ_DELAY_MS);
  
  // Bir satir oku (\r\n gelene kadar, timeout: READ_TIMEOUT_MS)
  char buffer[64];
  int index = 0;
  unsigned long startTime = millis();
  bool lineComplete = false;
  
  while (millis() - startTime < READ_TIMEOUT_MS && index < 63) {
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

    if (valueIndex >= 7) {
      intake1_fan_raw = values[4] / 10.0;
      intake2_fan_raw = values[5] / 10.0;
      exthaust_fan_raw = values[6] / 10.0;
    }
    
    if (valueIndex >= 8) {
      gesture_type = values[7];
      if (gesture_type < GESTURE_NONE || gesture_type > GESTURE_RIGHT) {
        gesture_type = GESTURE_NONE;
      }
      // Gesture ekranindaysak hemen ciz (gecikmesiz, loop beklemeden)
      if (currentMenu == MENU_GESTURE) {
        drawGestureScreen();
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
    if (valueIndex >= 7)
      n += snprintf(line + n, sizeof(line) - n, " %.1f %.1f %.1f",
                    intake1_fan_raw, intake2_fan_raw, exthaust_fan_raw);
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
  // Ustte "IR Temp" basligini ortala
  drawCenteredText(0, "IR Temp", 1);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  if (!irHasResult) {
    display.print("BEKLEME");
  } else {
    display.print(irStatusSuccess ? "SUCCESS" : "FAIL");
  }

  display.setCursor(0, 28);
  display.print("Deger: ");
  if (irHasResult && irStatusSuccess) {
    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "%.1f C", irLastTemp);
    display.print(tempStr);
  } else {
    display.print("--.- C");
  }

  // Alt satir: Test / Cikis secenekleri
  int y1 = 44;
  display.setCursor(0, y1);
  display.print(irSelection == 0 ? ">" : " ");
  display.print(" Test icin tikla");

  int y2 = 54;
  display.setCursor(0, y2);
  display.print(irSelection == 1 ? ">" : " ");
  display.print(" Cikis");

  display.display();
}

void drawNTCScreen() {
  display.clearDisplay();
  // Ustte "NTC Test" basligini ortala
  drawCenteredText(0, "NTC Test", 1);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 16);
  display.print("Durum: ");
  if (ntcTestRunning) {
    display.print("TESTING");
  } else if (ntcHasResult) {
    display.print(ntcStatusSuccess ? "SUCCESS" : "FAIL");
  } else {
    display.print("BEKLEME");
  }

  display.setCursor(0, 28);
  display.print("Deger: ");
  if (ntcHasResult) {
    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "%.1f C", ntcAverageTemp);
    display.print(tempStr);
  } else {
    display.print("--.- C");
  }

  if (ntcTestRunning) {
    // Test devam ederken sadece bilgi mesaji goster
    drawCenteredText(44, "Olcum yapiliyor...", 1);
  } else {
    // Test bitmis veya hic baslamamis: altta secilebilir iki satir
    // 0: Test icin tikla, 1: Cikis
    display.setTextSize(1);

    int y1 = 44;
    display.setCursor(0, y1);
    display.print(ntcSelection == 0 ? ">" : " ");
    display.print(" Test icin tikla");

    int y2 = 54;
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
  int g = gesture_type;
  if (g < 0 || g > 4) g = 0;
  
  // Gesture adı merkezde büyük fontla
  display.setTextSize(2);
  display.setCursor(0, 24);
  display.print(gestureNames[g]);
  
  // Alt bilgi
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("Type: ");
  display.print(gesture_type);
  display.print(" / ");
  display.print(gestureNames[g]);
  
  display.display();
}

void drawZRefScreen() {
  display.clearDisplay();
  drawHeader("Z Reference");
  
  // Durum merkezde büyük fontla
  const char* statusText = (z_tmc_status_stop_r == 1) ? "BASILI" : "PASIF";
  drawCenteredText(28, statusText, 2);
  
  // Alt bilgi
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("Status: ");
  display.print(z_tmc_status_stop_r);
  display.print(" (");
  display.print(statusText);
  display.print(")");
  
  display.display();
}

void drawYRefScreen() {
  display.clearDisplay();
  drawHeader("Y Reference");
  
  // Right durumu merkezde büyük fontla
  const char* rightStatus = (y_tmc_status_stop_r == 1) ? "R: BASILI" : "R: PASIF";
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(rightStatus);
  
  // Left durumu altta
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Left: ");
  display.print(y_tmc_status_stop_l == 1 ? "BASILI" : "PASIF");
  display.print(" (");
  display.print(y_tmc_status_stop_l);
  display.print(")");
  
  display.display();
}

void drawCVR1RefScreen() {
  display.clearDisplay();
  drawHeader("CVR1 Reference");
  
  // Right durumu merkezde büyük fontla
  const char* rightStatus = (cvr1_tmc_status_stop_r == 1) ? "R: BASILI" : "R: PASIF";
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(rightStatus);
  
  // Left durumu altta
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Left: ");
  display.print(cvr1_tmc_status_stop_l == 1 ? "BASILI" : "PASIF");
  display.print(" (");
  display.print(cvr1_tmc_status_stop_l);
  display.print(")");
  
  display.display();
}

void drawCVR2RefScreen() {
  display.clearDisplay();
  drawHeader("CVR2 Reference");
  
  // Right durumu merkezde büyük fontla
  const char* rightStatus = (cvr2_tmc_status_stop_r == 1) ? "R: BASILI" : "R: PASIF";
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.print(rightStatus);
  
  // Left durumu altta
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print("Left: ");
  display.print(cvr2_tmc_status_stop_l == 1 ? "BASILI" : "PASIF");
  display.print(" (");
  display.print(cvr2_tmc_status_stop_l);
  display.print(")");
  
  display.display();
}

void drawBrakeMotorScreen() {
  display.clearDisplay();
  drawHeader("BRAKE MOTOR");
  
  // Durum merkezde büyük fontla
  const char* statusText = brakeMotorActive ? "AKTIF" : "PASIF";
  drawCenteredText(28, statusText, 2);
  
  // Alt bilgi
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print("Komut: $B");
  display.print(brakeMotorActive ? "1" : "0");
  
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

// Z Motor ekranı
void drawZMotorScreen() {
  display.clearDisplay();
  drawHeader("Z Motor");

  // Satirlar:
  // 0: Durum, 1: Yon, 2: Mesafe, 3: Hiz, 4: Hareket, 5: Geri
  int y = 16;
  for (int row = 0; row < 6; row++) {
    display.setCursor(0, y);
    if (row == zMotorParamSelection) {
      display.print(zMotorEditMode ? "*" : ">");
    } else {
      display.print(" ");
    }
    display.print(" ");

    if (row == 0) {
      display.print("Durum: ");
      display.print(zMotorEnabled ? "AKTIF" : "PASIF");
    } else if (row == 1) {
      display.print("Yon  : ");
      display.print(zMotorDir == 1 ? "ILERI" : "GERI");
    } else if (row == 2) {
      display.print("Mesafe: ");
      display.print(zMotorDistanceSteps);
      display.print(" ustep");
    } else if (row == 3) {
      display.print("Hiz  : ");
      display.print(zMotorSpeedStepsPerS);
      display.print(" ustep/s");
    } else if (row == 4) {
      display.print("Hareket: Z komutu");
    } else if (row == 5) {
      display.print("Geri");
    }

    y += 9;
  }

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

// Y Motor ekranı (mikrostep tabanlı)
void drawYMotorScreen() {
  display.clearDisplay();
  drawHeader("Y Motor");

  // Satirlar:
  // 0: Durum, 1: Yon, 2: Mesafe, 3: Hiz, 4: Hareket, 5: Geri
  int y = 16;
  for (int row = 0; row < 6; row++) {
    display.setCursor(0, y);
    if (row == yMotorParamSelection) {
      display.print(yMotorEditMode ? "*" : ">");
    } else {
      display.print(" ");
    }
    display.print(" ");

    if (row == 0) {
      display.print("Durum: ");
      display.print(yMotorEnabled ? "AKTIF" : "PASIF");
    } else if (row == 1) {
      display.print("Yon  : ");
      display.print(yMotorDir == 1 ? "ILERI" : "GERI");
    } else if (row == 2) {
      display.print("Mesafe: ");
      display.print(yMotorDistanceSteps);
      display.print(" ustep");
    } else if (row == 3) {
      display.print("Hiz  : ");
      display.print(yMotorSpeedStepsPerS);
      display.print(" ustep/s");
    } else if (row == 4) {
      display.print("Hareket: SY komutu");
    } else if (row == 5) {
      display.print("Geri");
    }

    y += 9;
  }

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

// CVR Motor ekranı (mikrostep tabanlı, CVR-1 ve CVR-2)
void drawCVRMotorScreen() {
  display.clearDisplay();
  drawHeader("CVR Motor");

  int m = cvrMotorSelected; // 1 veya 2

  // Satirlar:
  // 0: Motor, 1: Durum, 2: Yon, 3: Mesafe, 4: Hiz, 5: Hareket, 6: Geri
  int y = 16;
  for (int row = 0; row < 7; row++) {
    display.setCursor(0, y);
    if (row == cvrMotorParamSelection) {
      display.print(cvrMotorEditMode ? "*" : ">");
    } else {
      display.print(" ");
    }
    display.print(" ");

    if (row == 0) {
      display.print("Motor: CVR-");
      display.print(cvrMotorSelected);
    } else if (row == 1) {
      display.print("Durum: ");
      display.print(cvrMotorEnabled[m] ? "AKTIF" : "PASIF");
    } else if (row == 2) {
      display.print("Yon  : ");
      display.print(cvrMotorDir[m] == 1 ? "ILERI" : "GERI");
    } else if (row == 3) {
      display.print("Mesafe: ");
      display.print(cvrMotorDistanceSteps[m]);
      display.print(" ustep");
    } else if (row == 4) {
      display.print("Hiz  : ");
      display.print(cvrMotorSpeedStepsPerS[m]);
      display.print(" ustep/s");
    } else if (row == 5) {
      display.print("Hareket: S");
      display.print(cvrMotorSelected);
      display.print(" komutu");
    } else if (row == 6) {
      display.print("Geri");
    }

    y += 9;
  }

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

  const int rowCount = 3;
  int selected = projeksiyonParamSelection;
  if (selected < 0) selected = 0;
  if (selected >= rowCount) selected = rowCount - 1;

  for (int row = 0; row < rowCount; row++) {
    int y = 18 + row * 9;
    display.setCursor(0, y);
    if (row == selected) {
      display.print(">");
    } else {
      display.print(" ");
    }
    display.setCursor(6, y);

    if (row == 0) {
      display.print("Durum: ");
      display.print(projeksiyonLedOn ? "ACIK" : "KAPALI");
    } else if (row == 1) {
      display.print("Akim  : ");
      display.print(projeksiyonAkim);
      if (projeksiyonEditMode) display.print(" *");
    } else if (row == 2) {
      display.print("Geri");
    }

    y += 9;
  }

  display.display();
}

void drawIntakeFanScreen() {
  display.clearDisplay();
  drawHeader("INTAKE FAN");
  
  // Hiz yüzdesi büyük fontla
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(fanSpeedPercent);
  display.setTextSize(1);
  display.print("%");
  
  // Progress bar
  drawProgressBar(0, 32, 128, fanSpeedPercent);
  
  // Komut durumu
  if (fanSpeedSent) {
    display.setCursor(100, 16);
    display.print("[OK]");
  }
  
  // RPM degerleri
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("F1: ");
  display.print(intake1_fan_raw, 1);
  display.print(" RPM");
  
  display.setCursor(0, 52);
  display.print("F2: ");
  display.print(intake2_fan_raw, 1);
  display.print(" RPM");
  
  display.display();
}

void drawExhaustFanScreen() {
  display.clearDisplay();
  drawHeader("EXHAUST FAN");
  
  // Hiz yüzdesi büyük fontla
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(exhaustFanSpeedPercent);
  display.setTextSize(1);
  display.print("%");
  
  // Progress bar
  drawProgressBar(0, 32, 128, exhaustFanSpeedPercent);
  
  // Komut durumu
  if (exhaustFanSpeedSent) {
    display.setCursor(100, 16);
    display.print("[OK]");
  }
  
  // RPM degeri
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print("RPM: ");
  display.print(exthaust_fan_raw, 1);
  
  display.display();
}

void drawRGBLedScreen() {
  display.clearDisplay();
  drawHeader("RGB LED");
  
  if (!rgbMode) {
    // Parametre secimi modu - daha okunabilir liste
    const char* params[] = {"Hue", "Saturation", "Value", "Cikis"};
    int values[] = {rgbHue, rgbSaturation, rgbValue, 0};
    
    for (int i = 0; i < 4; i++) {
      int yPos = 16 + (i * 11);
      display.setCursor(0, yPos);
      
      // Seçili parametre için > işareti
      if (rgbParamSelection == i) {
        display.print("> ");
      } else {
        display.print("  ");
      }
      
      display.print(params[i]);
      if (i < 3) {
        display.print(": ");
        display.print(values[i]);
        if (i == 0) display.print(" deg");
        else display.print("%");
      }
    }
  } else {
    // Deger ayarlama modu - büyük değer gösterimi
    const char* paramNames[] = {"Hue", "Saturation", "Value"};
    int values[] = {rgbHue, rgbSaturation, rgbValue};
    int maxValues[] = {360, 100, 100};
    
    display.setCursor(0, 16);
    display.print(paramNames[rgbSelectedParam]);
    display.print(":");
    
    // Değer büyük fontla
    display.setTextSize(2);
    display.setCursor(0, 28);
    display.print(values[rgbSelectedParam]);
    display.setTextSize(1);
    
    // Progress bar benzeri gösterim
    int barWidth = 120;
    int filled = (barWidth * values[rgbSelectedParam]) / maxValues[rgbSelectedParam];
    display.setCursor(0, 48);
    display.print("[");
    for (int i = 0; i < barWidth / 4; i++) {
      if (i < filled / 4) display.print("=");
      else display.print(" ");
    }
    display.print("]");
    
    // Komut durumu
    if (rgbCommandSent) {
      display.setCursor(100, 0);
      display.print("[OK]");
    }
  }
  
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
}

void updateMenu() {
  // Encoder ile menü seçimi veya hız ayarlama
  if (encoderPos != lastEncoderPos) {
    int diff = encoderPos - lastEncoderPos;
    if (currentMenu == MENU_MAIN) {
      menuSelection += diff;
      if (menuSelection < 0) menuSelection = menuItemCount - 1;
      if (menuSelection >= menuItemCount) menuSelection = 0;
      drawMenu();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_INTAKE_FAN) {
      // INTAKE FAN ekraninda encoder ile hiz ayarla (%10'luk adimlarla)
      fanSpeedPercent += diff * 10; // Her adimda %10 artir/azalt
      if (fanSpeedPercent < 0) fanSpeedPercent = 0;
      if (fanSpeedPercent > 100) fanSpeedPercent = 100;
      // Encoder her cevrildiginde otomatik komut gonder
      sendIntakeFanCommand();
      drawIntakeFanScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_EXHAUST_FAN) {
      // EXHAUST FAN ekraninda encoder ile hiz ayarla (%10'luk adimlarla)
      exhaustFanSpeedPercent += diff * 10; // Her adimda %10 artir/azalt
      if (exhaustFanSpeedPercent < 0) exhaustFanSpeedPercent = 0;
      if (exhaustFanSpeedPercent > 100) exhaustFanSpeedPercent = 100;
      // Encoder her cevrildiginde otomatik komut gonder
      sendExhaustFanCommand();
      drawExhaustFanScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_RGB_LED) {
      if (!rgbMode) {
        // Parametre secimi modu: H, S, V, Cikis arasinda gez
        rgbParamSelection += diff;
        if (rgbParamSelection < 0) rgbParamSelection = 3;
        if (rgbParamSelection > 3) rgbParamSelection = 0;
        drawRGBLedScreen();
        screenNeedsUpdate = false;
      } else {
        // Deger ayarlama modu: secili parametrenin degerini ayarla
        if (rgbSelectedParam == 0) {
          // Hue: 0-360 arasi, her adimda 10 artir/azalt
          rgbHue += diff * 10;
          if (rgbHue < 0) rgbHue = 0;
          if (rgbHue > 360) rgbHue = 360;
        } else if (rgbSelectedParam == 1) {
          // Saturation: 0-100 arasi, her adimda 10 artir/azalt
          rgbSaturation += diff * 10;
          if (rgbSaturation < 0) rgbSaturation = 0;
          if (rgbSaturation > 100) rgbSaturation = 100;
        } else if (rgbSelectedParam == 2) {
          // Value: 0-100 arasi, her adimda 10 artir/azalt
          rgbValue += diff * 10;
          if (rgbValue < 0) rgbValue = 0;
          if (rgbValue > 100) rgbValue = 100;
        }
        // Encoder her cevrildiginde otomatik komut gonder
        sendRGBLedCommand();
        drawRGBLedScreen();
        screenNeedsUpdate = false;
      }
    } else if (currentMenu == MENU_BRAKE_MOTOR) {
      // BRAKE MOTOR ekraninda encoder ile ac/kapat (toggle)
      // Encoder saga/sola donduruldugunde durumu degistir
      if (diff > 0) {
        // Saga donduruldu: Aktif yap
        brakeMotorActive = true;
      } else if (diff < 0) {
        // Sola donduruldu: Pasif yap
        brakeMotorActive = false;
      }
      // Encoder her cevrildiginde otomatik komut gonder
      sendBrakeMotorCommand(brakeMotorActive);
      drawBrakeMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_Z_MOTOR) {
      // Z MOTOR ekraninda encoder ile satir secimi veya deger ayarlama
      if (!zMotorEditMode) {
        // Satir secimi (0-5)
        zMotorParamSelection += diff;
        if (zMotorParamSelection < 0) zMotorParamSelection = 5;
        if (zMotorParamSelection > 5) zMotorParamSelection = 0;
      } else {
        // Deger ayarlama modu
        if (zMotorParamSelection == 1) {
          // Yon: 0 veya 1
          if (diff > 0) zMotorDir = 1;
          if (diff < 0) zMotorDir = 0;
        } else if (zMotorParamSelection == 2) {
          // Mesafe: 1-100000 mikrostep, her adimda 100 mikrostep
          zMotorDistanceSteps += diff * 100;
          if (zMotorDistanceSteps < 100) zMotorDistanceSteps = 100;
          if (zMotorDistanceSteps > 100000) zMotorDistanceSteps = 100000;
        } else if (zMotorParamSelection == 3) {
          // Hiz: 100-20000 mikrostep/s, her adimda 100 mikrostep/s
          zMotorSpeedStepsPerS += diff * 100;
          if (zMotorSpeedStepsPerS < 100) zMotorSpeedStepsPerS = 100;
          if (zMotorSpeedStepsPerS > 20000) zMotorSpeedStepsPerS = 20000;
        }
      }
      drawZMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_Y_MOTOR) {
      // Y MOTOR ekraninda encoder ile satir secimi veya deger ayarlama
      if (!yMotorEditMode) {
        // Satir secimi (0-5)
        yMotorParamSelection += diff;
        if (yMotorParamSelection < 0) yMotorParamSelection = 5;
        if (yMotorParamSelection > 5) yMotorParamSelection = 0;
      } else {
        // Deger ayarlama modu
        if (yMotorParamSelection == 1) {
          // Yon: 0 veya 1
          if (diff > 0) yMotorDir = 1;
          if (diff < 0) yMotorDir = 0;
        } else if (yMotorParamSelection == 2) {
          // Mesafe: 1-100000 mikrostep, her adimda 100 mikrostep
          yMotorDistanceSteps += diff * 100;
          if (yMotorDistanceSteps < 100) yMotorDistanceSteps = 100;
          if (yMotorDistanceSteps > 100000) yMotorDistanceSteps = 100000;
        } else if (yMotorParamSelection == 3) {
          // Hiz: 100-20000 mikrostep/s, her adimda 100 mikrostep/s
          yMotorSpeedStepsPerS += diff * 100;
          if (yMotorSpeedStepsPerS < 100) yMotorSpeedStepsPerS = 100;
          if (yMotorSpeedStepsPerS > 20000) yMotorSpeedStepsPerS = 20000;
        }
      }
      drawYMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_CVR_MOTOR) {
      // CVR MOTOR ekraninda encoder ile satir secimi veya deger ayarlama
      if (!cvrMotorEditMode) {
        // Satir secimi (0-6)
        cvrMotorParamSelection += diff;
        if (cvrMotorParamSelection < 0) cvrMotorParamSelection = 6;
        if (cvrMotorParamSelection > 6) cvrMotorParamSelection = 0;
      } else {
        int m = cvrMotorSelected; // 1 veya 2
        if (cvrMotorParamSelection == 0) {
          // Motor secimi: 1 <-> 2
          if (diff != 0) {
            cvrMotorSelected = (cvrMotorSelected == 1) ? 2 : 1;
          }
        } else if (cvrMotorParamSelection == 2) {
          // Yon: 0 veya 1
          if (diff > 0) cvrMotorDir[m] = 1;
          if (diff < 0) cvrMotorDir[m] = 0;
        } else if (cvrMotorParamSelection == 3) {
          // Mesafe: 1-100000 mikrostep, her adimda 100 mikrostep
          cvrMotorDistanceSteps[m] += diff * 100;
          if (cvrMotorDistanceSteps[m] < 100) cvrMotorDistanceSteps[m] = 100;
          if (cvrMotorDistanceSteps[m] > 100000) cvrMotorDistanceSteps[m] = 100000;
        } else if (cvrMotorParamSelection == 4) {
          // Hiz: 100-20000 mikrostep/s, her adimda 100 mikrostep/s
          cvrMotorSpeedStepsPerS[m] += diff * 100;
          if (cvrMotorSpeedStepsPerS[m] < 100) cvrMotorSpeedStepsPerS[m] = 100;
          if (cvrMotorSpeedStepsPerS[m] > 20000) cvrMotorSpeedStepsPerS[m] = 20000;
        }
      }
      drawCVRMotorScreen();
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_PROJEKSIYON) {
      // Projeksiyon ekraninda encoder: satir secimi veya akim ayari
      if (!projeksiyonEditMode) {
        projeksiyonParamSelection += diff;
        if (projeksiyonParamSelection < 0) projeksiyonParamSelection = 2;
        if (projeksiyonParamSelection > 2) projeksiyonParamSelection = 0;
      } else {
        // Akim satirinda: 0-1023, ornek adim 10
        projeksiyonAkim += diff * 10;
        if (projeksiyonAkim < 91) projeksiyonAkim = 91;
        if (projeksiyonAkim > 1023) projeksiyonAkim = 1023;
        sendProjeksiyonCurrent();
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
      // IR Temp ekraninda encoder ile alt secenekler (Test / Cikis) arasında gez
      irSelection += diff;
      if (irSelection < 0) irSelection = 1;
      if (irSelection > 1) irSelection = 0;
      drawIRTempScreen();
      screenNeedsUpdate = false;
    }
    lastEncoderPos = encoderPos;
  }
  
  // Buton ile seçim/geri dön
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(ENCODER_SW);
  
  // Buton basıldı (HIGH -> LOW geçişi, pull-up olduğu için LOW = basılı)
  if (lastButtonState == HIGH && currentButtonState == LOW && millis() - lastButtonPress > 300) {
    lastButtonPress = millis();
    delay(20); // Debounce
    
    if (currentMenu == MENU_MAIN) {
      // Menüden seçim yap
      if (menuSelection == 0) {
        currentMenu = MENU_IR_TEMP;
        // IR Temp menusu icin durum sifirlama
        irHasResult = false;
        irLastTemp  = 0.0f;
        irSelection = 0; // varsayilan secim: Test
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
        drawNTCScreen();
      } else if (menuSelection == 2) {
        currentMenu = MENU_INTAKE_FAN;
        fanSpeedPercent = 0; // Hizi %0 ile baslat
        fanSpeedSent = false; // Komut gonderilmedi
        lastEncoderPos = encoderPos; // Encoder pozisyonunu koru, direkt hiz ayarlanabilsin
        drawIntakeFanScreen();
      } else if (menuSelection == 3) {
        currentMenu = MENU_EXHAUST_FAN;
        exhaustFanSpeedPercent = 0; // Hizi %0 ile baslat
        exhaustFanSpeedSent = false; // Komut gonderilmedi
        lastEncoderPos = encoderPos; // Encoder pozisyonunu koru, direkt hiz ayarlanabilsin
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
        zMotorDistanceSteps = 1600;   // 1 tur
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
        currentMenu = MENU_PROJEKSIYON;
        projeksiyonLedOn = false;
        projeksiyonAkim = 512;
        projeksiyonParamSelection = 0;
        projeksiyonEditMode = false;
        encoderPos = 0;
        lastEncoderPos = 0;
        drawProjeksiyonScreen();
      }
    } else if (currentMenu == MENU_NTC) {
      // NTC menusu: buton islemleri
      if (!ntcTestRunning) {
        if (ntcSelection == 0) {
          // Test icin tikla: once sensor durumunu kontrol et
          if (!isNTCSensorOk()) {
            ntcTestRunning   = false;
            ntcHasResult     = true;
            ntcStatusSuccess = false;
            drawNTCScreen();
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
            drawNTCScreen();
          }
        } else if (ntcSelection == 1) {
          // Cikis: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      }
      screenNeedsUpdate = false;
    } else if (currentMenu == MENU_INTAKE_FAN) {
      // Butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_EXHAUST_FAN) {
      // Butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_RGB_LED) {
      if (!rgbMode) {
        // Parametre secimi modunda: secimi yap ve deger ayarlama moduna gec
        if (rgbParamSelection == 3) {
          // Cikis secildi: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        } else {
          // H, S veya V secildi: deger ayarlama moduna gec
          rgbSelectedParam = rgbParamSelection;
          rgbMode = true;
          rgbCommandSent = false;
          encoderPos = 0; // Encoder pozisyonunu sifirla
          lastEncoderPos = 0;
          drawRGBLedScreen();
        }
      } else {
        // Deger ayarlama modunda: parametre secimi moduna don
        rgbMode = false;
        drawRGBLedScreen();
      }
    } else if (currentMenu == MENU_GESTURE) {
      // Butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_IR_TEMP) {
      // IR Temp menusu: buton islemleri
      if (irSelection == 0) {
        // Test icin tikla: once IR sensor durumunu kontrol et
        if (!isIRSensorOk()) {
          irHasResult     = true;
          irStatusSuccess = false;
          drawIRTempScreen();
        } else {
          irLastTemp      = resin_temp_raw;
          irHasResult     = true;
          irStatusSuccess = true;
          drawIRTempScreen();
        }
      } else if (irSelection == 1) {
        // Cikis: ana menuye don
        currentMenu = MENU_MAIN;
        drawMenu();
      }
    } else if (currentMenu == MENU_Z_REF || currentMenu == MENU_Y_REF || 
               currentMenu == MENU_CVR1_REF || currentMenu == MENU_CVR2_REF) {
      // TMC Ref ekranlarindan butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_BRAKE_MOTOR) {
      // Butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_Z_MOTOR) {
      // Z Motor ekraninda buton davranisi
      if (!zMotorEditMode) {
        // Satir secimi modunda
        if (zMotorParamSelection == 0) {
          // Durum: enable/disable
          zMotorEnabled = !zMotorEnabled;
          if (!zMotorEnabled) {
            // Devre disi alirken once durdur, sonra disable
            sendZMotorStop();
          }
          sendZMotorEnable(zMotorEnabled);
          drawZMotorScreen();
        } else if (zMotorParamSelection == 1 ||
                   zMotorParamSelection == 2 ||
                   zMotorParamSelection == 3) {
          // Yon / Mesafe / Hiz icin deger ayarlama moduna gec
          zMotorEditMode = true;
          drawZMotorScreen();
        } else if (zMotorParamSelection == 4) {
          // Hareket: enable degilse once enable, sonra hareket komutu
          if (!zMotorEnabled) {
            zMotorEnabled = true;
            sendZMotorEnable(true);
          }
          // STM32 tarafinin enable komutunu isleyebilmesi icin kisa bekleme
          delay(100); // 100ms
          sendZMotorMove();
          drawZMotorScreen();
        } else if (zMotorParamSelection == 5) {
          // Geri: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      } else {
        // Deger ayarlama modundan cik
        zMotorEditMode = false;
        drawZMotorScreen();
      }
    } else if (currentMenu == MENU_Y_MOTOR) {
      // Y Motor ekraninda buton davranisi
      if (!yMotorEditMode) {
        // Satir secimi modunda
        if (yMotorParamSelection == 0) {
          // Durum: enable/disable
          yMotorEnabled = !yMotorEnabled;
          if (!yMotorEnabled) {
            // Devre disi alirken once stop, sonra disable
            sendYMotorStop();
          }
          sendYMotorEnable(yMotorEnabled);
          drawYMotorScreen();
        } else if (yMotorParamSelection == 1 ||
                   yMotorParamSelection == 2 ||
                   yMotorParamSelection == 3) {
          // Yon / Mesafe / Hiz icin deger ayarlama moduna gec
          yMotorEditMode = true;
          drawYMotorScreen();
        } else if (yMotorParamSelection == 4) {
          // Hareket: enable degilse once enable, sonra hareket komutu
          if (!yMotorEnabled) {
            yMotorEnabled = true;
            sendYMotorEnable(true);
          }
          // STM32 tarafinin enable komutunu isleyebilmesi icin kisa bekleme
          delay(100); // 100ms
          sendYMotorMove();
          drawYMotorScreen();
        } else if (yMotorParamSelection == 5) {
          // Geri: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      } else {
        // Deger ayarlama modundan cik
        yMotorEditMode = false;
        drawYMotorScreen();
      }
    } else if (currentMenu == MENU_CVR_MOTOR) {
      // CVR Motor ekraninda buton davranisi
      if (!cvrMotorEditMode) {
        int m = cvrMotorSelected; // 1 veya 2
        if (cvrMotorParamSelection == 0) {
          // Motor: 1 <-> 2
          cvrMotorSelected = (cvrMotorSelected == 1) ? 2 : 1;
          drawCVRMotorScreen();
        } else if (cvrMotorParamSelection == 1) {
          // Durum: enable/disable
          cvrMotorEnabled[m] = !cvrMotorEnabled[m];
          if (!cvrMotorEnabled[m]) {
            // Devre disi alirken once stop, sonra disable
            sendCVRMotorStop(m);
          }
          sendCVRMotorEnable(m, cvrMotorEnabled[m]);
          drawCVRMotorScreen();
        } else if (cvrMotorParamSelection == 2 ||
                   cvrMotorParamSelection == 3 ||
                   cvrMotorParamSelection == 4) {
          // Yon / Mesafe / Hiz icin deger ayarlama moduna gec
          cvrMotorEditMode = true;
          drawCVRMotorScreen();
        } else if (cvrMotorParamSelection == 5) {
          // Hareket: enable degilse once enable, sonra hareket komutu
          if (!cvrMotorEnabled[m]) {
            cvrMotorEnabled[m] = true;
            sendCVRMotorEnable(m, true);
          }
          sendCVRMotorMove(m);
          drawCVRMotorScreen();
        } else if (cvrMotorParamSelection == 6) {
          // Geri: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      } else {
        // Deger ayarlama modundan cik
        cvrMotorEditMode = false;
        drawCVRMotorScreen();
      }
    } else if (currentMenu == MENU_PROJEKSIYON) {
      // Projeksiyon ekraninda buton davranisi
      if (!projeksiyonEditMode) {
        if (projeksiyonParamSelection == 0) {
          // Durum: LED ac/kapa
          projeksiyonLedOn = !projeksiyonLedOn;
          if (projeksiyonLedOn) {
            sendProjeksiyonOn();
          } else {
            sendProjeksiyonOff();
          }
          drawProjeksiyonScreen();
        } else if (projeksiyonParamSelection == 1) {
          // Akim: deger ayarlama moduna gec
          projeksiyonEditMode = true;
          drawProjeksiyonScreen();
        } else if (projeksiyonParamSelection == 2) {
          // Geri: ana menuye don
          currentMenu = MENU_MAIN;
          drawMenu();
        }
      } else {
        // Akim ayarlama modundan cik
        projeksiyonEditMode = false;
        drawProjeksiyonScreen();
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
  drawProjeksiyonScreen  // MENU_PROJEKSIYON
};

// Optimize edilmiş ekran çizim fonksiyonu
void drawCurrentScreen() {
  if (currentMenu >= 0 && currentMenu < sizeof(drawScreenFunctions) / sizeof(drawScreenFunctions[0])) {
    drawScreenFunctions[currentMenu]();
  }
}

// $X komutu ile NTC ve IR sensor durumlarini oku
bool getSensorStatus(int &ntcStatus, int &irStatus) {
  ntcStatus = 1;
  irStatus  = 1;

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

  int values[2] = {0, 0};
  int valueIndex = 0;
  int numValue = 0;
  bool inNumber = false;

  for (int i = 1; i <= index; i++) {
    char c = (i < index) ? buffer[i] : '\0';
    if (c >= '0' && c <= '9') {
      numValue = numValue * 10 + (c - '0');
      inNumber = true;
    } else if (c == ',' || c == '\0' || c == '\r' || c == '\n') {
      if (inNumber && valueIndex < 2) {
        values[valueIndex++] = numValue;
        numValue = 0;
        inNumber = false;
      }
      if (c == '\0' || c == '\r' || c == '\n') break;
    }
  }

  if (inNumber && valueIndex < 2) {
    values[valueIndex++] = numValue;
  }

  if (valueIndex < 2) return false;

  ntcStatus = values[0];
  irStatus  = values[1];

  // Debug: $X cevabini ve parse edilen status degerlerini goster
  Serial.print("X cevabi: ");
  Serial.println(buffer);
  Serial.print("NTC status = ");
  Serial.print(ntcStatus);
  Serial.print(" , IR status = ");
  Serial.println(irStatus);

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
  
  // Sensör verisi: Gesture ekranindayken daha sik istek,
  // NTC testi sirasinda daha seyrek (100ms), diger durumlarda standart READ_INTERVAL_MS
  unsigned long readInterval;
  if (currentMenu == MENU_GESTURE) {
    readInterval = GESTURE_READ_MS;
  } else if (currentMenu == MENU_NTC && ntcTestRunning) {
    readInterval = NTC_SAMPLE_INTERVAL_MS;
  } else {
    readInterval = READ_INTERVAL_MS;
  }
  if (now - lastRead >= readInterval) {
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

  // NTC testi icin timeout kontrolu:
  // Belirli sure icinde yeterli olcum gelmezse eldeki orneklerle ortalama al,
  // aralik disina cikarsa veya hic ornek yoksa FAIL olarak sonlandir.
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
  
  delay(LOOP_DELAY_MS);
}
