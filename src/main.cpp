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

// OLED Ekran - 128x64, I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Encoder Pinleri - Rotary Encoder KY-040 veya benzeri
// Bağlantı: Encoder CLK -> Feather D23 (GPIO 23)
//           Encoder DT  -> Feather D12 (GPIO 12)
//           Encoder SW  -> Feather D27 (GPIO 27)
//           Encoder VCC -> 3.3V
//           Encoder GND -> GND
#define ENCODER_CLK 33  // CLK pini (D23, GPIO 23)
#define ENCODER_DT 12   // DT pini (D12, GPIO 12)
#define ENCODER_SW 27   // Buton pini (D27, GPIO 27)

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
  MENU_BRAKE_MOTOR
};

MenuState currentMenu = MENU_MAIN;
int menuSelection = 0; // 0-10 arasi menü seçimi
const char* menuItems[] = {"IR Temp Sensor", "NTC", "INTAKE FAN", "EXHAUST FAN", "RGB LED", "Gesture Sensor", "Z Ref", "Y Ref", "CVR1 Ref", "CVR2 Ref", "BRAKE MOTOR"};
const int menuItemCount = 11;
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
void sendBrakeMotorCommand(bool active);
void sendIntakeFanCommand();
void sendExhaustFanCommand();
void sendRGBLedCommand();
void updateMenu();

// Helper fonksiyonlar - UI iyilestirmeleri
void drawHeader(const char* title);
void drawProgressBar(int x, int y, int width, int percent);
void drawCenteredText(int y, const char* text, int textSize = 2);
void drawStatusScreen(const char* title, const char* statusText, bool isActive);

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
    display.display(); // Ekrani hemen aktif et
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
  
  // Menu'yu hemen goster ve ekrani aktif et
  drawMenu();
  display.display(); // Ekrani kesinlikle goster
  
  // Ilk veriyi al
  delay(200);
  readSTM32Data();
  lastRead = millis();
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
  
  // En az 4 sayi varsa guncelle (geri uyumluluk icin)
  if (valueIndex >= 4) {
    mcu_load_raw = values[0] / 10.0;
    pcb_temp_raw = values[1] / 10.0;
    plate_temp_raw = values[2] / 10.0;
    resin_temp_raw = values[3] / 10.0;
    
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
  drawHeader("IR Temp Sensor");
  
  // Deger ortada büyük fontla
  char tempStr[16];
  snprintf(tempStr, sizeof(tempStr), "%.1f C", resin_temp_raw);
  drawCenteredText(28, tempStr, 2);
  
  // Alt bilgi
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("Resin Temp");
  
  display.display();
}

void drawNTCScreen() {
  display.clearDisplay();
  drawHeader("NTC Temperature");
  
  // Deger ortada büyük fontla
  char tempStr[16];
  snprintf(tempStr, sizeof(tempStr), "%.1f C", plate_temp_raw);
  drawCenteredText(28, tempStr, 2);
  
  // Alt bilgi
  display.setTextSize(1);
  display.setCursor(0, 52);
  display.print("Plate Temp");
  
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
        drawIRTempScreen();
      } else if (menuSelection == 1) {
        currentMenu = MENU_NTC;
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
      }
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
    } else if (currentMenu == MENU_Z_REF || currentMenu == MENU_Y_REF || 
               currentMenu == MENU_CVR1_REF || currentMenu == MENU_CVR2_REF) {
      // TMC Ref ekranlarindan butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
    } else if (currentMenu == MENU_BRAKE_MOTOR) {
      // Butona basinca ana menuye don
      currentMenu = MENU_MAIN;
      drawMenu();
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
  drawIRTempScreen,     // MENU_IR_TEMP
  drawNTCScreen,        // MENU_NTC
  drawIntakeFanScreen,   // MENU_INTAKE_FAN
  drawExhaustFanScreen,  // MENU_EXHAUST_FAN
  drawRGBLedScreen,      // MENU_RGB_LED
  drawGestureScreen,      // MENU_GESTURE
  drawZRefScreen,        // MENU_Z_REF
  drawYRefScreen,        // MENU_Y_REF
  drawCVR1RefScreen,     // MENU_CVR1_REF
  drawCVR2RefScreen,     // MENU_CVR2_REF
  drawBrakeMotorScreen   // MENU_BRAKE_MOTOR
};

// Optimize edilmiş ekran çizim fonksiyonu
void drawCurrentScreen() {
  if (currentMenu >= 0 && currentMenu < sizeof(drawScreenFunctions) / sizeof(drawScreenFunctions[0])) {
    drawScreenFunctions[currentMenu]();
  }
}

void loop() {
  // Menu guncelle
  updateMenu();
  
  unsigned long now = millis();
  
  // Sensör verisi: Gesture ekranindayken daha sik istek (GESTURE_READ_MS), degilse READ_INTERVAL_MS
  unsigned long readInterval = (currentMenu == MENU_GESTURE) ? GESTURE_READ_MS : READ_INTERVAL_MS;
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
  
  delay(LOOP_DELAY_MS);
}
