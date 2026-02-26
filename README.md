# Edge-Pro-Power-Test-Kiti-

Edge Pro Power Test Kiti için **ESP32 Feather** tabanlı kontrol ve izleme arayüzü. STM32 ile UART üzerinden haberleşir; sensör verilerini okur, fan/RGB/fren motorunu komutlar, 128x64 OLED ve rotary encoder ile menü sunar.

---

## İçindekiler

- [Genel Bakış](#genel-bakış)
- [Donanım](#donanım)
- [Yazılım Mimarisi (Ana Kod)](#yazılım-mimarisi-ana-kod)
- [Menü Sistemi](#menü-sistemi)
- [Proje Yapısı ve Dokümantasyon](#proje-yapısı-ve-dokümantasyon)
- [Derleme ve Yükleme](#derleme-ve-yükleme)

---

## Genel Bakış

| Özellik | Açıklama |
|--------|----------|
| **Kart** | Adafruit HUZZAH32 ESP32 Feather |
| **Haberleşme** | UART (Serial1) ile STM32 – 115200, 8N1 |
| **Ekran** | 128x64 monokrom OLED (I2C, SSD1306, 0x3C) |
| **Kontrol** | Rotary encoder (döndürme + buton) |
| **Veri kaynağı** | Tüm sensör/fan/TMC verileri STM32’den `$A` komutu ile alınır |
| **Kontrol komutları** | Fan hızı (`$F1`, `$F2`, `$F3`), RGB LED (`$LA`), Fren motoru (`$B0`/`$B1`) |

ESP32, kullanıcı arayüzünü (OLED + encoder) yönetir; sensör ölçümü, fan sürme ve TMC okuma STM32 tarafında yapılır. Protokol detayı için **SERI_HABERLESME.md** kullanılır.

---

## Donanım

### Kullanılan Kart: Adafruit HUZZAH32 ESP32 Feather

- **MCU:** ESP32 (WiFi/Bluetooth kapalı, sadece UART + I2C + GPIO)
- **Framework:** Arduino (PlatformIO, `featheresp32`)

### Pin Özeti (ESP32 Feather)

| İşlev | GPIO | Açıklama |
|-------|------|----------|
| **UART (STM32)** | RX = 16, TX = 17 | STM32 TX → 16, STM32 RX → 17, GND ortak |
| **OLED (I2C)** | SDA/SCL (donanım I2C) | 128x64 SSD1306, adres 0x3C |
| **Encoder CLK** | 33 | Rotary encoder saat pini |
| **Encoder DT** | 12 | Rotary encoder veri pini |
| **Encoder SW** | 27 | Encoder butonu (menü seç / geri) |

Bağlantı detayları ve voltaj uyumu için **PIN_BAGLANTILARI.md** dosyasına bakın.

---

## Yazılım Mimarisi (Ana Kod)

Kaynak: **`src/main.cpp`**

### Başlangıç: `setup()`

1. **I2C ve OLED:** `Wire.begin()` → `display.begin()` (SSD1306, 0x3C) → ilk çerçeve çizilir.
2. **Serial:** Debug için `Serial` 115200.
3. **UART:** `Serial1.setPins(16, 17)`, `Serial1.begin(115200)`, buffer temizlenir.
4. **Encoder:** CLK/DT/SW pinleri `INPUT_PULLUP`; CLK için `attachInterrupt` ile `encoderISR` (encoderPos artır/azalt).
5. **İlk ekran:** `drawMenu()` ile ana menü gösterilir.
6. **İlk veri:** Kısa gecikme sonrası `readSTM32Data()` bir kez çağrılır, `lastRead` ayarlanır.

### Ana Döngü: `loop()`

1. **Menü ve kullanıcı girişi:** `updateMenu()` – encoder pozisyonu ve buton durumu okunur; menü seçimi, alt menüde değer değişimi (fan %, RGB, fren) ve komut gönderimi yapılır.
2. **Periyodik veri okuma:**  
   - Normal: her **50 ms**’de bir `readSTM32Data()`.  
   - Gesture ekranındayken: her **20 ms**’de bir (daha hızlı güncelleme).
3. **Ekran güncellemesi:**  
   - Veri geldiğinde `screenNeedsUpdate` set edilir; aynı turda `drawCurrentScreen()` ile ilgili ekran yenilenir.  
   - Ayrıca periyodik olarak (50 ms, Gesture’da 30 ms) `drawCurrentScreen()` çağrılır.
4. **Gecikme:** `delay(LOOP_DELAY_MS)` (5 ms) ile döngü yükü azaltılır.

### Veri Okuma: `readSTM32Data()`

- **Gönderim:** `$A\r\n` (STM32’den anlık veri isteği).
- **Alım:** Bir satır okunur (`\r` veya `\n`’e kadar, timeout 150 ms). İlk karakter `$` değilse satır yok sayılır.
- **Parse:** Virgülle ayrılmış sayılar alınır (en fazla 15 alan):
  - **1–4:** MCU load, PCB temp, plate temp (NTC), resin temp (IR) → 10’a bölünerek float.
  - **5–7:** İntake 1/2 ve exhaust fan RPM → 10’a bölünerek float.
  - **8:** Gesture tipi (0–4).
  - **9–15:** TMC durumları (Z, Y, CVR1, CVR2 – sağ/sol stop).
- **Güncelleme:** Parse sonrası global değişkenler yazılır; Gesture veya TMC Ref ekranındaysa ilgili ekran hemen çizilir.

### Menü Mantığı: `updateMenu()`

- **Encoder döndürme:**  
  - Ana menüde: seçili satır değişir (kaydırmalı liste).  
  - Alt menüde: ilgili değer değişir (fan %10 adım, RGB H/S/V, fren aç/kapa) ve komut anında gönderilir (`sendIntakeFanCommand`, `sendExhaustFanCommand`, `sendRGBLedCommand`, `sendBrakeMotorCommand`).
- **Encoder butonu:**  
  - Ana menüde: seçili satıra girilir (ekran değişir).  
  - Alt menüde: çoğunda ana menüye dönülür; RGB LED’de parametre seçimi / değer modu veya çıkış.

### Ekran Çizimi

- Ortak yardımcılar: `drawHeader()`, `drawProgressBar()`, `drawCenteredText()`.
- Her ekran için ayrı fonksiyon: `drawMenu()`, `drawIRTempScreen()`, `drawNTCScreen()`, fan ekranları, `drawRGBLedScreen()`, `drawGestureScreen()`, Z/Y/CVR1/CVR2 Ref, `drawBrakeMotorScreen()`.
- Hangi ekranın çizileceği `currentMenu` ve bir fonksiyon pointer dizisi (`drawScreenFunctions[]`) ile tek noktadan `drawCurrentScreen()` ile çağrılır.

---

## Menü Sistemi

| # | Menü | İçerik | Encoder | Buton |
|---|------|--------|---------|--------|
| 1 | IR Temp Sensor | Resin sıcaklığı (IR) | – | Ana menü |
| 2 | NTC | Plate sıcaklığı (NTC) | – | Ana menü |
| 3 | INTAKE FAN | İntake 1/2 RPM, % hız, progress bar | % 0–100 (%10 adım), komut otomatik | Ana menü |
| 4 | EXHAUST FAN | Exhaust RPM, % hız, progress bar | % 0–100 (%10 adım), komut otomatik | Ana menü |
| 5 | RGB LED | HSV (Hue/Sat/Value) | Parametre seçimi veya değer; komut otomatik | Alt menü/çıkış |
| 6 | Gesture Sensor | Gesture tipi (NONE/UP/DOWN/LEFT/RIGHT) | – | Ana menü |
| 7 | Z Ref | Z TMC stop (BASILI/PASIF) | – | Ana menü |
| 8 | Y Ref | Y TMC sağ/sol stop | – | Ana menü |
| 9 | CVR1 Ref | CVR1 TMC sağ/sol stop | – | Ana menü |
| 10 | CVR2 Ref | CVR2 TMC sağ/sol stop | – | Ana menü |
| 11 | BRAKE MOTOR | Fren motoru AKTIF/PASIF | Sağ = aktif ($B1), sol = pasif ($B0) | Ana menü |

Ana menüde 6 satır görünür, seçim kaydırmalıdır.

---

## Proje Yapısı ve Dokümantasyon

```
EdgePro Power Test Kiti/
├── src/
│   └── main.cpp              # Tüm uygulama kodu (UART, menü, OLED, encoder)
├── platformio.ini             # Kart: featheresp32, kütüphaneler, upload/monitor
├── README.md                  # Bu dosya – genel bakış ve ana kod açıklaması
├── SERI_HABERLESME.md         # UART protokolü, komutlar, veri formatı
├── PIN_BAGLANTILARI.md        # ESP32/STM32 pinleri ve bağlantı özeti
├── include/                   # Ek header’lar (şu an boş/README)
├── lib/                       # Yerel kütüphaneler (şu an boş/README)
└── test/                      # Test dosyaları (şu an boş/README)
```

- **Protokol ve komutlar:** **SERI_HABERLESME.md**  
- **Pinler ve bağlantı:** **PIN_BAGLANTILARI.md**

---

## Derleme ve Yükleme

**Gereksinim:** PlatformIO (VS Code eklentisi veya CLI).

```bash
# Derleme
pio run

# Yükleme (COM port platformio.ini içinde upload_port ile ayarlanır)
pio run -t upload

# Seri monitör (115200)
pio device monitor
```

`platformio.ini` içinde `upload_port = COM6` ve `monitor_speed = 115200` kullanılır; gerekirse portu değiştirin.

---

## Özet

- **Ana kod** tek dosyada: `setup` (OLED, UART, encoder), `loop` (updateMenu + periyodik okuma + ekran), `readSTM32Data` (parse + globals + anlık ekran), `updateMenu` (encoder/buton + komut gönderimi), ekran fonksiyonları.
- **README** projeyi, donanımı, yazılım akışını ve menüyü açıklar; detaylı protokol ve pin bilgisi ilgili .md dosyalarına bırakılmıştır.
