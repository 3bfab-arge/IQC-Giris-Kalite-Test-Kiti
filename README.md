# EdgePro Power Test Kiti

## Genel Bakış
Bu proje, EdgePro Power Test Kiti için geliştirilmiş bir Arduino Mega Mini tabanlı kontrol sistemidir.

## Donanım Özellikleri

### Mikrodenetleyici
- **Kart**: Arduino Mega Mini
- **Mikroişlemci**: ATmega2560
- **Platform**: PlatformIO

### Bağlantılar

#### NTC Sıcaklık Sensörü
- **Pin**: A0 (Analog Giriş)
- **Açıklama**: NTC (Negative Temperature Coefficient) sıcaklık sensörü analog olarak okunur
- **Kullanım**: Sıcaklık ölçümü için analog değer okunur ve sıcaklığa dönüştürülür

#### DS18B20 Sıcaklık Sensörü
- **Pin**: D4 (Dijital Giriş - OneWire)
- **Açıklama**: DS18B20 dijital sıcaklık sensörü OneWire protokolü ile çalışır
- **Kullanım**: DallasTemperature kütüphanesi ile sıcaklık okunur
- **Kütüphaneler**: OneWire, DallasTemperature
- **Not**: 4.7kΩ pull-up direnci gerekebilir (VCC ile DATA arası)

#### RGB LED
- **Kırmızı Pin**: D9 (PWM Çıkış)
- **Yeşil Pin**: D10 (PWM Çıkış)
- **Mavi Pin**: D11 (PWM Çıkış)
- **Güç Kaynağı**: 12V harici güç kaynağı
- **Açıklama**: RGB LED PWM sinyalleri ile kontrol edilir
- **Kullanım**: `analogWrite()` fonksiyonu ile 0-255 arası değerler verilerek renk kontrolü yapılır
- **Başlangıç Durumu**: Tüm kanallar 0 (kapalı)
- **Not**: 12V güç kaynağı harici olarak bağlanmalıdır

#### Rotary Encoder
- **DT Pin**: D22 (Dijital Giriş - INPUT_PULLUP)
- **CLK Pin**: D23 (Dijital Giriş - INPUT_PULLUP)
- **SW Pin**: D26 (Dijital Giriş - INPUT_PULLUP)
- **Açıklama**: Rotary encoder döndürme ve buton basma işlemleri için kullanılır
- **Kullanım**: DT ve CLK pinleri ile döndürme yönü tespit edilir, SW pini ile buton durumu okunur
- **Pull-up**: Tüm pinler INPUT_PULLUP modunda yapılandırılmıştır
- **Not**: Encoder okuma için interrupt veya polling yöntemi kullanılabilir

#### Buzzer
- **Pin**: D12 (PWM Çıkış)
- **Açıklama**: Sesli uyarı ve bildirimler için buzzer
- **Kullanım**: `digitalWrite()` ile basit bip sesi veya `tone()` fonksiyonu ile farklı frekanslarda ses üretilebilir
- **Başlangıç Durumu**: LOW (kapalı)
- **Not**: PWM pin olduğu için farklı tonlar üretilebilir

#### Buton
- **Pin**: D32 (Dijital Giriş - INPUT_PULLUP)
- **Açıklama**: Buton basma algılama için kullanılır
- **Bağlantı**: Pull-up bağlantılı (harici pull-up direnci mevcut)
- **Kullanım**: `digitalRead()` ile okunur, basıldığında LOW, basılmadığında HIGH değer döner
- **Not**: Debounce işlemi yapılması önerilir

#### OLED 1.3 inç Ekran (SH1106)
- **SDA Pin**: D20 (I2C Data - Arduino Mega I2C)
- **SCL Pin**: D21 (I2C Clock - Arduino Mega I2C)
- **Protokol**: I2C
- **Çözünürlük**: 128x64 piksel
- **Sürücü**: SH1106
- **I2C Adresi**: 0x3C veya 0x3D
- **Açıklama**: 1.3 inç monokrom OLED ekran, SH1106 sürücüsü ile çalışır
- **Kütüphaneler**: Adafruit GFX Library, Adafruit SH110X
- **Not**: Arduino Mega'da I2C pinleri otomatik olarak SDA=20, SCL=21'dir. Reset pin yoksa OLED_RESET=-1 kullanılır.

#### ST7789 TFT Ekran (Eski - Artık Kullanılmıyor)
- **CS Pin**: D53 (Dijital Çıkış)
- **DC Pin**: D48 (Dijital Çıkış)
- **RES Pin**: D49 (Dijital Çıkış)
- **SDA Pin**: D24 (Dijital Çıkış – Software SPI veri)
- **SCL Pin**: D25 (Dijital Çıkış – Software SPI saat)
- **Protokol**: Software SPI (ekran TMC2130 ile pin çakışmasın diye D51/D52 kullanılmıyor)
- **Açıklama**: ST7789 TFT ekran kontrolcüsü ile çalışan ekran (artık kullanılmıyor, yerine OLED kullanılıyor)
- **Kütüphaneler**: Adafruit GFX Library, Adafruit ST7735 and ST7789 Library
- **Not**: Eski bağlantı (sadece ekran varken) SDA=D51, SCL=D52 idi; TMC2130 eklendiği için ekran SDA/SCL D24/D25’e taşındı (aşağıda açıklama).

#### TMC2130 Step Motor Sürücü
- **DIR Pin**: D44 (Yön)
- **STEP Pin**: D45 (Adım)
- **EN Pin**: D46 (Enable, LOW = motor açık)
- **CS Pin**: D47 (SPI Chip Select)
- **SPI**: MISO=D50, MOSI=D51, SCK=D52 (donanım SPI, ekran ile paylaşılmıyor)
- **Açıklama**: Trinamic TMC2130 step motor sürücü; SPI ile ayar, STEP/DIR ile hareket

#### Brake Çıkışı
- **Pin**: D35 (Dijital Çıkış)
- **Açıklama**: Fren/brake kontrolü için dijital çıkış pini
- **Kullanım**: HIGH/LOW durumları ile brake kontrolü yapılır
- **Başlangıç Durumu**: LOW (kapalı)

#### Fanlar (25 kHz PWM + TACH)
- **PWM (Timer3, 25 kHz)**: Intake=D5, Exhaust=D2, H4=D3
- **TACH (RPM geri bildirimi)**: Intake=D24, Exhaust=D25, H4=D27 (INPUT_PULLUP)
- **Kullanım**: Menüden hangi fana tıklanırsa o fan aktif; encoder ile %10 kademe (0–100%); ekranda PWM % ve RPM gösterilir
- **Not**: 4-pin fan standardı 25 kHz PWM; TACH tipik 2 darbe/devir

#### Diwell DTP-UART-H04 IR Sıcaklık Sensörü
- **UART**: Serial3 (Arduino Mega TX3=D14, RX3=D15)
- **Baud**: 19200, 8N1
- **Bağlantı**: Sensör TX → Mega RX3 (D15), Sensör RX → Mega TX3 (D14), GND, VCC
- **Kullanım**: Menüden "IR temp. sens." seçilir; ekranda obje ve ortam (iç) sıcaklığı gösterilir; OK ile çıkış
- **Protokol**: Sorgu 4 byte (0x11 0x03 0x01 0x98), yanıt 8 byte (0x16 0x04 + obje 2B + ortam 2B + CRC + 0x9C), sıcaklık = 16 bit / 10 (°C)

#### DFPlayer Mini (Speakers)
- **UART**: Serial2 (Arduino Mega TX2=D16, RX2=D17)
- **Baud**: 9600, 8N1
- **Bağlantı**: DFPlayer RX → Mega TX2 (D16), DFPlayer TX → Mega RX2 (D17), VCC (5V), GND
- **Kullanım**: Menüden "Speakers" seçilir; track 1 çalar; encoder ile volume (0-30) ayarlanır; OK = pause/resume, tekrar OK = çıkış
- **Not**: SD kartta MP3 dosyaları olmalı (01.mp3, 02.mp3 vb.)

## Proje Yapısı

```
EdgePro Power Test Kiti/
├── src/
│   └── main.cpp          # Ana program dosyası
├── include/              # Header dosyaları
├── lib/                  # Kütüphane dosyaları
├── test/                 # Test dosyaları
├── platformio.ini        # PlatformIO yapılandırma dosyası
└── README.md            # Bu dosya
```

## Pin Tanımlamaları

| Bileşen | Pin | Tip | Açıklama |
|---------|-----|-----|----------|
| NTC Sensörü | A0 | Analog Giriş | Sıcaklık ölçümü |
| DS18B20 Sensörü | D4 | Dijital Giriş (OneWire) | Dijital sıcaklık ölçümü |
| RGB LED - Kırmızı | D9 | PWM Çıkış | RGB LED kırmızı kanal |
| RGB LED - Yeşil | D10 | PWM Çıkış | RGB LED yeşil kanal |
| RGB LED - Mavi | D11 | PWM Çıkış | RGB LED mavi kanal |
| RGB LED Güç | 12V | Harici Güç | RGB LED için 12V güç kaynağı |
| Rotary Encoder - DT | D22 | Dijital Giriş (INPUT_PULLUP) | Encoder data pini |
| Rotary Encoder - CLK | D23 | Dijital Giriş (INPUT_PULLUP) | Encoder clock pini |
| Rotary Encoder - SW | D26 | Dijital Giriş (INPUT_PULLUP) | Encoder switch/button pini |
| Buzzer | D12 | PWM Çıkış | Sesli uyarı ve bildirimler |
| Buton | D32 | Dijital Giriş (INPUT_PULLUP) | Buton basma algılama (pull-up bağlantılı) |
| OLED - SDA | D20 | I2C Data | OLED ekran I2C veri (SH1106) |
| OLED - SCL | D21 | I2C Clock | OLED ekran I2C saat (SH1106) |
| ST7789 - CS | D53 | Dijital Çıkış | Ekran chip select (Eski - Artık kullanılmıyor) |
| ST7789 - DC | D48 | Dijital Çıkış | Ekran data/command (Eski - Artık kullanılmıyor) |
| ST7789 - RST | D49 | Dijital Çıkış | Ekran reset (Eski - Artık kullanılmıyor) |
| ST7789 - SDA | D24 | Dijital Çıkış | Ekran veri (Software SPI) (Eski - Artık kullanılmıyor) |
| ST7789 - SCL | D25 | Dijital Çıkış | Ekran saat (Software SPI) (Eski - Artık kullanılmıyor) |
| TMC2130 - DIR | D44 | Dijital Çıkış | Step motor yön |
| TMC2130 - STEP | D45 | Dijital Çıkış | Step motor adım |
| TMC2130 - EN | D46 | Dijital Çıkış | Step motor enable |
| TMC2130 - CS | D47 | Dijital Çıkış | TMC2130 SPI chip select |
| TMC2130 - SPI | D50,D51,D52 | SPI | MISO, MOSI, SCK |
| Brake Çıkışı | D35 | Dijital Çıkış | Fren kontrolü |
| Intake Fan PWM | D5 | Timer3 25 kHz | Intake fan hız |
| Exhaust Fan PWM | D2 | Timer3 25 kHz | Exhaust fan hız |
| H4 Fan PWM | D3 | Timer3 25 kHz | H4 fan hız |
| Intake Fan TACH | D24 | Dijital Giriş | Intake RPM |
| Exhaust Fan TACH | D25 | Dijital Giriş | Exhaust RPM |
| H4 Fan TACH | D27 | Dijital Giriş | H4 RPM |
| DTP-UART-H04 (TX) | D14 | Serial3 TX | IR sensör komut (Mega→Sensör) |
| DTP-UART-H04 (RX) | D15 | Serial3 RX | IR sensör veri (Sensör→Mega) |
| DFPlayer Mini (TX) | D16 | Serial2 TX | DFPlayer komut (Mega→DFPlayer) |
| DFPlayer Mini (RX) | D17 | Serial2 RX | DFPlayer yanıt (DFPlayer→Mega) |

## Kurulum

1. PlatformIO IDE veya PlatformIO CLI kurulumu yapın
2. Projeyi klonlayın veya indirin
3. PlatformIO'da projeyi açın
4. Arduino Mega Mini'yi USB üzerinden bilgisayara bağlayın
5. Projeyi derleyin ve yükleyin

## Kullanım

### Derleme
```bash
pio run
```

### Yükleme
```bash
pio run -t upload
```

### Seri Monitör
```bash
pio device monitor
```

## Kod Yapısı

### Pin Tanımlamaları
```cpp
#define NTC_PIN A0          // NTC sıcaklık sensörü analog pin
#define DS18B20_PIN 4       // DS18B20 sıcaklık sensörü dijital pin (D4)
#define BRAKE_PIN 35        // Brake çıkışı dijital pin (D35)

// RGB LED pin tanımlamaları
#define RGB_RED_PIN 9       // RGB LED kırmızı kanal (D9 - PWM)
#define RGB_GREEN_PIN 10    // RGB LED yeşil kanal (D10 - PWM)
#define RGB_BLUE_PIN 11     // RGB LED mavi kanal (D11 - PWM)
// Not: RGB LED 12V harici güç kaynağı ile beslenir

// Rotary Encoder pin tanımlamaları
#define ENCODER_DT_PIN 22   // Rotary encoder DT (Data) pini (D22)
#define ENCODER_CLK_PIN 23  // Rotary encoder CLK (Clock) pini (D23)
#define ENCODER_SW_PIN 26   // Rotary encoder SW (Switch/Button) pini (D26)

// Buzzer pin tanımlaması
#define BUZZER_PIN 12       // Buzzer dijital pin (D12 - PWM)

// Buton pin tanımlaması
#define BUTTON_PIN 32       // Buton dijital pin (D32 - INPUT_PULLUP)

// OLED 1.3 inç Ekran pin tanımlamaları (I2C - SH1106)
// Arduino Mega'da I2C pinleri otomatik: SDA=20, SCL=21
#define OLED_WIDTH 128      // OLED genişlik
#define OLED_HEIGHT 64      // OLED yükseklik
#define OLED_RESET -1        // Reset pin yoksa -1
// I2C adresi: 0x3C veya 0x3D

// ST7789 Ekran pin tanımlamaları (Eski - Artık kullanılmıyor)
#define ST7789_CS_PIN 53    // ST7789 Chip Select pini (D53)
#define ST7789_DC_PIN 48    // ST7789 Data/Command pini (D48)
#define ST7789_RES_PIN 49   // ST7789 Reset pini (D49)
#define ST7789_SDA_PIN 24   // ST7789 Serial Data pini (D24 - Software SPI)
#define ST7789_SCL_PIN 25   // ST7789 Serial Clock pini (D25 - Software SPI)
```

### Kütüphaneler
- **OneWire**: DS18B20 sensörü için OneWire protokol desteği
- **DallasTemperature**: DS18B20 sensörü için sıcaklık okuma kütüphanesi
- **Adafruit GFX Library**: Grafik çizim fonksiyonları için temel kütüphane
- **Adafruit SH110X**: OLED 1.3 inç ekran (SH1106) için kütüphane
- **Adafruit ST7789 Library**: ST7789 ekran kontrolcüsü için özel kütüphane (Eski - Artık kullanılmıyor)

### Setup Fonksiyonu
- Seri haberleşme başlatılır (9600 baud)
- Pin yapılandırmaları yapılır
- Başlangıç durumları ayarlanır

### Loop Fonksiyonu
- Şu an boş, ileride eklemeler yapılacak

## Pin Değişikliği: Ekran SDA/SCL Neden D24 ve D25?

README’de eskiden ekran için **SDA=D51, SCL=D52** yazıyordu. Bu pinler Arduino Mega’nın **donanım SPI** pinleri (MOSI=51, SCK=52). Aynı SPI hattına **TMC2130 step motor sürücü** de bağlandığı için:

1. **Çakışma:** Hem ekran hem TMC2130 D51 (MOSI) ve D52 (SCK) kullanıyor. Program çalışırken TMC2130 SPI’ye yazınca ekran da aynı hattan sinyal alıyor → ekranda çizgiler, bozulma.
2. **Yükleme sırasında düzgün görünmesi:** Kod atılırken sadece bootloader/upload trafiği var; TMC2130 henüz SPI kullanmıyor. Program başlayınca TMC2130 da SPI kullanmaya başlayınca çakışma oluşuyor.

**Çözüm:** Ekranı bu ortak SPI hattından çıkarmak. Ekran için **Software SPI** kullanıp veri ve saati **başka iki pine** taşımak:

- **Eski (çakışan):** Ekran SDA → D51, SCL → D52 (TMC2130 ile ortak).
- **Yeni (çakışma yok):** Ekran SDA → **D24**, SCL → **D25** (sadece ekran; TMC2130 50, 51, 52’de kalıyor).

**Yapman gereken:** Ekran modülündeki SDA kablosunu D51’den çıkarıp **D24**’e, SCL kablosunu D52’den çıkarıp **D25**’e tak. CS (D53), DC (D48), RST (D49) bağlantıları aynı kalır. Kod tarafında ekran zaten D24/D25 (Software SPI) olarak ayarlı.

**Özet:** D51/D52 artık sadece TMC2130 için; ekran SDA/SCL için D24 ve D25 kullanılıyor. Bu sayede pin çakışması olmaz ve program çalışırken ekran bozulmaz.

## Geliştirme Notları

- Proje aktif olarak geliştirilmektedir
- Yeni özellikler ve bağlantılar eklenecektir
- Pin tanımlamaları sabit kalacak şekilde tasarlanmıştır

## Gelecek Geliştirmeler

- [ ] NTC sıcaklık okuma ve dönüşüm fonksiyonları
- [ ] DS18B20 sıcaklık okuma fonksiyonları
- [ ] RGB LED renk kontrolü ve efekt fonksiyonları
- [ ] Rotary encoder okuma ve yön tespiti fonksiyonları
- [ ] Buzzer ton ve melodi fonksiyonları
- [ ] Buton okuma ve debounce fonksiyonları
- [ ] ST7789 ekran başlatma ve grafik çizim fonksiyonları
- [ ] Brake kontrol algoritması
- [ ] Ek sensör ve çıkış bağlantıları
- [ ] Seri haberleşme protokolü
- [ ] Hata yönetimi ve güvenlik özellikleri

## Lisans

Bu proje özel bir projedir.

## İletişim

Proje hakkında sorularınız için lütfen proje sahibi ile iletişime geçin.
