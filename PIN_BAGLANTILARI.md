# Pin Bağlantıları

Bu dosya **IQC Giriş Kalite Test Kiti** projesinde kullanılan kartların pin tanımlarını ve bağlantılarını açıklar. Ana kontrol kartı **Adafruit HUZZAH32 ESP32 Feather**’dır; sensör ve çıkışlar **STM32** tarafında olup ESP32 ile UART üzerinden haberleşir.

---

## 1. Adafruit HUZZAH32 ESP32 Feather (Ana Kart)

Proje bu kart üzerinde çalışır. OLED, encoder ve STM32 bağlantıları aşağıdadır.

### 1.1. Özet Tablo

| İşlev | GPIO | Feather Pin / Not | Açıklama |
|-------|------|-------------------|----------|
| **UART – RX** | 16 | RX (Serial1) | STM32’den gelen veri (STM32 TX → bu pin) |
| **UART – TX** | 17 | TX (Serial1) | STM32’ye giden komut (bu pin → STM32 RX) |
| **Encoder CLK** | 33 | – | Rotary encoder saat (KY-040 vb.) |
| **Encoder DT** | 12 | – | Rotary encoder veri |
| **Encoder SW** | 27 | – | Encoder butonu (menü seç / geri) |
| **OLED** | I2C | SDA, SCL | 128x64 SSD1306, adres 0x3C (donanım I2C) |

Kodda kullanılan sabitler: `UART_RX 16`, `UART_TX 17`, `ENCODER_CLK 33`, `ENCODER_DT 12`, `ENCODER_SW 27`, `SCREEN_ADDRESS 0x3C`.

### 1.2. UART (STM32) Bağlantısı

- **STM32 TX** → **ESP32 GPIO 16 (RX)**  
- **ESP32 GPIO 17 (TX)** → **STM32 RX**  
- **GND** ↔ **GND** (ortak toprak zorunlu)

Baud: **115200**, format **8N1**. STM32 5V çıkış kullanıyorsa ESP32 3.3V uyumu için voltaj dönüştürücü veya bölücü kullanın (örn. **VOLTAJ_DONUSTURUCU.md**).

**Sık hata:** STM32 TX’i ESP32 TX’e bağlamak. Veri almak için STM32 TX mutlaka ESP32 **RX (16)**’e gitmelidir.

### 1.3. OLED (I2C)

- **Feather SDA** → OLED SDA  
- **Feather SCL** → OLED SCL  
- **VCC** → 3.3V, **GND** → GND  

Sürücü: SSD1306, adres 0x3C. Kütüphane: Adafruit SSD1306 + Adafruit GFX.

### 1.4. Rotary Encoder (KY-040 veya benzeri)

- **CLK** → GPIO 33  
- **DT** → GPIO 12  
- **SW** → GPIO 27  
- **VCC** → 3.3V  
- **GND** → GND  

Tüm encoder pinleri kodda `INPUT_PULLUP`; harici pull-up gerekmez.

---

## 2. STM32 Tarafı (Kısa Özet)

STM32, sensörleri (NTC, IR, gesture vb.) ve fan/RGB/TMC birimlerini yönetir. ESP32 yalnızca:

- **Veri isteği:** `$A\r\n` gönderir; STM32 cevaben `$VAL1,VAL2,...,VAL15\r\n` formatında satır döner.
- **Komut gönderir:** Fan (`$F1`, `$F2`, `$F3`), RGB (`$LA...`), Fren (`$B0`/`$B1`).

Pin detayları ve protokol STM32 projesinde ve **SERI_HABERLESME.md** içinde tanımlıdır.

---

## 3. Kontrol Listesi (ESP32 + STM32)

- [ ] STM32 TX → ESP32 RX (GPIO 16)
- [ ] ESP32 TX (GPIO 17) → STM32 RX
- [ ] GND ortak
- [ ] Her iki tarafta baud 115200, 8N1
- [ ] OLED I2C (SDA/SCL, 0x3C) bağlı
- [ ] Encoder CLK=33, DT=12, SW=27, VCC/GND bağlı

---

## 4. Eski Kart: Arduino Mega (Serial2) – Referans

Proje artık **ESP32 Feather** ile sürüyor. Aşağıdaki bilgi yalnızca referans içindir.

### 4.1. Serial2 Pinleri (Mega)

- **TX2:** D16 (STM32’ye giden)
- **RX2:** D17 (STM32’den gelen)

### 4.2. Voltaj Uyumu (5V → 3.3V)

Arduino Mega TX (D16) 5V çıkış verir; STM32 RX 3.3V ise voltaj bölücü kullanılır:

- D16 → 2.2kΩ → ortak nokta → 4.7kΩ → GND  
- STM32 RX → ortak nokta  
- STM32 TX → Mega D17 (direkt, 3.3V genelde Mega RX için uygun)

GND mutlaka ortak olmalıdır.

---

## 5. Sorun Giderme

**ESP32’de veri görünmüyorsa:**

1. STM32 TX’in ESP32 **RX (16)**’a gittiğini kontrol edin (TX–TX değil).  
2. GND ortak mı kontrol edin.  
3. Baud 115200, 8N1 her iki tarafta aynı mı bakın.  
4. Serial Monitor’da `$A` gönderildikten sonra `$` ile başlayan satır geliyor mu izleyin (kod sadece `$` ile başlayan satırları işler).

**OLED açılmıyorsa:**

- I2C adresini (0x3C / 0x3D) ve SDA/SCL bağlantısını kontrol edin.  
- Kodda `SCREEN_ADDRESS 0x3C` kullanılıyor.

**Encoder tepki vermiyorsa:**

- CLK/DT/SW pinlerinin 33, 12, 27 olduğunu ve VCC/GND’in doğru bağlandığını kontrol edin.
