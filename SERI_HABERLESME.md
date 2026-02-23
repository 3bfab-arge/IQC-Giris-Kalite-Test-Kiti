# Seri Haberleşme Protokolü Dokümantasyonu

## Genel Bakış

Bu dokümantasyon, ESP32 Feather (Adafruit HUZZAH32) ile STM32 mikrodenetleyici arasındaki UART seri haberleşme protokolünü açıklar. Sistem, sensör verilerini okumak, fan hızlarını kontrol etmek, RGB LED ayarlamak, fren motorunu kontrol etmek ve TMC status bilgilerini görüntülemek için kullanılır.

---

## 1. Donanım Bağlantıları

### Pin Bağlantıları
- **STM32 TX** → **ESP32 D16 (GPIO 16, RX)**
- **STM32 RX** → **ESP32 D17 (GPIO 17, TX)**
- **GND** → **GND** (Ortak toprak)

### UART Ayarları
- **Baud Rate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Format:** 8N1

**Not:** STM32'nin 5V çıkışı varsa, ESP32'nin 3.3V seviyesine uyum için voltaj dönüştürücü kullanılmalıdır. Detaylar için `VOLTAJ_DONUSTURUCU.md` dosyasına bakınız.

---

## 2. Veri Okuma Protokolü

### 2.1. Veri İsteme Komutu

ESP32'den STM32'ye gönderilen komut:

```
$A\r\n
```

**Format Açıklaması:**
- `$` - Komut başlangıç karakteri
- `A` - Veri isteme komutu
- `\r\n` - Satır sonu karakterleri (CRLF)

**Gönderme:**
```cpp
Serial1.print("$A\r\n");
Serial1.flush();
delay(10); // STM32 cevabı için kısa bekleme
```

### 2.2. STM32'den Gelen Veri Formatı

STM32, aşağıdaki formatta veri gönderir:

```
$VAL1,VAL2,VAL3,VAL4,VAL5,VAL6,VAL7,VAL8,VAL9,VAL10,VAL11,VAL12,VAL13,VAL14,VAL15\r\n
```

**Örnek:**
```
$222,286,264,0,150,200,180,1,1,0,1,1,0,1,0\r\n
```

**Veri Açıklaması:**
- `$` - Veri başlangıç karakteri (zorunlu)
- `VAL1` - MCU Load (0-9999, gerçek değer = VAL1/10)
- `VAL2` - PCB Temperature (0-9999, gerçek değer = VAL2/10)
- `VAL3` - Plate Temperature (NTC) (0-9999, gerçek değer = VAL3/10)
- `VAL4` - Resin Temperature (IR Sensor) (0-9999, gerçek değer = VAL4/10)
- `VAL5` - Intake Fan 1 RPM (0-9999, gerçek değer = VAL5/10)
- `VAL6` - Intake Fan 2 RPM (0-9999, gerçek değer = VAL6/10)
- `VAL7` - Exhaust Fan RPM (0-9999, gerçek değer = VAL7/10)
- `VAL8` - Gesture Type (0-4: GESTURE_NONE=0, GESTURE_UP=1, GESTURE_DOWN=2, GESTURE_LEFT=3, GESTURE_RIGHT=4)
- `VAL9` - Z TMC Status Stop Right (1 veya 0)
- `VAL10` - Y TMC Status Stop Right (1 veya 0)
- `VAL11` - Y TMC Status Stop Left (1 veya 0)
- `VAL12` - CVR1 TMC Status Stop Right (1 veya 0)
- `VAL13` - CVR1 TMC Status Stop Left (1 veya 0)
- `VAL14` - CVR2 TMC Status Stop Right (1 veya 0)
- `VAL15` - CVR2 TMC Status Stop Left (1 veya 0)
- `\r\n` - Satır sonu karakterleri

**Not:** 
- İlk 7 değer (VAL1-VAL7) 10'a bölünerek gerçek değerlere dönüştürülür. Örneğin `222` → `22.2`
- Gesture Type (VAL8) direkt kullanılır (0-4 arası)
- TMC Status değerleri (VAL9-VAL15) direkt kullanılır (1 = BASILI, 0 = BASILI DEGIL)

### 2.3. Veri Okuma Algoritması

1. **Komut Gönderme:**
   - `$A\r\n` komutu gönderilir
   - `Serial1.flush()` ile gönderim tamamlanır
   - 30ms bekleme yapılır

2. **Veri Okuma:**
   - Timeout: 150ms (gecikmeyi azaltmak için kısaltıldı)
   - Buffer boyutu: 64 byte
   - `\r` veya `\n` karakterine kadar okunur
   - Sadece printable ASCII karakterler (32-126) kabul edilir

3. **Veri Doğrulama:**
   - İlk karakter `$` olmalıdır
   - `$` ile başlamayan satırlar yok sayılır

4. **Veri Parse Etme:**
   - Virgülle ayrılmış sayılar parse edilir (15 değer bekleniyor)
   - İlk 7 değer 10'a bölünerek float değere dönüştürülür
   - Gesture Type (8. değer) direkt kullanılır (0-4)
   - TMC Status değerleri (9-15. değerler) direkt kullanılır (1 veya 0)
   - En az 4 değer beklenir (geri uyumluluk için)
   - 7 değer varsa fan RPM değerleri güncellenir
   - 8 değer varsa gesture_type güncellenir
   - 15 değer varsa tüm TMC status değerleri güncellenir

### 2.4. Veri Okuma Sıklığı

- **Normal Okuma Aralığı:** Her 50ms (saniyede 20 kez)
- **Gesture Ekranında:** Her 20ms (saniyede ~50 kez - en hızlı algılama)
- **Timeout:** 150ms
- **Bekleme Süresi:** 10ms (STM32 cevap vermesi için)

**Kod:**
```cpp
unsigned long readInterval = (currentMenu == MENU_GESTURE) ? GESTURE_READ_MS : READ_INTERVAL_MS;
if (now - lastRead >= readInterval) {
  lastRead = now;
  readSTM32Data();
}
```

**Hızlı Güncelleme:**
- Veri parse edildikten hemen sonra, ilgili ekrandaysa ekran anında güncellenir (loop beklemeden)
- Gesture ve TMC Ref ekranları için özel optimizasyon yapılmıştır

---

## 3. Fan Kontrol Komutları

### 3.1. Intake Fan Kontrolü

ESP32'den STM32'ye gönderilen komutlar:

**Fan 1:**
```
$F1HHHH\r\n
```

**Fan 2:**
```
$F2HHHH\r\n
```

**Format Açıklaması:**
- `$` - Komut başlangıç karakteri
- `F` - Fan kontrol komutu
- `1` veya `2` - Fan numarası (1 = Intake Fan 1, 2 = Intake Fan 2)
- `HHHH` - Hız değeri (0-1999 arası, 4 haneli)
- `\r\n` - Satır sonu karakterleri

**Hız Hesaplama:**
```
Hız Değeri = (Yüzde * 1999) / 100
```

**Örnekler:**
- %0 → `$F10\r\n` ve `$F20\r\n`
- %50 → `$F1999\r\n` ve `$F2999\r\n` (999 = 1999/2)
- %100 → `$F11999\r\n` ve `$F21999\r\n`

**Not:** Her iki fan için aynı hız değeri gönderilir. Encoder ile hız ayarlandığında otomatik olarak her iki fan için komut gönderilir.

**Gönderme:**
```cpp
// Fan 1
Serial1.print("$F1");
Serial1.print(fanSpeedValue);
Serial1.print("\r\n");
Serial1.flush();

delay(25); // F1/F2 arası kısa gecikme

// Fan 2
Serial1.print("$F2");
Serial1.print(fanSpeedValue);
Serial1.print("\r\n");
Serial1.flush();
```

### 3.2. Exhaust Fan Kontrolü

ESP32'den STM32'ye gönderilen komut:

```
$F3HHHH\r\n
```

**Format Açıklaması:**
- `$` - Komut başlangıç karakteri
- `F` - Fan kontrol komutu
- `3` - Fan numarası (3 = Exhaust Fan)
- `HHHH` - Hız değeri (0-1999 arası, 4 haneli)
- `\r\n` - Satır sonu karakterleri

**Hız Hesaplama:**
```
Hız Değeri = (Yüzde * 1999) / 100
```

**Örnekler:**
- %0 → `$F30\r\n`
- %50 → `$F3999\r\n` (999 = 1999/2)
- %100 → `$F31999\r\n`

**Gönderme:**
```cpp
Serial1.print("$F3");
Serial1.print(exhaustFanSpeedValue);
Serial1.print("\r\n");
Serial1.flush();
```

### 3.3. Fan Hız Ayarlama

**Encoder Kontrolü:**
- Encoder her çevrildiğinde hız %10 artar veya azalır
- Hız aralığı: 0-100 (%10'luk adımlarla: 0, 10, 20, 30, ..., 100)
- Encoder hareket ettiğinde komut otomatik olarak gönderilir

**Başlangıç:**
- Fan menülerine girildiğinde hız %0 ile başlar
- Encoder pozisyonu korunur (sıfırlanmaz)

### 3.4. RGB LED Kontrolü

ESP32'den STM32'ye gönderilen komut:

```
$LAHHH,SSS,VVV\r\n
```

**Format Açıklaması:**
- `$` - Komut başlangıç karakteri
- `LA` - RGB LED kontrol komutu (sabit)
- `HHH` - Hue değeri (0-360 arası)
- `SSS` - Saturation değeri (0-100 arası)
- `VVV` - Value/Brightness değeri (0-100 arası)
- `\r\n` - Satır sonu karakterleri

**Örnekler:**
- HSV(0, 100, 100) → `$LA0,100,100\r\n`
- HSV(320, 50, 80) → `$LA320,50,80\r\n`
- HSV(180, 100, 100) → `$LA180,100,100\r\n`

**Kontrol:**
- Encoder ile parametre seçimi (H, S, V arasında)
- Encoder ile değer ayarlama (%10'luk adımlarla)
- Encoder hareket ettiğinde komut otomatik olarak gönderilir

**Gönderme:**
```cpp
Serial1.print("$LA");
Serial1.print(rgbHue);
Serial1.print(",");
Serial1.print(rgbSaturation);
Serial1.print(",");
Serial1.print(rgbValue);
Serial1.print("\r\n");
Serial1.flush();
```

### 3.5. Fren Motoru Kontrolü

ESP32'den STM32'ye gönderilen komutlar:

**Aktif:**
```
$B1\r\n
```

**Pasif:**
```
$B0\r\n
```

**Format Açıklaması:**
- `$` - Komut başlangıç karakteri
- `B` - Fren motoru kontrol komutu
- `1` veya `0` - Durum (1 = Aktif, 0 = Pasif)
- `\r\n` - Satır sonu karakterleri

**Kontrol:**
- Encoder sağa döndürülünce → Aktif ($B1)
- Encoder sola döndürülünce → Pasif ($B0)
- Encoder hareket ettiğinde komut otomatik olarak gönderilir
- Menüye girildiğinde durum pasif (false) ile başlar

**Gönderme:**
```cpp
Serial1.print("$B");
Serial1.print(active ? "1" : "0");
Serial1.print("\r\n");
Serial1.flush();
```

---

## 4. Veri Değişkenleri

### 4.1. Okunan Veriler

**Sensör Verileri (Float):**
```cpp
float mcu_load_raw = 0.0;        // MCU yükü (gerçek değer)
float pcb_temp_raw = 0.0;        // PCB sıcaklığı (°C)
float plate_temp_raw = 0.0;       // Plate sıcaklığı - NTC (°C)
float resin_temp_raw = 0.0;       // Resin sıcaklığı - IR Sensor (°C)
float intake1_fan_raw = 0.0;      // Intake Fan 1 RPM
float intake2_fan_raw = 0.0;      // Intake Fan 2 RPM
float exthaust_fan_raw = 0.0;     // Exhaust Fan RPM
```

**Gesture Sensor:**
```cpp
int gesture_type = 0;  // 0-4 arası (GESTURE_NONE=0, GESTURE_UP=1, GESTURE_DOWN=2, GESTURE_LEFT=3, GESTURE_RIGHT=4)
```

**TMC Status Değerleri (1 veya 0):**
```cpp
int z_tmc_status_stop_r = 0;      // Z TMC Status Stop Right (1=BASILI, 0=BASILI DEGIL)
int y_tmc_status_stop_r = 0;      // Y TMC Status Stop Right
int y_tmc_status_stop_l = 0;      // Y TMC Status Stop Left
int cvr1_tmc_status_stop_r = 0;   // CVR1 TMC Status Stop Right
int cvr1_tmc_status_stop_l = 0;   // CVR1 TMC Status Stop Left
int cvr2_tmc_status_stop_r = 0;   // CVR2 TMC Status Stop Right
int cvr2_tmc_status_stop_l = 0;   // CVR2 TMC Status Stop Left
```

### 4.2. Fan Kontrol Değişkenleri

**Intake Fan:**
```cpp
int fanSpeedPercent = 0;          // 0-100 arası, %10'luk adımlarla
bool fanSpeedSent = false;        // Komut gönderildi mi?
```

**Exhaust Fan:**
```cpp
int exhaustFanSpeedPercent = 0;   // 0-100 arası, %10'luk adımlarla
bool exhaustFanSpeedSent = false; // Komut gönderildi mi?
```

**RGB LED:**
```cpp
int rgbHue = 0;                   // Hue: 0-360 arası
int rgbSaturation = 100;          // Saturation: 0-100 arası
int rgbValue = 100;                // Value/Brightness: 0-100 arası
int rgbSelectedParam = 0;         // 0: Hue, 1: Saturation, 2: Value
int rgbParamSelection = 0;         // Parametre seçimi (0-3)
bool rgbMode = false;              // false: parametre seçimi, true: değer ayarlama
bool rgbCommandSent = false;       // Komut gönderildi mi?
```

**Fren Motoru:**
```cpp
bool brakeMotorActive = false;     // false: pasif ($B0), true: aktif ($B1)
```

---

## 5. Komut Özet Tablosu

| Komut | Açıklama | Gönderen | Alıcı | Format |
|-------|----------|----------|-------|--------|
| `$A\r\n` | Sensör verilerini iste | ESP32 | STM32 | `$A\r\n` |
| `$F1HHHH\r\n` | Intake Fan 1 hızını ayarla | ESP32 | STM32 | `$F1` + hız (0-1999) + `\r\n` |
| `$F2HHHH\r\n` | Intake Fan 2 hızını ayarla | ESP32 | STM32 | `$F2` + hız (0-1999) + `\r\n` |
| `$F3HHHH\r\n` | Exhaust Fan hızını ayarla | ESP32 | STM32 | `$F3` + hız (0-1999) + `\r\n` |
| `$LAHHH,SSS,VVV\r\n` | RGB LED HSV ayarla | ESP32 | STM32 | `$LA` + Hue + `,` + Sat + `,` + Val + `\r\n` |
| `$B0\r\n` | Fren motorunu pasif yap | ESP32 | STM32 | `$B0\r\n` |
| `$B1\r\n` | Fren motorunu aktif yap | ESP32 | STM32 | `$B1\r\n` |
| `$VAL1,...,VAL15\r\n` | Sensör verileri (15 değer) | STM32 | ESP32 | `$` + virgülle ayrılmış değerler + `\r\n` |

---

## 6. Zamanlama Diyagramı

### Veri Okuma (Her 50ms, Gesture ekranında 20ms)
```
ESP32                    STM32
  |                        |
  |---[$A\r\n]----------->|
  |                        |
  |<--[$222,286,...,1,1,0,1,1,0,1,0\r\n]--|
  |                        |
  |  (10ms bekleme)        |
  |                        |
```

### Fan Kontrol (Encoder Hareketi)
```
ESP32                    STM32
  |                        |
  |---[$F1550\r\n]------->|
  |                        |
  |  (25ms bekleme)        |
  |                        |
  |---[$F2550\r\n]------->|
  |                        |
```

### RGB LED Kontrol (Encoder Hareketi)
```
ESP32                    STM32
  |                        |
  |---[$LA320,100,100\r\n]->|
  |                        |
```

### Fren Motoru Kontrol (Encoder Hareketi)
```
ESP32                    STM32
  |                        |
  |---[$B1\r\n]----------->|  (Encoder sağa → Aktif)
  |                        |
  |---[$B0\r\n]----------->|  (Encoder sola → Pasif)
  |                        |
```

---

## 7. Hata Durumları ve Çözümler

### 7.1. Veri Okunamıyor
- **Sebep:** Timeout (500ms içinde veri gelmedi)
- **Çözüm:** STM32'nin çalıştığından ve bağlantıların doğru olduğundan emin olun

### 7.2. Veri Parse Edilemiyor
- **Sebep:** `$` ile başlamayan satırlar
- **Çözüm:** STM32'nin doğru formatta veri gönderdiğinden emin olun

### 7.3. Fan Komutları Çalışmıyor
- **Sebep:** Yanlış format veya hız değeri
- **Çözüm:** Komut formatını kontrol edin (`$F1`, `$F2`, `$F3` + hız + `\r\n`)

### 7.4. UART Veri Bozulması
- **Sebep:** Baud rate uyumsuzluğu veya voltaj seviyesi sorunu
- **Çözüm:** Baud rate'i kontrol edin (115200) ve voltaj dönüştürücü kullanın

---

## 8. Örnek Kullanım Senaryoları

### Senaryo 1: Sensör Verilerini Okuma
```
1. ESP32: $A\r\n gönderir
2. STM32: $222,286,264,0,150,200,180,1,1,0,1,1,0,1,0\r\n gönderir
3. ESP32: Verileri parse eder ve değişkenlere kaydeder
   - mcu_load_raw = 22.2
   - pcb_temp_raw = 28.6
   - plate_temp_raw = 26.4
   - resin_temp_raw = 0.0
   - intake1_fan_raw = 15.0
   - intake2_fan_raw = 20.0
   - exthaust_fan_raw = 18.0
   - gesture_type = 1 (GESTURE_UP)
   - z_tmc_status_stop_r = 1 (BASILI)
   - y_tmc_status_stop_r = 1, y_tmc_status_stop_l = 0
   - cvr1_tmc_status_stop_r = 1, cvr1_tmc_status_stop_l = 0
   - cvr2_tmc_status_stop_r = 1, cvr2_tmc_status_stop_l = 0
```

### Senaryo 2: Intake Fan Hızını %50'ye Ayarlama
```
1. Kullanıcı encoder ile hızı %50'ye ayarlar
2. ESP32 otomatik olarak:
   - Hız değeri = (50 * 1999) / 100 = 999
   - $F1999\r\n gönderir
   - 50ms bekler
   - $F2999\r\n gönderir
```

### Senaryo 3: Exhaust Fan Hızını %100'e Ayarlama
```
1. Kullanıcı encoder ile hızı %100'e ayarlar
2. ESP32 otomatik olarak:
   - Hız değeri = (100 * 1999) / 100 = 1999
   - $F31999\r\n gönderir
```

### Senaryo 4: RGB LED HSV Ayarlama
```
1. Kullanıcı RGB LED menüsüne girer
2. Encoder ile parametre seçer (H, S, V)
3. Encoder ile değeri ayarlar (örn: Hue=320, Saturation=100, Value=100)
4. ESP32 otomatik olarak:
   - $LA320,100,100\r\n gönderir
```

### Senaryo 5: TMC Status Okuma
```
1. ESP32: $A\r\n gönderir
2. STM32: $222,286,264,0,150,200,180,1,1,0,1,1,0,1,0\r\n gönderir
3. ESP32 parse eder:
   - Z Ref ekranında: z_tmc_status_stop_r = 1 → "BASILI" gösterilir
   - Y Ref ekranında: y_tmc_status_stop_r = 1 (Right BASILI), y_tmc_status_stop_l = 0 (Left BASILI DEGIL)
   - CVR1 Ref ekranında: cvr1_tmc_status_stop_r = 1, cvr1_tmc_status_stop_l = 0
   - CVR2 Ref ekranında: cvr2_tmc_status_stop_r = 1, cvr2_tmc_status_stop_l = 0
```

### Senaryo 6: Fren Motoru Kontrolü
```
1. Kullanıcı BRAKE MOTOR menüsüne girer
2. Encoder sağa döndürülünce:
   - brakeMotorActive = true
   - ESP32 otomatik olarak: $B1\r\n gönderir
   - Ekranda "AKTIF" gösterilir
3. Encoder sola döndürülünce:
   - brakeMotorActive = false
   - ESP32 otomatik olarak: $B0\r\n gönderir
   - Ekranda "PASIF" gösterilir
4. Butona basınca ana menüye dönülür
```

---

## 9. Teknik Detaylar

### 9.1. UART Başlatma
```cpp
Serial1.setPins(UART_RX, UART_TX);  // Pin tanımlama
Serial1.begin(UART_BAUD);             // 115200 baud
Serial1.flush();                      // Buffer temizleme
```

### 9.2. Veri Gönderme
```cpp
Serial1.print("$A\r\n");  // Komut gönderme
Serial1.flush();          // Gönderimin tamamlanmasını bekleme
```

### 9.3. Veri Okuma
```cpp
if (Serial1.available()) {
  char c = Serial1.read();
  // Karakter işleme
}
```

### 9.4. Buffer Yönetimi
- Okuma buffer'ı: 64 byte
- Timeout: 150ms (gecikmeyi azaltmak için kısaltıldı)
- Printable ASCII kontrolü: 32-126 arası karakterler

### 9.5. Hızlı Yazdırma Optimizasyonları
- **Serial Print:** Tek `Serial.println()` kullanılarak çoklu `Serial.print()` çağrılarından kaçınılır
- **Ekran Güncelleme:** Veri parse edildikten hemen sonra ilgili ekran anında güncellenir (loop beklemeden)
- **Gesture Özel:** Gesture ekranında okuma sıklığı 20ms'ye düşürülür (saniyede ~50 istek)
- **Periyodik Yenileme:** Ekran her 50ms'de bir yenilenir (saniyede 20 kez)

---

## 10. Test ve Debug

### Serial Monitor Çıktıları

**Veri Okuma:**
```
$222,286,264,0,150,200,180,1,1,0,1,1,0,1,0 | 22.2 28.6 26.4 0.0 15.0 20.0 18.0 g1 z1 y1,0 c11,0 c21,0
```

**Fan Komutları:**
```
Gonderildi: $F1999\r\n
 F2999\r\n
```

**RGB LED Komutu:**
```
Gonderildi: $LA320,100,100\r\n
```

**Fren Motoru Komutu:**
```
Brake Motor: $B1  (Aktif)
Brake Motor: $B0  (Pasif)
```

**TMC Status:**
- Z Ref ekranında: "BASILI" veya "BASILI DEGIL" + değer (1 veya 0)
- Y/CVR1/CVR2 Ref ekranlarında: Right ve Left durumları gösterilir

### Debug İpuçları
- Serial Monitor'da `\r\n` karakterleri görünmeyebilir, bu normaldir
- Gerçek gönderimde `\r\n` karakterleri gönderilir
- Timeout durumunda veri okunmaz, bir sonraki okuma denemesinde tekrar denenir

---

## 11. Menü Sistemi

### 11.1. Ana Menü Öğeleri
1. **IR Temp Sensor** - Resin sıcaklığını gösterir
2. **NTC** - Plate sıcaklığını gösterir
3. **INTAKE FAN** - Intake fan hızını ayarlar (%0-100)
4. **EXHAUST FAN** - Exhaust fan hızını ayarlar (%0-100)
5. **RGB LED** - HSV renk ayarları (Hue: 0-360, Saturation: 0-100, Value: 0-100)
6. **Gesture Sensor** - Gesture tipini gösterir (GESTURE_NONE, GESTURE_UP, vb.)
7. **Z Ref** - Z TMC Status Stop Right durumunu gösterir (BASILI/BASILI DEGIL)
8. **Y Ref** - Y TMC Status Stop Right ve Left durumlarını gösterir
9. **CVR1 Ref** - CVR1 TMC Status Stop Right ve Left durumlarını gösterir
10. **CVR2 Ref** - CVR2 TMC Status Stop Right ve Left durumlarını gösterir
11. **BRAKE MOTOR** - Fren motorunu kontrol eder (Aktif/Pasif)

### 11.2. TMC Ref Ekranları
- **Durum Gösterimi:** Merkezde büyük fontla "BASILI" veya "BASILI DEGIL"
- **Değer Gösterimi:** Alt kısımda sayısal değer (1 veya 0)
- **Y/CVR1/CVR2:** Right durumu merkezde, Left durumu altta gösterilir
- **Buton:** Encoder butonuna basınca ana menüye dönülür

### 11.3. Fren Motoru Ekranı
- **Başlık:** "BRAKE MOTOR" (üst kısımda)
- **Durum Gösterimi:** Merkezde büyük fontla (TextSize 2) "AKTIF" veya "PASIF"
- **Encoder Kontrolü:** 
  - Sağa döndürme → Aktif ($B1)
  - Sola döndürme → Pasif ($B0)
  - Encoder her hareket ettiğinde komut otomatik gönderilir
- **Alt Bilgi Satırları:**
  - Satır 1 (48. pixel): "Encoder: AKTIF" veya "Encoder: PASIF"
  - Satır 2 (56. pixel): "Komut: $B1" veya "Komut: $B0"
- **Buton:** Encoder butonuna basınca ana menüye dönülür
- **Başlangıç:** Menüye girildiğinde durum pasif (false) ile başlar
- **Ekran Güncelleme:** Encoder hareket ettiğinde ekran anında güncellenir

---

## 12. Versiyon Bilgisi

- **Protokol Versiyonu:** 2.1
- **Son Güncelleme:** 2026-01-30
- **ESP32 Kart:** Adafruit HUZZAH32 Feather
- **STM32:** (Kart modeli belirtilmeli)
- **Yeni Özellikler (v2.1):**
  - Fren motoru kontrolü ($B0/$B1)
  - BRAKE MOTOR menü öğesi eklendi
- **Önceki Özellikler (v2.0):**
  - Gesture sensor desteği
  - RGB LED HSV kontrolü
  - TMC Status monitoring (Z, Y, CVR1, CVR2)
  - Hızlı veri okuma optimizasyonları (50ms normal, 20ms gesture)
  - Anında ekran güncelleme (loop beklemeden)

---

## 13. İletişim ve Destek

Sorularınız veya sorunlarınız için:
- Kod dosyası: `src/main.cpp`
- Pin bağlantıları: `PIN_BAGLANTILARI.md`
- Voltaj dönüştürücü: `VOLTAJ_DONUSTURUCU.md`

---

**Not:** Bu dokümantasyon, mevcut kod implementasyonuna göre hazırlanmıştır. STM32 tarafında yapılan değişiklikler bu dokümantasyona yansıtılmalıdır.
