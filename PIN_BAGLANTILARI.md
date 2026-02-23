# UART Pin Bağlantıları

---

## Adafruit HUZZAH32 ESP32 Feather (Şu an kullanılan)

**Docklight'ta görünüyor, Feather'da görünmüyorsa kontrol listesi:**

1. **Bağlantı (mutlaka böyle olmalı):**
   - **STM32 TX** → **Feather RX (GPIO 16)**  
   - **STM32 RX** → **Feather TX (GPIO 17)**  
   - **Feather GND** ↔ **STM32 GND** (ortak toprak, olmazsa olmaz!)

2. **Sık yapılan hata:**  
   STM32 TX'i Feather **TX**'e bağlamak → veri gelmez.  
   STM32 TX her zaman **Feather RX**'e gitmeli.

3. **Pin karışıklığı:**  
   Kart üzerinde "RX" yazan pin = GPIO 16 (giriş).  
   "TX" yazan pin = GPIO 17 (çıkış).  
   STM32'nin **gönderen (TX)** çıkışı Feather'ın **RX**'ine bağlanmalı.

4. **Baud rate:**  
   STM32 ve kod 115200 ise aynı kalmalı.

5. **Deneme:**  
   Feather'da RX/TX etiketli pinler 16/17 değilse koda `Serial1.setPins(23, 22);` veya kart pinout'una göre doğru GPIO'yu yazıp tekrar dene.

---

## STM32 - Arduino Mega Serial2 Pin Bağlantıları (Eski kart)

## Serial2 Pin Tanımları
- **Arduino Mega TX2**: D16 (TX çıkışı)
- **Arduino Mega RX2**: D17 (RX girişi)

## Bağlantı Şeması

### 1. Arduino TX (D16) → STM32 RX (Voltaj Bölücü ile)

```
Arduino Mega D16 (TX2) 
    |
    |--[2.2kΩ]--+--[4.7kΩ]--GND
    |           |
    |        STM32 RX (3.41V)
```

**Adım adım bağlantı:**
1. **2.2kΩ direnç:**
   - Bir ucu → Arduino Mega D16 pinine
   - Diğer ucu → Ortak nokta (iki direncin birleştiği yer)

2. **4.7kΩ direnç:**
   - Bir ucu → Ortak nokta (2.2k ile birleşen yer)
   - Diğer ucu → GND

3. **STM32 RX:**
   - Ortak noktaya bağlanır (2.2k ve 4.7k'ın birleştiği yer)
   - Bu noktada voltaj: **~3.41V**

### 2. STM32 TX → Arduino RX (D17) - Direkt Bağlantı

```
STM32 TX (3.3V) -------- Arduino Mega D17 (RX2)
                         (Direkt bağlantı, direnç YOK)
```

**Adım adım bağlantı:**
- STM32 TX pininden direkt Arduino Mega D17 pinine bağlayın
- **DİKKAT:** Burada voltaj bölücü YOK, direkt bağlantı!
- Arduino Mega RX pinleri genellikle 5V tolerant'tır, 3.3V'u kabul eder

### 3. GND Bağlantısı (ÇOK ÖNEMLİ!)

```
Arduino Mega GND ←→ STM32 GND
```

**Mutlaka bağlanmalı:**
- Arduino Mega'nın GND pininden STM32'nin GND pinine bağlayın
- **Ortak toprak şart!** Yoksa haberleşme çalışmaz

## Bağlantı Özeti

| Arduino Mega | Bağlantı | STM32 |
|--------------|----------|-------|
| D16 (TX2) | → [2.2kΩ] → [4.7kΩ] → GND | → RX (ortak nokta) |
| D17 (RX2) | ← (direkt) | ← TX |
| GND | ←→ | ←→ GND |

## Voltaj Seviyeleri

- **Arduino TX (D16)**: 5V (idle HIGH)
- **Voltaj bölücü çıkışı**: ~3.41V (STM32 RX'e giden)
- **STM32 TX**: 3.3V (idle HIGH)
- **Arduino RX (D17)**: 3.3V alır (5V tolerant)

## Kontrol Listesi

- [ ] Arduino D16 → 2.2kΩ → 4.7kΩ → GND bağlantısı yapıldı
- [ ] STM32 RX → Voltaj bölücünün ortak noktasına bağlandı
- [ ] STM32 TX → Arduino D17'e direkt bağlandı (direnç YOK)
- [ ] Arduino GND ↔ STM32 GND bağlandı (ortak toprak)
- [ ] Her iki tarafta da baud rate 115200
- [ ] Her iki tarafta da 8N1 (8 data bits, no parity, 1 stop bit)

## Sorun Giderme

**Veri gelmiyorsa kontrol edin:**

1. **Multimetre ile voltaj ölçümü:**
   - Arduino D16'da → ~5V olmalı
   - Voltaj bölücü çıkışında → ~3.4V olmalı
   - STM32 TX'te → ~3.3V olmalı
   - Arduino D17'de → STM32 TX ile aynı voltaj olmalı

2. **GND bağlantısı:**
   - Mutlaka ortak GND olmalı
   - GND bağlantısı kopuk olabilir

3. **Pin bağlantıları:**
   - D16 ve D17 pinlerinin doğru olduğundan emin olun
   - Başka bir pinle karışmış olabilir

4. **Serial2 başlatma:**
   - Kodda `Serial2.begin(115200)` çağrıldığından emin olun
   - Baud rate her iki tarafta da aynı olmalı

## Test

1. Arduino kodunu yükleyin
2. Serial Monitor'u açın (115200 baud)
3. STM32'den veri gönderin
4. Serial Monitor'da "D17 pin degisti" mesajlarını kontrol edin
5. "Available" değerinin artıp artmadığını kontrol edin
