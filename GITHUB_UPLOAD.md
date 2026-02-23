# GitHub'a Yükleme

Repo adı: **Edge-Pro-Power-Test-Kiti-**

## 1. Proje klasöründe Terminal açın

- VS Code / Cursor: `Terminal > New Terminal` (proje kökünde açılır)
- veya Windows: Proje klasörüne gidip adres çubuğuna `cmd` yazıp Enter

## 2. Aşağıdaki komutları sırayla çalıştırın

**GitHub kullanıcı adınızı** aşağıdaki `KULLANICI_ADINIZ` yerine yazın.

```bash
git init
git add .
git commit -m "EdgePro Power Test Kiti - ESP32 Feather, STM32 UART, OLED, Encoder menü"
git branch -M main
git remote add origin https://github.com/KULLANICI_ADINIZ/Edge-Pro-Power-Test-Kiti-.git
git push -u origin main
```

## 3. İlk kez push'ta

- GitHub kullanıcı adı ve şifre (veya Personal Access Token) istenebilir.
- Şifre yerine **Personal Access Token** kullanmanız gerekebilir: GitHub → Settings → Developer settings → Personal access tokens.

## 4. Repo GitHub'da zaten doluysa (README vs. eklediyseniz)

```bash
git pull origin main --allow-unrelated-histories
git push -u origin main
```

## Yüklenecek başlıca dosyalar

- `src/main.cpp` – Ana kod
- `platformio.ini` – PlatformIO ayarları
- `SERI_HABERLESME.md` – Seri haberleşme dokümantasyonu
- `PIN_BAGLANTILARI.md` – Pin bağlantıları
- `README.md` – Proje açıklaması
- `.gitignore` – Git hariç tutulanlar

`.pio` (build ve kütüphaneler) yüklenmez; `.gitignore` ile hariç tutulur.
