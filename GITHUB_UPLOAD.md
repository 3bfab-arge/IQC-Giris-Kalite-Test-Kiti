# GitHub'a Yükleme

Repo adı: **IQC-Giris-Kalite-Test-Kiti-**

## Yöntem 1: Script ile (en kolay)

1. Cursor'da **Terminal > New Terminal** açın (proje klasöründe açılır).
2. Şu komutu çalıştırın:

```powershell
.\push-github.ps1
```

3. İstenirse GitHub kullanıcı adınızı yazın, Enter'a basın.
4. Şifre/Token istenirse girin (Token: GitHub → Settings → Developer settings → Personal access tokens).

---

## Yöntem 2: Komutları elle yazın

**Proje klasöründe** Terminal açıp aşağıdaki iki satırı çalıştırın.  
`KULLANICI_ADINIZ` yerine **kendi GitHub kullanıcı adınızı** yazın.

```bash
git remote add origin https://github.com/KULLANICI_ADINIZ/IQC-Giris-Kalite-Test-Kiti-.git
git push -u origin main
```

Not: Commit zaten yapıldı, sadece remote ekleyip push etmeniz yeterli.

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
