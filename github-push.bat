@echo off
chcp 65001 >nul
cd /d "%~dp0"

echo ========================================
echo  Edge-Pro-Power-Test-Kiti- GitHub Push
echo ========================================
echo.

if not exist .git (
    echo Git init yapiliyor...
    git init
    echo.
)

echo Dosyalar ekleniyor...
git add .
echo.

echo Commit yapiliyor...
git commit -m "EdgePro Power Test Kiti - ESP32 Feather, STM32 UART, OLED, Encoder menu"
if errorlevel 1 (
    echo Uyari: Commit atlandi - degisiklik yok veya zaten commit edildi.
)
echo.

git branch -M main 2>nul

echo Remote kontrol ediliyor...
git remote show origin >nul 2>&1
if errorlevel 1 (
    set /p GITHUB_USER="GitHub kullanici adinizi girin: "
    git remote add origin https://github.com/%GITHUB_USER%/Edge-Pro-Power-Test-Kiti-.git
    echo Remote eklendi.
) else (
    echo Remote zaten var: origin
)
echo.

echo Push yapiliyor (origin main)...
git push -u origin main
echo.

echo Tamamlandi.
pause
