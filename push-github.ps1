# Edge-Pro-Power-Test-Kiti- GitHub'a yukle
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$repo = "Edge-Pro-Power-Test-Kiti-"

# Remote var mi?
$origin = git remote get-url origin 2>$null
if (-not $origin) {
    $user = Read-Host "GitHub kullanici adinizi girin"
    if (-not $user) { Write-Host "Kullanici adi gerekli."; exit 1 }
    git remote add origin "https://github.com/$user/$repo.git"
    Write-Host "Remote eklendi: origin -> https://github.com/$user/$repo.git"
} else {
    Write-Host "Remote zaten var: $origin"
}

Write-Host "Push yapiliyor..."
git push -u origin main
Write-Host "Tamamlandi."
