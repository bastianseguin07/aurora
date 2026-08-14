# Crea el entorno virtual (si no existe) e instala las dependencias del proyecto.
# Uso: desde PowerShell, parado en la carpeta Aurora:
#   .\setup.ps1

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if (-not (Test-Path ".\venv\Scripts\python.exe")) {
    Write-Host "Creando entorno virtual (venv)..."
    python -m venv venv
} else {
    Write-Host "El entorno virtual ya existe, se reutiliza."
}

Write-Host "Instalando dependencias (puede tardar unos minutos por Open3D)..."
& ".\venv\Scripts\python.exe" -m pip install --upgrade pip
& ".\venv\Scripts\python.exe" -m pip install -r requirements.txt

Write-Host ""
Write-Host "Listo. Para abrir la interfaz grafica:"
Write-Host "  .\venv\Scripts\python.exe scripts\gui.py"
