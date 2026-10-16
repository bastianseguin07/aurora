# Ejecuta en Windows la misma interfaz scripts/gui_gtk.py que se usa en Ubuntu.
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$pixi = Get-Command pixi -ErrorAction SilentlyContinue
if (-not $pixi) {
    $localPixi = Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"
    if (Test-Path $localPixi) { $pixi = $localPixi }
}
if (-not $pixi) { throw "No se encontro Pixi. Ejecuta primero .\setup_gtk_windows.ps1 y revisa la instalacion de Pixi." }
& $pixi run gui
exit $LASTEXITCODE
