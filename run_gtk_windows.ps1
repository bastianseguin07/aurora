# Ejecuta en Windows la misma interfaz scripts/gui_gtk.py que se usa en Ubuntu.
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$pixi = Get-Command pixi -ErrorAction SilentlyContinue
if (-not $pixi) {
    $localPixi = Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"
    if (Test-Path $localPixi) { $pixi = $localPixi }
}
if (-not $pixi) { throw "No se encontro Pixi. Ejecuta primero .\setup_gtk_windows.ps1 y revisa la instalacion de Pixi." }
$schemaDir = Join-Path $PSScriptRoot ".pixi\envs\default\Library\share\glib-2.0\schemas"
$schemaCompiler = Join-Path $PSScriptRoot ".pixi\envs\default\Library\bin\glib-compile-schemas.exe"
if (-not (Test-Path $schemaCompiler)) { throw "No se encontro glib-compile-schemas. Ejecuta .\setup_gtk_windows.ps1." }
& $schemaCompiler $schemaDir
if ($LASTEXITCODE -ne 0) { throw "No se pudieron compilar los esquemas GSettings de GTK." }
$env:GSETTINGS_SCHEMA_DIR = $schemaDir
& $pixi run gui
exit $LASTEXITCODE
