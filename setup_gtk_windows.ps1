# Prepara el entorno nativo de Windows para ejecutar la misma GUI GTK3 que en Ubuntu.
# Requiere Pixi: https://pixi.sh/latest/#installation
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

$pixi = Get-Command pixi -ErrorAction SilentlyContinue
if (-not $pixi) {
    $localPixi = Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"
    if (Test-Path $localPixi) { $pixi = $localPixi }
}
if (-not $pixi) {
    throw "No se encontro Pixi. Instalalo desde https://pixi.sh/latest/#installation y vuelve a ejecutar .\setup_gtk_windows.ps1"
}

& $pixi install
if ($LASTEXITCODE -ne 0) { throw "Pixi no pudo preparar las dependencias GTK/Open3D." }
$schemaDir = Join-Path $PSScriptRoot ".pixi\envs\default\Library\share\glib-2.0\schemas"
$schemaCompiler = Join-Path $PSScriptRoot ".pixi\envs\default\Library\bin\glib-compile-schemas.exe"
if (-not (Test-Path $schemaCompiler)) { throw "No se encontro glib-compile-schemas en el entorno GTK de Pixi." }
& $schemaCompiler $schemaDir
if ($LASTEXITCODE -ne 0) { throw "No se pudieron compilar los esquemas GSettings de GTK." }
Write-Host "Esquemas GSettings de GTK preparados."
& $pixi run python -c "import gi, cairo, open3d; gi.require_version('Gtk', '3.0'); from gi.repository import Gtk; print('GTK', Gtk.MAJOR_VERSION, 'Open3D', open3d.__version__)"
if ($LASTEXITCODE -ne 0) { throw "El entorno se instalo, pero GTK3/Open3D no se pudieron importar." }
Write-Host "Listo. Ejecuta .\run_gtk_windows.ps1 para abrir Aurora GTK."
