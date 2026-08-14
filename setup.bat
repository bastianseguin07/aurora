@echo off
REM Wrapper para poder correr el setup desde CMD o con doble clic
REM (setup.ps1 es un script de PowerShell, CMD no lo ejecuta directo).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup.ps1"
pause
