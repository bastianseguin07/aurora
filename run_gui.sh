#!/usr/bin/env bash
# Abre la GUI de Aurora (GTK3). Pensado para lanzarse con doble click desde
# el acceso directo del Escritorio (Aurora.desktop), no solo desde terminal.
set -e
cd "$(dirname "$0")"

if [ ! -x "./venv/bin/python3" ]; then
    echo "No se encontro el entorno virtual (./venv). Corre ./setup.sh primero."
    read -n 1 -s -r -p "Presiona una tecla para cerrar..."
    exit 1
fi

if ./venv/bin/python3 scripts/gui_gtk.py; then
    exit 0
else
    status=$?
fi

if [ $status -ne 0 ]; then
    echo ""
    echo "Aurora se cerro con un error (codigo $status)."
    read -n 1 -s -r -p "Presiona una tecla para cerrar..."
fi
