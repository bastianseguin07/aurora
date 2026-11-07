#!/usr/bin/env bash
# Abre la GUI de Aurora (GTK3). Pensado para lanzarse con doble click desde
# el acceso directo del Escritorio (Aurora.desktop), no solo desde terminal.
set -u
cd "$(dirname "$(readlink -f "$0")")"

if [ ! -x "./venv/bin/python3" ]; then
    message="No se encontro el entorno virtual (./venv). Corre ./setup.sh primero."
    if command -v zenity >/dev/null 2>&1; then
        zenity --error --title="Aurora" --text="$message"
    else
        echo "$message" >&2
        read -r -n 1 -s -p "Presiona una tecla para cerrar..." || true
        echo
    fi
    exit 1
fi

# Fuerza el renderizado con la GPU NVIDIA (PRIME render offload) en vez de la
# Intel integrada: en esta laptop (Intel + NVIDIA en modo "on-demand"), las
# ventanas 3D de Open3D no respondian al scroll/zoom del mouse cuando
# renderizaban con la Intel.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

./venv/bin/python3 scripts/gui_gtk.py "$@"
status=$?

if [ $status -ne 0 ]; then
    message="Aurora se cerro con un error (codigo $status)."
    if command -v zenity >/dev/null 2>&1; then
        zenity --error --title="Aurora" --text="$message"
    else
        echo "$message" >&2
        read -r -n 1 -s -p "Presiona una tecla para cerrar..." || true
        echo
    fi
fi
