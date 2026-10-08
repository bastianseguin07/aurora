#!/usr/bin/env bash
# Crea el entorno virtual e instala las dependencias del proyecto en Linux o
# macOS (ver README para detalles y limitaciones por sistema operativo).
# Uso, parado en la carpeta Aurora:
#   ./setup.sh

set -e
cd "$(dirname "$0")"

OS_NAME="$(uname -s)"

if [ "$OS_NAME" = "Linux" ]; then
    REQUIRED_PKGS="python3-venv python3-tk libgl1-mesa-glx libgomp1 python3-gi gir1.2-gtk-3.0"
    missing=0
    for pkg in $REQUIRED_PKGS; do
        dpkg -s "$pkg" >/dev/null 2>&1 || missing=1
    done
    if [ "$missing" = "1" ]; then
        echo "Instalando dependencias del sistema ($REQUIRED_PKGS)..."
        echo "Esto requiere sudo."
        sudo apt-get update
        sudo apt-get install -y $REQUIRED_PKGS
    fi
    PYTHON_BIN="python3"

elif [ "$OS_NAME" = "Darwin" ]; then
    if ! command -v brew >/dev/null 2>&1; then
        echo "No se encontro Homebrew. Instalalo primero desde https://brew.sh y volve a correr este script."
        exit 1
    fi
    echo "Instalando dependencias del sistema via Homebrew (gtk+3, pygobject3, python)..."
    brew install python gtk+3 pygobject3
    # Usar el python de Homebrew explicitamente (no el de Xcode/sistema), es el
    # que tiene gtk+3/pygobject3 instalados.
    PYTHON_BIN="$(brew --prefix)/bin/python3"

else
    echo "Sistema operativo no reconocido ($OS_NAME). Este script soporta Linux y macOS."
    echo "Para Windows, usa setup.ps1 (PowerShell) o setup.bat (CMD) en su lugar."
    exit 1
fi

if [ ! -f "venv/bin/python3" ]; then
    echo "Creando entorno virtual (venv)..."
    # --system-site-packages: los bindings de GTK3 (python3-gi / pygobject3) se
    # instalan a nivel de sistema (apt/brew), no via pip, asi que el venv
    # necesita heredarlos. El resto de las dependencias (open3d, numpy, etc.)
    # se instalan aparte dentro del venv normalmente.
    "$PYTHON_BIN" -m venv --system-site-packages venv
else
    echo "El entorno virtual ya existe, se reutiliza."
fi

echo "Instalando dependencias de Python (puede tardar unos minutos por Open3D)..."
./venv/bin/python3 -m pip install --upgrade pip
./venv/bin/python3 -m pip install -r requirements.txt

echo ""
echo "Listo. Para abrir la interfaz grafica (GTK3):"
echo "  ./venv/bin/python3 scripts/gui_gtk.py"
