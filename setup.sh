#!/usr/bin/env bash
# Crea el entorno virtual e instala las dependencias de Aurora en Ubuntu.
# Uso, parado en la carpeta Aurora:
#   ./setup.sh

set -e
cd "$(dirname "$0")"

if [ "$(uname -s)" != "Linux" ] || ! command -v apt-get >/dev/null 2>&1; then
    echo "Este proyecto esta preparado para Ubuntu Linux."
    exit 1
fi

REQUIRED_PKGS="python3-venv libgl1-mesa-glx libgomp1 python3-gi gir1.2-gtk-3.0"
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

if [ ! -f "venv/bin/python3" ]; then
    echo "Creando entorno virtual (venv)..."
    # --system-site-packages: los bindings de GTK3 (python3-gi / pygobject3) se
    # instalan a nivel de sistema (apt), no via pip, asi que el venv
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
