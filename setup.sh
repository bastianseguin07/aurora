#!/usr/bin/env bash
# Crea el entorno virtual e instala las dependencias del proyecto en Linux
# (probado para Ubuntu 20.04). Uso, parado en la carpeta Aurora:
#   ./setup.sh

set -e
cd "$(dirname "$0")"

if ! dpkg -s python3-venv >/dev/null 2>&1 || ! dpkg -s python3-tk >/dev/null 2>&1 \
   || ! dpkg -s libgl1-mesa-glx >/dev/null 2>&1 || ! dpkg -s libgomp1 >/dev/null 2>&1; then
    echo "Instalando dependencias del sistema (python3-venv, python3-tk, libgl1-mesa-glx, libgomp1)..."
    echo "Esto requiere sudo."
    sudo apt-get update
    sudo apt-get install -y python3-venv python3-tk libgl1-mesa-glx libgomp1
fi

if [ ! -f "venv/bin/python3" ]; then
    echo "Creando entorno virtual (venv)..."
    python3 -m venv venv
else
    echo "El entorno virtual ya existe, se reutiliza."
fi

echo "Instalando dependencias de Python (puede tardar unos minutos por Open3D)..."
./venv/bin/python3 -m pip install --upgrade pip
./venv/bin/python3 -m pip install -r requirements.txt

echo ""
echo "Listo. Para abrir la interfaz grafica:"
echo "  ./venv/bin/python3 scripts/gui.py"
