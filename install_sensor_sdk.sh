#!/usr/bin/env bash
# Descarga y prepara el SDK oficial de Slamtec para el Ubuntu de esta maquina.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
VENV_PYTHON="$PROJECT_DIR/venv/bin/python3"
SDK_DIR="$PROJECT_DIR/aurora_sdk_src"
SDK_URL="https://github.com/Slamtec/py_aurora_remote.git"
SDK_TAG="2.1.1-rtm"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "El SDK del sensor se prepara aqui solo para Linux/Ubuntu." >&2
    exit 1
fi
if [[ ! -x "$VENV_PYTHON" ]]; then
    echo "No existe el entorno virtual de Aurora. Ejecuta primero ./setup.sh." >&2
    exit 1
fi
if ! command -v git >/dev/null 2>&1; then
    echo "Falta git. Instálalo con: sudo apt install git" >&2
    exit 1
fi

case "$(uname -m)" in
    x86_64|amd64) SDK_PLATFORM="linux_x86_64" ;;
    aarch64|arm64) SDK_PLATFORM="linux_aarch64" ;;
    *)
        echo "Arquitectura no soportada por este instalador: $(uname -m)" >&2
        exit 1
        ;;
esac

if [[ -e "$SDK_DIR" ]]; then
    if [[ ! -d "$SDK_DIR/.git" ]]; then
        echo "$SDK_DIR existe pero no es un clon Git; no lo voy a sobrescribir." >&2
        exit 1
    fi
    if [[ -n "$(git -C "$SDK_DIR" status --porcelain)" ]]; then
        echo "El clon del SDK tiene cambios locales. Guárdalos o limpia $SDK_DIR antes de continuar." >&2
        exit 1
    fi
    echo "Actualizando el clon local al tag oficial $SDK_TAG..."
    git -C "$SDK_DIR" fetch --depth 1 origin "refs/tags/$SDK_TAG:refs/tags/$SDK_TAG"
    git -C "$SDK_DIR" checkout --detach "$SDK_TAG"
    git -C "$SDK_DIR" submodule update --init --recursive --depth 1
else
    echo "Descargando el SDK oficial Slamtec ($SDK_TAG)..."
    git clone --depth 1 --branch "$SDK_TAG" --recurse-submodules --shallow-submodules \
        "$SDK_URL" "$SDK_DIR"
fi

echo "Instalando herramientas de empaquetado del SDK..."
"$VENV_PYTHON" -m pip install -r "$SDK_DIR/requirements-dev.txt"

wheel_dir="$SDK_DIR/wheels"
mkdir -p "$wheel_dir"
rm -f "$wheel_dir"/slamtec_aurora_python_sdk_"$SDK_PLATFORM"-*.whl
echo "Construyendo el SDK para $SDK_PLATFORM..."
"$VENV_PYTHON" "$SDK_DIR/tools/build_package.py" --platforms "$SDK_PLATFORM"

shopt -s nullglob
wheels=("$wheel_dir"/slamtec_aurora_python_sdk_"$SDK_PLATFORM"-*.whl)
if [[ ${#wheels[@]} -eq 0 ]]; then
    echo "La compilacion termino sin generar el wheel esperado para $SDK_PLATFORM." >&2
    exit 1
fi

echo "Instalando ${wheels[0]} en el entorno de Aurora..."
"$VENV_PYTHON" -m pip install --force-reinstall "${wheels[0]}"
echo "Verificando el paquete Python y la biblioteca nativa del sensor..."
"$VENV_PYTHON" -c 'from slamtec_aurora_sdk.c_bindings import CBindings; CBindings(); print("SDK de Aurora listo.")'

echo "Instalacion del SDK completada. El clon se guardo en: $SDK_DIR"
