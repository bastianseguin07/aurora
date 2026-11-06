#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFERENCE_DIR="${1:-$PROJECT_DIR/../unitree-reference}"
if [[ ! -d "$REFERENCE_DIR/.git" ]]; then
    git clone --depth 1 --filter=blob:none --sparse https://github.com/iNakzu/Proyecto-24CVEK-264744.git "$REFERENCE_DIR"
    git -C "$REFERENCE_DIR" sparse-checkout set py unitree_lidar_sdk unitree_lidar_ros unitree_lidar_ros2
fi
SDK_DIR="$REFERENCE_DIR/unitree_lidar_sdk"
case "$(uname -m)" in
    x86_64|amd64) SDK_ARCH=x86_64 ;;
    aarch64|arm64) SDK_ARCH=aarch64 ;;
    *) echo "Arquitectura no soportada" >&2; exit 1 ;;
esac
command -v g++ >/dev/null || { echo "Instala g++ (build-essential) para compilar." >&2; exit 1; }
mkdir -p "$PROJECT_DIR/build/unitree"
g++ -std=c++17 -O2 -Wall -Wextra -pthread -I"$SDK_DIR/include" \
    "$PROJECT_DIR/native/unitree_bridge.cpp" "$SDK_DIR/lib/$SDK_ARCH/libunilidar_sdk2.a" \
    -o "$PROJECT_DIR/build/unitree/unitree_bridge"
# Compile the original example too, to check the documented upstream build route.
g++ -std=c++17 -O2 -pthread -I"$SDK_DIR/include" \
    "$SDK_DIR/examples/example_lidar_udp.cpp" "$SDK_DIR/lib/$SDK_ARCH/libunilidar_sdk2.a" \
    -o "$PROJECT_DIR/build/unitree/example_lidar_udp"
cp "$REFERENCE_DIR/LICENSE" "$PROJECT_DIR/build/unitree/LICENSE.unitree"
git -C "$REFERENCE_DIR" rev-parse HEAD > "$PROJECT_DIR/build/unitree/reference_commit.txt"
echo "SDK y puente compilados en $PROJECT_DIR/build/unitree"
