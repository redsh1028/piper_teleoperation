#!/usr/bin/env bash
set -euo pipefail

LIBREALSENSE_TAG="${LIBREALSENSE_TAG:-v2.57.7}"
LIBREALSENSE_DIR="${LIBREALSENSE_DIR:-$HOME/librealsense}"
BUILD_DIR="$LIBREALSENSE_DIR/build"

echo "[1/7] Installing librealsense build dependencies"
sudo apt-get update
sudo apt-get install -y \
  git wget cmake build-essential pkg-config \
  libssl-dev libusb-1.0-0-dev libudev-dev \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

echo "[2/7] Cloning librealsense ${LIBREALSENSE_TAG} into ${LIBREALSENSE_DIR}"
if [ ! -d "$LIBREALSENSE_DIR/.git" ]; then
  git clone https://github.com/realsenseai/librealsense.git "$LIBREALSENSE_DIR"
fi
cd "$LIBREALSENSE_DIR"
git fetch --tags
git checkout "$LIBREALSENSE_TAG"

echo "[3/7] Installing RealSense udev rules"
sudo ./scripts/setup_udev_rules.sh

echo "[4/7] Patching RealSense kernel modules for Ubuntu LTS/HWE"
sudo ./scripts/patch-realsense-ubuntu-lts-hwe.sh

echo "[5/7] Building librealsense SDK"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_WITH_DDS=OFF \
  -DCHECK_FOR_UPDATES=OFF \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE="$(command -v python3)"
make -j"$(nproc)"

echo "[6/7] Installing librealsense SDK"
sudo make install
sudo ldconfig

echo "[7/7] Verifying installed tools"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  rs-enumerate-devices || true
else
  echo "rs-enumerate-devices was not found in PATH. Try opening a new terminal."
fi

echo
echo "Done. Unplug/replug the RealSense camera, then run:"
echo "  python3 - <<'PY'"
echo "import pyrealsense2 as rs"
echo "ctx = rs.context()"
echo "print('device_count', len(ctx.query_devices()))"
echo "PY"
