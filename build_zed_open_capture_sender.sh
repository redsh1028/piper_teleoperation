#!/usr/bin/env bash
set -euo pipefail

ZED_OPEN_CAPTURE_DIR="${ZED_OPEN_CAPTURE_DIR:-$HOME/zed-open-capture}"

cmake \
  -S "$ZED_OPEN_CAPTURE_DIR" \
  -B "$ZED_OPEN_CAPTURE_DIR/build" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DBUILD_SENSORS=OFF \
  -DBUILD_EXAMPLES=ON

cmake --build "$ZED_OPEN_CAPTURE_DIR/build" -j"$(nproc)"

g++ -std=c++14 -O2 send_zed_open_capture_udp.cpp \
  -o send_zed_open_capture_udp \
  -I"$ZED_OPEN_CAPTURE_DIR/include" \
  -L"$ZED_OPEN_CAPTURE_DIR/build" \
  -Wl,-rpath,"$ZED_OPEN_CAPTURE_DIR/build" \
  -lzed_open_capture \
  $(pkg-config --cflags --libs opencv4) \
  -lusb-1.0
