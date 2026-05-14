#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-172.16.64.161,172.16.64.159}"

MAIN_SERIAL="${MAIN_SERIAL:-139522076807}"
LEFT_WRIST_SERIAL="${LEFT_WRIST_SERIAL:-116622072176}"
RIGHT_WRIST_SERIAL="${RIGHT_WRIST_SERIAL:-134322071792}"

MAIN_PORT="${MAIN_PORT:-5020}"
LEFT_WRIST_PORT="${LEFT_WRIST_PORT:-5021}"
RIGHT_WRIST_PORT="${RIGHT_WRIST_PORT:-5022}"

WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
JPEG_QUALITY="${JPEG_QUALITY:-60}"
BACKEND="${BACKEND:-realsense}"
COLOR_AUTO_EXPOSURE="${COLOR_AUTO_EXPOSURE:-false}"
COLOR_EXPOSURE="${COLOR_EXPOSURE:-3000}"
COLOR_GAIN="${COLOR_GAIN:-16}"

PIDS=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --host)
      HOST="$2"
      shift 2
      ;;
    --width)
      WIDTH="$2"
      shift 2
      ;;
    --height)
      HEIGHT="$2"
      shift 2
      ;;
    --fps)
      FPS="$2"
      shift 2
      ;;
    --jpeg-quality)
      JPEG_QUALITY="$2"
      shift 2
      ;;
    --color-auto-exposure)
      COLOR_AUTO_EXPOSURE="true"
      shift
      ;;
    --no-color-auto-exposure)
      COLOR_AUTO_EXPOSURE="false"
      shift
      ;;
    --color-exposure)
      COLOR_EXPOSURE="$2"
      shift 2
      ;;
    --color-gain)
      COLOR_GAIN="$2"
      shift 2
      ;;
    *)
      HOST="$1"
      shift
      ;;
  esac
done

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_camera() {
  local name="$1"
  local serial="$2"
  local port="$3"
  echo "Starting ${name}: serial=${serial}, host=${HOST}, port=${port}"
  python3 send_realsense_udp.py \
    --host "$HOST" \
    --port "$port" \
    --backend "$BACKEND" \
    --serial "$serial" \
    --stream color \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --fps "$FPS" \
    --jpeg-quality "$JPEG_QUALITY" \
    "--$([ "$COLOR_AUTO_EXPOSURE" = "true" ] && echo color-auto-exposure || echo no-color-auto-exposure)" \
    --color-exposure "$COLOR_EXPOSURE" \
    --color-gain "$COLOR_GAIN" &
  PIDS+=("$!")
}

start_camera "main" "$MAIN_SERIAL" "$MAIN_PORT"
start_camera "left_wrist" "$LEFT_WRIST_SERIAL" "$LEFT_WRIST_PORT"
start_camera "right_wrist" "$RIGHT_WRIST_SERIAL" "$RIGHT_WRIST_PORT"

echo "All RealSense UDP senders started. Press Ctrl+C to stop."
wait
