#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-172.16.64.161,172.16.64.159}"

MAIN_BACKEND="${MAIN_BACKEND:-zed_open_capture}"
MAIN_V4L2_DEVICE="${MAIN_V4L2_DEVICE:-6}"
MAIN_V4L2_FOURCC="${MAIN_V4L2_FOURCC:-YUYV}"
MAIN_ZED_DEVICE_ID="${MAIN_ZED_DEVICE_ID:--1}"
MAIN_ZED_RESOLUTION="${MAIN_ZED_RESOLUTION:-HD720}"
MAIN_CROP="${MAIN_CROP:-right-half}"
MAIN_WIDTH="${MAIN_WIDTH:-2560}"
MAIN_HEIGHT="${MAIN_HEIGHT:-720}"

LEFT_WRIST_SERIAL="${LEFT_WRIST_SERIAL:-116622072176}"
RIGHT_WRIST_SERIAL="${RIGHT_WRIST_SERIAL:-134322071792}"

MAIN_PORT="${MAIN_PORT:-5020}"
LEFT_WRIST_PORT="${LEFT_WRIST_PORT:-5021}"
RIGHT_WRIST_PORT="${RIGHT_WRIST_PORT:-5022}"

WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
JPEG_QUALITY="${JPEG_QUALITY:-60}"
WRIST_BACKEND="${WRIST_BACKEND:-realsense}"
COLOR_AUTO_EXPOSURE="${COLOR_AUTO_EXPOSURE:-false}"
COLOR_EXPOSURE="${COLOR_EXPOSURE:-100}"
COLOR_GAIN="${COLOR_GAIN:-0}"
COLOR_AUTO_WHITE_BALANCE="${COLOR_AUTO_WHITE_BALANCE:-false}"
COLOR_WHITE_BALANCE="${COLOR_WHITE_BALANCE:-3800}"
LEFT_COLOR_EXPOSURE="${LEFT_COLOR_EXPOSURE:-$COLOR_EXPOSURE}"
LEFT_COLOR_GAIN="${LEFT_COLOR_GAIN:-$COLOR_GAIN}"
LEFT_COLOR_WHITE_BALANCE="${LEFT_COLOR_WHITE_BALANCE:-3500}"
RIGHT_COLOR_EXPOSURE="${RIGHT_COLOR_EXPOSURE:-100}"
RIGHT_COLOR_GAIN="${RIGHT_COLOR_GAIN:-$COLOR_GAIN}"
RIGHT_COLOR_WHITE_BALANCE="${RIGHT_COLOR_WHITE_BALANCE:-3600}"

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
    --main-v4l2-device)
      MAIN_V4L2_DEVICE="$2"
      shift 2
      ;;
    --main-v4l2-fourcc)
      MAIN_V4L2_FOURCC="$2"
      shift 2
      ;;
    --main-width)
      MAIN_WIDTH="$2"
      shift 2
      ;;
    --main-height)
      MAIN_HEIGHT="$2"
      shift 2
      ;;
    --main-crop)
      MAIN_CROP="$2"
      shift 2
      ;;
    --main-zed-device-id)
      MAIN_ZED_DEVICE_ID="$2"
      shift 2
      ;;
    --main-zed-resolution)
      MAIN_ZED_RESOLUTION="$2"
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
    --color-auto-white-balance)
      COLOR_AUTO_WHITE_BALANCE="true"
      shift
      ;;
    --no-color-auto-white-balance)
      COLOR_AUTO_WHITE_BALANCE="false"
      shift
      ;;
    --color-white-balance)
      COLOR_WHITE_BALANCE="$2"
      LEFT_COLOR_WHITE_BALANCE="$2"
      RIGHT_COLOR_WHITE_BALANCE="$2"
      shift 2
      ;;
    --left-color-exposure)
      LEFT_COLOR_EXPOSURE="$2"
      shift 2
      ;;
    --left-color-gain)
      LEFT_COLOR_GAIN="$2"
      shift 2
      ;;
    --left-color-white-balance)
      LEFT_COLOR_WHITE_BALANCE="$2"
      shift 2
      ;;
    --right-color-exposure)
      RIGHT_COLOR_EXPOSURE="$2"
      shift 2
      ;;
    --right-color-gain)
      RIGHT_COLOR_GAIN="$2"
      shift 2
      ;;
    --right-color-white-balance)
      RIGHT_COLOR_WHITE_BALANCE="$2"
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
  local exposure="$4"
  local gain="$5"
  local white_balance="$6"
  echo "Starting ${name}: serial=${serial}, host=${HOST}, port=${port}, exposure=${exposure}, gain=${gain}, white_balance=${white_balance}"
  python3 send_realsense_udp.py \
    --host "$HOST" \
    --port "$port" \
    --backend "$WRIST_BACKEND" \
    --serial "$serial" \
    --stream color \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --fps "$FPS" \
    --jpeg-quality "$JPEG_QUALITY" \
    "--$([ "$COLOR_AUTO_EXPOSURE" = "true" ] && echo color-auto-exposure || echo no-color-auto-exposure)" \
    $([ "$COLOR_AUTO_EXPOSURE" = "true" ] || printf '%s ' --color-exposure "$exposure" --color-gain "$gain") \
    "--$([ "$COLOR_AUTO_WHITE_BALANCE" = "true" ] && echo color-auto-white-balance || echo no-color-auto-white-balance)" \
    $([ "$COLOR_AUTO_WHITE_BALANCE" = "true" ] || printf '%s ' --color-white-balance "$white_balance") &
  PIDS+=("$!")
}

start_main_camera() {
  if [ "$MAIN_BACKEND" = "zed_open_capture" ]; then
    echo "Starting main ZED 2i with zed-open-capture: resolution=${MAIN_ZED_RESOLUTION}, crop=${MAIN_CROP}, host=${HOST}, port=${MAIN_PORT}"
    ./send_zed_open_capture_udp \
      --host "$HOST" \
      --port "$MAIN_PORT" \
      --device-id "$MAIN_ZED_DEVICE_ID" \
      --resolution "$MAIN_ZED_RESOLUTION" \
      --fps "$FPS" \
      --jpeg-quality "$JPEG_QUALITY" \
      --crop "$MAIN_CROP" &
  else
    echo "Starting main ZED 2i with V4L2: v4l2_device=${MAIN_V4L2_DEVICE}, crop=${MAIN_CROP}, host=${HOST}, port=${MAIN_PORT}"
    python3 send_realsense_udp.py \
      --host "$HOST" \
      --port "$MAIN_PORT" \
      --backend "$MAIN_BACKEND" \
      --v4l2-device "$MAIN_V4L2_DEVICE" \
      --v4l2-fourcc "$MAIN_V4L2_FOURCC" \
      --stream color \
      --width "$MAIN_WIDTH" \
      --height "$MAIN_HEIGHT" \
      --fps "$FPS" \
      --jpeg-quality "$JPEG_QUALITY" \
      --crop "$MAIN_CROP" &
  fi
  PIDS+=("$!")
}

start_main_camera
start_camera "left_wrist" "$LEFT_WRIST_SERIAL" "$LEFT_WRIST_PORT" "$LEFT_COLOR_EXPOSURE" "$LEFT_COLOR_GAIN" "$LEFT_COLOR_WHITE_BALANCE"
start_camera "right_wrist" "$RIGHT_WRIST_SERIAL" "$RIGHT_WRIST_PORT" "$RIGHT_COLOR_EXPOSURE" "$RIGHT_COLOR_GAIN" "$RIGHT_COLOR_WHITE_BALANCE"

echo "All camera UDP senders started. Press Ctrl+C to stop."
wait
