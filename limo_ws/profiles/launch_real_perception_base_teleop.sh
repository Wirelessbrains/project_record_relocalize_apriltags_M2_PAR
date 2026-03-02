#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 3 ]; then
  echo "Usage: $0 <tag_size> [calibration_yaml] [video_device]"
  echo "  tag_size can be meters (0.16) or centimeters (16)"
  echo "  calibration_yaml defaults to workspace config if omitted"
  echo "  video_device defaults to /dev/video0"
  echo "  set JOY_DEVICE_ID to choose joystick device (default: 0)"
  exit 1
fi

TAG_SIZE="$1"
CALIBRATION_YAML="${2:-}"
VIDEO_DEVICE="${3:-/dev/video0}"
JOY_DEVICE_ID="${JOY_DEVICE_ID:-0}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cleanup() {
  kill "${PER_PID:-}" "${TEL_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[1/2] Starting real perception pipeline..."
if [ -n "$CALIBRATION_YAML" ]; then
  bash "$SCRIPT_DIR/launch_real_perception.sh" "$TAG_SIZE" "$CALIBRATION_YAML" "$VIDEO_DEVICE" &
else
  bash "$SCRIPT_DIR/launch_real_perception.sh" "$TAG_SIZE" "" "$VIDEO_DEVICE" &
fi
PER_PID=$!

sleep 2

echo "[2/2] Starting real base + teleop stack..."
bash "$SCRIPT_DIR/launch_real_base_teleop.sh" --joy-device-id "$JOY_DEVICE_ID" &
TEL_PID=$!

wait "$PER_PID"

