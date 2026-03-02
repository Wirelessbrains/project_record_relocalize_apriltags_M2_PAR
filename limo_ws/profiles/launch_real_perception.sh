#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 3 ]; then
  echo "Usage: $0 <tag_size> [calibration_yaml] [video_device]"
  echo "  tag_size can be meters (0.16) or centimeters (16)"
  echo "  calibration_yaml defaults to workspace config if omitted"
  echo "  video_device defaults to /dev/video0"
  echo "  set START_CAMERA=false to skip camera startup"
  exit 1
fi

TAG_SIZE_INPUT="${1}"
TAG_SIZE_METERS="$TAG_SIZE_INPUT"

if ! [[ "$TAG_SIZE_INPUT" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Error: tag_size must be numeric (example: 0.16 or 16)"
  exit 1
fi

if awk "BEGIN{exit !($TAG_SIZE_INPUT > 1.0)}"; then
  TAG_SIZE_METERS="$(awk "BEGIN{printf \"%.6f\", $TAG_SIZE_INPUT/100.0}")"
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
CALIB_DEFAULT_SRC="$WS_ROOT/src/limo_apriltag_tools/config/webcam_calibration_robot.yaml"
CALIB_DEFAULT_INSTALL="$WS_ROOT/install/limo_apriltag_tools/share/limo_apriltag_tools/config/webcam_calibration_robot.yaml"
VIDEO_DEVICE="${3:-/dev/video0}"
START_CAMERA="${START_CAMERA:-true}"

# Camera defaults tuned for higher FPS with lower CPU overhead.
CAM_WIDTH="${CAM_WIDTH:-640}"
CAM_HEIGHT="${CAM_HEIGHT:-480}"
CAM_FPS="${CAM_FPS:-30}"
CAM_PIXEL_FORMAT="${CAM_PIXEL_FORMAT:-MJPG}"
CAM_OUTPUT_ENCODING="${CAM_OUTPUT_ENCODING:-rgb8}"
CAM_CAMERA_NAME="${CAM_CAMERA_NAME:-dabai_dc1}"

if [ -n "${2:-}" ]; then
  CALIBRATION_FILE="$2"
elif [ -f "$CALIB_DEFAULT_SRC" ]; then
  CALIBRATION_FILE="$CALIB_DEFAULT_SRC"
elif [ -f "$CALIB_DEFAULT_INSTALL" ]; then
  CALIBRATION_FILE="$CALIB_DEFAULT_INSTALL"
else
  echo "Error: calibration file not found."
  echo "Expected one of:"
  echo "  - $CALIB_DEFAULT_SRC"
  echo "  - $CALIB_DEFAULT_INSTALL"
  echo "Or pass a custom file as second argument."
  exit 1
fi

cd "$WS_ROOT"
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
  source install/setup.bash
else
  echo "Error: workspace is not built. Run 'bash profiles/build_real.sh' first."
  exit 1
fi
set -u

echo "[config] tag_size=${TAG_SIZE_METERS} m"
echo "[config] calibration_file=${CALIBRATION_FILE}"

if [ "$START_CAMERA" = "true" ]; then
  echo "[config] camera_start=true device=${VIDEO_DEVICE}"
  echo "[config] camera format: ${CAM_WIDTH}x${CAM_HEIGHT} ${CAM_PIXEL_FORMAT} @ ${CAM_FPS} fps (${CAM_OUTPUT_ENCODING})"
  ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:="${VIDEO_DEVICE}" \
    -p camera_name:="${CAM_CAMERA_NAME}" \
    -p camera_info_url:="file://${CALIBRATION_FILE}" \
    -p image_size:="[${CAM_WIDTH},${CAM_HEIGHT}]" \
    -p time_per_frame:="[1,${CAM_FPS}]" \
    -p pixel_format:="${CAM_PIXEL_FORMAT}" \
    -p output_encoding:="${CAM_OUTPUT_ENCODING}" \
    -r /camera_info:=/camera_info_raw &
  CAM_PID=$!

  cleanup() {
    kill "${PER_PID:-}" "${CAM_PID:-}" 2>/dev/null || true
  }
  trap cleanup EXIT
  sleep 1
else
  echo "[config] camera_start=false (expecting external /image_raw publisher)"
fi

ros2 launch limo_apriltag_tools apriltag_camera_pipeline.launch.py \
    tag_size:="${TAG_SIZE_METERS}" \
    calibration_file:="${CALIBRATION_FILE}" \
    camera_image_topic:=/image_raw \
    image_topic:=/image_raw \
    camera_info_topic:=/camera_info \
    camera_frame:=camera_optical \
    optical_frame:=camera_optical \
    relay_image:=false &
PER_PID=$!

wait "$PER_PID"
