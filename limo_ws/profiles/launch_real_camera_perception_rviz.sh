#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 4 ]; then
  echo "Usage: $0 <tag_size> [video_device] [calibration_yaml] [no_rviz]"
  echo "  tag_size can be meters (0.16) or centimeters (16)"
  echo "  video_device defaults to /dev/video0"
  echo "  calibration_yaml defaults to workspace config if omitted"
  echo "  no_rviz: true|false (default: false)"
  exit 1
fi

TAG_SIZE="$1"
VIDEO_DEVICE="${2:-/dev/video0}"
CALIBRATION_YAML="${3:-}"
NO_RVIZ="${4:-false}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
CALIB_DEFAULT_SRC="$WS_ROOT/src/limo_apriltag_tools/config/webcam_calibration_robot.yaml"
CALIB_DEFAULT_INSTALL="$WS_ROOT/install/limo_apriltag_tools/share/limo_apriltag_tools/config/webcam_calibration_robot.yaml"

if [ -z "$CALIBRATION_YAML" ]; then
  if [ -f "$CALIB_DEFAULT_SRC" ]; then
    CALIBRATION_YAML="$CALIB_DEFAULT_SRC"
  elif [ -f "$CALIB_DEFAULT_INSTALL" ]; then
    CALIBRATION_YAML="$CALIB_DEFAULT_INSTALL"
  else
    echo "Error: calibration file not found."
    echo "Expected one of:"
    echo "  - $CALIB_DEFAULT_SRC"
    echo "  - $CALIB_DEFAULT_INSTALL"
    echo "Or pass a custom file as argument 3."
    exit 1
  fi
fi

if [ ! -f "$CALIBRATION_YAML" ]; then
  echo "Error: calibration file not found: $CALIBRATION_YAML"
  exit 1
fi

# Camera defaults tuned for higher FPS with lower CPU overhead.
CAM_WIDTH="${CAM_WIDTH:-640}"
CAM_HEIGHT="${CAM_HEIGHT:-480}"
CAM_FPS="${CAM_FPS:-30}"
CAM_PIXEL_FORMAT="${CAM_PIXEL_FORMAT:-MJPG}"
CAM_OUTPUT_ENCODING="${CAM_OUTPUT_ENCODING:-rgb8}"
CAM_CAMERA_NAME="${CAM_CAMERA_NAME:-dabai_dc1}"

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

RVIZ_CFG_SRC="$WS_ROOT/src/limo_apriltag_tools/rviz/real_perception.rviz"
RVIZ_CFG_INSTALL="$WS_ROOT/install/limo_apriltag_tools/share/limo_apriltag_tools/rviz/real_perception.rviz"
if [ -f "$RVIZ_CFG_SRC" ]; then
  RVIZ_CFG="$RVIZ_CFG_SRC"
else
  RVIZ_CFG="$RVIZ_CFG_INSTALL"
fi

echo "[1/3] Starting v4l2 camera on ${VIDEO_DEVICE}..."
echo "[config] camera calibration: ${CALIBRATION_YAML}"
echo "[config] camera format: ${CAM_WIDTH}x${CAM_HEIGHT} ${CAM_PIXEL_FORMAT} @ ${CAM_FPS} fps (${CAM_OUTPUT_ENCODING})"
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="${VIDEO_DEVICE}" \
  -p camera_name:="${CAM_CAMERA_NAME}" \
  -p camera_info_url:="file://${CALIBRATION_YAML}" \
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

echo "[2/3] Starting perception pipeline..."
START_CAMERA=false bash "$SCRIPT_DIR/launch_real_perception.sh" "$TAG_SIZE" "$CALIBRATION_YAML" "$VIDEO_DEVICE" &
PER_PID=$!

sleep 2

if [ "$NO_RVIZ" = "true" ]; then
  echo "[3/3] RViz disabled (no_rviz=true)."
  wait "$PER_PID"
else
  echo "[3/3] Starting RViz..."
  rviz2 -d "$RVIZ_CFG"
fi
