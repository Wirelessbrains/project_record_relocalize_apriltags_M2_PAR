#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 4 ]; then
  echo "Usage: $0 <tag_size> [video_device] [calibration_yaml] [no_rviz]"
  echo "  tag_size can be meters (0.16) or centimeters (16)"
  echo "  video_device defaults to /dev/video0"
  echo "  no_rviz: true|false (default: false)"
  exit 1
fi

TAG_SIZE="$1"
VIDEO_DEVICE="${2:-/dev/video0}"
CALIBRATION_YAML="${3:-}"
NO_RVIZ="${4:-false}"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
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
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="${VIDEO_DEVICE}" \
  -r /camera_info:=/camera_info_raw &
CAM_PID=$!

cleanup() {
  kill "${PER_PID:-}" "${CAM_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 1

echo "[2/3] Starting perception pipeline..."
if [ -n "$CALIBRATION_YAML" ]; then
  bash "$SCRIPT_DIR/launch_real_perception.sh" "$TAG_SIZE" "$CALIBRATION_YAML" &
else
  bash "$SCRIPT_DIR/launch_real_perception.sh" "$TAG_SIZE" &
fi
PER_PID=$!

sleep 2

if [ "$NO_RVIZ" = "true" ]; then
  echo "[3/3] RViz disabled (no_rviz=true)."
  wait "$PER_PID"
else
  echo "[3/3] Starting RViz..."
  rviz2 -d "$RVIZ_CFG"
fi
