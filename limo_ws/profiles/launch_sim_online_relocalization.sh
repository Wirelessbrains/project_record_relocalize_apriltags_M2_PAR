#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <reference_csv_path> [pose_topic] [pose_msg_type] [frame_id] [trajectory_plane]"
  echo "Example:"
  echo "  bash $0 outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv"
  echo "  bash $0 outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv /tag_only_pose pose_stamped map xz"
  exit 1
fi

REFERENCE_CSV="$1"
POSE_TOPIC="${2:-/tag_only_pose}"
POSE_MSG_TYPE="${3:-pose_stamped}"
FRAME_ID="${4:-map}"
TRAJECTORY_PLANE="${5:-xz}"
REFERENCE_AXIS_MODE_DEFAULT="identity"
AUTO_ALIGN_DEFAULT="false"

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
  echo "Error: workspace is not built. Run 'bash profiles/build_sim.sh' first."
  exit 1
fi
set -u

RVIZ_CFG="$WS_ROOT/src/limo_online_relocalization/rviz/online_relocalization_map.rviz"

echo "[config] pose_topic=$POSE_TOPIC"
echo "[config] pose_msg_type=$POSE_MSG_TYPE"
echo "[config] frame_id=$FRAME_ID"
echo "[config] trajectory_plane=$TRAJECTORY_PLANE"
echo "[config] reference_axis_mode=$REFERENCE_AXIS_MODE_DEFAULT"
echo "[config] auto_align_xyyaw=$AUTO_ALIGN_DEFAULT"

if [ ! -f "$REFERENCE_CSV" ]; then
  ALT_CSV="${REFERENCE_CSV/dataset\//outputs/}"
  if [ -f "$ALT_CSV" ]; then
    echo "[info] Reference CSV not found at '$REFERENCE_CSV'. Using '$ALT_CSV' instead."
    REFERENCE_CSV="$ALT_CSV"
  else
    echo "[error] Reference CSV not found: $REFERENCE_CSV"
    echo "[hint] Example existing file:"
    find "$WS_ROOT/outputs" -type f -name trajetoria_camera.csv 2>/dev/null | head -n 3
    exit 1
  fi
fi

echo "[1/2] Starting online relocalization node..."
ros2 launch limo_online_relocalization online_relocalization.launch.py \
  reference_csv:="$REFERENCE_CSV" \
  frame_id:="$FRAME_ID" \
  pose_topic:="$POSE_TOPIC" \
  pose_msg_type:="$POSE_MSG_TYPE" \
  distance_mode:=2d \
  publish_rate:=12.0 \
  path_downsample_step:=3 \
  auto_align_xyyaw:="$AUTO_ALIGN_DEFAULT" \
  trajectory_plane:="$TRAJECTORY_PLANE" \
  reference_axis_mode:="$REFERENCE_AXIS_MODE_DEFAULT" &
REL_PID=$!

cleanup() {
  kill "${REL_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[2/2] Starting RViz..."
rviz2 -d "$RVIZ_CFG"
