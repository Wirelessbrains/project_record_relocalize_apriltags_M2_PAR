#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <reference_csv_path>"
  echo "Example: bash $0 outputs/walls_tags_run_01_outputs/trajetoria_camera.csv"
  exit 1
fi

REFERENCE_CSV="$1"

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

if [ ! -f "$REFERENCE_CSV" ]; then
  echo "[error] Reference CSV not found: $REFERENCE_CSV"
  exit 1
fi

RVIZ_CFG="$WS_ROOT/src/limo_online_relocalization/rviz/online_relocalization_map.rviz"

echo "[1/2] Starting online relocalization (map/tag_only_base_pose)..."
ros2 launch limo_online_relocalization online_relocalization.launch.py \
  reference_csv:="$REFERENCE_CSV" \
  frame_id:=map \
  pose_topic:=/tag_only_base_pose \
  pose_msg_type:=pose_stamped \
  distance_mode:=2d \
  publish_rate:=12.0 \
  path_downsample_step:=3 \
  auto_align_xyyaw:=false \
  trajectory_plane:=xy \
  reference_axis_mode:=identity &
REL_PID=$!

cleanup() {
  kill "${REL_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[2/2] Starting RViz (Fixed Frame = map)..."
rviz2 -d "$RVIZ_CFG"
