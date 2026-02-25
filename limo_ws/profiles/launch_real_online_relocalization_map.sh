#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 4 ]; then
  echo "Usage: $0 <reference_csv_path> [pose_topic] [pose_msg_type] [mode]"
  echo "  mode: full | progress (default: full)"
  exit 1
fi

REFERENCE_CSV="$1"
POSE_TOPIC="${2:-/tag_only_pose}"
POSE_MSG_TYPE="${3:-auto}"
MODE="${4:-full}"

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

if [ ! -f "$REFERENCE_CSV" ]; then
  echo "Error: reference CSV not found: $REFERENCE_CSV"
  exit 1
fi

if [ "$MODE" != "full" ] && [ "$MODE" != "progress" ]; then
  echo "Error: mode must be 'full' or 'progress'."
  exit 1
fi

if [ "$MODE" = "progress" ]; then
  RVIZ_CFG="$WS_ROOT/src/limo_online_relocalization/rviz/online_relocalization_progress_clean.rviz"
else
  RVIZ_CFG="$WS_ROOT/src/limo_online_relocalization/rviz/online_relocalization_map.rviz"
fi

echo "[1/2] Starting online relocalization ($MODE)..."
bash "$SCRIPT_DIR/launch_real_online_relocalization.sh" \
  "$REFERENCE_CSV" \
  "$POSE_TOPIC" \
  "$POSE_MSG_TYPE" \
  "$MODE" &
REL_PID=$!

cleanup() {
  kill "${REL_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[2/2] Starting RViz..."
rviz2 -d "$RVIZ_CFG"
