#!/usr/bin/env bash
set -euo pipefail

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

RUN_NAME="${1:-sim_tags_$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="dataset/${RUN_NAME}"
MODE="${2:-minimal}"

echo "Recording rosbag to: ${OUT_DIR}"
echo "Mode: ${MODE}"
echo "Press Ctrl+C to stop recording."

if [[ "${MODE}" == "full" ]]; then
  ros2 bag record \
    /model/limo_01/pose \
    /model/limo_01/cmd_vel \
    /rgb_camera \
    /rgb_camera_sync \
    /camera_info \
    /detections \
    /tag_only_base_pose \
    -o "${OUT_DIR}"
else
  # Minimal dataset for mapping/relocalization (small bag size).
  ros2 bag record \
    /model/limo_01/pose \
    /model/limo_01/cmd_vel \
    /camera_info \
    /detections \
    /tag_only_base_pose \
    -o "${OUT_DIR}"
fi
