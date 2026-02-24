#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <map_yaml_path>"
  echo "Example:"
  echo "  bash $0 outputs/walls_tags_run_01_outputs/tag_map_walls.yaml"
  exit 1
fi

MAP_YAML="$1"

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

if [ ! -f "$MAP_YAML" ]; then
  echo "[error] map_yaml not found: $MAP_YAML"
  exit 1
fi

ros2 launch control_limo sim_tags_localization.launch.py \
  map_yaml:="$MAP_YAML"
