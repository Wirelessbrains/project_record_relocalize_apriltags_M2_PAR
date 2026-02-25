#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 3 ]; then
  echo "Usage: $0 <tag_size> [map_yaml] [base_frame]"
  echo "  tag_size can be meters (0.16) or centimeters (16)"
  echo "  map_yaml should be the generated map file (required; prompted if omitted)"
  echo "  base_frame defaults to base_footprint"
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

MAP_YAML="${2:-}"
BASE_FRAME="${3:-base_footprint}"

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

echo "[config] tag_size=${TAG_SIZE_METERS} m"
echo "[config] base_frame=${BASE_FRAME}"

if [ -z "$MAP_YAML" ]; then
  echo ""
  echo "Map YAML is required."
  read -r -p "Enter generated map YAML path: " MAP_YAML
fi

if [ ! -f "$MAP_YAML" ]; then
  echo "Error: map_yaml file not found: $MAP_YAML"
  exit 1
fi

echo "[config] map_yaml=${MAP_YAML}"
ros2 launch limo_apriltag_tools ippe_parking_localization.launch.py \
  tag_size:="${TAG_SIZE_METERS}" \
  map_yaml:="${MAP_YAML}" \
  base_frame:="${BASE_FRAME}"
