#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$WS_ROOT"
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install \
  --packages-select control_limo limo_apriltag_tools limo_online_relocalization

set +u
source install/setup.bash
set -u
echo "REAL build completed."
