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
  echo "Error: workspace is not built. Run 'bash profiles/build_real.sh' first."
  exit 1
fi
set -u

# Wrapper around the known-good teleop runner logic:
# base + joy + teleop + cmd_vel_curve (no perception here).
exec python3 "$WS_ROOT/scripts/manual_control/rodar_teleop_camera_ign.py" "$@"
