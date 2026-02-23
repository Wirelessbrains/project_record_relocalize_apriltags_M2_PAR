#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$WS_ROOT"

set +u
source /opt/ros/humble/setup.bash

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: Workspace not built. Run 'colcon build' in: $WS_ROOT"
    exit 1
fi
set -u

echo "Launching Parking Control Scenario (Gare)"
# tags_parking_full.launch.py already includes bridge, perception, and control.
ros2 launch control_limo tags_parking_full.launch.py
