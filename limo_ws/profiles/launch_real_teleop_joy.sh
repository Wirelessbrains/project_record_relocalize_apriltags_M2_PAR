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

# Defaults aligned with the simulation teleop profile, but for the real robot.
AXIS_LINEAR=1
AXIS_ANGULAR=0
ENABLE_BUTTON=7
SCALE_LINEAR=0.8
SCALE_ANGULAR=1.3
JOY_DEADZONE=0.02
JOY_AUTOREPEAT=50.0
JOY_COALESCE=0.0
JOY_DEVICE_ID=0
REQUIRE_ENABLE=false
CMD_TOPIC="/cmd_vel"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --axis-linear) AXIS_LINEAR="$2"; shift 2 ;;
    --axis-angular) AXIS_ANGULAR="$2"; shift 2 ;;
    --enable-button) ENABLE_BUTTON="$2"; shift 2 ;;
    --scale-linear) SCALE_LINEAR="$2"; shift 2 ;;
    --scale-angular) SCALE_ANGULAR="$2"; shift 2 ;;
    --joy-deadzone) JOY_DEADZONE="$2"; shift 2 ;;
    --joy-autorepeat) JOY_AUTOREPEAT="$2"; shift 2 ;;
    --joy-coalesce) JOY_COALESCE="$2"; shift 2 ;;
    --joy-device-id) JOY_DEVICE_ID="$2"; shift 2 ;;
    --require-enable) REQUIRE_ENABLE=true; shift 1 ;;
    --cmd-topic) CMD_TOPIC="$2"; shift 2 ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--axis-linear N] [--axis-angular N] [--enable-button N] [--scale-linear V] [--scale-angular V] [--joy-device-id N] [--require-enable] [--cmd-topic /cmd_vel]"
      exit 1
      ;;
  esac
done

echo "[1/2] Starting joy_node..."
ros2 run joy joy_node --ros-args \
  -p device_id:="${JOY_DEVICE_ID}" \
  -p autorepeat_rate:="${JOY_AUTOREPEAT}" \
  -p coalesce_interval:="${JOY_COALESCE}" \
  -p deadzone:="${JOY_DEADZONE}" &
JOY_PID=$!

cleanup() {
  kill "${JOY_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "[2/2] Starting teleop_twist_joy -> ${CMD_TOPIC} ..."
ros2 run teleop_twist_joy teleop_node --ros-args \
  -r /cmd_vel:="${CMD_TOPIC}" \
  -p enable_button:="${ENABLE_BUTTON}" \
  -p require_enable_button:="${REQUIRE_ENABLE}" \
  -p axis_linear.x:="${AXIS_LINEAR}" \
  -p axis_angular.yaw:="${AXIS_ANGULAR}" \
  -p scale_linear.x:="${SCALE_LINEAR}" \
  -p scale_angular.yaw:="${SCALE_ANGULAR}"
