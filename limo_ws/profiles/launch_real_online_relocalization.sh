#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ] || [ $# -gt 4 ]; then
  echo "Usage: $0 <reference_csv_path> [pose_topic] [pose_msg_type] [mode]"
  echo "  pose_msg_type: pose_stamped | pose | odom"
  echo "  if omitted, auto-detect from topic type (fallback: pose_stamped)"
  echo "  mode: full | progress (default: full)"
  exit 1
fi

REFERENCE_CSV="$1"
POSE_TOPIC="${2:-/tag_only_pose}"
POSE_MSG_TYPE="${3:-auto}"
RELOCALIZATION_MODE="${4:-full}"

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

if [ "$POSE_MSG_TYPE" = "auto" ]; then
  TOPIC_TYPE=""
  for _ in $(seq 1 30); do
    TOPIC_TYPE="$(ros2 topic type "$POSE_TOPIC" 2>/dev/null || true)"
    [ -n "$TOPIC_TYPE" ] && break
    sleep 0.2
  done
  case "$TOPIC_TYPE" in
    geometry_msgs/msg/PoseStamped) POSE_MSG_TYPE="pose_stamped" ;;
    geometry_msgs/msg/Pose)        POSE_MSG_TYPE="pose" ;;
    nav_msgs/msg/Odometry)         POSE_MSG_TYPE="odom" ;;
    *)
      POSE_MSG_TYPE="pose_stamped"
      echo "[warn] Could not auto-detect type for $POSE_TOPIC. Falling back to pose_stamped."
      echo "[hint] Start rosbag first or pass explicit type as 3rd argument."
      ;;
  esac
fi

echo "[config] reference_csv=$REFERENCE_CSV"
echo "[config] pose_topic=$POSE_TOPIC"
echo "[config] pose_msg_type=$POSE_MSG_TYPE"
echo "[config] mode=$RELOCALIZATION_MODE"

ros2 launch limo_online_relocalization online_relocalization.launch.py \
  reference_csv:="$REFERENCE_CSV" \
  frame_id:=map \
  pose_topic:="$POSE_TOPIC" \
  pose_msg_type:="$POSE_MSG_TYPE" \
  distance_mode:=2d \
  publish_rate:=10.0 \
  relocalization_mode:="$RELOCALIZATION_MODE" \
  auto_align_xyyaw:=false \
  trajectory_plane:=xy \
  reference_axis_mode:=identity
