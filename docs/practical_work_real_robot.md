# Practical Work - Real Robot (Primary Track)

This practical work focuses on **AprilTag-based trajectory recording and relocalization** on the physical LIMO robot.

## Objective

Students must:

1. Drive the robot manually.
2. Record a dataset.
3. Build an offline reference trajectory/map.
4. Run online relocalization.
5. Analyze distance and heading error outputs.

## Mandatory Requirement

Manual control is required.  
The operator must have a joystick/teleop controller to drive the robot during data recording.

## Workflow

From `limo_ws/`:

```bash
source /opt/ros/humble/setup.bash
bash profiles/build_real.sh
```

Start perception first:

```bash
bash profiles/launch_real_perception.sh 0.16
```

Joystick teleop (real robot):

```bash
bash profiles/launch_real_teleop_joy.sh
```

Single-command run (perception + teleop):

```bash
bash profiles/launch_real_perception_teleop.sh 0.16
```

Recommended for class/demo use (camera + perception + RViz):

```bash
bash profiles/launch_real_camera_perception_rviz.sh 0.16
```

Camera calibration YAML must be available at:

```bash
src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

Or pass a custom file explicitly:

```bash
bash profiles/launch_real_perception.sh 0.16 src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

If tag size differs from 16 cm, pass your value (meters or cm):

```bash
bash profiles/launch_real_perception.sh 0.20
# or
bash profiles/launch_real_perception.sh 20
```

Start localization only when needed:

```bash
bash profiles/launch_real_localization.sh 0.16 outputs/real_run_01_outputs/tag_map.yaml
```

Record a run while manually driving:

```bash
source install/setup.bash
ros2 bag record \
  /camera_info \
  /detections \
  -o dataset/real_run_01
```

Optional: include `/tag_only_base_pose` if you need map-based pose in the recording.

Build offline reference:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/real_run_01 \
  outputs/real_run_01_outputs
```

Run online relocalization:

```bash
bash profiles/launch_real_online_relocalization.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_pose
```

## Expected Outputs

- `outputs/real_run_01_outputs/trajetoria_camera.csv`
- `outputs/real_run_01_outputs/tag_map.yaml`
- `/relocalization/distance_m`
- `/relocalization/heading_error_deg`
- RViz markers/path topics from relocalization

## Scope Reminder

This practical work is about relocalization quality and trajectory deviation metrics.  
It is **not** a full navigation stack assignment.
