# Practical Work - Real Robot (Primary Track)

This practical work focuses on **AprilTag-based trajectory recording and relocalization** on the physical LIMO robot.

## Objective

Students must:

1. Drive the robot manually (teach run).
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

Start perception/localization:

```bash
bash profiles/launch_real_perception.sh
```

Camera calibration YAML must be available at:

```bash
limo_ws/src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
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

Record a run while manually driving:

```bash
source install/setup.bash
ros2 bag record \
  /camera_info \
  /detections \
  /tag_only_base_pose \
  -o dataset/real_run_01
```

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
  /tag_only_base_pose
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
