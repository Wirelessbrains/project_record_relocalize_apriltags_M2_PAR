# Workspace Profiles

This folder provides helper scripts for two workflows:

1. **REAL (primary):** physical LIMO robot
2. **SIM (bonus):** Gazebo/Ignition validation

---

## REAL (Primary)

Manual driving is required for dataset recording (joystick/teleop controller).

Build:

```bash
bash profiles/build_real.sh
```

Run perception in a single command (starts camera + AprilTag detector):

```bash
bash profiles/launch_real_perception.sh 0.16
```

If camera is already running externally, disable internal camera startup:

```bash
START_CAMERA=false bash profiles/launch_real_perception.sh 0.16
```

Run joystick teleop for the real robot:

```bash
bash profiles/launch_real_teleop_joy.sh
```

Run perception + joystick teleop together (single command):

```bash
bash profiles/launch_real_perception_teleop.sh 0.16
```

Run camera + perception + RViz together (recommended):

```bash
bash profiles/launch_real_camera_perception_rviz.sh 0.16
```

This command now enforces the project calibration file on the camera driver
(`camera_info_url`) and uses a high-FPS default profile:
- `640x480`
- `YUYV`
- `30 fps`
- `mono8` output (better for AprilTag CPU usage)
- `io_method` uses driver default (set `CAM_IO_METHOD` only if needed)

Custom camera device example:

```bash
bash profiles/launch_real_camera_perception_rviz.sh 0.16 /dev/video1
```

Custom calibration file with one-command startup:

```bash
bash profiles/launch_real_camera_perception_rviz.sh 0.16 /dev/video0 src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

Optional camera tuning (applies to `launch_real_camera_perception_rviz.sh`):

```bash
CAM_WIDTH=640 CAM_HEIGHT=480 CAM_FPS=30 CAM_PIXEL_FORMAT=YUYV CAM_OUTPUT_ENCODING=mono8 \
bash profiles/launch_real_camera_perception_rviz.sh 0.16 /dev/video0
```

Optional override if needed:

```bash
CAM_IO_METHOD=mmap bash profiles/launch_real_camera_perception_rviz.sh 0.16 /dev/video0
```

Default calibration file path used by the script:

```bash
src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

If this file does not exist, it falls back to:

```bash
install/limo_apriltag_tools/share/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

You can also pass a custom calibration file as second argument:

```bash
bash profiles/launch_real_perception.sh 0.16 src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

Set tag size explicitly (meters or centimeters):

```bash
bash profiles/launch_real_perception.sh 0.16
# or
bash profiles/launch_real_perception.sh 16
```

Start localization only when needed (map + IPPE).
The script requires a generated map YAML (it prompts if omitted):

```bash
bash profiles/launch_real_localization.sh 0.16 outputs/real_run_01_outputs/tag_map.yaml
```

Run minimal navigation:

```bash
bash profiles/launch_real_nav_minimal.sh
```

Run online relocalization:

```bash
bash profiles/launch_real_online_relocalization.sh <reference_csv_path> [pose_topic] [pose_msg_type] [mode]
```

Example:

```bash
bash profiles/launch_real_online_relocalization.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_pose
```

If `pose_msg_type` is omitted, the script auto-detects the topic type.
`mode` can be:
- `full` (default): reference path + nearest point + distance + heading + markers
- `progress`: reference path + current pose only (clean step-by-step visualization)

Run online relocalization + RViz together (recommended for rosbag validation):

```bash
bash profiles/launch_real_online_relocalization_map.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_pose pose_stamped full
```

Rosbag replay with map/tags visible (no topic conflict):

1. Start map + tag TFs from generated YAML:

```bash
bash profiles/launch_real_localization.sh 0.20 outputs/<run>_outputs/tag_map.yaml
```

2. Start online relocalization reading a dedicated replay topic:

```bash
bash profiles/launch_real_online_relocalization_map.sh \
  outputs/<run>_outputs/trajetoria_camera.csv \
  /tag_only_pose_bag pose_stamped full
```

3. Replay bag remapping pose topic:

```bash
ros2 bag play <bag_path> --clock \
  --remap /tag_only_pose:=/tag_only_pose_bag
```

This avoids two publishers on `/tag_only_pose` (`localization` + `bag`) and keeps distance/heading consistent.

---

## SIM (Bonus)

Manual driving is required in simulation as well to generate the teach trajectory.

Build:

```bash
bash profiles/build_sim.sh
```

### Parking demo

```bash
bash profiles/launch_sim_parking.sh
```

### Dataset + online relocalization demo (walls scenario)

1) Launch scenario:

```bash
bash profiles/launch_sim_tags_dataset.sh 7
```

2) Start joystick teleop:

```bash
bash profiles/launch_teleop_joy_sim.sh
```

3) Record bag:

```bash
bash profiles/record_sim_dataset.sh walls_tags_run_01 minimal
```

4) Build offline outputs:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

5) Launch simulation with selected YAML:

```bash
bash profiles/launch_sim_tags_online_detection.sh \
  outputs/walls_tags_run_01_outputs/tag_map.yaml
```

6) Start online relocalization + RViz:

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera.csv \
  /tag_only_pose pose_stamped map xy
```

7) Keep teleop running while testing relocalization:

```bash
bash profiles/launch_teleop_joy_sim.sh
```

---

## Notes

- Use `minimal` recording mode to keep bag size small.
- `launch_sim_online_relocalization.sh` default profile:
  - `pose_topic=/tag_only_pose`
  - `pose_msg_type=pose_stamped`
  - `frame_id=map`
  - `trajectory_plane=xy`
- RViz simulation convention for online relocalization:
  - `Fixed Frame = map`
  - `Grid Plane = XZ`
  - map trajectory lives in `XY`
- `src/control_limo` is an optional support package for control demos; it is not the project core.
- For additional control-law study:
  - https://github.com/AtsushiSakai/PythonRobotics/tree/master
