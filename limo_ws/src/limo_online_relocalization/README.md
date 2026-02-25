# limo_online_relocalization

Online relocalization node that compares live robot pose against a recorded reference trajectory.

## Inputs

- Reference CSV (`trajetoria_camera.csv`) with at least `x,y,z`.
- Live pose topic in one of these message types:
  - `geometry_msgs/msg/PoseStamped`
  - `geometry_msgs/msg/Pose`
  - `nav_msgs/msg/Odometry`

## Output topics

- `/relocalization/reference_path` (`nav_msgs/msg/Path`)
- `/relocalization/current_pose` (`geometry_msgs/msg/PoseStamped`)
- Full mode only:
  - `/relocalization/nearest_pose` (`geometry_msgs/msg/PoseStamped`)
  - `/relocalization/markers` (`visualization_msgs/msg/MarkerArray`)
  - `/relocalization/distance_m` (`std_msgs/msg/Float64`)
  - `/relocalization/heading_error_deg` (`std_msgs/msg/Float64`)

## Launch

```bash
ros2 launch limo_online_relocalization online_relocalization.launch.py \
  reference_csv:=outputs/<run>/trajetoria_camera.csv \
  pose_topic:=/tag_only_pose \
  pose_msg_type:=pose_stamped \
  relocalization_mode:=full \
  frame_id:=map \
  auto_align_xyyaw:=false
```

Modes:
- `full`: full relocalization diagnostics (nearest point, distance, heading, markers).
- `progress`: clean teaching visualization (reference path + current pose only).
- Recommended with `trajetoria_camera.csv`: use camera pose topic `/tag_only_pose`.

## RViz displays

- `Path` on `/relocalization/reference_path`
- `Pose` on `/relocalization/current_pose`
- `Pose` on `/relocalization/nearest_pose`
- `MarkerArray` on `/relocalization/markers`

## Simulation quick run

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera.csv \
  /tag_only_pose pose_stamped map xy
```

RViz simulation convention:
- `Fixed Frame = map`
- `Grid Plane = XZ`
- map trajectory plane is `XY`

For clean progress view in RViz:

```bash
rviz2 -d src/limo_online_relocalization/rviz/online_relocalization_progress_clean.rviz
```

## Rosbag replay with localization running

If localization is running at the same time, avoid publishing replay pose on `/tag_only_pose` directly.
Use a dedicated replay topic to prevent duplicate publishers and biased distance values:

```bash
# terminal 1
bash profiles/launch_real_localization.sh 0.20 outputs/<run>_outputs/tag_map.yaml

# terminal 2
bash profiles/launch_real_online_relocalization_map.sh \
  outputs/<run>_outputs/trajetoria_camera.csv \
  /tag_only_pose_bag pose_stamped full

# terminal 3
ros2 bag play <bag_path> --clock \
  --remap /tag_only_pose:=/tag_only_pose_bag
```
