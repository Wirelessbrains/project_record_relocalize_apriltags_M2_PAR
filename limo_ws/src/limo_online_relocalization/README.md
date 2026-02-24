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
- `/relocalization/nearest_pose` (`geometry_msgs/msg/PoseStamped`)
- `/relocalization/markers` (`visualization_msgs/msg/MarkerArray`)
- `/relocalization/distance_m` (`std_msgs/msg/Float64`)
- `/relocalization/heading_error_deg` (`std_msgs/msg/Float64`)

## Launch

```bash
ros2 launch limo_online_relocalization online_relocalization.launch.py \
  reference_csv:=outputs/<run>/trajetoria_camera.csv \
  pose_topic:=/tag_only_base_pose \
  pose_msg_type:=pose_stamped \
  frame_id:=map \
  auto_align_xyyaw:=false
```

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
- `Grid Plane = YZ`
- map trajectory plane is `XY`
