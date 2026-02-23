# limo_apriltag_tools

ROS 2 package for AprilTag perception, camera-info handling, overlay visualization, and offline mapping/relocalization wrappers.

## Nodes

- `camera_info_publisher_node`: Publishes `CameraInfo` from YAML calibration.
- `tag_overlay_node`: Draws detected tags over image output.
- `ippe_localizer`: Computes pose from tag detections and static tag map.
- `static_map_publisher`: Publishes static TF transforms for tags.

## Launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch limo_apriltag_tools apriltag_camera_pipeline.launch.py
```

## Offline tools

- `build_tag_map_offline.py`: Generates reference maps from bags.
- `analyze_distance_angle_to_trajectory.py`: Compares runs against references.

## Config

- `config/apriltag_params.yaml`
- `config/tag_map_parking.yaml`
