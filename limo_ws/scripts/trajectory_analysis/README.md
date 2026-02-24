# Offline Trajectory Mapping and Relocalization Analysis

Main scripts:

- `build_tag_map_offline.py`
- `analyze_distance_angle_to_trajectory.py`

## 1) Build map + trajectory from rosbag2

```bash
cd <workspace_root>/limo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py <bag_path> <output_dir>
```

Example:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

Notes:

- `bag_path` must be the rosbag2 folder (contains `metadata.yaml` and `.db3` files).
- Default behavior includes best-lap selection and robust map optimization.

## 2) Compare relocalization run against a reference trajectory

Recommended usage with output directories:

```bash
python3 scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py \
  <reference_output_dir> \
  <query_output_dir> \
  <compare_output_dir> \
  --mode point \
  --viz-mode debug
```

Direct CSV usage:

```bash
python3 scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py \
  <reference_trajetoria_camera.csv> \
  <query_trajetoria_camera.csv> \
  <output_dir> \
  --mode point \
  --viz-mode debug \
  --query-bag <query_bag_db3_or_folder>
```

## 3) Outputs

Typical generated outputs:

- `mapa_tags.csv`
- `tag_map.yaml`
- `trajetoria_camera.csv`
- `distance_angle_metrics.csv`
- `distance_angle_analysis.html`
- `distance_angle_point_plot.pdf`
- `distance_angle_point_plot.svg`
- `point_pose_didactic.csv`

## 4) Alignment and scale safety

The comparison script supports robust alignment with RANSAC and scale checks.

Relevant options:

- `--align-model {similarity,rigid}`
- `--align-ransac-thresh-m`
- `--align-ransac-iters`
- `--align-scale-min`
- `--align-scale-max`
- `--allow-bad-scale`

## 5) Common issue

If you see an error indicating `/camera_info` could not be read from the bag:

- The topic may exist in metadata but have zero recorded messages.
- Re-record the dataset and ensure `/camera_info` is being published and recorded.
