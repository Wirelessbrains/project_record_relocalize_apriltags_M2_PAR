# LIMO Workspace Technical Guide

This document is the **workspace-level operational guide**.

It explains:
- what each part of `limo_ws` is used for,
- how to run the workflows,
- what outputs are generated,
- how to interpret those outputs,
- what the current limitations are,
- what students should observe and deliver in practical sessions.

---

## 1) Workspace Purpose

`limo_ws` implements an AprilTag-based trajectory learning and relocalization workflow for a mobile robot.

Main objective:
- measure robot deviation from a learned trajectory using:
  - nearest-point distance,
  - heading error.

The workspace supports:
- **Offline processing** (from rosbag datasets),
- **Online relocalization** (live topic-based estimation),
- **Simulation validation** (bonus track).

---

## 2) Folder Map

```bash
limo_ws/
├── dataset/                    # Recorded bags (not intended for large files in Git)
├── outputs/                    # Generated artifacts (maps, trajectories, analysis)
├── profiles/                   # Short build/run scripts
├── scripts/
│   ├── trajectory_analysis/    # Offline mapping + comparison
│   └── manual_control/         # Utility scripts for teleop/control shaping
├── src/
│   ├── control_limo/           # Simulation parking + control example
│   ├── gz_apriltag_env/        # Gazebo worlds and models
│   ├── limo_apriltag_tools/    # Detection/localization/perception tools
│   └── limo_online_relocalization/
└── README.md
```

---

## 3) Build and Environment

From `limo_ws/`:

```bash
source /opt/ros/humble/setup.bash
```

Build real profile:

```bash
bash profiles/build_real.sh
```

Build simulation profile:

```bash
bash profiles/build_sim.sh
```

---

## 4) Real Robot Workflow (Primary)

### 4.0 Manual control is required

The dataset recording phase requires manual driving (joystick/teleop controller).

### 4.1 Run perception/localization stack

```bash
bash profiles/launch_real_perception.sh
```

Camera calibration file (default expected path):

```bash
src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

If needed, provide a custom calibration YAML as second argument:

```bash
bash profiles/launch_real_perception.sh 0.16 src/limo_apriltag_tools/config/webcam_calibration_robot.yaml
```

If your printed tags have a different size, pass it explicitly:

```bash
bash profiles/launch_real_perception.sh 0.16
# or in centimeters
bash profiles/launch_real_perception.sh 16
```

### 4.2 Record a dataset

```bash
source install/setup.bash
ros2 bag record \
  /camera_info \
  /detections \
  /tag_only_base_pose \
  -o dataset/real_run_01
```

### 4.3 Build offline reference artifacts

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/real_run_01 \
  outputs/real_run_01_outputs
```

### 4.4 Run online relocalization

```bash
bash profiles/launch_real_online_relocalization.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_base_pose
```

---

## 5) Simulation Workflow (Bonus)

### 5.0 Launch parking scenario
```bash
bash profiles/launch_sim_parking.sh
```

### 5.1 Launch multi-tag scenario

```bash
bash profiles/launch_sim_tags_dataset.sh 7
```

### 5.2 Start joystick teleop

```bash
bash profiles/launch_teleop_joy_sim.sh
```

Manual driving is mandatory here as well to generate the teach trajectory used by relocalization.

### 5.3 Record compact dataset

```bash
bash profiles/record_sim_dataset.sh walls_tags_run_01 minimal
```

### 5.4 Build offline artifacts

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

### 5.5 Run online relocalization + RViz

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv \
  /tag_only_pose pose_stamped map xz
```

---

## 6) What Outputs Are Generated

Typical offline outputs in `outputs/<run>_outputs/`:

- `trajetoria_camera.csv`
  - reference trajectory samples.
- `mapa_tags.csv`
  - estimated tag poses.
- `tag_map.yaml`
  - map for localization nodes.
- `diagnostico_erros.csv`
  - diagnostic metrics from reconstruction.
- `mapa_trajetoria_interativo.html`
  - interactive map/trajectory view.

Comparison outputs (from analysis script):

- `distance_angle_metrics.csv`
- `distance_angle_analysis.html`
- `distance_angle_point_plot.pdf`
- `distance_angle_point_plot.svg`
- `point_pose_didactic.csv` (point mode)

Online relocalization topics:

- `/relocalization/reference_path`
- `/relocalization/current_pose`
- `/relocalization/nearest_pose`
- `/relocalization/markers`
- `/relocalization/distance_m`
- `/relocalization/heading_error_deg`

---

## 7) How to Interpret the Main Metrics

- `distance_m`
  - Euclidean distance between current robot pose and nearest reference trajectory point.
  - Lower is better.

- `heading_error_deg`
  - Angular difference between robot heading and local trajectory heading.
  - Closer to 0 deg is better.

Interpretation guideline:
- low distance + low heading error => good relocalization agreement,
- low distance + high heading error => position close but orientation mismatch,
- high distance => robot is far from learned path segment.

---

## 8) Control Law Example Included in Simulation

The workspace includes an example control stack in `src/control_limo`:

- `go_to_pose`: low-level pose controller (goal -> cmd_vel),
- `parking_spot_navigator`: staged parking goal publication,
- `tags_parking_full.launch.py`: complete simulation orchestration.

This is provided as a practical control example linked to localization/perception.
It is not the core objective of this repository.

It is useful for:
- observing closed-loop behavior in simulation,
- understanding interaction between localization and command generation,
- testing parameter sensitivity.

For additional trajectory-control law references, students can use:

- https://github.com/AtsushiSakai/PythonRobotics/tree/master

---

## 9) What Students Should Observe in Practical Work

During runs, students should observe:

1. Detection health
- stable `/detections` stream,
- camera info availability.

2. Pose stream quality
- continuity of `/tag_only_base_pose` (or configured pose topic),
- absence of long gaps and severe jumps.

3. Offline artifacts quality
- coherent trajectory shape,
- plausible tag map geometry.

4. Online metrics behavior
- nearest-point marker progression along path,
- distance/heading error evolution while moving.

Suggested practical deliverables:
- one recorded run,
- one offline output set,
- one online relocalization screenshot/video,
- short discussion of metric trends and limitations.

---

## 10) Current Limitations

This workspace is not a full generic navigation stack.

Known boundaries:
- depends on AprilTag visibility and calibration quality,
- not designed for dynamic obstacle avoidance,
- not a global SLAM solution,
- not intended for arbitrary unknown large-scale worlds,
- performance degrades with poor detections or sparse tag visibility.

---

## 11) Troubleshooting Checklist

If behavior is incorrect:

1. Check topic availability:

```bash
ros2 topic list
```

2. Confirm detections and pose are active:

```bash
ros2 topic echo /detections --once
ros2 topic echo /tag_only_base_pose --once
```

3. Confirm reference CSV path exists before online relocalization.

4. Ensure `frame_id`, `pose_topic`, and `pose_msg_type` are consistent with the current run.

5. For large bags, prefer `minimal` recording mode to reduce processing time and size.

---

## 12) Additional Documents

- `../docs/roadmap_2025_2026.md`
- `../docs/project_scope_requirements.md`
- `../docs/practical_work_real_robot.md`
- `../docs/practical_work_simulation.md`
- `profiles/README.md`
- `scripts/trajectory_analysis/README.md`
