# Record, Map and Relocalize with AprilTags

![Project Avatar](https://www.generationrobots.com/19724-product_cover/robot-mobile-open-source-limo-compatible-ros1-et-ros2-limo-standard-version.jpg)

**AprilTag-based workflows for LIMO: primary real-robot offline relocalization + bonus simulation support**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

---

## Abstract

This workspace is organized with a primary and a bonus track:

1. **Primary (Real robot offline pipeline)**: record data, build trajectory/map from bag files, and compute relocalization metrics (nearest-point distance and heading error).
2. **Bonus (Simulation support)**: validate parking/navigation logic in Gazebo/Ignition.

Current real-robot relocalization in this repository is **offline**.

---

## Project Scope

### 1) Primary: Real Robot Offline Pipeline

Purpose:
- process recorded rosbag data from the real robot;
- build route/map outputs from AprilTag observations;
- evaluate relocalization against a reference route.

Main scripts:
- `scripts/trajectory_analysis/build_tag_map_offline.py`
- `scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py`

Build:

```bash
cd /home/jpdark/Downloads/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select limo_apriltag_tools
source install/setup.bash
```

Generate route/map from bag:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py <bag_path> <output_dir>
```

Analyze relocalization (distance + angle):

```bash
python3 scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py \
  <reference_output_dir> \
  <query_output_dir> \
  <compare_output_dir> \
  --mode point \
  --viz-mode debug
```

Detailed reference:
- `scripts/trajectory_analysis/README.md`

### 2) Bonus: Simulation Pipeline

Purpose:
- run autonomous parking logic in a controlled simulation environment.

Main packages:
- `src/control_limo`
- `src/limo_apriltag_tools`
- `src/gz_apriltag_env`

Quick run:

```bash
cd /home/jpdark/Downloads/robot_ws
bash profiles/build_sim.sh
bash profiles/launch_sim_parking.sh
```

Detailed reference:
- `src/control_limo/README_PARKING.md`

---

## Workspace Overview

```bash
robot_ws/
├── dataset/                          # rosbag captures and datasets
├── outputs/                          # generated maps, trajectories, and comparison outputs
├── profiles/                         # short scripts for build/launch per profile
├── scripts/
│   └── trajectory_analysis/          # offline mapping/relocalization analysis scripts
├── src/
│   ├── control_limo/                 # parking/navigation logic (simulation-focused)
│   ├── limo_apriltag_tools/          # AprilTag tools, localization, offline wrappers
│   └── gz_apriltag_env/              # Gazebo/Ignition worlds and assets
└── README.md
```

---

## Notes

- Real-robot flow in this repository is currently **offline**.
- **Online relocalization** is planned as the next implementation step.
