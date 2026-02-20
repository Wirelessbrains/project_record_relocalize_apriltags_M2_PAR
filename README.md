# Record, Map, and Relocalize with AprilTags

![Project Avatar](https://www.generationrobots.com/19724-product_cover/robot-mobile-open-source-limo-compatible-ros1-et-ros2-limo-standard-version.jpg)

Primary track: real LIMO robot pipeline (offline + online relocalization).  
Bonus track: simulation workflow for validation.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

---

## Authors

| Name | Role | Contact |
|------|------|----------|
| **MARTINS DO LAGO REIS João Pedro** | Real Robot Pipeline, Relocalization, Evaluation | joao_pedro.martins_do_lago_reis@etu.uca.fr |
| **DA SILVA RAMOS Yann Kelvem** | Simulation, ROS Architecture, Integration | yann_kelvem.da_silva_ramos@etu.uca.fr |

---

## Repository Structure

```bash
.
├── docs/
├── limo_ws/
│   ├── dataset/
│   ├── outputs/
│   ├── profiles/
│   ├── scripts/
│   └── src/
└── README.md
```

All runtime commands below are executed from `limo_ws/`.

---

## Quick Start

```bash
git clone <your-repo-url>
cd <repo-name>/limo_ws
source /opt/ros/humble/setup.bash
```

Build (real profile):

```bash
bash profiles/build_real.sh
```

Build (simulation profile):

```bash
bash profiles/build_sim.sh
```

---

## Primary Workflow (Real Robot)

1. Start perception/localization:

```bash
bash profiles/launch_real_perception.sh
```

2. Record rosbag:

```bash
source install/setup.bash
ros2 bag record \
  /camera_info \
  /detections \
  /tag_only_base_pose \
  -o dataset/real_run_01
```

3. Build map + trajectory offline:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/real_run_01 \
  outputs/real_run_01_outputs
```

4. Run online relocalization:

```bash
bash profiles/launch_real_online_relocalization.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_base_pose
```

---

## Bonus Workflow (Simulation)

1. Launch walls scenario:

```bash
bash profiles/launch_sim_tags_dataset.sh
```

2. Start joystick teleop:

```bash
bash profiles/launch_teleop_joy_sim.sh
```

3. Record compact dataset:

```bash
bash profiles/record_sim_dataset.sh walls_tags_run_01 minimal
```

4. Build map + trajectory offline:

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

5. Run online relocalization + RViz:

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv \
  /tag_only_pose pose_stamped map xz
```

---

## Project Governance

- Roadmap (Oct 2025 to Feb 2026): `docs/roadmap_2025_2026.md`
- Scope and requirements (cahier des charges style): `docs/project_scope_requirements.md`

---

## Scope Boundary

This repository focuses on AprilTag-based trajectory learning and relocalization (offline + online). It is not a full generic navigation stack for arbitrary environments.
