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

## Repository Layout

```bash
.
├── docs/                        # Roadmap and scope/requirements docs
├── limo_ws/                     # Main ROS 2 workspace
│   ├── dataset/
│   ├── outputs/
│   ├── profiles/
│   ├── scripts/
│   └── src/
└── README.md
```

---

## Where to Start

- Full technical guide: `limo_ws/README.md`
- Profile scripts guide: `limo_ws/profiles/README.md`
- Offline analysis guide: `limo_ws/scripts/trajectory_analysis/README.md`
- Practical work (real robot): `docs/practical_work_real_robot.md`
- Practical work (simulation bonus annex): `docs/practical_work_simulation.md`

---

## Quick Setup

```bash
git clone <your-repo-url>
cd <repo-name>/limo_ws
source /opt/ros/humble/setup.bash
bash profiles/build_real.sh
```

For simulation build:

```bash
bash profiles/build_sim.sh
```

---

## Project Governance

- Roadmap (Oct 2025 to Feb 2026): `docs/roadmap_2025_2026.md`
- Scope and requirements (cahier des charges style): `docs/project_scope_requirements.md`

---

## Scope Boundary

This repository is focused on AprilTag-based trajectory learning and relocalization (offline + online).  
It is not a full generic navigation stack for arbitrary unknown environments.

## Control Package Positioning

`limo_ws/src/control_limo` is provided as an optional didactic support package (example control integration in simulation).  
It is not the core objective of the project.

Students who want to study or implement additional control laws can use:

- https://github.com/AtsushiSakai/PythonRobotics/tree/master
