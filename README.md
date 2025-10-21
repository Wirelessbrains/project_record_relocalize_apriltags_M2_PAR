# limo_apriltag_navigation

![Project Avatar](https://www.generationrobots.com/19724-product_cover/robot-mobile-open-source-limo-compatible-ros1-et-ros2-limo-standard-version.jpg)

**Wireless Brains - AprilTag-Based Autonomous Navigation for the LIMO Robot**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Abstract

This project presents an autonomous navigation system for the AgileX LIMO robot, developed in ROS 2, which utilizes AprilTags as visual fiducial markers. The system implements a "Teach, Replay & Resume" methodology, enabling the robot to: (1) Learn a route via manual teleoperation; (2) Replay the route autonomously; and (3) Localize itself and resume the route from any point on the path. The entire development process is first validated in a Gazebo simulation environment before deployment on the physical hardware.

1. **Teach** – Learn a route via manual teleoperation using a joystick.  
2. **Replay** – Follow the learned route autonomously based on recorded AprilTag positions.  
3. **Resume** – Localize itself from the nearest tag and continue the route from any point on the path.
---

### 👥 Authors

| Name | Role | Contact |
|------|------|----------|
| **MARTINS DO LAGO REIS João Pedro** | Simulation & ROS Architect | joao_pedro.martins_do_lago_reis@etu.uca.fr |
| **DA SILVA RAMOS Yann Kelvem** | Navigation Logic & State Machine Engineer | yann_kelvem.da_silva_ramos@etu.uca.fr |


---

## Project Documentation

All detailed project documentation has been moved to the `/docs` folder.

| Document | Description |
| :--- | :--- |
| **[Project Specifications](./docs/project_specifications.md)** | The project specifications: objectives and functional/non-functional requirements. |
| **[System Architecture](./docs/architecture.md)** | The core methodology (Teach, Replay, Resume) and software architecture (ROS Nodes). |
| **[Project Timeline](./docs/gantt.png)** | The gantt chart detailing the project plan and task division. |
| **[Experiments & Results](./docs/experiments.md)** | Demonstration GIFs, videos, and validation of the system. |
| **[References](./docs/references.bib)** | All scientific and technical references in BibTeX format. |


---

## Workspace Overview

> The workspace follows a **multi-package ROS 2 structure**, enabling modular development by task:
> - **limo_mapping:** teach-run and AprilTag logging  
> - **limo_route_follow:** autonomous path replay  
> - **limo_relocalization:** localization and resume behavior  
> - **limo_simulation:** Gazebo & RViz configuration for testing


## Code Structure

```bash
limo_ws/
├─ README.md
├─ LICENSE
├─ .gitignore
├─ docs/
│  ├─ cahier_des_charges.md
│  ├─ gantt.png
│  ├─ architecture.md
│  ├─ experiments.md
│  ├─ references.bib
│  └─ system_architecture.png
├─ .github/
│  ├─ ISSUE_TEMPLATE/
│  │  └─ task.md
│  ├─ PULL_REQUEST_TEMPLATE.md
│  └─ CODEOWNERS
└─ src/
   ├─ limo_mapping/
   │  ├─ package.xml
   │  ├─ CMakeLists.txt
   │  ├─ launch/mapping.launch.py
   │  ├─ config/apriltag.yaml
   │  └─ src/mapping_node.py
   ├─ limo_route_follow/
   │  ├─ package.xml
   │  ├─ CMakeLists.txt
   │  ├─ launch/route_follow.launch.py
   │  ├─ config/controller.yaml
   │  └─ src/route_follow_node.py
   ├─ limo_relocalization/
   │  ├─ package.xml
   │  ├─ CMakeLists.txt
   │  ├─ launch/relocalization.launch.py
   │  └─ src/relocalization_node.py
   └─ limo_simulation/
      ├─ package.xml
      ├─ CMakeLists.txt
      ├─ rviz/mapping_view.rviz
      └─ worlds/track.world


