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

## Authors

| Name | Role | Contact |
|------|------|----------|
| **MARTINS DO LAGO REIS João Pedro** | Autonomous Navigation and Visual Relocalization | joao_pedro.martins_do_lago_reis@etu.uca.fr |
| **DA SILVA RAMOS Yann Kelvem** |  Simulation & ROS Architect | yann_kelvem.da_silva_ramos@etu.uca.fr |


---

## Project Documentation

All detailed project documentation has been moved to the `/docs` folder.

| Document | Description |
| :--- | :--- |
| **[Project Specifications](./docs/project_specifications.md)** | The project specifications: objectives and functional/non-functional requirements. |
| **[System Architecture](./docs/architecture.md)** | The core methodology (Teach, Replay, Resume) and software architecture (ROS Nodes). |
| **[Project Timeline](./docs/gantt.md)** | The gantt chart detailing the project plan and task division. |
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
├── docs
│   ├── architecture.md
│   ├── experiments.md
│   ├── gantt.md
│   ├── project_specifications.md
│   ├── references.bib
│   └── system_architecture.png
├── LICENSE
├── limo_ws
│   ├── docs
│   └── src
│       ├── limo_apriltag_tools
│       │   ├── config
│       │   │   ├── apriltag_params.yaml
│       │   │   └── webcam_calibration.yaml
│       │   ├── launch
│       │   │   └── apriltag_webcam_full.launch.py
│       │   ├── limo_apriltag_tools
│       │   │   ├── camera_info_publisher.py
│       │   │   ├── __init__.py
│       │   │   └── __pycache__
│       │   │       ├── camera_info_publisher.cpython-310.pyc
│       │   │       └── __init__.cpython-310.pyc
│       │   ├── package.xml
│       │   ├── README.md
│       │   ├── resource
│       │   │   └── limo_apriltag_tools
│       │   ├── scripts
│       │   │   └── camera_info_publisher_node
│       │   └── setup.py
│       ├── limo_mapping
│       │   ├── config
│       │   ├── launch
│       │   └── src
│       ├── limo_relocalization
│       │   ├── launch
│       │   └── src
│       ├── limo_route_follow
│       │   ├── config
│       │   ├── launch
│       │   └── src
│       └── limo_simulation
│           ├── rviz
│           └── worlds
└── README.md

