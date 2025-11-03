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
| **MARTINS DO LAGO REIS João Pedro** | April tag detectation | joao_pedro.martins_do_lago_reis@etu.uca.fr |
| **DA SILVA RAMOS Yann Kelvem** | Navigation Logic and teleoperation | yann_kelvem.da_silva_ramos@etu.uca.fr |


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
├── docs
│   ├── architecture.md
│   ├── experiments.md
│   ├── gantt.md
│   ├── project_specifications.md
│   ├── references.bib
│   ├── Report
│   ├── system_architecture.png
│   └── tasks.md
├── LICENSE
├── limo_ws
│   ├── docs
│   └── src
│       └── limo_apriltag_tools
│           ├── config
│           │   ├── apriltag_params.yaml
│           │   ├── camera_v4l2.yaml
│           │   └── webcam_calibration.yaml
│           ├── launch
│           │   └── apriltag_full_v4l2_yuyv.launch.py
│           ├── limo_apriltag_tools
│           │   ├── __init__.py
│           │   └── yuyv_to_mono_node.py
│           ├── package.xml
│           ├── README.md
│           ├── resource
│           │   └── limo_apriltag_tools
│           ├── setup.cfg
│           └── setup.py
├── limo_WS
│   ├── docs
│   │   └── ros2_ws_backup_simulation
│   │       └── src
│   │           └── limo_ros2
│   └── src
│       ├── limo_joystick
│       │   ├── launch
│       │   │   ├── limo_joystick.launch.py
│       │   │   └── mode_limo_joystick.launch.py
│       │   ├── limo_joystick
│       │   │   ├── __init__.py
│       │   │   └── mode_controller_node.py
│       │   ├── package.xml
│       │   ├── README.md
│       │   ├── resource
│       │   │   └── limo_joystick
│       │   ├── setup.cfg
│       │   ├── setup.py
│       │   └── test
│       │       ├── test_copyright.py
│       │       ├── test_flake8.py
│       │       └── test_pep257.py
│       └── limo_ros2
├── pc_WS
│   └── src
│       ├── limo_joystick_simu
│       │   ├── config
│       │   ├── launch
│       │   │   ├── simu_and_mode_joystick.launch.py
│       │   │   └── simu_and_teleop.launch.py
│       │   ├── limo_joystick_simu
│       │   │   ├── __init__.py
│       │   │   └── mode_controller_node.py
│       │   ├── package.xml
│       │   ├── README.md
│       │   ├── resource
│       │   │   └── limo_joystick_simu
│       │   ├── setup.cfg
│       │   ├── setup.py
│       │   └── test
│       │       ├── test_copyright.py
│       │       ├── test_flake8.py
│       │       └── test_pep257.py
│       └── limo_ros2
│           ├── limo_base
│           │   ├── CMakeLists.txt
│           │   ├── include
│           │   │   └── limo_base
│           │   │       ├── limo_driver.h
│           │   │       ├── limo_protocol.h
│           │   │       └── serial_port.h
│           │   ├── launch
│           │   │   ├── limo_base.launch.py
│           │   │   ├── open_ydlidar_launch.py
│           │   │   └── start_limo.launch.py
│           │   ├── package.xml
│           │   ├── scripts
│           │   │   └── tf_pub.py
│           │   └── src
│           │       ├── limo_base_node.cpp
│           │       ├── limo_driver.cpp
│           │       ├── serial_port.cpp
│           │       └── tf_pub.cpp
│           ├── limo_car
│           │   ├── CMakeLists.txt
│           │   ├── gazebo
│           │   │   ├── ackermann_with_sensor.xacro
│           │   │   ├── ackermann.xacro
│           │   │   └── sensor.xacro
│           │   ├── launch
│           │   │   ├── ackermann_gazebo.launch.py
│           │   │   ├── ackermann.launch.py
│           │   │   └── display_ackermann.launch.py
│           │   ├── meshes
│           │   │   ├── limo_base.dae
│           │   │   ├── limo_base.stl
│           │   │   ├── limo_wheel.dae
│           │   │   └── limo_wheel.stl
│           │   ├── package.xml
│           │   ├── Readme.md
│           │   ├── README.txt
│           │   ├── rviz
│           │   │   ├── gazebo.rviz
│           │   │   └── urdf.rviz
│           │   ├── src
│           │   ├── urdf
│           │   │   ├── limo_ackerman_base.xacro
│           │   │   ├── limo_anteil.xacro
│           │   │   └── limo_steering_hinge.xacro
│           │   └── worlds
│           ├── limo_description
│           │   ├── CMakeLists.txt
│           │   ├── launch
│           │   │   ├── display_models_diff.launch.py
│           │   │   └── gazebo_models_diff.launch.py
│           │   ├── meshes
│           │   │   ├── limo_base.dae
│           │   │   ├── limo_base.stl
│           │   │   ├── limo_wheel.dae
│           │   │   └── limo_wheel.stl
│           │   ├── package.xml
│           │   ├── rviz
│           │   │   ├── model_display.rviz
│           │   │   └── urdf.rviz
│           │   └── urdf
│           │       ├── limo_ackerman.gazebo
│           │       ├── limo_ackerman.xacro
│           │       ├── limo_four_diff_2.gazebo
│           │       ├── limo_four_diff.gazebo
│           │       ├── limo_four_diff.xacro
│           │       ├── limo_gazebo.gazebo
│           │       ├── limo_steering_hinge.xacro
│           │       └── limo_xacro.xacro
│           ├── limo_msgs
│           │   ├── CMakeLists.txt
│           │   ├── msg
│           │   │   └── LimoStatus.msg
│           │   └── package.xml
│           └── Readme.md
├── README.md
├── tutorials
│   ├── cofig_Limo_ROS2_Humble_eviroment.md
│   ├── Exemple_LIMO_gazebo_with_joystick.md
│   └── Real_Limo_gazebo_with_joystick.md
└── venv
    ├── bin
    │   ├── python -> python3
    │   ├── python3 -> /usr/bin/python3
    │   └── python3.10 -> python3
    ├── include
    ├── lib
    │   └── python3.10
    │       └── site-packages
    ├── lib64 -> lib
    └── pyvenv.cfg



