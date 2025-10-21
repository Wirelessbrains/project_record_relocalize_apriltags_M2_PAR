# limo_apriltag_navigation

![Project Avatar](https://www.generationrobots.com/19724-product_cover/robot-mobile-open-source-limo-compatible-ros1-et-ros2-limo-standard-version.jpg)

**Wireless Brains - AprilTag-Based Autonomous Navigation for the LIMO Robot**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Abstract

This project presents an autonomous navigation system for the AgileX LIMO robot, developed in ROS 2, which utilizes AprilTags as visual fiducial markers. The system implements a "Teach, Replay & Resume" methodology, enabling the robot to: (1) Learn a route via manual teleoperation; (2) Replay the route autonomously; and (3) Localize itself and resume the route from any point on the path. The entire development process is first validated in a Gazebo simulation environment before deployment on the physical hardware.

---

## Authors

* **Person 1:** [MARTINS DO LAGO REIS João Pedro] - (Simulation & ROS Architect) - [joao_pedro.martins_do_lago_reis@etu.uca.fr]
* **Person 2:** [Teammate's Name] - (Navigation Logic & State Machine Engineer) - [teammate.email@example.com]

---

---

## Project Documentation

All detailed project documentation has been moved to the `/docs` folder.

| Document | Description |
| :--- | :--- |
| **[Project Specifications](./docs/cahier_des_charges.md)** | The "Cahier de Charges": objectives and functional/non-functional requirements. |
| **[System Architecture](./docs/architecture.md)** | The core methodology (Teach, Replay, Resume) and software architecture (ROS Nodes). |
| **[Project Timeline](./docs/gantt.png)** | The Gantt chart detailing the project plan and task division. |
| **[Experiments & Results](./docs/experiments.md)** | Demonstration GIFs, videos, and validation of the system. |
| **[References](./docs/references.bib)** | All scientific and technical references in BibTeX format. |

---

## Code Structure

The repository is organized as follows:


