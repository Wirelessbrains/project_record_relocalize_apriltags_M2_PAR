# limo_apriltag_navigation

![Project Avatar](URL_TO_AVATAR_IMAGE_HERE)

**Wireless Brains - AprilTag-Based Autonomous Navigation for the LIMO Robot**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Abstract

This project presents an autonomous navigation system for the AgileX LIMO robot, developed in ROS 2, which utilizes AprilTags as visual fiducial markers. The system implements a "Teach, Replay & Resume" methodology, enabling the robot to: (1) Learn a route via manual teleoperation; (2) Replay the route autonomously; and (3) Localize itself and resume the route from any point on the path. The entire development process is first validated in a Gazebo simulation environment before deployment on the physical hardware.

---

## Authors (The Team)

* **Person 1:** [Your Name] - (Simulation & ROS Architect) - [your.email@example.com]
* **Person 2:** [Teammate's Name] - (Navigation Logic & State Machine Engineer) - [teammate.email@example.com]

---

## 1. Introduction and Problem Statement

Autonomous navigation in indoor environments presents significant challenges... The goal of this work is to propose a lightweight and robust solution using visual markers...

---

## 2. Objectives and Requirements (Cahier de Charges)

### 2.1. Project Objectives
* **Main Objective:** To develop an autonomous navigation system for the LIMO robot based on AprilTags.
* **Secondary Objectives:**
    * To validate the system in a simulation environment (ROS 2/Gazebo).
    * To create a modular and scalable software architecture in ROS 2.
    * To document the development process for academic and scientific purposes.

### 2.2. Functional Requirements (FRs)
* **FR01:** The system must allow manual control (teleoperation) to record a trajectory.
* **FR02:** The system must detect and identify AprilTags via the robot's camera.
* **FR03:** The system must save the sequence of detected waypoints (AprilTags) during the recording phase.
* **FR04:** The system must be able to autonomously navigate the saved waypoint sequence ("Replay" mode).
* **FR05:** The system must, when started in an unknown location, enter a "Search" mode to find an AprilTag.
* **FR06:** Upon finding a tag, the system must localize itself on the waypoint map and resume the trajectory from the correct point ("Resume" mode).

### 2.3. Non-Functional Requirements (NFRs)
* **NFR01:** Platform: ROS 2 Humble.
* **NFR02:** Logic Language: Python.
* **NFR03:** Validation: Gazebo Simulation.
* **NFR04:** Version Control: Git/GitHub.

---

## 3. Methodology

Our system operates as a three-stage state machine:

1.  **Stage 1: Teach (Recording):** The robot is teleoperated. A `teach_node.py` script listens to the `/odom` and `/tag_detections` topics to build an ordered map of waypoints.
2.  **Stage 2: Replay (Navigation):** The robot is started at the beginning of the track. A `replay_node.py` script loads the waypoint map and sends sequential goals to the Nav2 `MapsToPose` action server.
3.  **Stage 3: Resume (Localization & Resumption):** The robot is started at a random location. The main `resume_node.py` (state machine) enters "Search" mode, rotating until a tag is visible. Upon seeing `Tag N`, it calculates its pose, determines the next goal is `Tag N+1`, and transitions to "Replay" mode from that point.

---

## 4. Project Timeline (Gantt Chart)

```mermaid
gantt
    title Project Plan: LIMO AprilTag Navigation
    dateFormat  YYYY-MM-DD
    axisFormat %m-%d
    
    section Phase 0: Setup & Research (1 Week)
    Define Cahier de Charges :done, both, 2024-10-21, 2d
    Setup ROS 2 Workspace   :done, p1, 2024-10-21, 3d
    Setup Git & Repo        :done, p2, 2024-10-21, 1d
    Research (AprilTag ROS2):done, p1, 2024-10-23, 3d
    Research (Nav2 API)     :done, p2, 2024-10-23, 3d

    section Phase 1: Simulation & "Teach" (2 Weeks)
    Create Gazebo World (Track) :p1, 2024-10-28, 5d
    Spawn LIMO & AprilTags    :p1, 2024-11-04, 3d
    Teleop & Recording Script :p2, 2024-11-04, 5d
    Test Tag Detection        :both, 2024-11-07, 2d
    Waypoint Extraction Script :p2, 2024-11-11, 3d

    section Phase 2: Navigation "Replay" (1 Week)
    Configure Nav2 in Gazebo  :p1, 2024-11-11, 4d
    Waypoint Nav Script (Action Client) :p2, 2024-11-13, 5d
    Integration Test (Phase 2) :both, 2024-11-18, 2d

    section Phase 3: Localization "Resume" (2 Weeks)
    State Machine Script (Search Mode) :p2, 2024-11-20, 4d
    Localization Logic (TF2)  :p2, 2024-11-25, 4d
    Modify Nav2 for "Resume"  :p1, 2024-11-25, 3d
    Integration Test (Phase 3) :both, 2024-12-02, 3d

    section Phase 4: Finalization (1 Week)
    Test on Physical Robot  :both, 2024-12-05, 3d
    Write Final Paper/Report :both, 2024-12-05, 7d
    Record Demo Video       :p1, 2024-12-10, 2d
    Code Cleanup & README   :p2, 2024-12-10, 3d
    Final Presentation      :active, both, 2024-12-16, 1d
