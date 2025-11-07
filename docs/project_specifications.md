# Project Specifications

## 1. Project Overview
The project aims to implement an **autonomous navigation system** for the **AgileX LIMO robot**, using **AprilTags** for environment referencing and localization.

The development follows a **“Teach, Replay & Resume”** methodology:
1. **Teach:** The robot learns a path through manual teleoperation (joystick).
2. **Replay:** It autonomously reproduces the learned route using recorded tag poses.
3. **Resume:** It identifies the closest tag and continues from the correct point.

---

## 2. Objectives
- Achieve fully autonomous navigation using AprilTags as fiducial markers.
- Validate all stages in **ROS 2 Humble**, within **Gazebo** and **RViz**.
- Build a modular software architecture divided into four ROS2 packages:
  - `limo_mapping`
  - `limo_route_follow`
  - `limo_relocalization`
  - `limo_simulation`

---

## 3. Functional Requirements
| ID | Requirement | Description |
|----|--------------|-------------|
| FR1 | AprilTag Detection | Detect AprilTags at 0.5–3.0 m with ≥ 20 FPS |
| FR2 | Teach Mode | Record AprilTag poses and odometry data during teleoperation |
| FR3 | Replay Mode | Follow the stored route with lateral error ≤ 10 cm |
| FR4 | Resume Mode | Identify the nearest tag and continue the trajectory |
| FR5 | Simulation | Validate all components virtually before physical testing |

---

## 4. Non-Functional Requirements
- **Performance:** Control loop ≥ 30 Hz, latency ≤ 50 ms  
- **Robustness:** Handle tag occlusion and temporary loss  
- **Safety:** Safe manual override via joystick at any moment  
- **Maintainability:** Modular ROS2 nodes, clean topic interfaces  

---

## 5. Constraints
- ROS 2 Humble + Ubuntu 22.04 environment  
- Simulation in Gazebo and visualization in RViz  
- Use of AprilTag family `tag36h11`  
- Camera: ≥ 640×480 @ 30 FPS  

---

## 6. Deliverables
- Complete ROS2 workspace (`limo_ws/`)  
- Documentation (`/docs/`)  
- Simulation worlds and configuration files  
- Final report with performance metrics  

---

## 7. Evaluation Metrics
- Mean pose error (m, °)  
- Lap time difference vs manual run  
- Resume latency after relocalization (s)  
- Number of tags detected per run  
