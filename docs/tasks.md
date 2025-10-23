# Detailed Breakdown of Micro-Tasks by Module (Subtask)

## ➡️ M1 - Mapping

### 1.1 Environment Setup and Docs (Primary Lead: Yann)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Set up Git repository and folder structure. | **Yann** | :arrows_counterclockwise: Active | 1 day |
| Install ROS/Package dependencies in the development environment. | **Yann** | :arrows_counterclockwise: Active | 2 days |
| Create and configure initial README and Docs template. | **João** | :large_blue_circle: To Do | 2 days |
| Test basic communication between hardware and environment. | **Both** | :arrows_counterclockwise: Active | 2 days |

### 1.2 AprilTag Detection and Calibration (Primary Lead: Yann)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Select and integrate the AprilTag Detection package. | **Yann** | :large_blue_circle: To Do | 3 days |
| Implement the camera reading node and Tag publication. | **Yann** | :large_blue_circle: To Do | 4 days |
| Create intrinsic and extrinsic camera calibration routine. | **João** | :large_blue_circle: To Do | 3 days |
| Validate detection accuracy at different distances/angles. | **Both** | :large_blue_circle: To Do | 2 days |

### 1.3 Teach Run Manual Teleoperation (Primary Lead: João)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Configure and test joystick/keyboard interface for teleoperation. | **João** | :large_blue_circle: To Do | 2 days |
| Implement node for recording odometry topic and commands. | **João** | :large_blue_circle: To Do | 4 days |
| Define and document the standard procedure for "Teach Run." | **Yann** | :large_blue_circle: To Do | 2 days |
| Field test of teleoperation and initial recording. | **Both** | :large_blue_circle: To Do | 2 days |

### 1.4 Dataset Recording and Validation (Primary Lead: João)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Conduct dataset recordings in multiple environments/scenarios. | **João** | :large_blue_circle: To Do | 3 days |
| Develop script for checking dataset consistency. | **Yann** | :large_blue_circle: To Do | 2 days |
| Noise analysis and initial filtering of odometry data. | **João** | :large_blue_circle: To Do | 2 days |
| **(Milestone) M1 Delivery - Docs and Demo** | Prepare slides and final documentation for M1. | **Both** | :checkered_flag: Milestone | 0 days |

---

## ➡️ M2 - Replay

### 2.1 Waypoint Generation and Smoothing (Primary Lead: Yann)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Implement routine for reading and parsing the recorded dataset. | **Yann** | :large_blue_circle: To Do | 3 days |
| Create logic to generate discrete waypoints from odometry. | **Yann** | :large_blue_circle: To Do | 4 days |
| Apply filters (e.g., moving average) for path smoothing. | **João** | :large_blue_circle: To Do | 3 days |

### 2.2 Path Following Controller PID (Primary Lead: João)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Write the PID controller node for lateral and angular error. | **João** | :large_blue_circle: To Do | 5 days |
| Tune initial parameters (Kp, Ki, Kd) in a test environment. | **João** | :large_blue_circle: To Do | 3 days |
| Define the logic for progression between waypoints. | **Yann** | :large_blue_circle: To Do | 2 days |

### 2.3 Simulation Tests Gazebo (Primary Lead: João)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Create or adapt a Gazebo environment for Replay tests. | **João** | :large_blue_circle: To Do | 3 days |
| Execute robustness tests of the controller in simulation scenarios. | **João** | :large_blue_circle: To Do | 4 days |
| Document precision and path tracking results. | **Yann** | :large_blue_circle: To Do | 2 days |
| **(Milestone) M2 Delivery - Autonomous Replay** | Prepare slides and final documentation for M2. | **Both** | :checkered_flag: Milestone | 0 days |

---

## ➡️ M3 - Resume

### 3.1 Nearest Tag Search Algorithm (Primary Lead: Yann)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Algorithm for searching and identifying the nearest Tag. | **Yann** | :large_blue_circle: To Do | 4 days |
| Calculate transformation from Tag pose to robot pose. | **Yann** | :large_blue_circle: To Do | 4 days |
| (Other micro-tasks here...) | ... | ... | ... |

### 3.2 Resume Logic Integration (Primary Lead: Yann)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Implement the FSM (Finite State Machine) for the "Lost" -> "Resume" transition. | **Yann** | :large_blue_circle: To Do | 4 days |
| Logic for initial realignment with the closest Waypoint. | **Yann** | :large_blue_circle: To Do | 4 days |
| (Other micro-tasks here...) | ... | ... | ... |

### 3.3 Simulation Validation Random Starts (Primary Lead: João)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
| Develop script for random initial positioning of the robot. | **João** | :large_blue_circle: To Do | 3 days |
| Execute and log success/failure tests for the Resume function. | **João** | :large_blue_circle: To Do | 4 days |
| **(Milestone) M3 Delivery - Resume Phase** | Prepare slides and final documentation for M3. | **Both** | :checkered_flag: Milestone | 0 days |

---

## ➡️ M4 - Integration

### 4.1 Full System Integration (Primary Lead: Yann)

| Micro-Task | Responsible | Status | Estimated Duration |
| :--- | :--- | :--- | :--- |
