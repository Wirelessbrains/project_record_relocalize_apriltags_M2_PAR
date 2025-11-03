# LIMO Joystick Control (Real Robot Version)

This package (`limo_joystick`) provides joystick control functionality for the real LIMO robot using ROS 2.  
It includes two different modes of operation, each launched through separate launch files.

---

### ** IMPORTANT PREREQUISITE**
This package relies on the **`limo_ros2`** package being installed and configured in your workspace for the simulation models to be loaded correctly. Please refer to the specific configuration guide for your environment:

* **TUTORIAL PATH:** [LIMO ROS2 HUMBLE CONFIGURATION](https://github.com/Wirelessbrains/limo_autonomy_project_M2_PAR/blob/versionyk/tutorials/cofig_Limo_ROS2_Humble_eviroment.md)

  * **Or copy and paste (clone) the folder** [limo_ros2](https://github.com/Wirelessbrains/limo_autonomy_project_M2_PAR/blob/versionyk/PC_WS/src/limo_ros2) **and build it in your** `workspace/src`

* **Before proceeding**, make sure your system is up to date and install the required ROS2 packages for the joystick configuration:

```bash
sudo apt update
sudo apt install ros-humble-joy \
                 ros-humble-joy-linux \
                 ros-humble-teleop-twist-joy
```

---
##  Package Overview

**Package name:** `limo_joystick`  
**Main node:** `mode_controller`  
**Dependencies:** `rclpy`, `sensor_msgs`, `geometry_msgs`, `csv`, `os`, `time`

---

##  Launch Files

###  Teleoperation Mode (Basic Control)

**File:** `limo_joystick.launch.py`  

This mode is used for **manual driving** of the robot.  
It configures a standard Xbox joystick by default, but the parameters can be changed for any other controller.

- This mode **does not** include the record or playback functions.  
- It only publishes velocity commands (`/cmd_vel`) directly from joystick input.

**Run command:**

```bash
ros2 launch limo_joystick limo_joystick.launch.py
```

---

###  CSV Mode (Real Trajectory Recording & Playback)

**File:** `mode_limo_joystick.launch.py`  

This mode enables a **three-state control system**:
- **Free Mode:** manual teleoperation (default)
- **Record Mode:** saves trajectory to a `.csv` file
- **Play Mode:** executes the recorded trajectory

The system uses a single joystick to toggle between these modes dynamically.

**Run command:**

```bash
ros2 launch limo_joystick mode_limo_joystick.launch.py
```

---

##  Default Xbox Controller Mapping

| Action | Button | Description |
|:--------|:--------:|:-------------|
| **Enable movement** | RB (7) | Must be held to send velocity commands |
| **Free mode** | A (0) | Switch to manual mode |
| **Record mode** | B (1) | Start/stop recording to CSV |
| **Play mode** | Y (4) | Start/stop playback from CSV |

>  These button indices and axis parameters can be changed in the launch file.

---

##  CSV File Output

When in **Record Mode**, the node creates a file named:

```
trajetoria_limo.csv
```

Each line contains the following data:

```
relative_time, linear_x, angular_z
```

This file is then used automatically when entering **Play Mode** to reproduce the recorded trajectory.

---

##  Notes

- The **recorded file** is overwritten each time a new recording starts.  
- If the **Enable button (RB)** is not pressed, motion is set to zero even if the joystick is moved.  
- Make sure your LIMO robot is connected and running the correct ROS 2 bridge.  
- Recommended update rate: 50 Hz (0.02 s timer).

---

##  Summary

| Mode | Description | Command |
|:------|:-------------|:---------|
| **Teleop** | Manual control only | `ros2 launch limo_joystick limo_joystick.launch.py` |
| **Mode Controller** | Free / Record / Play modes | `ros2 launch limo_joystick mode_limo_joystick.launch.py` |

---

**Author:** Wireless Brains  
**Package:** `limo_joystick`  
**Environment:** ROS 2 eloquente / Ubuntu 18.04  
