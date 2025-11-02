# 🏎️ Package `limo_joystick_simu`: Advanced LIMO Simulation and Control

The `limo_joystick_simu` package provides a robust solution for **simulation, teleoperation, and trajectory automation** of the LIMO robot.  
It extends the basic ROS 2 teleoperation capabilities by implementing a **custom state machine** that enables recording and playback of movement commands with precision and flexibility.

### **⚠️ IMPORTANT PREREQUISITE**
This package relies on the **`limo_ros2`** package being installed and configured in your workspace for the simulation models to be loaded correctly. Please refer to the specific configuration guide for your environment:

* **TUTORIAL PATH:** [LIMO ROS2 HUMBLE CONFIGURATION](https://github.com/Wirelessbrains/limo_autonomy_project_M2_PAR/blob/versionyk/tutorials/cofig_Limo_ROS2_Humble_eviroment.md)

** **Or copy and paste (clone) the folder** [limo_ros2](https://github.com/Wirelessbrains/limo_autonomy_project_M2_PAR/blob/versionyk/PC_WS/src/limo_ros2) **and build it in your** `workspace/src`

* **Before proceeding**, make sure your system is up to date and install the required ROS2 packages for the joystick configuration:

```bash
sudo apt update
sudo apt install ros-humble-joy \
                 ros-humble-joy-linux \
                 ros-humble-teleop-twist-joy
```
---

### 🧭 **Motion Control Overview**
The LIMO robot operates in **differential drive mode** (`diff_drive`), where velocity messages (`/cmd_vel`) are interpreted as:
- **Linear velocity** (`linear.x`)
- **Angular velocity** (`angular.z`)  
These parameters are used to compute the wheel speeds that control the robot’s motion.

---

## ⚙️ Main Components

This package manages both the **ROS 2** and **Gazebo** environments and is built around one custom node and two main launch files:

| Type | Name | Function | Main Controller |
| :--- | :--- | :--- | :--- |
| **Custom Node** | `mode_controller_node` | The **core of the package.** Implements a three-state control logic (FREE, RECORD, PLAY) to manage the robot and external `rosbag` processes. | N/A |
| **Launch File 1** | `simu_and_mode_joystick.launch.py` | **Advanced Mode:** Starts the simulation and enables trajectory recording and playback through a state machine. | `mode_controller_node` |
| **Launch File 2** | `simu_and_teleop.launch.py` | **Simple Mode:** Starts the simulation with standard joystick control only (no state machine logic). | `teleop_twist_joy` |

---

## 🎮 Joystick Configuration

The package is **preconfigured for a standard Xbox controller**, but unlike typical setups that rely on a `.yaml` file,  
**this configuration is done directly in the code or within the launch command**.

Example (inside the launch command or node parameters):
```bash
ros2 run teleop_twist_joy teleop_twist_joy_node --ros-args -p axis_linear:=1 -p axis_angular:=3 -p enable_button:=5 -p enable_turbo_button:=7 -p scale_linear:=0.5 -p scale_angular:=0.8
```

You can freely modify these parameters to adapt the control scheme to **any joystick or gamepad**, such as PlayStation or generic USB controllers.  
The mappings for axes and buttons can be customized according to your hardware and preferences.

**Default button mapping (Xbox controller):**
| Function | Button | Description |
| :--- | :--- | :--- |
| **Free Mode** | A | Return to manual mode |
| **Record Mode** | B | Start/Stop recording trajectory |
| **Play Mode** | Y | Play recorded trajectory |

---

## 🚀 How to Run

Choose one of the available modes.  
In both cases, the **Gazebo simulation** and **joystick driver (`joy_node`)** are automatically launched.

---

### 1. 🤖 **Advanced Mode: Trajectory Recording and Playback**  
(`simu_and_mode_joystick.launch.py`)

This mode launches the **custom `mode_controller_node`**, which acts as a **state machine** enabling:
- Manual control (FREE)
- Trajectory recording (RECORD)
- Autonomous playback (PLAY)

```bash
ros2 launch limo_joystick_simu simu_and_mode_joystick.launch.py
```

| Component | Executed Node | Key Functionality |
| :--- | :--- | :--- |
| State Control | `mode_controller_node` (Custom) | Enables **FREE**, **RECORD**, and **PLAY** modes through joystick buttons. |
| Monitors | `xterm` (/joy, /cmd_vel) | Displays joystick inputs and robot velocity commands. |

#### ⚙️ Detailed Operation

**FREE Mode (Button A):**
- Default starting state.
- Manual joystick control publishes to `/cmd_vel`.

**RECORD Mode (Button B):**
1. Press **B** → transitions to `MODE_RECORD`.
2. The node executes:
   ```bash
   ros2 bag record -o [file_name] /cmd_vel
   ```
3. The user drives the robot manually while motion data is recorded.
4. Pressing **B** again (or **A**) stops and saves the `.bag` file.

**PLAY Mode (Button Y):**
1. Press **Y** → transitions to `MODE_PLAY`.
2. The node executes:
   ```bash
   ros2 bag play [file_name]
   ```
3. Manual control is disabled; the robot replays the recorded motion autonomously.
4. When playback finishes, or if **Y/A** is pressed again, the system returns to FREE mode.

---

### 2. 🕹️ **Simple Mode: Direct Manual Control Only**  
(`simu_and_teleop.launch.py`)

This mode runs **without the custom state machine** — it provides only **direct joystick control** of the robot.  
It is ideal for **basic manual operation**, testing joystick input, or simple movement.

```bash
ros2 launch limo_joystick_simu simu_and_teleop.launch.py
```

| Component | Executed Node | Functionality |
| :--- | :--- | :--- |
| Motion Control | `teleop_twist_joy` (Standard ROS 2 Node) | Direct mapping of joystick axes to `/cmd_vel`. |
| Monitors | `xterm` (/joy, /cmd_vel) | Displays joystick and velocity data in real time. |

#### ⚙️ Detailed Operation
- The `teleop_twist_joy` node **does not include state logic** (no recording or playback).  
- The robot is controlled directly via joystick axes.
- Motion is **enabled only** when the **enable_button** (e.g., Button 5) is pressed — providing a built-in safety feature.
- This mode simply transmits live commands to control the robot manually.

---

## 🧠 Node Details: `ModeController` (Advanced Mode Only)

The `mode_controller_node` handles:
- Reading joystick button inputs.
- Managing transitions between FREE, RECORD, and PLAY states.
- Launching and safely stopping `ros2 bag` processes.
- Disabling manual input during playback to ensure accurate execution.

---

### 🕹️ State Machine Summary

| Mode | State | Control Type | Description |
| :--- | :--- | :--- | :--- |
| **FREE** | `MODE_FREE` | Manual | Direct joystick control (publishes `/cmd_vel`). |
| **RECORD** | `MODE_RECORD` | Manual + Recording | Keeps manual control while saving `/cmd_vel` data to a `.bag` file. |
| **PLAY** | `MODE_PLAY` | Autonomous | Replays recorded trajectories using `ros2 bag play`. |

---

## 🛑 Process Management

The node uses Python’s **`subprocess`** module to launch and terminate `rosbag` processes safely.  
It sends `SIGINT` signals to ensure clean transitions and prevent background processes from persisting.

---

## ⚠️ Safe Terminal Shutdown

Both launch files include a **shutdown routine** that ensures all `xterm` topic monitoring windows (`/joy`, `/cmd_vel`) close properly.

- **Implementation:**  
  Monitors are launched with `xterm`. When `Ctrl+C` is pressed, the system executes:
  ```bash
  pkill -9 -f 'ros2 topic echo /topic'
  ```
  to terminate all running topic echoes.

- **Automatic Closure:**  
  Each `xterm` window closes once its command ends.

- **Verification:**  
  `[WARNING]` messages appear in the main terminal to confirm clean shutdown.

---

💡 **Final Summary**

`limo_joystick_simu` provides a **complete and modular simulation system** for the LIMO robot, offering:
- Full **Gazebo integration**
- **Real-time joystick control**
- **Trajectory recording and playback**
- **Direct Xbox controller setup (editable via launch command)**
- **No .yaml files required**
- **Clean process and window management**

You can operate the robot in either:
- **Simple Manual Mode** (`simu_and_teleop.launch.py`) → direct joystick control only  
- **Advanced Mode** (`simu_and_mode_joystick.launch.py`) → record, save, and replay robot trajectories automatically


**Author:** Wireless Brains  
**Package:** `limo_joystick`  
**Environment:** ROS 2 humble / Ubuntu 22.04  
