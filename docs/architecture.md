# System Architecture

## 1. Overview
The software is organized into four ROS 2 packages, each addressing a different phase of the navigation pipeline.

| Package | Function |
|----------|-----------|
| **limo_mapping** | Records AprilTag poses and odometry during manual teaching |
| **limo_route_follow** | Executes trajectory following between recorded tags |
| **limo_relocalization** | Finds nearest AprilTag and resumes route autonomously |
| **limo_simulation** | Provides Gazebo world, robot model, and RViz configuration |

---

## 2. Teach, Replay & Resume Pipeline

[Joystick] → [Mapping Node] → [Trajectory File] → [Route Follow Node] → [Relocalization Node]

- **Mapping Node:** captures AprilTag IDs, their 3D poses, and robot odometry.  
- **Route Follow Node:** reads saved waypoints and publishes velocity commands.  
- **Relocalization Node:** matches current camera detections to known tag poses and restarts from the correct waypoint.  

---

## 3. ROS2 Node Graph (example)

/camera/image_raw → /apriltag_ros
/apriltag_detections → /limo_mapping
/limo_mapping → /limo_route_follow
/limo_relocalization → /cmd_vel

---

## 4. TF Tree Example

---

## 5. Topics and Parameters (excerpt)
| Node | Subscribes | Publishes | Parameters |
|------|-------------|-----------|-------------|
| `mapping_node` | `/apriltag_detections`, `/odom` | `/mapping_data` | `tag_family`, `save_rate` |
| `route_follow_node` | `/mapping_data`, `/odom` | `/cmd_vel` | `linear_speed`, `angular_speed` |
| `relocalization_node` | `/apriltag_detections`, `/odom` | `/cmd_vel` | `resume_tolerance` |

---

## 6. Files and Configurations
- **Config:** YAML files for AprilTag detection, controller gains, etc.  
- **Launch:** Python launch files for each phase.  
- **RViz:** visualization layouts for debugging.  
- **Worlds:** Gazebo environments for training and validation.

---

## 7. System Architecture Diagram
*(To be represented in `system_architecture.png`)*



