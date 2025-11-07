=========================================
 APRILTAG TOOLS PACKAGE (limo_apriltag_tools)
=========================================

1. OVERVIEW
-----------------

This package provides a complete system for detecting and "latching" AprilTags using a standard V4L2 webcam in ROS2 Humble.

It is composed of two main functionalities:
1. A launch file (`apriltag_webcam_full.launch.py`) that starts the camera and the `apriltag_node` for detection.
2. A custom node (`tag_latcher_node`) that "remembers" the last known position of a tag and keeps it published relative to the map, even when the camera can no longer see it.


2. CORE FUNCTIONALITIES
-----------------------------

* **Tag Detection:** Uses `v4l2_camera` for image capture and `apriltag_ros` for real-time detection.
* **TF Publishing:** The `apriltag_node` publishes the transform (TF) from the camera to the detected tag (e.g., `camera` -> `tag36h11:0`).
* **Pose Latching (Custom Node):** The `tag_latcher_node` "listens" to the TF tree. When it sees a tag connected to the map (e.g., `map` -> `camera` -> `tag36h11:0`), it stores this pose (`map` -> `tag36h11:0`).
* **Continuous Publishing:** The `tag_latcher_node` then continuously republishes this "latched" pose at 10Hz, ensuring the tag remains visible in RViz even after being lost by the camera.
* **Utilities:** Includes a camera info publisher (`camera_info_publisher_node`) for manual synchronization, although the main launch file uses the `camera_info_url` feature from `v4l2_camera`.


3. PREREQUISITES
--------------------

* **Hardware:**
    * A V4L2-compatible USB webcam.
    * Printed AprilTags (the system is configured for the `tag36h11` family).

* **Software (ROS Dependencies):**
    * `v4l2_camera`
    * `apriltag_ros`
    * `apriltag_msgs`
    * `tf2_ros`
    * `launch_ros`
    * (Other standard dependencies like `rclpy` and `sensor_msgs`).

* **Dependency Installation (Example):**
    ```bash
    sudo apt-get install ros-humble-v4l2-camera ros-humble-apriltag-ros ros-humble-apriltag-msgs
    ```


4. CONFIGURATION (BEFORE RUNNING)
----------------------------------

**ESSENTIAL: You MUST do this before using the package.**

1.  **Camera Calibration:**
    * Calibrate your specific camera (e.g., using ROS `camera_calibration`).
    * Replace the contents of `config/webcam_calibration.yaml` with your calibration values. 3D pose detection will NOT work correctly without this.

2.  **Tag Size (IMPORTANT):**
    * With a ruler, measure the side of the BLACK border of your printed AprilTag (in meters).
    * Open the file `launch/apriltag_webcam_full.launch.py`.
    * Edit the `size` parameter to match your measurement.
        ```python
        # Example inside apriltag_webcam_full.launch.py:
        {'size': 0.10},               # <-- Change this value! (currently 10cm)
        ```

3.  **Build:**
    * In your workspace root (e.g., `~/limo_ws`), build the package:
        ```bash
        colcon build --packages-select limo_apriltag_tools
        ```

4.  **Sourcing:**
    * Remember to source your workspace in EVERY terminal you use:
        ```bash
        source ~/path_to_your_ws/install/setup.bash
        ```


5. HOW TO RUN AND TEST (SCENARIOS)
------------------------------------

There are two main scenarios for running the package.

### SCENARIO 1: Basic Detection Test (No Latching)

Use this scenario to check if your camera and tag detection are working.

1.  **Terminal 1 (Main System):**
    ```bash
    ros2 launch limo_apriltag_tools apriltag_webcam_full.launch.py
    ```

2.  **Terminal 2 (RViz):**
    ```bash
    rviz2
    ```
    * In RViz, change the "Fixed Frame" to `camera`.
    * Add an **Image** display and select the `/image_raw` topic (to see the camera) or `/apriltag/image_detections` (to see detections).
    * Add a **TF** display.

3.  **Test:**
    * Show an AprilTag to the camera.
    * **Expected Result:** You will see the tag's frame (e.g., `tag36h11:0`) appear in RViz, "floating" in front of the `camera` frame. If you hide the tag, the frame will disappear.

---

### SCENARIO 2: Full Test with Latching (Locking the Tag to the Map)

This is the full operational mode using the `tag_latcher_node`. Since you (probably) don't have a SLAM or odometry node running, we will create a "fake map" for the test.

1.  **Terminal 1 (Main System):**
    * Starts the camera and the `apriltag_node`.
    ```bash
    ros2 launch limo_apriltag_tools apriltag_webcam_full.launch.py
    ```

2.  **Terminal 2 (The "Fake Map"):**
    * Publishes a static TF that "pins" the `camera` frame to the origin of the `map` frame.
    ```bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera
    ```

3.  **Terminal 3 (The Latcher):**
    * Starts your custom node that will "remember" the tags.
    ```bash
    ros2 run limo_apriltag_tools tag_latcher
    ```

4.  **Terminal 4 (RViz):**
    ```bash
    rviz2
    ```
    * In RViz, change the "Global Options" -> **"Fixed Frame"** to `map`.
    * Add a **TF** display.

5.  **Test:**
    * **Step A:** Show an AprilTag (e.g., `tag36h11:0`) to the camera.
    * **Step B:** In RViz, you will see the `tag36h11:0` frame appear, positioned relative to the `map`.
    * **Step C:** Move the camera away or cover the tag.
    * **Expected Result:** The `tag36h11:0` frame **will remain visible** in RViz, "latched" in its last known position. The `tag_latcher` is working.


6. DEBUGGING AND USEFUL TOOLS
-----------------------------

* **Check if the camera is publishing:**
    ```bash
    ros2 topic hz /image_raw
    ```

* **Check if calibration is publishing:**
    ```bash
    ros2 topic echo /camera_info --once
    ```

* **Check if detection is occurring (Topics):**
    The `apriltag_node` publishes several topics. The most important are:
    * `/detections`: Contains IDs and pixels (no 3D pose).
    * `/tag_detections_pose`: Contains IDs and 3D poses (if calibration is correct).
    * `/tf`: Where the `camera` -> `tag...` pose is published.

* **Check TFs (The most important tool):**
    * **View the TF tree:**
        ```bash
        ros2 run tf2_tools view_frames
        ```
    * **Test detection (Scenario 1):**
        * (While the tag is visible)
        ```bash
        ros2 run tf2_ros tf2_echo camera tag36h11:0
        ```
    * **Test Latching (Scenario 2):**
        * (After the tag has been seen and hidden)
        ```bash
        ros2 run tf2_ros tf2_echo map tag36h11:0
        ```
        * If this command keeps printing a pose, your `tag_latcher` is working.


7. NOTES AND "GOTCHAS"
------------------------------

* **Camera Frame ID:** The system is configured to use `camera` as the primary `frame_id`. This is defined in `apriltag_webcam_full.launch.py`.
* **Tag Naming (IMPORTANT):**
    * The `apriltag_webcam_full.launch.py` file *tries* to set the tag prefix to `target_`.
    * However, the `apriltag_node` (Humble version) seems to ignore this parameter and defaults to using the tag family name.
    * Therefore, the actual tag name published to `/tf` will be `tag36h11:0`, `tag36h11:1`, etc.
    * Your `tag_latcher_node.py` is already **correctly** configured to look for `tag36h11:`. Do not change this unless the behavior of `apriltag_node` changes.
