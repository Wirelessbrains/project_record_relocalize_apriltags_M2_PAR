# LIMO AprilTag Tools

[![ROS 2 Eloquent](https://img.shields.io/badge/ROS%202-Eloquent-blue)](https://docs.ros.org/en/eloquent/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-yellow.svg)](https://www.apache.org/licenses/LICENSE-2.0)

A ROS 2 perception pipeline for the LIMO robot, providing robust, real-time AprilTag detection from a standard `v4l2` webcam.

This package is designed to solve a common challenge in ROS 2: efficiently converting `YUYV` webcam streams to `MONO8` and ensuring **perfect timestamp synchronization** between the `Image` and `CameraInfo` messages. This synchronization is critical for the `apriltag_ros` node to accurately compute 3D poses.

## Core Problem & Solution

The `apriltag_ros` package requires a `MONO8` encoded image and a corresponding `CameraInfo` message. Most standard webcams, when accessed via `v4l2_camera_node`, provide a `YUYV` stream.

This package provides the central "glue" node, `yuyv_to_mono_node`, which:
1.  Subscribes to `/image_raw` (`YUYV`) and `/camera_info`.
2.  Performs a highly efficient `numpy`-based conversion from `YUYV` to `MONO8`.
3.  **Crucially, it copies the *exact* header (including the timestamp) from the input image to the output `CameraInfo` message**. This eliminates timestamp-related synchronization failures in the `apriltag_node`.
4.  Publishes the synchronized `/image_mono` and `/camera_info_mono` topics.

## System Pipeline

The entire pipeline is orchestrated by the `apriltag_full_v4l2_yuyv.launch.py` launch file.

1.  **Node: `v4l2_camera_node`**
    * **Action**: Captures 640x480 @ 30fps `YUYV` stream.
    * **Calibration**: Loads `config/webcam_calibration.yaml` using the `camera_info_url` parameter.
    * **Publishes**: `/image_raw` (YUYV) and `/camera_info` (Calibrated).

2.  **Node: `yuyv_to_mono_node`**
    * **Action**: Subscribes to the topics above. Converts `YUYV` -> `MONO8` and ensures timestamp synchronization.
    * **Publishes**: `/image_mono` (MONO8) and `/camera_info_mono` (Synced).

3.  **Node: `apriltag_node`**
    * **Action**: Subscribes (via remap) to `/image_mono` and `/camera_info_mono`.
    * **Configuration**: Loads `config/apriltag_params.yaml`.
    * **Publishes**: `/apriltag/detections` and `/tf` (from `camera` frame to `tag_X` frames).

### A Note on QoS
This pipeline is explicitly configured for `RELIABLE` QoS. The `yuyv_to_mono_node` uses a `Reliable` policy, and the launch file instructs the `v4l2_camera_node` to use `use_sensor_data_qos: False`, forcing it to match the reliable profile for robust message delivery, especially for the initial `CameraInfo`.

## Prerequisites

* ROS 2 Eloquent
* ROS 2 Dependencies (System-installed):
    ```bash
    sudo apt update
    sudo apt install ros-eloquent-v4l2-camera ros-eloquent-apriltag-ros
    ```
    *(Note: This assumes `apriltag_msgs` and `apriltag_ros` are installed via `apt` and **not** included in the `src/` directory, as per the `.gitignore` configuration.)*

## Build Instructions

1.  Clone this repository into your workspace's `src/` folder:
    ```bash
    cd ~/limo_ws/src
    git clone [https://github.com/Wirelessbrains/limo_autonomy_project_M2_PAR.git](https://github.com/Wirelessbrains/limo_autonomy_project_M2_PAR.git)
    ```
2.  Navigate to the workspace root and build the package:
    ```bash
    cd ~/limo_ws
    colcon build --packages-select limo_apriltag_tools
    ```

## Usage

1.  Ensure your webcam is connected (e.g., at `/dev/video0`).
2.  Source your workspace:
    ```bash
    source ~/limo_ws/install/setup.bash
    ```
3.  Run the main launch file:
    ```bash
    ros2 launch limo_apriltag_tools apriltag_full_v4l2_yuyv.launch.py
    ```

## System Verification (RViz2)

1.  Open RViz2: `rviz2`
2.  Set the **Global Options** -> **Fixed Frame** to `camera`.
3.  Add the following displays to visualize the pipeline:
    * **Image**: Subscribe to `/image_mono` to see the monochrome image being processed.
    * **TF**: To visualize the `camera` and `tag_X` coordinate frames.
    * **MarkerArray**: Subscribe to `/apriltag/detections` to see the detected tags visualized.

## Configuration Files

* `config/webcam_calibration.yaml`: **(Critical)** Contains the camera's intrinsic calibration matrices (K, D, P). This file is loaded by `v4l2_camera_node` at startup.
* `config/apriltag_params.yaml`: **(Critical)** Configures the `apriltag_node` detector.
    * `tag_family: "tag36h11"`: The dictionary of tags to detect.
    * `tag_size: 0.150`: The physical size (15cm) of the tag's black border in meters. **This must be accurate** for correct pose calculation.
    * `camera_frame: "camera"`: The parent frame for all tag poses.

---
*Apache-2.0 License (as per `package.xml`)*
