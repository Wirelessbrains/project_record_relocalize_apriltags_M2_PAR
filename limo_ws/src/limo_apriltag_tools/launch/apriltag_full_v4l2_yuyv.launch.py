# ~/limo_apriltag_tools/launch/apriltag_full_v4l2_yuyv.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('limo_apriltag_tools')

    calibration_file = os.path.join(pkg_share_dir, 'config', 'webcam_calibration.yaml')
    camera_info_url = 'file://' + calibration_file
    apriltag_params_file = os.path.join(pkg_share_dir, 'config', 'apriltag_params.yaml')

    # --- câmera ---
    v4l2 = Node(
        package='v4l2_camera',
        node_executable='v4l2_camera_node',
        node_name='v4l2_camera_node',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'time_per_frame': [1, 30],
            'camera_name': 'usb_2.0_camera',
            'use_sensor_data_qos': False,      # força QoS Reliable (combina com o subscriber)
            'frame_id': 'camera',
            'camera_info_url': camera_info_url
        }],
        output='screen'
    )

    # --- conversor ---
    mono = Node(
        package='limo_apriltag_tools',
        node_executable='yuyv_to_mono',
        node_name='yuyv_to_mono',
        output='screen'
    )

    # --- apriltag ---
    apriltag = Node(
        package='apriltag_ros',
        node_executable='apriltag_node',
        node_name='apriltag',
        remappings=[
            ('image', '/image_mono'),
            ('camera_info', '/camera_info_mono'),
            # 👇 acrescenta o remap do tópico “interno” que o apriltag cria:
            ('/apriltag/camera_info', '/camera_info_mono')
        ],
        parameters=[apriltag_params_file],
        output='screen'
    )

    return LaunchDescription([v4l2, mono, apriltag])

