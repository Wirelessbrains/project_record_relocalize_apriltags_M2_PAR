import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'limo_apriltag_tools'
    pkg_share_dir = get_package_share_directory(pkg_name)

    calibration_file = os.path.join(
        pkg_share_dir,
        'config',
        'webcam_calibration.yaml'
    )

    # --- NÓ DA CÂMERA ---
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[
            {'camera_info_url': 'file://' + calibration_file},
            {'frame_id': 'camera'},
        ]
    )

    # --- NÓ APRILTAG (parâmetros diretos — OPÇÃO C) ---
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[
            # ---- Detecção ----
            {'tag_family': 'tag36h11'},
            {'size': 0.10},               # <-- 10 cm (lado preto)
            {'quad_decimate': 2.0},
            {'quad_sigma': 0.0},
            {'nthreads': 4},

            # ---- Sincronização / transporte ----
            {'sync_approximate': True},
            {'use_image_transport': True},

            # ---- Frames ----
            {'camera_frame': 'camera'},
            {'tag_frame_prefix': 'target_'},

            # ---- Saídas (ESSENCIAL p/ distância) ----
            {'publish_tag_detections': True},
            {'publish_tag_detections_pose': True},
            {'publish_tf': True},

            # ---- Visual debug (opcional) ----
            {'publish_detected_image': True},
        ],
    )

    return LaunchDescription([
        camera_node,
        apriltag_node,
    ])

