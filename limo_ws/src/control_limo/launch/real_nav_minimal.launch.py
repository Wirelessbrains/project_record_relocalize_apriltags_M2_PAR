from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pose_topic', default_value='/tag_only_base_pose'),
        DeclareLaunchArgument('distance_tolerance', default_value='0.20'),
        DeclareLaunchArgument('require_final_orientation', default_value='true'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.35'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.80'),
        Node(
            package='control_limo',
            executable='go_to_pose',
            name='go_to_pose_node',
            output='screen',
            parameters=[{
                'pose_source': 'tag_only',
                'pose_topic': LaunchConfiguration('pose_topic'),
                'path_frame_id': 'map',
                'kp_linear': 1.0,
                'kp_angular': 2.8,
                'distance_tolerance': LaunchConfiguration('distance_tolerance'),
                'require_final_orientation': LaunchConfiguration('require_final_orientation'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'use_gazebo_model_states': False,
                'enable_reverse_motion': True,
                'startup_delay': 0.0,
            }],
        ),
    ])
