from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('reference_csv', description='Reference trajectory CSV (trajetoria_camera.csv)'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('pose_topic', default_value='/tag_only_pose'),
        DeclareLaunchArgument('pose_msg_type', default_value='pose_stamped'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('distance_mode', default_value='2d'),
        DeclareLaunchArgument('path_downsample_step', default_value='5'),
        DeclareLaunchArgument('relocalization_mode', default_value='full'),
        DeclareLaunchArgument('auto_align_xyyaw', default_value='false'),
        DeclareLaunchArgument('trajectory_plane', default_value='xy'),
        DeclareLaunchArgument('reference_axis_mode', default_value='identity'),
        Node(
            package='limo_online_relocalization',
            executable='online_relocalization_node',
            name='online_relocalization_node',
            output='screen',
            parameters=[{
                'reference_csv': LaunchConfiguration('reference_csv'),
                'frame_id': LaunchConfiguration('frame_id'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'pose_msg_type': LaunchConfiguration('pose_msg_type'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'distance_mode': LaunchConfiguration('distance_mode'),
                'path_downsample_step': LaunchConfiguration('path_downsample_step'),
                'relocalization_mode': LaunchConfiguration('relocalization_mode'),
                'auto_align_xyyaw': LaunchConfiguration('auto_align_xyyaw'),
                'trajectory_plane': LaunchConfiguration('trajectory_plane'),
                'reference_axis_mode': LaunchConfiguration('reference_axis_mode'),
            }],
        )
    ])
