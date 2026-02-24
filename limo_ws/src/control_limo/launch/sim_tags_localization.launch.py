import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_share = get_package_share_directory('control_limo')
    apriltag_share = get_package_share_directory('limo_apriltag_tools')
    gz_share = get_package_share_directory('gz_apriltag_env')

    default_world = os.path.join(gz_share, 'worlds', 'walls_apriltag_limo.sdf')

    sim_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'sim_perception.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    ippe_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(apriltag_share, 'launch', 'ippe_parking_localization.launch.py')
        ),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'detections_topic': '/detections',
            'camera_info_topic': '/camera_info',
            'base_frame': 'base_footprint',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument(
            'map_yaml',
            description='Tag map YAML path used by static map publisher and IPPE localizer',
        ),
        sim_perception,
        ippe_localization,
    ])
