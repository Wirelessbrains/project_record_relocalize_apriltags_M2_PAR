import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gz_env_share = get_package_share_directory('gz_apriltag_env')
    apriltag_share = get_package_share_directory('limo_apriltag_tools')

    default_world = os.path.join(gz_env_share, 'worlds', 'walls_apriltag_limo.sdf')
    models_path = os.path.join(gz_env_share, 'models')
    worlds_path = os.path.join(gz_env_share, 'worlds')

    world_arg = DeclareLaunchArgument('world', default_value=default_world)

    set_ign_resource = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=f'{models_path}:{worlds_path}')
    set_gz_resource = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=f'{models_path}:{worlds_path}')

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', LaunchConfiguration('world')], output='screen')

    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/model/limo_01/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose',
            '/model/limo_01/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/rgb_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        output='screen'
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(apriltag_share, 'launch', 'apriltag_ignition.launch.py'))
    )

    return LaunchDescription([
        world_arg,
        set_ign_resource,
        set_gz_resource,
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[apriltag_launch]),
    ])
