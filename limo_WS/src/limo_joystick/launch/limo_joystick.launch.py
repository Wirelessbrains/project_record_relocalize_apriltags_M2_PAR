import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ===============================================
    # 5. LIMO LAUNCH FILE INCLUSION CONFIGURATION
    # (MODIFIED to use the REAL robot's launch)
    # ===============================================
    limo_launch = None
    
    # ----------------------------------------------------------------------
    # REPLACED get_package_share_directory() logic to use the ABSOLUTE
    # path specified for the real robot's launch file.
    # ----------------------------------------------------------------------
    limo_launch_path = '/home/agilex/wb_ws/src/limo_ros2/limo_base/launch/limo_base.launch.py'
    
    # Check if the file exists before attempting to include
    if os.path.exists(limo_launch_path):
        try:
            limo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([limo_launch_path])
            )
            print(f"INFO: Including REAL LIMO launch: {limo_launch_path}")
        except Exception as e:
            # If inclusion fails for any reason, even if the file exists
            print(f"WARNING: Failed to include REAL LIMO launch. Error: {e}")
    else:
        print(f"WARNING: REAL LIMO launch file not found at path: {limo_launch_path}")
    
    # ===============================================
    # 1. JOYSTICK DRIVER NODE (joy_node)
    # ===============================================
    joy_node = Node(
        package='joy',             
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    # ===============================================
    # 2. TELEOPERATION NODE (teleop_node)
    # ===============================================
    teleop_params = {
        'axis_linear.x': 1,
        'axis_angular.yaw': 2,
        'enable_button': 7,
        'scale_linear.x': 0.8,
        'scale_angular.yaw': 0.5,
    }
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[teleop_params],
        output='screen',
        # Remapping '/cmd_vel' to a different topic if needed for LIMO base
        # remappings=[('/cmd_vel', '/limo/cmd_vel')] # Uncomment if LIMO uses a different topic name
    )
    
    # ===============================================
    # 3. RAW DATA VISUALIZATION COMMAND (/joy) (xterm)
    #    Uses 'xterm -e' to execute 'ros2 topic echo /joy' and close on exit.
    # ===============================================
    joy_echo = ExecuteProcess(
        # The 'bash -c' is used to ensure the command is executed and the terminal closes upon completion.
        # When ros2 launch is terminated, it sends signals to 'xterm', which closes, terminating 'ros2 topic echo'.
        cmd=['xterm', '-e', 'ros2 topic echo /joy'],
        name='joy_echo_monitor',
        output='screen',
        # Set the required flag to ensure this process is monitored by launch
        # and receives the kill signals upon shutdown.
        emulate_tty=True
    )

    # ===============================================
    # 4. VELOCITY VISUALIZATION COMMAND (/cmd_vel) (xterm)
    #    Uses 'xterm -e' to execute 'ros2 topic echo /cmd_vel' and close on exit.
    # ===============================================
    cmd_vel_echo = ExecuteProcess(
        # The 'bash -c' is used to ensure the command is executed and the terminal closes upon completion.
        # When ros2 launch is terminated, it sends signals to 'xterm', which closes, terminating 'ros2 topic echo'.
        cmd=['xterm', '-e', 'ros2 topic echo /cmd_vel'],
        name='cmd_vel_monitor',
        output='screen',
        emulate_tty=True
    )

    # Final list of actions to execute
    actions_to_launch = [
        joy_node,
        teleop_node,
        joy_echo,
        cmd_vel_echo
    ]
    
    # Add the LIMO launch to the list if it was loaded successfully
    if limo_launch:
        actions_to_launch.append(limo_launch)

    return LaunchDescription(actions_to_launch)
