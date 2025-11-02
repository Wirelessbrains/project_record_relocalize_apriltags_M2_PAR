import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ===============================================
    # 5. REAL LIMO LAUNCH FILE INCLUSION CONFIGURATION
    # ===============================================
    limo_launch = None
    limo_launch_path = '/home/agilex/wb_ws/src/limo_ros2/limo_base/launch/limo_base.launch.py'
    
    if os.path.exists(limo_launch_path):
        try:
            limo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([limo_launch_path])
            )
            print(f"INFO: Including REAL LIMO launch: {limo_launch_path}")
        except Exception as e:
            print(f"WARNING: Failed to include REAL LIMO launch. Error: {e}")
    else:
        print(f"WARNING: REAL LIMO launch file not found at path: {limo_launch_path}")
    
    # ===============================================
    # 1. JOYSTICK DRIVER NODE (joy_node)
    # ===============================================
    joy_node = Node(
        package='joy',             
        executable='joy_node', # Correctly using 'executable'
        name='joy_node',
        output='screen'
    )
    
    # ===============================================
    # 2. MODE CONTROLLER NODE (mode_controller_node)
    #    -> Replaces teleop_node to manage Record/Play.
    # ===============================================
    mode_controller_node = Node(
        package='limo_joystick',
        executable='mode_controller_node', # Assuming the executable name is 'mode_controller_node'
        name='mode_controller',
        output='screen',
        parameters=[
            # ADJUST THESE INDICES FOR YOUR REAL ROBOT (if different from the example)
            {'free_button': 0},         # Button A
            {'record_button': 1},       # Button B (For Record)
            {'play_button': 4},         # Button Y (For Play)
            {'enable_button': 7},       # Enable Button (Example: Start)
            {'axis_linear_x': 1},       # Left Y-Axis
            {'axis_angular_yaw': 2},    # Right X-Axis (Kept as in your code)
            {'scale_linear_x': 0.8},
            {'scale_angular_yaw': 0.5},
            {'cmd_vel_topic': '/cmd_vel'}, # Standard topic, usually correct for LIMO
            # Using 'trajetoria_limo.csv' as defined in the Python file, 
            # or changing the parameter name to match the python code's 'bag_file_name' if needed.
            # Assuming the name in the Python file is 'bag_file_name' for consistency
            {'bag_file_name': 'trajetoria_limo.csv'} 
        ]
    )
    
    # ===============================================
    # 3. RAW DATA VISUALIZATION COMMAND (/joy) (xterm)
    #    Uses 'xterm -e' to execute 'ros2 topic echo /joy' and close on exit.
    # ===============================================
    joy_echo = ExecuteProcess(
        # 'xterm -e' executes the following command and closes the terminal when the command terminates.
        cmd=['xterm', '-e', 'ros2 topic echo /joy'],
        name='joy_echo_monitor',
        output='screen',
        emulate_tty=True
    )

    # ===============================================
    # 4. VELOCITY VISUALIZATION COMMAND (/cmd_vel) (xterm)
    #    Uses 'xterm -e' to execute 'ros2 topic echo /cmd_vel' and close on exit.
    # ===============================================
    cmd_vel_echo = ExecuteProcess(
        # 'xterm -e' executes the following command and closes the terminal when the command terminates.
        cmd=['xterm', '-e', 'ros2 topic echo /cmd_vel'],
        name='cmd_vel_monitor',
        output='screen',
        emulate_tty=True
    )

    # Final list of actions to execute
    actions_to_launch = [
        joy_node,
        mode_controller_node, # USING MODE CONTROLLER NODE
        joy_echo,
        cmd_vel_echo
    ]
    
    # Add the LIMO launch to the list if it was loaded successfully
    if limo_launch:
        actions_to_launch.append(limo_launch)

    return LaunchDescription(actions_to_launch)
