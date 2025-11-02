import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch.actions import LogInfo
from launch.substitutions import TextSubstitution

def generate_launch_description():

    # ===============================================
    # 5. LIMO LAUNCH FILE INCLUSION CONFIGURATION
    # ===============================================
    limo_launch = None
    try:
        # Standard way to find the LIMO description package
        limo_description_pkg_share = get_package_share_directory('limo_description')
        limo_launch_path = os.path.join(
            limo_description_pkg_share, 
            'launch', 
            'gazebo_models_diff.launch.py'
        ) 
        limo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([limo_launch_path])
        )
        print(f"INFO: Including LIMO Simulation Launch: {limo_launch_path}")
    except Exception:
        print("WARNING: 'limo_description' package not found. Launching without LIMO simulation.")
        pass 
    
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
    # 2. MODE CONTROLLER NODE (mode_controller_node)
    # ===============================================
    mode_controller_node = Node(
        package='limo_joystick_simu', 
        executable='mode_controller_node', 
        name='mode_controller', 
        output='screen', 
        parameters=[
            {'free_button': 0}, {'record_button': 1}, {'play_button': 3}, {'enable_button': 5}, 
            {'axis_linear_x': 1}, {'axis_angular_yaw': 3}, {'scale_linear_x': 0.8}, 
            {'scale_angular_yaw': 0.5}, {'bag_file_name': 'trajetoria_limo.bag'}
        ]
    )

    # -----------------------------------------------
    # 3. RAW DATA MONITOR (/joy)
    # -----------------------------------------------
    JOY_TITLE = 'JOY_MONITOR_LIMO'
    JOY_COMMAND_STRING = 'ros2 topic echo /joy'
    # Use 'sh -c' for robustness in xterm command execution
    joy_echo = ExecuteProcess(
        # USES 'xterm' to automatically close. '-e' executes the command.
        # Adding 'exec' before 'ros2' ensures the shell process is replaced by the ros2 command, 
        # which helps with signal forwarding and reliable closure.
        cmd=['xterm', '-title', JOY_TITLE, '-e', 'sh', '-c', f'exec {JOY_COMMAND_STRING}'],
        name='joy_echo_monitor',
        output='screen',
        emulate_tty=True # Helps with signal handling for closure
    )

    # -----------------------------------------------
    # 4. VELOCITY MONITOR (/cmd_vel)
    # -----------------------------------------------
    CMD_VEL_TITLE = 'CMD_VEL_MONITOR_LIMO'
    CMD_VEL_COMMAND_STRING = 'ros2 topic echo /cmd_vel'
    cmd_vel_echo = ExecuteProcess(
        # USES 'xterm' to automatically close.
        cmd=['xterm', '-title', CMD_VEL_TITLE, '-e', 'sh', '-c', f'exec {CMD_VEL_COMMAND_STRING}'],
        name='cmd_vel_monitor',
        output='screen',
        emulate_tty=True # Helps with signal handling for closure
    )

    # ===============================================
    # 🛑 SHUTDOWN HANDLER (pkill -9 for forced closure)
    # ===============================================

    # Action to force-kill the internal ROS2 process (this makes xterm close)
    # Note: pkill uses regex matching, so be specific.
    kill_joy_echo = ExecuteProcess(
        cmd=['pkill', '-9', '-f', JOY_COMMAND_STRING],
        name='kill_joy_echo',
        on_exit=LogInfo(msg=f'[INFO] Internal JOY process ({JOY_COMMAND_STRING}) terminated. (xterm should close).')
    )
    
    # Action to force-kill the internal ROS2 process (this makes xterm close)
    kill_cmd_vel_echo = ExecuteProcess(
        cmd=['pkill', '-9', '-f', CMD_VEL_COMMAND_STRING],
        name='kill_cmd_vel_echo',
        on_exit=LogInfo(msg=f'[INFO] Internal CMD_VEL process ({CMD_VEL_COMMAND_STRING}) terminated. (xterm should close).')
    )

    # Register the OnShutdown event handler to execute the kill commands
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg=TextSubstitution(text='--- [WARNING] ROS2 Launch ended. Starting terminal shutdown routine ---')),
                ExecuteProcess(cmd=['sleep', '1.0']), # Wait slightly to ensure signals are sent first
                
                LogInfo(msg=TextSubstitution(text=f'[WARNING] Triggering KILL -9 for: {JOY_COMMAND_STRING}')),
                kill_joy_echo,
                
                LogInfo(msg=TextSubstitution(text=f'[WARNING] Triggering KILL -9 for: {CMD_VEL_COMMAND_STRING}')),
                kill_cmd_vel_echo,
                
                LogInfo(msg=TextSubstitution(text='--- [WARNING] Terminal shutdown routine finished ---'))
            ]
        )
    )

    # Final list of actions to execute
    actions_to_launch = [
        joy_node,
        mode_controller_node,
        joy_echo,
        cmd_vel_echo,
        shutdown_handler  # The handler must be included to be active
    ]
    
    # Add LIMO launch if successfully loaded
    if limo_launch:
        actions_to_launch.append(limo_launch)

    return LaunchDescription(actions_to_launch)
