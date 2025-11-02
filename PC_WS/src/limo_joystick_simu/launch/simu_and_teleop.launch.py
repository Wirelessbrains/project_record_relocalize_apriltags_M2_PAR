import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch.actions import LogInfo # Required for displaying shutdown messages

def generate_launch_description():

    # ===============================================
    # 5. LIMO LAUNCH FILE INCLUSION CONFIGURATION
    # ===============================================
    limo_launch = None
    try:
        limo_description_pkg_share = get_package_share_directory('limo_description')
        limo_launch_path = os.path.join(
            limo_description_pkg_share, 
            'launch', 
            'gazebo_models_diff.launch.py'
        ) 
        limo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([limo_launch_path])
        )
        # Using print() here, but in a real file, LogInfo would be better if executed via an action
        # print(f"INFO: Incluindo o launch do LIMO: {limo_launch_path}")
    except Exception as e:
        # print(f"AVISO: O pacote 'limo_description' ou o arquivo de launch não foi encontrado. Erro: {e}")
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
    # 2. TELEOPERATION NODE (teleop_node)
    # ===============================================
    teleop_params = {
        'axis_linear.x': 1,
        'axis_angular.yaw': 3,
        'enable_button': 5,
        'scale_linear.x': 0.8,
        'scale_angular.yaw': 0.5,
    }
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[teleop_params],
        output='screen'
    )
    
    # -----------------------------------------------
    # DEFINING COMMAND STRINGS for pkill (Unique identifiers)
    # -----------------------------------------------
    JOY_TITLE = 'JOY_MONITOR_TELEOP'
    JOY_COMMAND_STRING = 'ros2 topic echo /joy'
    
    CMD_VEL_TITLE = 'CMD_VEL_MONITOR_TELEOP'
    CMD_VEL_COMMAND_STRING = 'ros2 topic echo /cmd_vel'

    # ===============================================
    # 3. RAW DATA VISUALIZATION COMMAND (/joy) (Using XTERM)
    # ===============================================
    joy_echo = ExecuteProcess(
        # Changed to 'xterm' for guaranteed window closure
        cmd=['xterm', '-title', JOY_TITLE, '-e', 'bash', '-c', JOY_COMMAND_STRING],
        name='joy_echo_monitor',
        output='screen'
    )

    # ===============================================
    # 4. VELOCITY VISUALIZATION COMMAND (/cmd_vel) (Using XTERM)
    # ===============================================
    cmd_vel_echo = ExecuteProcess(
        # Changed to 'xterm' for guaranteed window closure
        cmd=['xterm', '-title', CMD_VEL_TITLE, '-e', 'bash', '-c', CMD_VEL_COMMAND_STRING],
        name='cmd_vel_monitor',
        output='screen'
    )

    # ===============================================
    # 🛑 FORCED SHUTDOWN HANDLER
    # ===============================================

    # Log message formatted as WARNING for JOY shutdown
    log_kill_joy = LogInfo(msg=f'[WARNING] DISPATCHING KILL -9: Searching and terminating process: {JOY_COMMAND_STRING}')
    
    # Action to terminate the ROS2 process (this will make xterm close)
    kill_joy_echo = ExecuteProcess(
        cmd=['pkill', '-9', '-f', JOY_COMMAND_STRING],
        name='kill_joy_echo',
        on_exit=LogInfo(msg=f'[WARNING] Internal JOY process ({JOY_COMMAND_STRING}) TERMINATED. (xterm should close).')
    )

    # Log message formatted as WARNING for CMD_VEL shutdown
    log_kill_cmd_vel = LogInfo(msg=f'[WARNING] DISPATCHING KILL -9: Searching and terminating process: {CMD_VEL_COMMAND_STRING}')

    # Action to terminate the ROS2 process (this will make xterm close)
    kill_cmd_vel_echo = ExecuteProcess(
        cmd=['pkill', '-9', '-f', CMD_VEL_COMMAND_STRING],
        name='kill_cmd_vel_echo',
        on_exit=LogInfo(msg=f'[WARNING] Internal CMD_VEL process ({CMD_VEL_COMMAND_STRING}) TERMINATED. (xterm should close).')
    )

    # Register the OnShutdown event handler to execute the kill commands
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='--- [WARNING] Main ROS2 Launch terminated. Starting terminal shutdown routine ---'),
                # Delay to ensure stability
                ExecuteProcess(cmd=['sleep', '1.0']), 
                
                # JOY Execution and Verification
                log_kill_joy,
                kill_joy_echo,
                
                # CMD_VEL Execution and Verification
                log_kill_cmd_vel,
                kill_cmd_vel_echo,
                
                LogInfo(msg='--- [WARNING] Terminal shutdown routine completed ---')
            ]
        )
    )

    # Final list of actions to execute
    actions_to_launch = [
        joy_node,
        teleop_node,
        joy_echo,
        cmd_vel_echo,
        shutdown_handler # Add the shutdown handler
    ]
    
    # Add LIMO launch if successfully loaded
    if limo_launch:
        actions_to_launch.append(limo_launch)

    # Ensure handler is in the list
    if shutdown_handler not in actions_to_launch:
        actions_to_launch.append(shutdown_handler)

    return LaunchDescription(actions_to_launch)
