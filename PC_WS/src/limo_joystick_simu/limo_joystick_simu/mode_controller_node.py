import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import subprocess
import os
import signal
import time
import shutil  # Needed to remove directories (folders)
from rclpy.qos import qos_profile_sensor_data

# --- Robot State Constants ---
MODE_FREE = 0      # Manual control via Joystick (Teleoperation)
MODE_RECORD = 1    # Recording /cmd_vel commands
MODE_PLAY = 2      # Executing recorded commands

class ModeController(Node):
    """
    Mode Controller for switching between Free, Record, and Playback trajectories
    using external ros2 bag processes.
    """
    def __init__(self):
        super().__init__('mode_controller')
        
        # --- Parameters ---
        self.declare_parameter('free_button', 0)
        self.declare_parameter('record_button', 2)
        self.declare_parameter('play_button', 3)
        self.declare_parameter('enable_button', 5)
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_yaw', 3)
        self.declare_parameter('scale_linear_x', 0.8)
        self.declare_parameter('scale_angular_yaw', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('bag_file_name', 'trajetoria_limo.bag')

        # Load Parameters
        self.free_button = self.get_parameter('free_button').get_parameter_value().integer_value
        self.record_button = self.get_parameter('record_button').get_parameter_value().integer_value
        self.play_button = self.get_parameter('play_button').get_parameter_value().integer_value
        self.enable_button = self.get_parameter('enable_button').get_parameter_value().integer_value
        self.axis_linear_x = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_angular_yaw = self.get_parameter('axis_angular_yaw').get_parameter_value().integer_value
        self.scale_linear_x = self.get_parameter('scale_linear_x').get_parameter_value().double_value
        self.scale_angular_yaw = self.get_parameter('axis_angular_yaw').get_parameter_value().double_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        # Note: ros2 bag names the output directory after the bag_file_name parameter.
        self.bag_file_name = self.get_parameter('bag_file_name').get_parameter_value().string_value
        
        # --- State Variables ---
        self.current_mode = MODE_FREE
        self.last_buttons = []
        self.record_process = None
        self.play_process = None
        
        self.get_logger().info(f'[{self.get_name()}] Initialized. Current Mode: FREE')
        self.get_logger().info(f'Free Button (A): Index {self.free_button} | Record (B): {self.record_button} | Play (Y): {self.play_button}')
        
        # --- Publishers and Subscribers ---
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            qos_profile_sensor_data
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)


    def joy_callback(self, msg):
        current_buttons = msg.buttons
        
        if not self.last_buttons or len(self.last_buttons) != len(current_buttons):
            self.last_buttons = list(current_buttons)
            return

        # --- Transition Logic: FREE Button (Forces return to Free mode) ---
        if (current_buttons[self.free_button] == 1 and self.last_buttons[self.free_button] == 0):
            if self.current_mode == MODE_RECORD:
                self.stop_record()
            elif self.current_mode == MODE_PLAY:
                self.stop_play()
            
            if self.current_mode != MODE_FREE:
                self.current_mode = MODE_FREE
                self.get_logger().info('*** FREE MODE ACTIVATED by dedicated button. ***')


        # --- Transition Logic: RECORD/PLAY Buttons ---
        
        # Record Button (Toggle)
        elif (current_buttons[self.record_button] == 1 and self.last_buttons[self.record_button] == 0):
            if self.current_mode == MODE_FREE:
                self.start_record()
            elif self.current_mode == MODE_RECORD:
                self.stop_record()

        # Play Button (Toggle)
        elif (current_buttons[self.play_button] == 1 and self.last_buttons[self.play_button] == 0):
            if self.current_mode == MODE_FREE:
                self.start_play()
            elif self.current_mode == MODE_PLAY:
                self.stop_play()


        # --- Execute Action in Current Mode (MOVEMENT HANDLING HERE) ---
        
        # The robot must be controllable (teleop) in FREE AND RECORD modes.
        if self.current_mode == MODE_FREE or self.current_mode == MODE_RECORD:
            # Publish teleoperation commands
            self.publish_teleop_twist(msg)
            
        elif self.current_mode == MODE_PLAY:
            # Check if the rosbag play process finished on its own
            if self.play_process and self.play_process.poll() is not None:
                self.get_logger().info('Trajectory playback finished automatically.')
                self.current_mode = MODE_FREE
                self.play_process = None
                self.get_logger().info(f'Current Mode: FREE')
            
        
        # Update button state
        self.last_buttons = list(current_buttons)


    def publish_teleop_twist(self, joy_msg):
        """Calculates and publishes the Twist message based on joystick axes."""
        twist = Twist()
        # Check if the enable button is pressed
        is_enabled = len(joy_msg.buttons) > self.enable_button and joy_msg.buttons[self.enable_button] == 1
        
        if is_enabled:
            # Apply scaling to linear and angular movement from joystick axes
            twist.linear.x = joy_msg.axes[self.axis_linear_x] * self.scale_linear_x
            twist.angular.z = joy_msg.axes[self.axis_angular_yaw] * self.scale_angular_yaw
        else:
            # Zero movement if not enabled
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

    # --- PROCESS CONTROL METHODS ---

    def start_record(self):
        """Starts ros2 bag recording, handling directory removal."""
        self.current_mode = MODE_RECORD
        self.get_logger().warn('*** RECORD MODE ACTIVATED! Movement and Recording in progress. Press RECORD or FREE again to STOP. ***')
        
        # --- CORRECTED REMOVAL LOGIC ---
        if os.path.exists(self.bag_file_name):
            self.get_logger().info(f'Removing existing bag item: {self.bag_file_name}')
            if os.path.isdir(self.bag_file_name):
                # Remove the entire directory for ros2 bag
                shutil.rmtree(self.bag_file_name) 
            else:
                # Remove a file if it exists (for older rosbag versions or misuse)
                os.remove(self.bag_file_name)
        # ------------------------------------

        # Command to record the /cmd_vel topic
        cmd_string = f'ros2 bag record -o {self.bag_file_name} {self.cmd_vel_topic}'
        
        try:
            self.record_process = subprocess.Popen(
                cmd_string, 
                shell=True, 
                executable='/bin/bash', 
                # Use os.setsid to create a new process group, allowing us to kill the whole group (ros2 bag and its children)
                preexec_fn=os.setsid 
            )
            self.get_logger().info(f'Recording command executed: {cmd_string}')
            
        except Exception as e:
            self.get_logger().error(f"FATAL ERROR starting rosbag record: {e}")
            self.current_mode = MODE_FREE
            return


    def stop_record(self):
        """Stops ros2 bag recording and returns to Free mode."""
        if self.record_process:
            # Send SIGINT to the process group to gracefully shut down ros2 bag
            os.killpg(os.getpgid(self.record_process.pid), signal.SIGINT)
            try:
                # Wait for the process to terminate, with a timeout
                self.record_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().error("rosbag record process took too long to terminate.")
            self.record_process = None
            self.current_mode = MODE_FREE
            self.get_logger().info(f'*** RECORD MODE DEACTIVATED. File saved as {self.bag_file_name} ***')
        else:
            self.current_mode = MODE_FREE 


    def start_play(self):
        """Starts ros2 bag playback."""
        # Check if the bag file/directory exists and is not empty
        is_empty_dir = os.path.isdir(self.bag_file_name) and not os.listdir(self.bag_file_name)
        if not os.path.exists(self.bag_file_name) or is_empty_dir:
            self.get_logger().error(f'Trajectory file not found or empty: {self.bag_file_name}. Record a trajectory first!')
            self.get_logger().warn('Play Mode not started. Remaining in FREE MODE.')
            return

        self.current_mode = MODE_PLAY
        self.get_logger().warn('*** PLAY MODE ACTIVATED! Playing Trajectory. Press PLAY or FREE again to INTERRUPT. ***')

        # Command to play the bag file
        cmd_string = f'ros2 bag play {self.bag_file_name}'
        
        try:
            self.play_process = subprocess.Popen(
                cmd_string, 
                shell=True,
                executable='/bin/bash',
                preexec_fn=os.setsid # New process group for easy termination
            )
            self.get_logger().info(f'Playback command executed: {cmd_string}')
            
        except Exception as e:
            self.get_logger().error(f"FATAL ERROR starting rosbag play: {e}")
            self.current_mode = MODE_FREE
            return


    def stop_play(self):
        """Interrupts ros2 bag playback and returns to Free mode."""
        if self.play_process:
            # Send SIGINT to the process group to gracefully shut down ros2 bag play
            os.killpg(os.getpgid(self.play_process.pid), signal.SIGINT)
            try:
                self.play_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().error("rosbag play process took too long to terminate.")
            self.play_process = None
            self.current_mode = MODE_FREE
            self.get_logger().info('*** PLAY MODE DEACTIVATED. Returning to Free Mode. ***')
        else:
            self.current_mode = MODE_FREE 


def main(args=None):
    rclpy.init(args=args)
    controller = ModeController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup upon exit (Ctrl+C)
        controller.get_logger().info('Cleaning up external processes...')
        if controller.record_process:
            controller.stop_record()
        if controller.play_process:
            controller.stop_play()
        
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
