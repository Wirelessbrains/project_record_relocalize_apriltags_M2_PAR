import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import os
import csv
import time

# --- Robot State Constants ---
MODE_FREE = 0
MODE_RECORD = 1
MODE_PLAY = 2

class ModeController(Node):
    """
    Mode Controller using CSV for trajectory Recording and Playback.
    Includes Enable Button logic and WARN log handling.
    """
    def __init__(self):
        super().__init__('mode_controller')
        
        # --- Configuration ---
        self.cmd_vel_topic = '/cmd_vel'
        self.bag_file_name = 'trajetoria_limo.csv' 

        # --- CSV/Playback State Variables ---
        self.current_mode = MODE_FREE
        self.last_buttons = []
        self.csv_file = None      
        self.csv_writer = None    
        self.last_record_time = None
        self.play_timer = None    
        self.trajectory_data = [] 
        self.current_index = 0    
        self.play_start_time = None

        # --- Button Flags (To avoid WARN repetition) ---
        self.record_button_pressed = False
        self.play_button_pressed = False
        self.free_button_pressed = False

        # --- Parameters (Names MUST correspond to the launch file) ---
        self.declare_parameter('axis_linear_x', 1) 
        self.declare_parameter('axis_angular_yaw', 2)
        self.declare_parameter('free_button', 0)   
        self.declare_parameter('record_button', 1) 
        self.declare_parameter('play_button', 4)   
        self.declare_parameter('enable_button', 7) 
        self.declare_parameter('scale_linear_x', 0.8)
        self.declare_parameter('scale_angular_yaw', 0.5)

        # Reading Parameters
        self.axis_linear = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_angular = self.get_parameter('axis_angular_yaw').get_parameter_value().integer_value
        self.button_free = self.get_parameter('free_button').get_parameter_value().integer_value
        self.button_record = self.get_parameter('record_button').get_parameter_value().integer_value
        self.button_play = self.get_parameter('play_button').get_parameter_value().integer_value
        self.button_enable = self.get_parameter('enable_button').get_parameter_value().integer_value 
        self.scale_linear = self.get_parameter('scale_linear_x').get_parameter_value().double_value
        self.scale_angular = self.get_parameter('scale_angular_yaw').get_parameter_value().double_value
        
        # --- Subscriptions and Publishers ---
        self.joy_subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        self.get_logger().warn('*** ModeController initialized in FREE MODE (CSV Strategy). ***') 

    def joy_callback(self, joy_msg):
        
        current_buttons = list(joy_msg.buttons)
        
        # --- ROBUSTNESS: Initialize last_buttons and check length ---
        if not self.last_buttons or len(self.last_buttons) != len(current_buttons):
            self.last_buttons = current_buttons
            return

        # --- BUTTON DETECTION LOGIC WITH FLAGS (SOLUTION TO WARN SPAM) ---
        
        # 1. FREE MODE BUTTON
        free_button_state = current_buttons[self.button_free] == 1
        if free_button_state and not self.free_button_pressed:
            if self.current_mode != MODE_FREE:
                self.stop_record() 
                self.stop_play()   
                self.current_mode = MODE_FREE
                self.get_logger().warn('*** FREE MODE ACTIVATED by button! ***')
            self.free_button_pressed = True
        elif not free_button_state:
            self.free_button_pressed = False
        
        # 2. RECORD BUTTON
        record_button_state = current_buttons[self.button_record] == 1
        if record_button_state and not self.record_button_pressed:
            if self.current_mode == MODE_FREE:
                self.start_record()
            elif self.current_mode == MODE_RECORD:
                self.stop_record()
            self.record_button_pressed = True
        elif not record_button_state:
            self.record_button_pressed = False
        
        # 3. PLAY BUTTON
        play_button_state = current_buttons[self.button_play] == 1
        if play_button_state and not self.play_button_pressed:
            if self.current_mode == MODE_FREE:
                self.start_play()
            elif self.current_mode == MODE_PLAY:
                self.stop_play()
            self.play_button_pressed = True
        elif not play_button_state:
            self.play_button_pressed = False

        # Publish movement commands (Teleop)
        if self.current_mode == MODE_FREE or self.current_mode == MODE_RECORD:
            self.publish_teleop_twist(joy_msg)
        
        # **IMPORTANT:** Update button state for the next iteration
        self.last_buttons = current_buttons

    # --- CSV RECORDING ---
    
    def start_record(self):
        """Starts CSV recording."""
        if os.path.exists(self.bag_file_name):
            try:
                os.remove(self.bag_file_name) 
            except Exception as e:
                self.get_logger().error(f"Could not remove old CSV file: {e}")
                return

        try:
            self.csv_file = open(self.bag_file_name, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['relative_time', 'linear_x', 'angular_z'])
            
            self.last_record_time = self.get_clock().now().nanoseconds
            
            self.current_mode = MODE_RECORD
            self.get_logger().warn(f'*** RECORD MODE ACTIVATED! Movement and Recording in progress. File: {self.bag_file_name} ***') 

        except Exception as e:
            self.get_logger().error(f"FATAL ERROR starting CSV recording: {e}")
            self.current_mode = MODE_FREE

    def stop_record(self):
        """Closes the CSV file and finishes recording."""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.current_mode = MODE_FREE
            self.get_logger().warn(f'*** RECORD MODE DEACTIVATED. CSV trajectory saved. ***') 
        else:
            self.current_mode = MODE_FREE

    def publish_teleop_twist(self, joy_msg):
        """Calculates, publishes, and records the Twist message (if in RECORD mode)."""
        
        twist = Twist()
        
        # --- Enable Button Logic ---
        is_enabled = len(joy_msg.buttons) > self.button_enable and joy_msg.buttons[self.button_enable] == 1

        if is_enabled:
            # Apply scale read from parameters
            twist.linear.x = float(joy_msg.axes[self.axis_linear] * self.scale_linear) 
            twist.angular.z = float(joy_msg.axes[self.axis_angular] * self.scale_angular)
        else:
            # Zero movement if the ENABLE button is not pressed
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
            if self.current_mode == MODE_RECORD:
                 # This message will repeat, but it is crucial to know why it is not moving.
                 # We don't convert it to a flag because it is a continuous status message.
                 self.get_logger().warn('Record Mode: Movement zeroed (ENABLE Button not pressed).')

        self.cmd_vel_publisher.publish(twist)

        # CSV RECORDING LOGIC
        if self.current_mode == MODE_RECORD and self.csv_writer:
            current_time = self.get_clock().now().nanoseconds
            
            relative_time_sec = (current_time - self.last_record_time) / 1e9 
            
            self.csv_writer.writerow([
                relative_time_sec, 
                twist.linear.x, 
                twist.angular.z
            ])
            
            self.last_record_time = current_time 
            
    # --- CSV PLAYBACK ---

    def read_trajectory_csv(self):
        """Reads the CSV file, calculates cumulative time, and stores the data."""
        data = []
        try:
            with open(self.bag_file_name, 'r') as f:
                reader = csv.DictReader(f)
                cumulative_time = 0.0
                for row in reader:
                    relative_time = float(row['relative_time'])
                    cumulative_time += relative_time 
                    
                    data.append({
                        'cumulative_time': cumulative_time,
                        'linear_x': float(row['linear_x']),
                        'angular_z': float(row['angular_z'])
                    })
        except Exception as e:
            self.get_logger().error(f"Error reading CSV file: {e}")
        return data

    def start_play(self):
        """Starts the playback of commands from the CSV."""
        if not os.path.exists(self.bag_file_name):
            self.get_logger().error(f"CSV trajectory file not found: {self.bag_file_name}. Record first!")
            self.current_mode = MODE_FREE
            return

        self.trajectory_data = self.read_trajectory_csv()
        if not self.trajectory_data:
            self.get_logger().error("CSV file is empty or unreadable.")
            self.current_mode = MODE_FREE
            return

        self.current_mode = MODE_PLAY
        self.current_index = 0
        self.play_start_time = self.get_clock().now().nanoseconds
        
        self.play_timer = self.create_timer(0.02, self.play_trajectory_csv) 
        self.get_logger().warn('*** PLAY MODE ACTIVATED! Reproducing Trajectory. ***') 

    def play_trajectory_csv(self):
        """Timer callback that publishes synchronized commands."""
        
        if self.current_index >= len(self.trajectory_data):
            self.get_logger().warn('Trajectory execution finished automatically.') 
            self.stop_play()
            return

        elapsed_time_sec = (self.get_clock().now().nanoseconds - self.play_start_time) / 1e9
        
        while (self.current_index < len(self.trajectory_data) and
               elapsed_time_sec >= self.trajectory_data[self.current_index]['cumulative_time']):
                
            command = self.trajectory_data[self.current_index]
            
            twist = Twist()
            twist.linear.x = command['linear_x']
            twist.angular.z = command['angular_z']
            self.cmd_vel_publisher.publish(twist)
            
            self.current_index += 1


    def stop_play(self):
        """Interrupts playback and returns to Free mode."""
        if self.play_timer:
            self.play_timer.cancel()
            self.play_timer = None
        
        self.cmd_vel_publisher.publish(Twist()) 
        
        self.current_mode = MODE_FREE
        self.get_logger().warn('*** PLAY MODE DEACTIVATED. Returning to Free Mode. ***') 

    # --- Final Cleanup ---
    def on_shutdown(self):
        self.stop_record() 
        self.stop_play()


def main(args=None):
    rclpy.init(args=args)
    node = ModeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
