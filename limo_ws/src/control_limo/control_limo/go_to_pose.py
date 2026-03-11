import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped, Pose, Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult

try:
    from gazebo_msgs.msg import ModelStates
except ImportError:
    ModelStates = None

class GoToPoseNode(Node):
    def __init__(self):
        super().__init__('go_to_pose_node')

        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('ki_linear', 0.0)
        self.declare_parameter('kd_linear', 0.0)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('ki_angular', 0.0)
        self.declare_parameter('kd_angular', 0.0)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('enable_reverse_motion', False)
        self.declare_parameter('reverse_yaw_bias', 0.1)

        self.declare_parameter('use_gazebo_model_states', True)
        self.declare_parameter('pose_source', 'model_states')
        self.declare_parameter('pose_topic', '/tag_only_pose')
        self.declare_parameter('robot_model_name', 'limo_description')
        self.declare_parameter('model_states_topic', '/model_states')
        self.declare_parameter('debug_log', False)
        self.declare_parameter('pid_angle_stop_threshold', 0.25)
        self.declare_parameter('path_frame_id', 'odom')
        self.declare_parameter('path_min_distance', 0.05)

        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.ki_linear = self.get_parameter('ki_linear').get_parameter_value().double_value
        self.kd_linear = self.get_parameter('kd_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.ki_angular = self.get_parameter('ki_angular').get_parameter_value().double_value
        self.kd_angular = self.get_parameter('kd_angular').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.enable_reverse_motion = self.get_parameter('enable_reverse_motion').get_parameter_value().bool_value
        self.reverse_yaw_bias = self.get_parameter('reverse_yaw_bias').get_parameter_value().double_value
        self.use_gazebo_model_states = self.get_parameter('use_gazebo_model_states').get_parameter_value().bool_value
        self.pose_source = self.get_parameter('pose_source').get_parameter_value().string_value.lower()
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.robot_model_name = self.get_parameter('robot_model_name').get_parameter_value().string_value
        self.model_states_topic = self.get_parameter('model_states_topic').get_parameter_value().string_value
        self.debug_log = self.get_parameter('debug_log').get_parameter_value().bool_value
        self.pid_angle_stop_threshold = self.get_parameter('pid_angle_stop_threshold').get_parameter_value().double_value
        self.path_frame_id = self.get_parameter('path_frame_id').get_parameter_value().string_value
        self.path_min_distance = self.get_parameter('path_min_distance').get_parameter_value().double_value
        self.declare_parameter('require_final_orientation', True)
        self.require_final_orientation = self.get_parameter('require_final_orientation').get_parameter_value().bool_value
        self.declare_parameter('startup_delay', 0.0)
        self.startup_delay = self.get_parameter('startup_delay').get_parameter_value().double_value

        self.subscription_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)

        self.subscription_pose = None
        self.subscription_model_states = None
        self._setup_pose_subscription(self.use_gazebo_model_states)

        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.path_frame_id
        self.last_path_pose: PoseStamped | None = None

        self.current_pose = None
        self.goal_pose = None
        self.last_time = self.get_clock().now()
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop)
        self.add_on_set_parameters_callback(self.on_parameter_change)
        self.get_logger().info('GoToPose Node has been started.')

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        self.get_logger().info(f'New goal received: x={msg.pose.position.x}, y={msg.pose.position.y}')

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self._maybe_append_path()

    def pose_only_callback(self, msg: Pose):
        self.current_pose = msg
        self._maybe_append_path()

    def model_states_callback(self, msg: ModelStates):
        if self.robot_model_name in msg.name:
            idx = msg.name.index(self.robot_model_name)
            self.current_pose = msg.pose[idx]
            self._maybe_append_path()
        else:
            self.get_logger().warn(
                f'Model "{self.robot_model_name}" not found in /gazebo/model_states. '
                f'Available names: {", ".join(msg.name)}',
                throttle_duration_sec=5.0
            )

    def _setup_pose_subscription(self, use_gazebo: bool):
        if self.subscription_pose:
            self.destroy_subscription(self.subscription_pose)
            self.subscription_pose = None
        if self.subscription_model_states:
            self.destroy_subscription(self.subscription_model_states)
            self.subscription_model_states = None

        if self.pose_source == 'tag_only':
            self.subscription_pose = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                10)
            return
        if self.pose_source == 'pose':
            self.subscription_pose = self.create_subscription(
                Pose,
                self.pose_topic,
                self.pose_only_callback,
                10)
            return

        if ModelStates is None:
            self.get_logger().warn(
                'gazebo_msgs is not available; falling back to PoseStamped subscription '
                f'on {self.pose_topic}.',
                throttle_duration_sec=5.0
            )
            self.subscription_pose = self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self.pose_callback,
                10)
            return

        self.subscription_model_states = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.model_states_callback,
            10)

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.orientation
        t3 = +2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        t4 = +1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(t3, t4)
        return yaw

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _maybe_append_path(self):
        if self.current_pose is None:
            return
        pose_st = PoseStamped()
        pose_st.header.frame_id = self.path_frame_id
        pose_st.header.stamp = self.get_clock().now().to_msg()
        pose_st.pose = self.current_pose

        if self.last_path_pose is None:
            self.path_msg.header.frame_id = self.path_frame_id
            self.path_msg.poses.append(pose_st)
            self.last_path_pose = pose_st
            self.path_pub.publish(self.path_msg)
            return

        dx = pose_st.pose.position.x - self.last_path_pose.pose.position.x
        dy = pose_st.pose.position.y - self.last_path_pose.pose.position.y
        if math.hypot(dx, dy) >= self.path_min_distance:
            self.path_msg.poses.append(pose_st)
            self.last_path_pose = pose_st
            self.path_pub.publish(self.path_msg)

    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            if self.current_pose is None:
                self.get_logger().warn('Waiting for pose.', throttle_duration_sec=5.0)
            if self.goal_pose is None:
                self.get_logger().warn('Waiting for goal on /goal_pose.', throttle_duration_sec=5.0)
            return

        if self.startup_delay > 0:
            sim_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if sim_elapsed < self.startup_delay:
                return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0:
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y

        distance = math.sqrt(dx**2 + dy**2)
        heading_to_target = math.atan2(dy, dx)
        target_yaw_goal = self.get_yaw_from_pose(self.goal_pose)
        target_yaw = heading_to_target
        use_reverse = False

        if self.enable_reverse_motion and distance > self.distance_tolerance:
            reverse_heading = self._normalize_angle(heading_to_target + math.pi)
            forward_alignment = abs(self._normalize_angle(target_yaw_goal - heading_to_target))
            reverse_alignment = abs(self._normalize_angle(target_yaw_goal - reverse_heading))
            use_reverse = reverse_alignment + self.reverse_yaw_bias < forward_alignment
            target_yaw = reverse_heading if use_reverse else heading_to_target

        align_distance = max(min(self.distance_tolerance, 0.05), 0.02)
        if self.require_final_orientation and distance <= align_distance:
            target_yaw = target_yaw_goal
        current_yaw = self.get_yaw_from_pose(self.current_pose)

        angle_error = self._normalize_angle(target_yaw - current_yaw)
        if abs(angle_error) < 0.03:
            angle_error = 0.0

        cmd_vel = Twist()

        force_zero_linear = False
        if distance < self.distance_tolerance:
            cmd_vel.linear.x = 0.0
            force_zero_linear = True
            if not self.require_final_orientation:
                self.reset_pid()
                self.publisher_cmd_vel.publish(cmd_vel)
                return
            if abs(angle_error) < self.angle_tolerance:
                self.reset_pid()
                self.publisher_cmd_vel.publish(cmd_vel)
                return

        near_target = distance < align_distance
        drive_sign = -1.0 if use_reverse and not force_zero_linear else 1.0
        signed_distance = drive_sign * distance
        dist_for_pid = 0.0 if force_zero_linear else signed_distance * (0.3 if near_target else 1.0)

        self.integral_angular += angle_error * dt
        d_angle = (angle_error - self.prev_angular_error) / dt if dt > 0 else 0.0
        angular_cmd = (self.kp_angular * angle_error +
                       self.ki_angular * self.integral_angular +
                       self.kd_angular * d_angle)
        self.prev_angular_error = angle_error

        self.integral_linear += dist_for_pid * dt
        d_linear = (dist_for_pid - self.prev_linear_error) / dt if dt > 0 else 0.0
        linear_cmd = (self.kp_linear * dist_for_pid +
                      self.ki_linear * self.integral_linear +
                      self.kd_linear * d_linear)
        self.prev_linear_error = dist_for_pid

        angle_stop_active = abs(angle_error) > self.pid_angle_stop_threshold and distance > align_distance * 2.0
        if angle_stop_active:
            linear_cmd = 0.0
            self.integral_linear = 0.0

        if near_target and force_zero_linear:
            linear_cmd = 0.0

        cmd_vel.linear.x = max(min(linear_cmd, self.max_linear_speed), -self.max_linear_speed)
        cmd_vel.angular.z = max(min(angular_cmd, self.max_angular_speed), -self.max_angular_speed)

        self.publisher_cmd_vel.publish(cmd_vel)
        if self.debug_log:
            self.get_logger().info(
                (f'PID dist={distance:.3f} ang_err={angle_error:.3f} '
                 f'cmd_lin={cmd_vel.linear.x:.3f} cmd_ang={cmd_vel.angular.z:.3f}'),
                throttle_duration_sec=1.0
            )

    def reset_pid(self):
        self.integral_linear = 0.0
        self.integral_angular = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

    def on_parameter_change(self, params):
        """Allow updating gains/limits/tolerances and pose source at runtime."""
        for param in params:
            if param.name == 'kp_linear':
                self.kp_linear = float(param.value)
            elif param.name == 'ki_linear':
                self.ki_linear = float(param.value)
            elif param.name == 'kd_linear':
                self.kd_linear = float(param.value)
            elif param.name == 'kp_angular':
                self.kp_angular = float(param.value)
            elif param.name == 'ki_angular':
                self.ki_angular = float(param.value)
            elif param.name == 'kd_angular':
                self.kd_angular = float(param.value)
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = float(param.value)
            elif param.name == 'angle_tolerance':
                self.angle_tolerance = float(param.value)
            elif param.name == 'max_linear_speed':
                self.max_linear_speed = float(param.value)
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = float(param.value)
            elif param.name == 'enable_reverse_motion':
                self.enable_reverse_motion = bool(param.value)
            elif param.name == 'reverse_yaw_bias':
                self.reverse_yaw_bias = max(0.0, float(param.value))
            elif param.name == 'debug_log':
                self.debug_log = bool(param.value)
            elif param.name == 'robot_model_name':
                self.robot_model_name = str(param.value)
            elif param.name == 'model_states_topic':
                new_topic = str(param.value)
                if new_topic != self.model_states_topic:
                    self.model_states_topic = new_topic
                    if self.use_gazebo_model_states:
                        self._setup_pose_subscription(True)
            elif param.name == 'use_gazebo_model_states':
                use_gazebo = bool(param.value)
                if use_gazebo != self.use_gazebo_model_states:
                    self.use_gazebo_model_states = use_gazebo
                    self._setup_pose_subscription(self.use_gazebo_model_states)
            elif param.name == 'pose_source':
                src = str(param.value).lower()
                if src != self.pose_source:
                    self.pose_source = src
                    self._setup_pose_subscription(self.use_gazebo_model_states)
            elif param.name == 'pose_topic':
                topic = str(param.value)
                if topic != self.pose_topic:
                    self.pose_topic = topic
                    if self.pose_source in ('tag_only', 'pose'):
                        self._setup_pose_subscription(self.use_gazebo_model_states)
            elif param.name == 'require_final_orientation':
                self.require_final_orientation = bool(param.value)
            elif param.name == 'pid_angle_stop_threshold':
                self.pid_angle_stop_threshold = max(0.0, float(param.value))
            elif param.name == 'startup_delay':
                self.startup_delay = max(0.0, float(param.value))

        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
