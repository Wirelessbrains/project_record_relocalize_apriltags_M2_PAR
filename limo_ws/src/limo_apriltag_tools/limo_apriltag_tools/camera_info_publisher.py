import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml
import os

class SynchronizedCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # 1) Declare and read parameters.
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('image_output_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('relay_image', False)
        self.declare_parameter('optical_frame', '')
        
        # File path comes from launch/CLI parameter.
        calibration_file_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        image_output_topic = self.get_parameter('image_output_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        relay_image = self.get_parameter('relay_image').get_parameter_value().bool_value
        self.optical_frame = self.get_parameter('optical_frame').get_parameter_value().string_value
        self._optical_published = False
        self._static_broadcaster = StaticTransformBroadcaster(self)

        if not calibration_file_path:
            self.get_logger().error("Missing required parameter: 'calibration_file'.")
            return

        # 2) Load calibration message.
        self.camera_info_msg = self._load_camera_info_from_yaml(calibration_file_path)
        
        if self.camera_info_msg is None:
            self.get_logger().error("Node startup aborted: invalid calibration data.")
            return

        # Publish with RELIABLE QoS to match consumers that require reliability
        # (e.g., apriltag/image_transport in some real-camera setups).
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 3) Configure publishers.
        self.pub_camera_info = self.create_publisher(
            CameraInfo,
            camera_info_topic,
            pub_qos,
        )
        self.pub_image = None
        if relay_image:
            self.pub_image = self.create_publisher(
                Image,
                image_output_topic,
                pub_qos,
            )

        # 4) Subscribe to image topic for timestamp sync.
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Synchronized CameraInfo publisher started using {calibration_file_path}"
        )
        if relay_image:
            self.get_logger().info(
                f"Relaying image {image_topic} -> {image_output_topic}"
            )


    def _load_camera_info_from_yaml(self, file_path):
        """Load and populate CameraInfo from YAML."""
        
        try:
            with open(file_path, 'r') as file:
                calib_data = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Calibration file not found: {file_path}")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            return None

        cinfo = CameraInfo()
        
        cinfo.height = calib_data['image_height']
        cinfo.width = calib_data['image_width']
        cinfo.distortion_model = calib_data['distortion_model']
        cinfo.k = calib_data['camera_matrix']['data']
        cinfo.d = calib_data['distortion_coefficients']['data']
        cinfo.r = calib_data['rectification_matrix']['data']
        cinfo.p = calib_data['projection_matrix']['data']
        
        cinfo.header.frame_id = 'camera_link_optical'

        self.get_logger().info("Camera calibration loaded successfully (PyYAML).")
        return cinfo


    def _image_callback(self, img_msg):
        """Image callback: copies timestamp/frame and publishes CameraInfo."""
        
        if self.camera_info_msg is None:
            return

        # Keep CameraInfo synchronized to image timestamp.
        self.camera_info_msg.header.stamp = img_msg.header.stamp
        if not self.optical_frame:
            self.optical_frame = f"{img_msg.header.frame_id}_optical"
        self.camera_info_msg.header.frame_id = self.optical_frame
        if not self._optical_published:
            if img_msg.header.frame_id != self.optical_frame:
                t = TransformStamped()
                t.header.stamp = img_msg.header.stamp
                t.header.frame_id = img_msg.header.frame_id
                t.child_frame_id = self.optical_frame
                t.transform.rotation.x = -0.5
                t.transform.rotation.y = 0.5
                t.transform.rotation.z = -0.5
                t.transform.rotation.w = 0.5
                self._static_broadcaster.sendTransform(t)
            self._optical_published = True
        self.pub_camera_info.publish(self.camera_info_msg)
        if self.pub_image is not None:
            img_msg.header.frame_id = self.optical_frame
            self.pub_image.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedCameraInfoPublisher() 
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
