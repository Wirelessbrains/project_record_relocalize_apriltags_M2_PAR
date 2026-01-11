#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

class TrajectorySimulator(Node):
    def __init__(self, scenario_file):
        super().__init__('trajectory_simulator')
        
        # Load Scenario
        with open(scenario_file, 'r') as f:
            self.scenario = yaml.safe_load(f)

        self.world_frame = self.scenario.get('world_frame', 'world')
        
        # Publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, '/camera_path', 10)
        # self.marker_pub = self.create_publisher(MarkerArray, '/world_tags', 10) # Removido a pedido
        
        # Setup Simulation State
        start_pos = self.scenario['camera_path']['start_position']
        self.current_pos = np.array([start_pos['x'], start_pos['y'], start_pos['z']])
        self.velocity = np.array([
            self.scenario['camera_path']['velocity_linear']['x'],
            self.scenario['camera_path']['velocity_linear']['y'],
            self.scenario['camera_path']['velocity_linear']['z']
        ])
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame
        
        # Timer
        self.fps = 10.0
        self.dt = 1.0 / self.fps
        self.timer = self.create_timer(self.dt, self.update)
        self.elapsed_time = 0.0
        self.total_time = self.scenario['camera_path']['total_time']

        # Publish Static World Tags (Como TFs fixas para visualização dos eixos)
        self.publish_static_tags()

        self.get_logger().info("Simulation Started. Visualization topics: /camera_path, /tf")

    def publish_static_tags(self):
        """Publica TFs estáticas para as tags fixas no mundo (Visualizar Eixos)"""
        static_transforms = []
        for tag_name, data in self.scenario['tags'].items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame
            t.child_frame_id = f"fixed_{tag_name}" # Ex: fixed_tag36h11:0
            
            t.transform.translation.x = data['position']['x']
            t.transform.translation.y = data['position']['y']
            t.transform.translation.z = data['position']['z']
            t.transform.rotation.x = data['orientation']['x']
            t.transform.rotation.y = data['orientation']['y']
            t.transform.rotation.z = data['orientation']['z']
            t.transform.rotation.w = data['orientation']['w']
            
            static_transforms.append(t)
        
        self.static_broadcaster.sendTransform(static_transforms)
        self.get_logger().info(f"Published {len(static_transforms)} static tag transforms.")

    def update(self):
        if self.elapsed_time > self.total_time:
            self.get_logger().info("Simulation Finished.")
            # Keep running to keep TFs alive for RViz, but stop moving
            # return 
        else:
            # Update Position
            self.current_pos += self.velocity * self.dt
            self.elapsed_time += self.dt

        now = self.get_clock().now().to_msg()

        # 1. Publish Camera TF (Robot Base)
        # World -> Camera Link (Base do robo/camera)
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = self.world_frame
        t_base.child_frame_id = "camera_link"
        t_base.transform.translation.x = self.current_pos[0]
        t_base.transform.translation.y = self.current_pos[1]
        t_base.transform.translation.z = self.current_pos[2]
        t_base.transform.rotation.w = 1.0 # Sem rotação (alinhado com mundo, X-frente)
        self.tf_broadcaster.sendTransform(t_base)
        
        # World -> Camera Optical (Para cálculos de visão)
        # Optical frame: Z-frente, X-direita, Y-baixo
        # Rotação de Camera Link (X-frente) para Optical: Roll=-90, Yaw=-90
        # Quat aprox: -0.5, 0.5, -0.5, 0.5
        optical_rot = R.from_quat([-0.5, 0.5, -0.5, 0.5])
        
        # Matriz T_world_optical
        T_world_link = np.eye(4)
        T_world_link[:3, 3] = self.current_pos
        # Assumindo link alinhado com mundo (rot identity)
        
        T_link_optical = np.eye(4)
        T_link_optical[:3, :3] = optical_rot.as_matrix()
        
        T_world_optical = T_world_link @ T_link_optical
        
        # Publish Optical Frame TF (Static relative to link, but dynamic relative to world)
        t_opt = TransformStamped()
        t_opt.header.stamp = now
        t_opt.header.frame_id = "camera_link"
        t_opt.child_frame_id = "camera_optical_frame"
        t_opt.transform.rotation.x = -0.5
        t_opt.transform.rotation.y = 0.5
        t_opt.transform.rotation.z = -0.5
        t_opt.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t_opt) # Link -> Optical

        # 2. Update Path Visualization
        pose = PoseStamped()
        pose.header = t_base.header
        pose.pose.position.x = self.current_pos[0]
        pose.pose.position.y = self.current_pos[1]
        pose.pose.position.z = self.current_pos[2]
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        # 3. Simulate Tag Detections
        # Check FOV and Range
        T_optical_world = np.linalg.inv(T_world_optical) # Inversa para pegar posições no frame da camera
        
        max_dist = self.scenario['camera_path'].get('max_distance', 4.0)
        fov_deg = self.scenario['camera_path'].get('fov_angle', 60.0)
        fov_rad = np.radians(fov_deg / 2.0) # Metade do angulo para cada lado

        for tag_name, data in self.scenario['tags'].items():
            # Tag pose in World
            tp = data['position']
            to = data['orientation']
            tag_pos_world = np.array([tp['x'], tp['y'], tp['z']])
            
            # Vector Camera -> Tag (in World)
            vec_cam_tag = tag_pos_world - self.current_pos
            dist = np.linalg.norm(vec_cam_tag)
            
            # Simple FOV check: Dot product with Camera Forward Vector (X-axis of camera_link)
            # Robot is moving in +X, camera points +X
            cam_forward = np.array([1.0, 0.0, 0.0]) 
            vec_norm = vec_cam_tag / dist if dist > 0 else np.array([1,0,0])
            angle = np.arccos(np.dot(cam_forward, vec_norm))

            if dist < max_dist and angle < fov_rad:
                # DETECTED! Calculate relative pose
                # T_optical_tag = T_optical_world * T_world_tag
                
                r_tag = R.from_quat([to['x'], to['y'], to['z'], to['w']])
                T_world_tag = np.eye(4)
                T_world_tag[:3, :3] = r_tag.as_matrix()
                T_world_tag[:3, 3] = tag_pos_world
                
                T_optical_tag = T_optical_world @ T_world_tag
                
                rel_pos = T_optical_tag[:3, 3]
                rel_quat = R.from_matrix(T_optical_tag[:3, :3]).as_quat()
                
                # Publish Detection TF
                t_det = TransformStamped()
                t_det.header.stamp = now
                t_det.header.frame_id = "camera_optical_frame"
                t_det.child_frame_id = tag_name # ex: tag36h11:0
                
                t_det.transform.translation.x = rel_pos[0]
                t_det.transform.translation.y = rel_pos[1]
                t_det.transform.translation.z = rel_pos[2]
                t_det.transform.rotation.x = rel_quat[0]
                t_det.transform.rotation.y = rel_quat[1]
                t_det.transform.rotation.z = rel_quat[2]
                t_det.transform.rotation.w = rel_quat[3]
                
                self.tf_broadcaster.sendTransform(t_det)
                
                # Debug print only on first detection or interval
                # self.get_logger().info(f"Detecting {tag_name} at dist {dist:.2f}m")
            
        # Republish markers occasionally (Removido pois usamos Static TF agora)
        # if int(self.elapsed_time * 10) % 20 == 0:
        #    self.publish_world_markers()

def main(args=None):
    # Instala scipy se necessario
    try:
        import scipy
        from scipy.spatial.transform import Rotation as R
    except ImportError:
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy"])
        import scipy
        from scipy.spatial.transform import Rotation as R

    rclpy.init(args=args)
    
    scenario_path = 'trajectory_scenario.yaml'
    if len(sys.argv) > 1:
        scenario_path = sys.argv[1]

    node = TrajectorySimulator(scenario_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
