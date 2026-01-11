#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import yaml
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class SCurveSimulator(Node):
    def __init__(self, scenario_file):
        super().__init__('s_curve_simulator')
        
        # Load Scenario
        with open(scenario_file, 'r') as f:
            self.scenario = yaml.safe_load(f)

        self.world_frame = self.scenario.get('world_frame', 'world')
        
        # Publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, '/camera_path', 10)
        
        # Simulation Params
        path_cfg = self.scenario['camera_path']
        self.vel_x = path_cfg.get('velocity_x', 0.5)
        self.amplitude = path_cfg.get('amplitude', 1.5)
        self.freq = path_cfg.get('frequency', 0.2)
        self.total_time = path_cfg.get('total_time', 30.0)
        
        self.current_pos = np.array([0.0, 0.0, 0.2]) # Z fixo
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame
        
        # Timer
        self.fps = 20.0 # Mais suave
        self.dt = 1.0 / self.fps
        self.timer = self.create_timer(self.dt, self.update)
        self.elapsed_time = 0.0

        # Publish Static Tags
        self.publish_static_tags()

        self.get_logger().info(f"Simulação em S Iniciada. A={self.amplitude}m, F={self.freq}")

    def publish_static_tags(self):
        static_transforms = []
        for tag_name, data in self.scenario['tags'].items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame
            t.child_frame_id = f"fixed_{tag_name}"
            
            t.transform.translation.x = data['position']['x']
            t.transform.translation.y = data['position']['y']
            t.transform.translation.z = data['position']['z']
            t.transform.rotation.x = data['orientation']['x']
            t.transform.rotation.y = data['orientation']['y']
            t.transform.rotation.z = data['orientation']['z']
            t.transform.rotation.w = data['orientation']['w']
            
            static_transforms.append(t)
        
        self.static_broadcaster.sendTransform(static_transforms)

    def update(self):
        if self.elapsed_time > self.total_time:
            self.get_logger().info("Fim da Simulação.")
            # self.timer.cancel()
            # return 
        else:
            self.elapsed_time += self.dt
            
            # 1. Calcular Posição (Senoide)
            # x = v * t
            x = self.vel_x * self.elapsed_time
            # y = A * sin(f * t)
            y = self.amplitude * math.sin(self.freq * self.elapsed_time)
            
            self.current_pos[0] = x
            self.current_pos[1] = y
            
            # 2. Calcular Yaw (Derivada para apontar na direção do movimento)
            # dx/dt = v
            # dy/dt = A * f * cos(f * t)
            vx = self.vel_x
            vy = self.amplitude * self.freq * math.cos(self.freq * self.elapsed_time)
            yaw = math.atan2(vy, vx)
            
            # Quaternion da Base do Robô
            q_base = R.from_euler('z', yaw).as_quat()

            now = self.get_clock().now().to_msg()

            # 3. Publicar TF da Base
            t_base = TransformStamped()
            t_base.header.stamp = now
            t_base.header.frame_id = self.world_frame
            t_base.child_frame_id = "camera_link"
            t_base.transform.translation.x = x
            t_base.transform.translation.y = y
            t_base.transform.translation.z = self.current_pos[2]
            t_base.transform.rotation.x = q_base[0]
            t_base.transform.rotation.y = q_base[1]
            t_base.transform.rotation.z = q_base[2]
            t_base.transform.rotation.w = q_base[3]
            self.tf_broadcaster.sendTransform(t_base)
            
            # 4. Publicar TF Óptica (Filha da Base)
            # Rotação fixa para alinhar Z-frente
            t_opt = TransformStamped()
            t_opt.header.stamp = now
            t_opt.header.frame_id = "camera_link"
            t_opt.child_frame_id = "camera_optical_frame"
            t_opt.transform.rotation.x = -0.5
            t_opt.transform.rotation.y = 0.5
            t_opt.transform.rotation.z = -0.5
            t_opt.transform.rotation.w = 0.5
            self.tf_broadcaster.sendTransform(t_opt)
            
            # 5. Visualizar Caminho
            pose = PoseStamped()
            pose.header = t_base.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.current_pos[2]
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)
            
            # 6. Simular Detecções
            self.simulate_vision(x, y, self.current_pos[2], q_base, now)

    def simulate_vision(self, rx, ry, rz, r_quat, time_stamp):
        # Matriz World -> Base
        r_mat_base = R.from_quat(r_quat).as_matrix()
        T_world_base = np.eye(4)
        T_world_base[:3, :3] = r_mat_base
        T_world_base[:3, 3] = [rx, ry, rz]
        
        # Matriz Base -> Optical
        r_mat_opt = R.from_quat([-0.5, 0.5, -0.5, 0.5]).as_matrix()
        T_base_opt = np.eye(4)
        T_base_opt[:3, :3] = r_mat_opt
        
        # T_world_optical
        T_world_opt = T_world_base @ T_base_opt
        T_opt_world = np.linalg.inv(T_world_opt)
        
        max_dist = self.scenario['camera_path'].get('max_distance', 3.0)
        fov_half_rad = np.radians(self.scenario['camera_path'].get('fov_angle', 60.0) / 2.0)
        
        for tag_name, data in self.scenario['tags'].items():
            tag_pos_world = np.array([data['position']['x'], data['position']['y'], data['position']['z']])
            
            # Posição da Tag no Frame Óptico
            tag_pos_opt = (T_opt_world @ np.append(tag_pos_world, 1.0))[:3]
            
            dist = np.linalg.norm(tag_pos_opt)
            
            # Lógica de Visão:
            # 1. Distância menor que max
            # 2. Z > 0 (na frente da câmera)
            # 3. Ângulo com eixo Z menor que FOV
            if dist < max_dist and tag_pos_opt[2] > 0.1:
                angle = math.atan2(math.sqrt(tag_pos_opt[0]**2 + tag_pos_opt[1]**2), tag_pos_opt[2])
                
                if angle < fov_half_rad:
                    # Detectado! Publicar TF
                    r_tag_world = R.from_quat([
                        data['orientation']['x'], data['orientation']['y'], 
                        data['orientation']['z'], data['orientation']['w']
                    ])
                    T_world_tag = np.eye(4)
                    T_world_tag[:3, :3] = r_tag_world.as_matrix()
                    T_world_tag[:3, 3] = tag_pos_world
                    
                    T_opt_tag = T_opt_world @ T_world_tag
                    
                    rel_pos = T_opt_tag[:3, 3]
                    rel_quat = R.from_matrix(T_opt_tag[:3, :3]).as_quat()
                    
                    t_det = TransformStamped()
                    t_det.header.stamp = time_stamp
                    t_det.header.frame_id = "camera_optical_frame"
                    t_det.child_frame_id = tag_name
                    t_det.transform.translation.x = rel_pos[0]
                    t_det.transform.translation.y = rel_pos[1]
                    t_det.transform.translation.z = rel_pos[2]
                    t_det.transform.rotation.x = rel_quat[0]
                    t_det.transform.rotation.y = rel_quat[1]
                    t_det.transform.rotation.z = rel_quat[2]
                    t_det.transform.rotation.w = rel_quat[3]
                    
                    self.tf_broadcaster.sendTransform(t_det)

def main(args=None):
    # Check scipy
    try:
        import scipy
    except ImportError:
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy"])

    rclpy.init(args=args)
    
    scenario_path = 's_curve_scenario.yaml'
    if len(sys.argv) > 1:
        scenario_path = sys.argv[1]

    node = SCurveSimulator(scenario_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
