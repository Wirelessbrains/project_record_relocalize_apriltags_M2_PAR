#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import yaml
import time
import sys
import os

class DatasetRecorder(Node):
    def __init__(self, output_file):
        super().__init__('dataset_recorder')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.output_file = output_file
        self.dataset = {
            'info': {
                'date': time.strftime("%Y-%m-%d %H:%M:%S"),
                'description': 'Trajectory dataset with Ground Truth camera pose and Tag detections.'
            },
            'frames': []
        }
        
        self.start_time = self.get_clock().now().nanoseconds
        
        # Gravar a 20Hz (mesma taxa da simulação)
        self.timer = self.create_timer(0.05, self.record_step)
        self.get_logger().info(f"Gravando dataset em: {output_file}")
        self.get_logger().info("Pressione Ctrl+C para salvar e sair.")

    def record_step(self):
        # 1. Obter Ground Truth (World -> Camera Optical)
        # Usamos 'camera_optical_frame' porque é o referencial das detecções
        try:
            # Lookup World -> Camera
            tf_gt = self.tf_buffer.lookup_transform(
                'world', 'camera_optical_frame', rclpy.time.Time())
            
            frame_data = {
                'timestamp': tf_gt.header.stamp.sec + tf_gt.header.stamp.nanosec * 1e-9,
                'ground_truth_pose': self.tf_to_dict(tf_gt.transform),
                'detections': {}
            }
            
            # 2. Verificar Detecções (Camera -> Tags)
            # Procuramos por transformações para todas as tags possíveis (0 a 10)
            # Se o tf_buffer tiver a transformada RECENTE, consideramos detectada
            for i in range(10):
                tag_frame = f"tag36h11:{i}"
                if self.tf_buffer.can_transform('camera_optical_frame', tag_frame, rclpy.time.Time()):
                    try:
                        # Pega a TF mais recente
                        tf_det = self.tf_buffer.lookup_transform(
                            'camera_optical_frame', tag_frame, rclpy.time.Time())
                        
                        # Verifica se é recente (menos de 0.2s de atraso)
                        now_sec = self.get_clock().now().nanoseconds * 1e-9
                        tf_sec = tf_det.header.stamp.sec + tf_det.header.stamp.nanosec * 1e-9
                        
                        if (now_sec - tf_sec) < 0.2:
                            frame_data['detections'][tag_frame] = self.tf_to_dict(tf_det.transform)
                            # self.get_logger().info(f"Gravado {tag_frame}")
                    except Exception:
                        pass
            
            self.dataset['frames'].append(frame_data)
            
        except Exception as e:
            # Pode falhar no inicio enquanto TFs não chegam
            pass

    def tf_to_dict(self, transform):
        return {
            'position': {
                'x': transform.translation.x,
                'y': transform.translation.y,
                'z': transform.translation.z
            },
            'orientation': {
                'x': transform.rotation.x,
                'y': transform.rotation.y,
                'z': transform.rotation.z,
                'w': transform.rotation.w
            }
        }

    def save_dataset(self):
        self.get_logger().info(f"Salvando {len(self.dataset['frames'])} frames no arquivo...")
        with open(self.output_file, 'w') as f:
            yaml.dump(self.dataset, f, default_flow_style=False)
        self.get_logger().info("Arquivo salvo com sucesso.")

def main(args=None):
    rclpy.init(args=args)
    
    output_path = 'trajectory_dataset.yaml'
    if len(sys.argv) > 1:
        output_path = sys.argv[1]
        
    recorder = DatasetRecorder(output_path)
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.save_dataset()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
