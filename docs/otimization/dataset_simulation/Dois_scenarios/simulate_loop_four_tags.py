#!/usr/bin/env python3
import math
import sys
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R


class LoopFourTagsSimulator(Node):
    def __init__(self, scenario_file: str):
        super().__init__("loop_four_tags_simulator")

        with open(scenario_file, "r") as f:
            self.scenario = yaml.safe_load(f)

        self.world_frame = self.scenario.get("world_frame", "world")
        path_cfg = self.scenario["camera_path"]
        self.speed = path_cfg.get("speed", 0.45)
        self.loop = path_cfg.get("loop", False)
        self.fov_half_rad = math.radians(path_cfg.get("fov_angle", 60.0) / 2.0)
        self.max_distance = path_cfg.get("max_distance", 3.0)

        # Waypoints
        wp = path_cfg["waypoints"]
        self.waypoints = [np.array([p["x"], p["y"], p["z"]], dtype=float) for p in wp]
        if len(self.waypoints) < 2:
            raise ValueError("Defina pelo menos dois waypoints para formar um caminho.")

        self.current_pos = self.waypoints[0].copy()
        self.current_yaw = 0.0

        # Precomputar segmentos (vetor direção normalizado e distância)
        self.segments = []
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]
            vec = end - start
            dist = np.linalg.norm(vec)
            if dist < 1e-6:
                continue
            direction = vec / dist
            duration = dist / max(self.speed, 1e-6)
            self.segments.append(
                {"start": start, "end": end, "dir": direction, "dist": dist, "duration": duration}
            )

        if not self.segments:
            raise ValueError("Caminho inválido: todos os segmentos têm distância zero.")

        self.segment_idx = 0
        self.segment_progress = 0.0  # metros percorridos no segmento atual

        # Publicadores
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, "/camera_path", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        # Timer
        self.fps = 20.0
        self.dt = 1.0 / self.fps
        self.timer = self.create_timer(self.dt, self.update)

        # Publicar tags fixas
        self.publish_static_tags()

        self.get_logger().info(
            f"Loop iniciado com {len(self.segments)} segmentos, velocidade={self.speed} m/s, loop={self.loop}"
        )

    def publish_static_tags(self):
        static_transforms = []
        for tag_name, data in self.scenario["tags"].items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame
            t.child_frame_id = f"fixed_{tag_name}"
            t.transform.translation.x = data["position"]["x"]
            t.transform.translation.y = data["position"]["y"]
            t.transform.translation.z = data["position"]["z"]
            t.transform.rotation.x = data["orientation"]["x"]
            t.transform.rotation.y = data["orientation"]["y"]
            t.transform.rotation.z = data["orientation"]["z"]
            t.transform.rotation.w = data["orientation"]["w"]
            static_transforms.append(t)

        self.static_broadcaster.sendTransform(static_transforms)

    def advance_segment(self):
        self.segment_idx += 1
        self.segment_progress = 0.0
        if self.segment_idx >= len(self.segments):
            if self.loop:
                self.segment_idx = 0
            else:
                self.segment_idx = len(self.segments)  # trava no fim

    def update(self):
        # Movimento
        if self.segment_idx < len(self.segments):
            seg = self.segments[self.segment_idx]
            step = self.speed * self.dt
            remaining = seg["dist"] - self.segment_progress

            if step >= remaining:
                # Chegou ao fim do segmento
                self.current_pos = seg["end"].copy()
                self.segment_progress = seg["dist"]
                self.current_yaw = math.atan2(seg["dir"][1], seg["dir"][0])
                self.advance_segment()
            else:
                self.segment_progress += step
                self.current_pos = seg["start"] + seg["dir"] * self.segment_progress
                self.current_yaw = math.atan2(seg["dir"][1], seg["dir"][0])

        # Publicar TFs
        now = self.get_clock().now().to_msg()
        q_base = R.from_euler("z", self.current_yaw).as_quat()

        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = self.world_frame
        t_base.child_frame_id = "camera_link"
        t_base.transform.translation.x = float(self.current_pos[0])
        t_base.transform.translation.y = float(self.current_pos[1])
        t_base.transform.translation.z = float(self.current_pos[2])
        t_base.transform.rotation.x = q_base[0]
        t_base.transform.rotation.y = q_base[1]
        t_base.transform.rotation.z = q_base[2]
        t_base.transform.rotation.w = q_base[3]
        self.tf_broadcaster.sendTransform(t_base)

        t_opt = TransformStamped()
        t_opt.header.stamp = now
        t_opt.header.frame_id = "camera_link"
        t_opt.child_frame_id = "camera_optical_frame"
        t_opt.transform.rotation.x = -0.5
        t_opt.transform.rotation.y = 0.5
        t_opt.transform.rotation.z = -0.5
        t_opt.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t_opt)

        # Visualização do caminho
        pose = PoseStamped()
        pose.header = t_base.header
        pose.pose.position.x = float(self.current_pos[0])
        pose.pose.position.y = float(self.current_pos[1])
        pose.pose.position.z = float(self.current_pos[2])
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        # Simular visão
        self.simulate_vision(self.current_pos, q_base, now)

    def simulate_vision(self, pos, base_quat, time_stamp):
        # Matriz World -> Base
        r_mat_base = R.from_quat(base_quat).as_matrix()
        T_world_base = np.eye(4)
        T_world_base[:3, :3] = r_mat_base
        T_world_base[:3, 3] = pos

        # Base -> Optical (rotação fixa)
        r_mat_opt = R.from_quat([-0.5, 0.5, -0.5, 0.5]).as_matrix()
        T_base_opt = np.eye(4)
        T_base_opt[:3, :3] = r_mat_opt

        T_world_opt = T_world_base @ T_base_opt
        T_opt_world = np.linalg.inv(T_world_opt)

        for tag_name, data in self.scenario["tags"].items():
            tag_pos_world = np.array(
                [data["position"]["x"], data["position"]["y"], data["position"]["z"]]
            )

            tag_pos_opt = (T_opt_world @ np.append(tag_pos_world, 1.0))[:3]
            dist = np.linalg.norm(tag_pos_opt)

            if dist < self.max_distance and tag_pos_opt[2] > 0.1:
                angle = math.atan2(
                    math.sqrt(tag_pos_opt[0] ** 2 + tag_pos_opt[1] ** 2), tag_pos_opt[2]
                )
                if angle < self.fov_half_rad:
                    r_tag_world = R.from_quat(
                        [
                            data["orientation"]["x"],
                            data["orientation"]["y"],
                            data["orientation"]["z"],
                            data["orientation"]["w"],
                        ]
                    )
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
    # Garantir scipy disponível
    try:
        import scipy  # noqa: F401
    except ImportError:
        import subprocess

        subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy"])

    rclpy.init(args=args)

    scenario_path = "loop_four_tags_scenario.yaml"
    if len(sys.argv) > 1:
        scenario_path = sys.argv[1]

    node = LoopFourTagsSimulator(scenario_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
