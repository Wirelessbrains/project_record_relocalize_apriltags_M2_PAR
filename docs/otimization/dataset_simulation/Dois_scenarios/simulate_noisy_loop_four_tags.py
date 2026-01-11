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


class NoisyLoopFourTagsSimulator(Node):
    def __init__(self, scenario_file: str):
        super().__init__("noisy_loop_four_tags_simulator")

        # Ruídos (mesmos parâmetros do simulador ruidoso em S)
        self.noise_robot_pos = 0.005   # m de jitter no robô
        self.noise_tag_pos = 0.06      # m de ruído na detecção da tag
        self.noise_tag_rot = 0.04      # rad em cada eixo da tag detectada
        self.noise_yaw = 0.02          # rad de ruído no heading do robô
        self.dropout_rate = 0.02       # probabilidade de falha de detecção

        with open(scenario_file, "r") as f:
            self.scenario = yaml.safe_load(f)

        self.world_frame = self.scenario.get("world_frame", "world")
        path_cfg = self.scenario["camera_path"]
        self.speed = path_cfg.get("speed", 0.45)
        self.loop = path_cfg.get("loop", False)
        self.fov_half_rad = math.radians(path_cfg.get("fov_angle", 60.0) / 2.0)
        self.max_distance = path_cfg.get("max_distance", 3.0)

        wp = path_cfg["waypoints"]
        self.waypoints = [np.array([p["x"], p["y"], p["z"]], dtype=float) for p in wp]
        if len(self.waypoints) < 2:
            raise ValueError("Defina pelo menos dois waypoints para formar um caminho.")

        self.ideal_pos = self.waypoints[0].copy()
        self.current_yaw = 0.0

        self.segments = []
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]
            vec = end - start
            dist = np.linalg.norm(vec)
            if dist < 1e-6:
                continue
            direction = vec / dist
            self.segments.append({"start": start, "end": end, "dir": direction, "dist": dist})

        if not self.segments:
            raise ValueError("Caminho inválido: todos os segmentos têm distância zero.")

        self.segment_idx = 0
        self.segment_progress = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, "/camera_path", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        self.fps = 20.0
        self.dt = 1.0 / self.fps
        self.timer = self.create_timer(self.dt, self.update)

        self.publish_static_tags()

        self.get_logger().info(
            f"Simulação LOOP ruidosa iniciada. Seg={len(self.segments)}, noise_pos={self.noise_tag_pos}m"
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
                self.segment_idx = len(self.segments)

    def update(self):
        if self.segment_idx >= len(self.segments):
            return

        seg = self.segments[self.segment_idx]
        step = self.speed * self.dt
        remaining = seg["dist"] - self.segment_progress

        if step >= remaining:
            self.ideal_pos = seg["end"].copy()
            self.segment_progress = seg["dist"]
            self.current_yaw = math.atan2(seg["dir"][1], seg["dir"][0])
            self.advance_segment()
        else:
            self.segment_progress += step
            self.ideal_pos = seg["start"] + seg["dir"] * self.segment_progress
            self.current_yaw = math.atan2(seg["dir"][1], seg["dir"][0])

        # Ruído no estado publicado (não altera o progresso ideal)
        noisy_pos = self.ideal_pos.copy()
        noisy_pos[0] += np.random.normal(0, self.noise_robot_pos)
        noisy_pos[1] += np.random.normal(0, self.noise_robot_pos)
        yaw_noisy = self.current_yaw + np.random.normal(0, self.noise_yaw)
        q_base = R.from_euler("z", yaw_noisy).as_quat()

        now = self.get_clock().now().to_msg()

        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = self.world_frame
        t_base.child_frame_id = "camera_link"
        t_base.transform.translation.x = float(noisy_pos[0])
        t_base.transform.translation.y = float(noisy_pos[1])
        t_base.transform.translation.z = float(noisy_pos[2])
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

        pose = PoseStamped()
        pose.header = t_base.header
        pose.pose.position.x = float(noisy_pos[0])
        pose.pose.position.y = float(noisy_pos[1])
        pose.pose.position.z = float(noisy_pos[2])
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        self.simulate_noisy_vision(noisy_pos, q_base, now)

    def simulate_noisy_vision(self, pos, base_quat, time_stamp):
        r_mat_base = R.from_quat(base_quat).as_matrix()
        T_world_base = np.eye(4)
        T_world_base[:3, :3] = r_mat_base
        T_world_base[:3, 3] = pos

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
                    if np.random.rand() < self.dropout_rate:
                        continue

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
                    real_rel_pos = T_opt_tag[:3, 3]
                    real_rel_rot = R.from_matrix(T_opt_tag[:3, :3])

                    noisy_rel_pos = real_rel_pos + np.random.normal(0, self.noise_tag_pos, 3)
                    euler = real_rel_rot.as_euler("xyz")
                    euler_noisy = euler + np.random.normal(0, self.noise_tag_rot, 3)
                    noisy_rel_quat = R.from_euler("xyz", euler_noisy).as_quat()

                    t_det = TransformStamped()
                    t_det.header.stamp = time_stamp
                    t_det.header.frame_id = "camera_optical_frame"
                    t_det.child_frame_id = tag_name
                    t_det.transform.translation.x = noisy_rel_pos[0]
                    t_det.transform.translation.y = noisy_rel_pos[1]
                    t_det.transform.translation.z = noisy_rel_pos[2]
                    t_det.transform.rotation.x = noisy_rel_quat[0]
                    t_det.transform.rotation.y = noisy_rel_quat[1]
                    t_det.transform.rotation.z = noisy_rel_quat[2]
                    t_det.transform.rotation.w = noisy_rel_quat[3]

                    self.tf_broadcaster.sendTransform(t_det)


def main(args=None):
    try:
        import scipy  # noqa: F401
    except ImportError:
        import subprocess

        subprocess.check_call([sys.executable, "-m", "pip", "install", "scipy"])

    rclpy.init(args=args)

    scenario_path = "loop_four_tags_scenario.yaml"
    if len(sys.argv) > 1:
        scenario_path = sys.argv[1]

    node = NoisyLoopFourTagsSimulator(scenario_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
