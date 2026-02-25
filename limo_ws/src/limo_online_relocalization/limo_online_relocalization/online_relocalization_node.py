#!/usr/bin/env python3
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class RefPoint:
    x: float
    y: float
    z: float
    yaw: float


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def forward_heading_xz_from_quat(x: float, y: float, z: float, w: float) -> float:
    # Forward vector for optical frame (+Z) projected to XZ plane.
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    r02 = 2.0 * (xz + wy)       # row0 col2
    r22 = 1.0 - 2.0 * (xx + yy) # row2 col2
    return math.atan2(r22, r02)


class OnlineRelocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('online_relocalization_node')

        self.declare_parameter('reference_csv', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_topic', '/tag_only_pose')
        self.declare_parameter('pose_msg_type', 'pose_stamped')  # pose_stamped | pose | odom
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('distance_mode', '2d')  # 2d | 3d
        self.declare_parameter('path_downsample_step', 5)
        self.declare_parameter('relocalization_mode', 'full')  # full | progress
        self.declare_parameter('auto_align_xyyaw', False)
        self.declare_parameter('trajectory_plane', 'xy')  # xy | xz
        self.declare_parameter('reference_axis_mode', 'identity')  # identity | yz_to_xz

        self.reference_csv = self.get_parameter('reference_csv').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.pose_msg_type = self.get_parameter('pose_msg_type').get_parameter_value().string_value.lower()
        self.publish_rate = max(self.get_parameter('publish_rate').get_parameter_value().double_value, 1.0)
        self.distance_mode = self.get_parameter('distance_mode').get_parameter_value().string_value.lower()
        self.path_downsample_step = max(int(self.get_parameter('path_downsample_step').value), 1)
        self.relocalization_mode = (
            self.get_parameter('relocalization_mode').get_parameter_value().string_value.lower()
        )
        if self.relocalization_mode not in ('full', 'progress'):
            self.relocalization_mode = 'full'
        self.auto_align_xyyaw = bool(self.get_parameter('auto_align_xyyaw').value)
        self.trajectory_plane = self.get_parameter('trajectory_plane').get_parameter_value().string_value.lower()
        if self.trajectory_plane not in ('xy', 'xz'):
            self.trajectory_plane = 'xy'
        self.reference_axis_mode = (
            self.get_parameter('reference_axis_mode').get_parameter_value().string_value.lower()
        )
        if self.reference_axis_mode not in ('identity', 'yz_to_xz'):
            self.reference_axis_mode = 'identity'

        if not self.reference_csv:
            raise RuntimeError('Parameter "reference_csv" is required.')

        self.ref_points = self._load_reference(Path(self.reference_csv))
        if len(self.ref_points) < 2:
            raise RuntimeError(f'Reference CSV has insufficient points: {self.reference_csv}')

        self.current_pose: Optional[Pose] = None

        self.path_pub = self.create_publisher(PathMsg, '/relocalization/reference_path', 10)
        self.current_pose_pub = self.create_publisher(PoseStamped, '/relocalization/current_pose', 10)
        self.nearest_pose_pub = None
        self.distance_pub = None
        self.heading_pub = None
        self.marker_pub = None
        if self.relocalization_mode == 'full':
            self.nearest_pose_pub = self.create_publisher(PoseStamped, '/relocalization/nearest_pose', 10)
            self.distance_pub = self.create_publisher(Float64, '/relocalization/distance_m', 10)
            self.heading_pub = self.create_publisher(Float64, '/relocalization/heading_error_deg', 10)
            self.marker_pub = self.create_publisher(MarkerArray, '/relocalization/markers', 10)

        self._alignment_initialized = False
        self._align_cos = 1.0
        self._align_sin = 0.0
        self._align_tx = 0.0
        self._align_ty = 0.0
        self._align_tz = 0.0
        self._align_yaw = 0.0

        if self.pose_msg_type == 'pose':
            self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.pose_cb, 10)
        elif self.pose_msg_type == 'odom':
            self.pose_sub = self.create_subscription(Odometry, self.pose_topic, self.odom_cb, 10)
        else:
            self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_stamped_cb, 10)

        self.reference_path_msg = self._build_path_msg()
        self.static_path_marker = self._build_path_marker()

        self.create_timer(1.0 / self.publish_rate, self.timer_cb)
        self.get_logger().info(
            f'Online relocalization started. mode={self.relocalization_mode}, '
            f'reference_csv={self.reference_csv}, points={len(self.ref_points)}'
        )

    def _load_reference(self, csv_path: Path) -> List[RefPoint]:
        if not csv_path.exists():
            raise FileNotFoundError(f'Reference CSV not found: {csv_path}')

        points: List[RefPoint] = []
        with csv_path.open('r', newline='') as f:
            reader = csv.DictReader(f)
            required = {'x', 'y', 'z'}
            if not required.issubset(set(reader.fieldnames or [])):
                raise RuntimeError(f'CSV must contain columns {required}. got={reader.fieldnames}')

            rows = list(reader)
            for i, row in enumerate(rows):
                x_raw = float(row['x'])
                y_raw = float(row['y'])
                z_raw = float(row.get('z', 0.0))
                x, y, z = self._convert_reference_axes(x_raw, y_raw, z_raw)

                # Prefer local tangent heading for smooth reference.
                if i == 0:
                    nx = float(rows[i + 1]['x'])
                    ny = float(rows[i + 1]['y'])
                    nz = float(rows[i + 1].get('z', 0.0))
                    if self.trajectory_plane == 'xz':
                        yaw = math.atan2(nz - z, nx - x)
                    else:
                        yaw = math.atan2(ny - y, nx - x)
                elif i == len(rows) - 1:
                    px = float(rows[i - 1]['x'])
                    py = float(rows[i - 1]['y'])
                    pz = float(rows[i - 1].get('z', 0.0))
                    if self.trajectory_plane == 'xz':
                        yaw = math.atan2(z - pz, x - px)
                    else:
                        yaw = math.atan2(y - py, x - px)
                else:
                    px = float(rows[i - 1]['x'])
                    py = float(rows[i - 1]['y'])
                    pz = float(rows[i - 1].get('z', 0.0))
                    nx = float(rows[i + 1]['x'])
                    ny = float(rows[i + 1]['y'])
                    nz = float(rows[i + 1].get('z', 0.0))
                    if self.trajectory_plane == 'xz':
                        yaw = math.atan2(nz - pz, nx - px)
                    else:
                        yaw = math.atan2(ny - py, nx - px)

                # If tangent degenerates, fallback to quaternion yaw when available.
                if abs(yaw) < 1e-9 and {'qx', 'qy', 'qz', 'qw'}.issubset(set(row.keys())):
                    yaw = yaw_from_quat(
                        float(row.get('qx', 0.0)),
                        float(row.get('qy', 0.0)),
                        float(row.get('qz', 0.0)),
                        float(row.get('qw', 1.0)),
                    )

                points.append(RefPoint(x=x, y=y, z=z, yaw=yaw))

        return points

    def _convert_reference_axes(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        if self.reference_axis_mode == 'yz_to_xz':
            # Reference was recorded mainly in YZ; remap it to XZ camera plane.
            # New x <- old y, keep z, old x becomes out-of-plane component.
            return y, x, z
        return x, y, z

    def _build_path_msg(self) -> PathMsg:
        msg = PathMsg()
        msg.header.frame_id = self.frame_id
        for i, p in enumerate(self.ref_points):
            if i % self.path_downsample_step != 0 and i != len(self.ref_points) - 1:
                continue
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            px, py, pz = self._project_for_display(p.x, p.y, p.z)
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = pz
            ps.pose.orientation.z = math.sin(0.5 * p.yaw)
            ps.pose.orientation.w = math.cos(0.5 * p.yaw)
            msg.poses.append(ps)
        return msg

    def _build_path_marker(self) -> Marker:
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.ns = 'relocalization'
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.frame_locked = True
        mk.scale.x = 0.03
        mk.color.r = 0.1
        mk.color.g = 0.8
        mk.color.b = 1.0
        mk.color.a = 1.0
        for i, p in enumerate(self.ref_points):
            if i % self.path_downsample_step != 0 and i != len(self.ref_points) - 1:
                continue
            pt = Point()
            px, py, pz = self._project_for_display(p.x, p.y, p.z)
            pt.x = px
            pt.y = py
            pt.z = pz
            mk.points.append(pt)
        return mk

    def pose_stamped_cb(self, msg: PoseStamped) -> None:
        self.current_pose = msg.pose

    def pose_cb(self, msg: Pose) -> None:
        self.current_pose = msg

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def _distance(self, rx: float, ry: float, rz: float, p: RefPoint) -> float:
        if self.trajectory_plane == 'xz':
            dx = p.x - rx
            dy = p.z - rz
        else:
            dx = p.x - rx
            dy = p.y - ry
        if self.distance_mode == '3d':
            dz = p.z - rz
            return math.sqrt(dx * dx + dy * dy + dz * dz)
        return math.hypot(dx, dy)

    def _project_for_display(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        if self.trajectory_plane == 'xz':
            return x, 0.0, z
        return x, y, z

    def _nearest_index(self, rx: float, ry: float, rz: float) -> int:
        best_i = 0
        best_d = float('inf')
        for i, p in enumerate(self.ref_points):
            d = self._distance(rx, ry, rz, p)
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _init_alignment_if_needed(self, rx: float, ry: float, rz: float, robot_yaw: float) -> None:
        if not self.auto_align_xyyaw or self._alignment_initialized:
            return

        idx = self._nearest_index(rx, ry, rz)
        rp = self.ref_points[idx]
        yaw_off = normalize_angle(rp.yaw - robot_yaw)

        c = math.cos(yaw_off)
        s = math.sin(yaw_off)
        if self.trajectory_plane == 'xz':
            tx = rp.x - (c * rx - s * rz)
            tz = rp.z - (s * rx + c * rz)
            ty = rp.y - ry
        else:
            tx = rp.x - (c * rx - s * ry)
            ty = rp.y - (s * rx + c * ry)
            tz = rp.z - rz

        self._align_cos = c
        self._align_sin = s
        self._align_tx = tx
        self._align_ty = ty
        self._align_tz = tz
        self._align_yaw = yaw_off
        self._alignment_initialized = True

        self.get_logger().info(
            f'Auto alignment initialized at reference index {idx}: '
            f't=({tx:.3f}, {ty:.3f}, {tz:.3f}), yaw_off={math.degrees(yaw_off):.2f} deg'
        )

    def _apply_alignment(self, rx: float, ry: float, rz: float, robot_yaw: float) -> tuple[float, float, float, float]:
        if not self.auto_align_xyyaw:
            return rx, ry, rz, robot_yaw

        self._init_alignment_if_needed(rx, ry, rz, robot_yaw)

        if self.trajectory_plane == 'xz':
            x = self._align_cos * rx - self._align_sin * rz + self._align_tx
            z = self._align_sin * rx + self._align_cos * rz + self._align_tz
            y = ry + self._align_ty
        else:
            x = self._align_cos * rx - self._align_sin * ry + self._align_tx
            y = self._align_sin * rx + self._align_cos * ry + self._align_ty
            z = rz + self._align_tz
        yaw = normalize_angle(robot_yaw + self._align_yaw)
        return x, y, z, yaw

    def timer_cb(self) -> None:
        # Keep reference path visible in RViz.
        self.reference_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.reference_path_msg)

        if self.current_pose is None:
            return

        rx = float(self.current_pose.position.x)
        ry = float(self.current_pose.position.y)
        rz = float(self.current_pose.position.z)
        qx = float(self.current_pose.orientation.x)
        qy = float(self.current_pose.orientation.y)
        qz = float(self.current_pose.orientation.z)
        qw = float(self.current_pose.orientation.w)
        if self.trajectory_plane == 'xz':
            robot_yaw = forward_heading_xz_from_quat(qx, qy, qz, qw)
        else:
            robot_yaw = yaw_from_quat(qx, qy, qz, qw)

        rx, ry, rz, robot_yaw = self._apply_alignment(rx, ry, rz, robot_yaw)

        current_pose = PoseStamped()
        current_pose.header.frame_id = self.frame_id
        current_pose.header.stamp = self.get_clock().now().to_msg()
        cpx, cpy, cpz = self._project_for_display(rx, ry, rz)
        current_pose.pose.position.x = cpx
        current_pose.pose.position.y = cpy
        current_pose.pose.position.z = cpz
        current_pose.pose.orientation.z = math.sin(0.5 * robot_yaw)
        current_pose.pose.orientation.w = math.cos(0.5 * robot_yaw)
        self.current_pose_pub.publish(current_pose)

        if self.relocalization_mode == 'progress':
            return

        idx = self._nearest_index(rx, ry, rz)
        rp = self.ref_points[idx]

        dist = self._distance(rx, ry, rz, rp)
        heading_error_rad = normalize_angle(robot_yaw - rp.yaw)
        heading_error_deg = math.degrees(heading_error_rad)

        nearest_pose = PoseStamped()
        nearest_pose.header.frame_id = self.frame_id
        nearest_pose.header.stamp = self.get_clock().now().to_msg()
        npx, npy, npz = self._project_for_display(rp.x, rp.y, rp.z)
        nearest_pose.pose.position.x = npx
        nearest_pose.pose.position.y = npy
        nearest_pose.pose.position.z = npz
        nearest_pose.pose.orientation.z = math.sin(0.5 * rp.yaw)
        nearest_pose.pose.orientation.w = math.cos(0.5 * rp.yaw)
        if self.nearest_pose_pub is not None:
            self.nearest_pose_pub.publish(nearest_pose)

        d_msg = Float64()
        d_msg.data = dist
        if self.distance_pub is not None:
            self.distance_pub.publish(d_msg)

        h_msg = Float64()
        h_msg.data = heading_error_deg
        if self.heading_pub is not None:
            self.heading_pub.publish(h_msg)

        markers = MarkerArray()

        path_marker = self.static_path_marker
        path_marker.header.stamp = nearest_pose.header.stamp
        markers.markers.append(path_marker)

        nearest_mk = Marker()
        nearest_mk.header.frame_id = self.frame_id
        nearest_mk.header.stamp = nearest_pose.header.stamp
        nearest_mk.ns = 'relocalization'
        nearest_mk.id = 1
        nearest_mk.type = Marker.SPHERE
        nearest_mk.action = Marker.ADD
        nearest_mk.frame_locked = True
        nearest_mk.pose.position.x = npx
        nearest_mk.pose.position.y = npy
        nearest_mk.pose.position.z = npz
        nearest_mk.pose.orientation.w = 1.0
        nearest_mk.scale.x = 0.12
        nearest_mk.scale.y = 0.12
        nearest_mk.scale.z = 0.12
        nearest_mk.color.r = 1.0
        nearest_mk.color.g = 0.2
        nearest_mk.color.b = 0.2
        nearest_mk.color.a = 1.0
        markers.markers.append(nearest_mk)

        err_line = Marker()
        err_line.header.frame_id = self.frame_id
        err_line.header.stamp = nearest_pose.header.stamp
        err_line.ns = 'relocalization'
        err_line.id = 2
        err_line.type = Marker.LINE_STRIP
        err_line.action = Marker.ADD
        err_line.frame_locked = True
        err_line.scale.x = 0.02
        err_line.color.r = 1.0
        err_line.color.g = 1.0
        err_line.color.b = 0.0
        err_line.color.a = 1.0
        p_robot = Point()
        p_robot.x = cpx
        p_robot.y = cpy
        p_robot.z = cpz
        p_near = Point()
        p_near.x = npx
        p_near.y = npy
        p_near.z = npz
        err_line.points = [p_robot, p_near]
        markers.markers.append(err_line)

        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = nearest_pose.header.stamp
        text.ns = 'relocalization'
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.frame_locked = True
        text.pose.position.x = cpx
        text.pose.position.y = cpy
        text.pose.position.z = cpz + 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.15
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f'dist={dist:.3f} m | angle={heading_error_deg:.1f} deg'
        markers.markers.append(text)

        if self.marker_pub is not None:
            self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OnlineRelocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
