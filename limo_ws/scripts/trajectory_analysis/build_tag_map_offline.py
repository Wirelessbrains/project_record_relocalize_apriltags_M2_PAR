#!/usr/bin/env python3
import argparse
import csv
import math
import os
import sqlite3
import shutil
import time
import warnings
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Callable, Dict, List, Optional, Tuple

# Keep rosbag2/rclcpp INFO logs from breaking the progress bar rendering.
os.environ.setdefault("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "30")
warnings.filterwarnings(
    "ignore",
    message=r"A NumPy version >=1\.17\.3 and <1\.25\.0 is required for this version of SciPy.*",
    category=UserWarning,
)

import cv2
import matplotlib
import numpy as np
import plotly.graph_objects as go
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import least_squares

matplotlib.use("Agg")
import matplotlib.pyplot as plt


class TerminalProgressBar:
    def __init__(self, width: int = 34) -> None:
        term_cols = shutil.get_terminal_size(fallback=(120, 30)).columns
        max_width = max(10, min(50, term_cols - 34))
        self.width = max(10, min(int(width), max_width))
        self._last_bucket = -1
        self._last_label = ""

    def update(self, percent: float, label: str = "", force: bool = False) -> None:
        p = max(0.0, min(100.0, float(percent)))
        bucket = int(p // 2)  # redraw every 2%
        if not force and bucket == self._last_bucket and label == self._last_label:
            return
        self._last_bucket = bucket
        self._last_label = label
        filled = int(round((p / 100.0) * self.width))
        bar = "#" * filled + "-" * (self.width - filled)
        suffix = f" | {label}" if label else ""
        line = f"[{bar}] {p:6.2f}%{suffix}"
        print(f"\r\033[2K{line}", end="", flush=True)

    def finish(self, label: str = "Done") -> None:
        self.update(100.0, label, force=True)
        print()


def _stage_progress(
    progress_bar: TerminalProgressBar,
    start_pct: float,
    end_pct: float,
    stage_name: str,
    step_idx: int,
    total_steps: int,
) -> Callable[[float], None]:
    span = max(0.0, end_pct - start_pct)
    label = f"Step {step_idx}/{total_steps}: {stage_name}"

    def _cb(local_fraction: float) -> None:
        f = max(0.0, min(1.0, float(local_fraction)))
        progress_bar.update(start_pct + (span * f), label)

    return _cb


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = rotation
    t[:3, 3] = translation.reshape(3)
    return t


def _find_db_file(bag_path: str) -> Optional[Path]:
    p = Path(bag_path)
    if p.is_file() and p.suffix == ".db3":
        return p
    if p.is_dir():
        cands = sorted(p.glob("*.db3"))
        if cands:
            return cands[0]
    return None


def get_topic_message_count_from_db(bag_path: str, topic_name: str) -> Optional[int]:
    db = _find_db_file(bag_path)
    if db is None:
        return None
    try:
        con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        cur = con.cursor()
        cur.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
        row = cur.fetchone()
        if row is None:
            return None
        topic_id = int(row[0])
        cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id = ?", (topic_id,))
        return int(cur.fetchone()[0])
    except Exception:
        return None


def run_lsq_with_progress(
    residual_fn,
    x0: np.ndarray,
    max_nfev: int,
    loss: str,
    f_scale: float,
    show_progress: bool,
    progress_label: str,
    progress_step_nfev: int,
    optimizer_verbose: int,
    progress_callback: Optional[Callable[[float], None]] = None,
):
    if not show_progress and progress_callback is None:
        return least_squares(
            residual_fn,
            x0,
            loss=loss,
            f_scale=f_scale,
            max_nfev=max_nfev,
            verbose=optimizer_verbose,
        )

    x = x0.copy()
    used = 0
    last = None
    step = max(5, int(progress_step_nfev))
    t0 = time.time()
    while used < max_nfev:
        budget = min(step, max_nfev - used)
        res = least_squares(
            residual_fn,
            x,
            loss=loss,
            f_scale=f_scale,
            max_nfev=budget,
            verbose=0,
        )
        last = res
        x = res.x
        used += int(res.nfev)
        pct = 100.0 * min(used, max_nfev) / float(max_nfev)
        elapsed = time.time() - t0
        if progress_callback is not None:
            progress_callback(min(1.0, pct / 100.0))
        else:
            print(
                f"[progress] {progress_label}: {pct:5.1f}% "
                f"(nfev {used}/{max_nfev}, cost={res.cost:.6f}, t={elapsed:.1f}s)"
            )
        if bool(res.success) and int(res.nfev) < budget:
            break

    if last is None:
        last = SimpleNamespace(
            x=x0,
            fun=np.array([], dtype=float),
            cost=0.0,
            nfev=0,
            success=False,
        )
    return last


def invert_transform(t: np.ndarray) -> np.ndarray:
    r = t[:3, :3]
    p = t[:3, 3]
    out = np.eye(4, dtype=float)
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ p
    return out


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    r, _ = cv2.Rodrigues(rotvec.reshape(3, 1))
    return r


def matrix_to_rotvec(r: np.ndarray) -> np.ndarray:
    v, _ = cv2.Rodrigues(r)
    return v.reshape(3)


def se3_from_vector(v: np.ndarray) -> np.ndarray:
    rot = rotvec_to_matrix(v[:3])
    trans = v[3:6]
    return make_transform(rot, trans)


def vector_from_transform(t: np.ndarray) -> np.ndarray:
    rv = matrix_to_rotvec(t[:3, :3])
    tr = t[:3, 3]
    return np.hstack([rv, tr])


def quat_from_rotation(r: np.ndarray) -> np.ndarray:
    tr = np.trace(r)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=float)
    n = np.linalg.norm(q)
    if n > 0:
        q /= n
    return q


def blend_quaternion(q_prev: np.ndarray, q_new: np.ndarray, alpha: float) -> np.ndarray:
    if np.dot(q_prev, q_new) < 0.0:
        q_new = -q_new
    q = (1.0 - alpha) * q_prev + alpha * q_new
    n = np.linalg.norm(q)
    if n > 0:
        q /= n
    return q


def rotation_from_quat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def average_transforms(transforms: List[np.ndarray]) -> np.ndarray:
    if len(transforms) == 1:
        return transforms[0]
    trans = np.array([t[:3, 3] for t in transforms], dtype=float)
    trans_mean = np.mean(trans, axis=0)
    quats = [quat_from_rotation(t[:3, :3]) for t in transforms]
    q0 = quats[0]
    aligned = []
    for q in quats:
        aligned.append(-q if np.dot(q, q0) < 0.0 else q)
    q_mean = np.mean(np.array(aligned), axis=0)
    q_mean /= np.linalg.norm(q_mean)
    x, y, z, w = q_mean
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return make_transform(r, trans_mean)


def smooth_trajectory_ema(
    traj: List[Tuple[int, np.ndarray]], alpha_pos: float, alpha_rot: float
) -> List[Tuple[int, np.ndarray]]:
    if not traj:
        return []
    if alpha_pos <= 0.0 and alpha_rot <= 0.0:
        return list(traj)

    alpha_pos = max(0.0, min(1.0, alpha_pos))
    alpha_rot = max(0.0, min(1.0, alpha_rot))

    out = []
    prev_t = traj[0][1]
    out.append((traj[0][0], prev_t.copy()))
    prev_q = quat_from_rotation(prev_t[:3, :3])
    prev_p = prev_t[:3, 3].copy()

    for stamp_ns, t in traj[1:]:
        cur_p = t[:3, 3]
        cur_q = quat_from_rotation(t[:3, :3])
        new_p = (1.0 - alpha_pos) * prev_p + alpha_pos * cur_p if alpha_pos > 0.0 else cur_p
        new_q = blend_quaternion(prev_q, cur_q, alpha_rot) if alpha_rot > 0.0 else cur_q
        new_t = make_transform(rotation_from_quat(new_q), new_p)
        out.append((stamp_ns, new_t))
        prev_p = new_p
        prev_q = new_q

    return out


def smooth_trajectory_global(
    traj: List[Tuple[int, np.ndarray]],
    data_weight_pos: float,
    data_weight_rot: float,
    smooth_weight_pos: float,
    smooth_weight_rot: float,
    show_progress: bool = False,
    progress_step_nfev: int = 25,
    optimizer_verbose: int = 0,
    progress_callback: Optional[Callable[[float], None]] = None,
) -> List[Tuple[int, np.ndarray]]:
    if len(traj) < 3:
        return list(traj)

    raw_vecs = np.array([vector_from_transform(t) for _, t in traj], dtype=float)
    n = raw_vecs.shape[0]
    x0 = raw_vecs.reshape(-1).copy()

    wp_data = max(1e-9, data_weight_pos)
    wr_data = max(1e-9, data_weight_rot)
    wp_smooth = max(1e-9, smooth_weight_pos)
    wr_smooth = max(1e-9, smooth_weight_rot)

    def residuals(x_flat: np.ndarray) -> np.ndarray:
        x = x_flat.reshape(n, 6)
        out = []

        # Fidelity to measured per-frame poses
        d = x - raw_vecs
        out.extend((math.sqrt(wr_data) * d[:, 0:3].reshape(-1)).tolist())
        out.extend((math.sqrt(wp_data) * d[:, 3:6].reshape(-1)).tolist())

        # Smoothness: second difference (acceleration-like penalty)
        dd = x[2:] - 2.0 * x[1:-1] + x[:-2]
        out.extend((math.sqrt(wr_smooth) * dd[:, 0:3].reshape(-1)).tolist())
        out.extend((math.sqrt(wp_smooth) * dd[:, 3:6].reshape(-1)).tolist())

        return np.array(out, dtype=float)

    res = run_lsq_with_progress(
        residuals,
        x0,
        max_nfev=200,
        loss="soft_l1",
        f_scale=0.05,
        show_progress=show_progress,
        progress_label="global trajectory smoothing",
        progress_step_nfev=progress_step_nfev,
        optimizer_verbose=optimizer_verbose,
        progress_callback=progress_callback,
    )

    x_opt = res.x.reshape(n, 6)
    out = []
    for i, (stamp_ns, _) in enumerate(traj):
        out.append((stamp_ns, se3_from_vector(x_opt[i])))
    return out


def _lap_metrics_xy(path_xy: np.ndarray) -> Tuple[float, float, float]:
    if len(path_xy) < 3:
        return 1e9, 1e9, 1e9
    steps = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    length = float(np.sum(steps))
    closure = float(np.linalg.norm(path_xy[-1] - path_xy[0]))
    if len(path_xy) > 3:
        v = np.diff(path_xy, axis=0)
        heading = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
        dheading = np.diff(heading)
        smooth = float(np.std(dheading))
    else:
        smooth = 1e9
    return closure, smooth, length


def select_best_single_lap(
    traj: List[Tuple[int, np.ndarray]],
    min_turns_required: float = 1.5,
    min_samples_per_lap: int = 40,
) -> Tuple[List[Tuple[int, np.ndarray]], List[Dict[str, float]]]:
    if len(traj) < min_samples_per_lap:
        return traj, []

    xz = np.array([[t[0, 3], t[2, 3]] for _, t in traj], dtype=float)
    center = np.mean(xz, axis=0)
    ang = np.unwrap(np.arctan2(xz[:, 1] - center[1], xz[:, 0] - center[0]))
    ang0 = ang - ang[0]
    total_turns = abs(float(ang0[-1])) / (2.0 * math.pi)
    if total_turns < min_turns_required:
        return traj, []

    direction = 1.0 if ang0[-1] >= 0.0 else -1.0
    max_k = int(abs(ang0[-1]) // (2.0 * math.pi))
    if max_k < 1:
        return traj, []

    boundaries = [0]
    for k in range(1, max_k + 1):
        target = direction * (2.0 * math.pi * k)
        idx = int(np.argmin(np.abs(ang0 - target)))
        if idx - boundaries[-1] >= min_samples_per_lap:
            boundaries.append(idx)
    if len(boundaries) < 2:
        return traj, []

    candidates = []
    for i in range(len(boundaries) - 1):
        a = boundaries[i]
        b = boundaries[i + 1]
        seg = xz[a : b + 1]
        closure, smooth, length = _lap_metrics_xy(seg)
        score = (3.0 * closure) + smooth
        candidates.append(
            {
                "lap_id": float(i + 1),
                "start_idx": float(a),
                "end_idx": float(b),
                "n_samples": float(b - a + 1),
                "closure_m": closure,
                "smooth_std_heading": smooth,
                "length_m": length,
                "score": score,
            }
        )
    if not candidates:
        return traj, []

    best = min(candidates, key=lambda c: c["score"])
    a = int(best["start_idx"])
    b = int(best["end_idx"])
    return traj[a : b + 1], candidates


@dataclass
class DetectionObs:
    stamp_ns: int
    tag: str
    margin: float
    t_cam_tag: np.ndarray


@dataclass
class EdgeObs:
    tag_i: str
    tag_j: str
    t_i_j: np.ndarray
    weight: float


def _open_reader(bag_path: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id="sqlite3"), ConverterOptions("", "cdr"))
    return reader


def read_camera_info_and_counts(
    bag_path: str, detections_topic: str, camera_info_topic: str
) -> Tuple[np.ndarray, np.ndarray, Counter]:
    reader = _open_reader(bag_path)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if detections_topic not in type_map:
        raise RuntimeError(f"Detections topic not found: {detections_topic}")
    if camera_info_topic not in type_map:
        raise RuntimeError(f"camera_info topic not found: {camera_info_topic}")

    det_msg_t = get_message(type_map[detections_topic])
    cam_msg_t = get_message(type_map[camera_info_topic])

    k = None
    d = None
    per_tag_count = Counter()

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == camera_info_topic and k is None:
            msg = deserialize_message(data, cam_msg_t)
            k = np.array(msg.k, dtype=float).reshape(3, 3)
            d = np.array(msg.d, dtype=float).reshape(-1, 1)
        elif topic == detections_topic:
            msg = deserialize_message(data, det_msg_t)
            for det in msg.detections:
                per_tag_count[f"{det.family}:{int(det.id)}"] += 1

    if k is None:
        raise RuntimeError(f"Could not read {camera_info_topic} in the bag.")
    return k, d, per_tag_count


def read_detections_with_pnp(
    bag_path: str,
    detections_topic: str,
    k: np.ndarray,
    d: np.ndarray,
    tag_size: float,
    min_margin: float,
    frame_stride: int = 1,
    show_progress: bool = False,
    progress_callback: Optional[Callable[[float], None]] = None,
) -> Dict[int, List[DetectionObs]]:
    reader = _open_reader(bag_path)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    if detections_topic not in type_map:
        raise RuntimeError(f"Detections topic not found: {detections_topic}")
    det_msg_t = get_message(type_map[detections_topic])

    half = tag_size / 2.0
    object_pts = np.array(
        [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]],
        dtype=np.float64,
    )

    frames: Dict[int, List[DetectionObs]] = defaultdict(list)
    total_det_msgs = get_topic_message_count_from_db(bag_path, detections_topic)
    det_msg_seen = 0
    last_print_pct = -1
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != detections_topic:
            continue
        det_msg_seen += 1
        if progress_callback is not None and total_det_msgs:
            progress_callback(min(1.0, det_msg_seen / float(total_det_msgs)))
        if show_progress:
            if total_det_msgs:
                pct = int(100.0 * det_msg_seen / float(total_det_msgs))
                if pct >= last_print_pct + 5:
                    last_print_pct = pct
                    if progress_callback is None:
                        print(f"[progress] PnP detections: {pct}% ({det_msg_seen}/{total_det_msgs})")
            elif det_msg_seen % 50 == 0:
                if progress_callback is None:
                    print(f"[progress] PnP detections messages: {det_msg_seen}")
        if frame_stride > 1 and (det_msg_seen - 1) % frame_stride != 0:
            continue
        msg = deserialize_message(data, det_msg_t)
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        for det in msg.detections:
            margin = float(getattr(det, "decision_margin", 0.0))
            if margin < min_margin:
                continue
            corners = np.array([[c.x, c.y] for c in det.corners], dtype=np.float64)
            if corners.shape != (4, 2):
                continue
            ok, rvec, tvec = cv2.solvePnP(object_pts, corners, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            r, _ = cv2.Rodrigues(rvec)
            t_cam_tag = make_transform(r, tvec.reshape(3))
            frames[stamp_ns].append(
                DetectionObs(
                    stamp_ns=stamp_ns,
                    tag=f"{det.family}:{int(det.id)}",
                    margin=margin,
                    t_cam_tag=t_cam_tag,
                )
            )
    return frames


def build_edges(frames: Dict[int, List[DetectionObs]]) -> List[EdgeObs]:
    edges = []
    for obs_list in frames.values():
        if len(obs_list) < 2:
            continue
        n = len(obs_list)
        for i in range(n):
            for j in range(i + 1, n):
                a = obs_list[i]
                b = obs_list[j]
                t_i_j = invert_transform(a.t_cam_tag) @ b.t_cam_tag
                w = max(1e-3, min(a.margin, b.margin))
                edges.append(EdgeObs(tag_i=a.tag, tag_j=b.tag, t_i_j=t_i_j, weight=w))
    return edges


def initial_map_from_edges(anchor_tag: str, edges: List[EdgeObs]) -> Dict[str, np.ndarray]:
    neighbors = defaultdict(list)
    for e in edges:
        neighbors[e.tag_i].append((e.tag_j, e.t_i_j))
        neighbors[e.tag_j].append((e.tag_i, invert_transform(e.t_i_j)))

    tag_map = {anchor_tag: np.eye(4)}
    frontier = [anchor_tag]
    while frontier:
        cur = frontier.pop()
        for nxt, t_cur_nxt in neighbors[cur]:
            if nxt in tag_map:
                continue
            tag_map[nxt] = tag_map[cur] @ t_cur_nxt
            frontier.append(nxt)
    return tag_map


def optimize_tag_map(
    anchor_tag: str,
    all_tags: List[str],
    initial_map: Dict[str, np.ndarray],
    edges: List[EdgeObs],
    show_progress: bool = False,
    progress_step_nfev: int = 25,
    optimizer_verbose: int = 0,
    progress_callback: Optional[Callable[[float], None]] = None,
) -> Tuple[Dict[str, np.ndarray], float]:
    opt_tags = [t for t in all_tags if t != anchor_tag and t in initial_map]
    idx = {t: i for i, t in enumerate(opt_tags)}
    x0 = np.zeros(len(opt_tags) * 6, dtype=float)
    for t in opt_tags:
        x0[idx[t] * 6 : idx[t] * 6 + 6] = vector_from_transform(initial_map[t])

    def pose_of(tag: str, x: np.ndarray) -> Optional[np.ndarray]:
        if tag == anchor_tag:
            return np.eye(4)
        if tag not in idx:
            return None
        base = idx[tag] * 6
        return se3_from_vector(x[base : base + 6])

    def residuals(x: np.ndarray) -> np.ndarray:
        out = []
        for e in edges:
            t_i = pose_of(e.tag_i, x)
            t_j = pose_of(e.tag_j, x)
            if t_i is None or t_j is None:
                continue
            pred = invert_transform(t_i) @ t_j
            err = invert_transform(e.t_i_j) @ pred
            rv = matrix_to_rotvec(err[:3, :3])
            tv = err[:3, 3]
            w = math.sqrt(min(5.0, max(0.1, e.weight / 20.0)))
            out.extend((w * tv).tolist())
            out.extend((w * rv).tolist())
        return np.array(out, dtype=float)

    res = run_lsq_with_progress(
        residuals,
        x0,
        max_nfev=300,
        loss="soft_l1",
        f_scale=0.1,
        show_progress=show_progress,
        progress_label="map optimization",
        progress_step_nfev=progress_step_nfev,
        optimizer_verbose=optimizer_verbose,
        progress_callback=progress_callback,
    )

    out_map = {anchor_tag: np.eye(4)}
    for t in opt_tags:
        base = idx[t] * 6
        out_map[t] = se3_from_vector(res.x[base : base + 6])
    rmse = float(math.sqrt(np.mean(res.fun * res.fun))) if len(res.fun) else 0.0
    return out_map, rmse


def _camera_pose_lsq_for_frame(
    obs_list: List[DetectionObs], tag_map: Dict[str, np.ndarray], initial_t_map_cam: np.ndarray
) -> Optional[np.ndarray]:
    valid_obs = [o for o in obs_list if o.tag in tag_map]
    if not valid_obs:
        return None

    x0 = vector_from_transform(initial_t_map_cam)

    def residuals(x: np.ndarray) -> np.ndarray:
        t_map_cam = se3_from_vector(x)
        out = []
        for obs in valid_obs:
            pred = invert_transform(t_map_cam) @ tag_map[obs.tag]
            err = invert_transform(obs.t_cam_tag) @ pred
            tv = err[:3, 3]
            rv = matrix_to_rotvec(err[:3, :3])
            w = math.sqrt(min(5.0, max(0.1, obs.margin / 20.0)))
            out.extend((w * tv).tolist())
            out.extend((w * rv).tolist())
        return np.array(out, dtype=float)

    res = least_squares(residuals, x0, loss="soft_l1", f_scale=0.05, max_nfev=60, verbose=0)
    if not res.success:
        return None
    return se3_from_vector(res.x)


def relocalize_camera(
    frames: Dict[int, List[DetectionObs]],
    tag_map: Dict[str, np.ndarray],
    min_tags_per_pose: int,
    mode: str,
    show_progress: bool = False,
    progress_callback: Optional[Callable[[float], None]] = None,
) -> List[Tuple[int, np.ndarray]]:
    traj = []
    last_pose = None
    ordered = sorted(frames.keys())
    total = len(ordered)
    for i, stamp_ns in enumerate(ordered):
        estimates = []
        valid_obs = []
        for obs in frames[stamp_ns]:
            if obs.tag not in tag_map:
                continue
            valid_obs.append(obs)
            estimates.append(tag_map[obs.tag] @ invert_transform(obs.t_cam_tag))

        if len(estimates) < min_tags_per_pose:
            continue

        if mode == "avg":
            pose = average_transforms(estimates)
        else:
            init = last_pose if last_pose is not None else average_transforms(estimates)
            pose = _camera_pose_lsq_for_frame(valid_obs, tag_map, init)
            if pose is None:
                pose = average_transforms(estimates)

        traj.append((stamp_ns, pose))
        last_pose = pose
        if progress_callback is not None and total > 0:
            progress_callback((i + 1) / float(total))
        if show_progress and (i % 50 == 0 or i == total - 1):
            pct = int(100.0 * (i + 1) / float(total))
            if progress_callback is None:
                print(f"[progress] relocalization: {pct}% ({i+1}/{total})")

    return traj


def save_map_csv(path: str, tag_map: Dict[str, np.ndarray], obs_count: Counter) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tag", "x", "y", "z", "qx", "qy", "qz", "qw", "n_obs"])
        for tag in sorted(tag_map.keys()):
            t = tag_map[tag]
            q = quat_from_rotation(t[:3, :3])
            p = t[:3, 3]
            w.writerow([tag, float(p[0]), float(p[1]), float(p[2]), float(q[0]), float(q[1]), float(q[2]), float(q[3]), int(obs_count.get(tag, 0))])


def save_map_yaml(path: str, tag_map: Dict[str, np.ndarray], anchor_tag: str, world_frame: str = "map") -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(f'world_frame: "{world_frame}"\n')
        f.write(f'anchor_tag: "{anchor_tag}"\n')
        f.write("tags:\n")
        for tag in sorted(tag_map.keys()):
            t = tag_map[tag]
            q = quat_from_rotation(t[:3, :3])
            p = t[:3, 3]
            f.write(f"  {tag}:\n")
            f.write(
                f"    position: {{x: {float(p[0]):.6f}, y: {float(p[1]):.6f}, z: {float(p[2]):.6f}}}\n"
            )
            f.write(
                f"    orientation: {{x: {float(q[0]):.6f}, y: {float(q[1]):.6f}, z: {float(q[2]):.6f}, w: {float(q[3]):.6f}}}\n"
            )


def save_traj_csv(path: str, traj: List[Tuple[int, np.ndarray]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_sec", "stamp_nanosec", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for stamp_ns, t in traj:
            sec = stamp_ns // 1_000_000_000
            nsec = stamp_ns % 1_000_000_000
            p = t[:3, 3]
            q = quat_from_rotation(t[:3, :3])
            w.writerow([sec, nsec, float(p[0]), float(p[1]), float(p[2]), float(q[0]), float(q[1]), float(q[2]), float(q[3])])


def save_diagnostics_csv(
    path: str,
    n_frames: int,
    n_frames_with_pose: int,
    n_tags_detected: int,
    n_tags_mapped: int,
    n_edges: int,
    rmse_map: float,
    relocalization_mode: str,
) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        w.writerow(["frames_total", n_frames])
        w.writerow(["frames_with_pose", n_frames_with_pose])
        w.writerow(["tags_detected", n_tags_detected])
        w.writerow(["tags_mapped", n_tags_mapped])
        w.writerow(["pairwise_edges", n_edges])
        w.writerow(["map_residual_rmse", rmse_map])
        w.writerow(["relocalization_mode", relocalization_mode])


def save_lap_selection_csv(path: str, lap_rows: List[Dict[str, float]]) -> None:
    if not lap_rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cols = [
        "lap_id",
        "start_idx",
        "end_idx",
        "n_samples",
        "closure_m",
        "smooth_std_heading",
        "length_m",
        "score",
    ]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for row in lap_rows:
            w.writerow(row)


def save_plot_3d(path: str, tag_map: Dict[str, np.ndarray], traj: List[Tuple[int, np.ndarray]], anchor_tag: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig = plt.figure(figsize=(11, 6))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax2d = fig.add_subplot(1, 2, 2)

    tags = sorted(tag_map.keys())
    tx = [tag_map[t][0, 3] for t in tags]
    ty = [tag_map[t][1, 3] for t in tags]
    tz = [tag_map[t][2, 3] for t in tags]

    ax3d.scatter(tx, ty, tz, c="orange", s=26, label="tags")
    ax2d.scatter(tx, tz, c="orange", s=26, label="tags")

    for name in tags:
        p = tag_map[name][:3, 3]
        lbl = name + (" (anchor)" if name == anchor_tag else "")
        ax3d.text(p[0], p[1], p[2], lbl, fontsize=7)
        ax2d.text(p[0], p[2], lbl, fontsize=7)

    if traj:
        px = [t[0, 3] for _, t in traj]
        py = [t[1, 3] for _, t in traj]
        pz = [t[2, 3] for _, t in traj]
        ax3d.plot(px, py, pz, c="tab:blue", label="camera")
        ax3d.scatter(px[0], py[0], pz[0], c="green", s=36, label="start")
        ax3d.scatter(px[-1], py[-1], pz[-1], c="red", s=36, label="end")
        ax2d.plot(px, pz, c="tab:blue", label="camera")
        ax2d.scatter(px[0], pz[0], c="green", s=36, label="start")
        ax2d.scatter(px[-1], pz[-1], c="red", s=36, label="end")

    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    ax3d.set_title("Tag map + trajectory (3D)")
    ax3d.legend(loc="upper right")

    ax2d.set_xlabel("X [m]")
    ax2d.set_ylabel("Z [m]")
    ax2d.set_title("Top view (X-Z)")
    ax2d.axis("equal")
    ax2d.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_plot_interactive_html(
    path: str,
    tag_map: Dict[str, np.ndarray],
    traj_raw: List[Tuple[int, np.ndarray]],
    anchor_tag: str,
    traj_smooth: Optional[List[Tuple[int, np.ndarray]]] = None,
    show_axes: bool = False,
    tag_axis_len: float = 0.08,
    cam_axis_len: float = 0.06,
    cam_axis_step: int = 8,
) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tags = sorted(tag_map.keys())
    tx = [tag_map[t][0, 3] for t in tags]
    ty = [tag_map[t][1, 3] for t in tags]
    tz = [tag_map[t][2, 3] for t in tags]
    labels = [t + (" (anchor)" if t == anchor_tag else "") for t in tags]

    data = [
        go.Scatter3d(
            x=tx,
            y=ty,
            z=tz,
            mode="markers+text",
            text=labels,
            textposition="top center",
            marker=dict(size=4, color="orange"),
            name="tags",
        )
    ]

    if traj_raw:
        px = [t[0, 3] for _, t in traj_raw]
        py = [t[1, 3] for _, t in traj_raw]
        pz = [t[2, 3] for _, t in traj_raw]
        data.append(
            go.Scatter3d(x=px, y=py, z=pz, mode="lines", line=dict(width=2, color="royalblue"), name="camera raw")
        )
        data.append(
            go.Scatter3d(
                x=[px[0], px[-1]],
                y=[py[0], py[-1]],
                z=[pz[0], pz[-1]],
                mode="markers+text",
                text=["start", "end"],
                textposition="top center",
                marker=dict(size=5, color=["green", "red"]),
                name="endpoints",
            )
        )
    if traj_smooth:
        sx = [t[0, 3] for _, t in traj_smooth]
        sy = [t[1, 3] for _, t in traj_smooth]
        sz = [t[2, 3] for _, t in traj_smooth]
        data.append(
            go.Scatter3d(x=sx, y=sy, z=sz, mode="lines", line=dict(width=5, color="crimson"), name="camera smooth")
        )

    if show_axes:
        def add_axes_traces(transforms: List[np.ndarray], length: float, prefix: str):
            if not transforms:
                return
            colors = [("X", "red"), ("Y", "green"), ("Z", "blue")]
            for axis_idx, (axis_name, color) in enumerate(colors):
                xs, ys, zs = [], [], []
                for t in transforms:
                    o = t[:3, 3]
                    d = t[:3, :3][:, axis_idx]
                    e = o + length * d
                    xs.extend([o[0], e[0], None])
                    ys.extend([o[1], e[1], None])
                    zs.extend([o[2], e[2], None])
                data.append(
                    go.Scatter3d(
                        x=xs,
                        y=ys,
                        z=zs,
                        mode="lines",
                        line=dict(width=3, color=color),
                        name=f"{prefix} {axis_name}",
                    )
                )

        tag_tfs = [tag_map[k] for k in sorted(tag_map.keys())]
        add_axes_traces(tag_tfs, tag_axis_len, "tag axis")

        cam_src = traj_smooth if traj_smooth else traj_raw
        cam_tfs = []
        if cam_src:
            for i, (_, t) in enumerate(cam_src):
                if i % max(1, cam_axis_step) == 0:
                    cam_tfs.append(t)
            if cam_src and cam_tfs[-1] is not cam_src[-1][1]:
                cam_tfs.append(cam_src[-1][1])
        add_axes_traces(cam_tfs, cam_axis_len, "camera axis")

    fig = go.Figure(data=data)
    fig.update_layout(
        title="Tag map + camera trajectory (interactive)",
        scene=dict(
            xaxis_title="X [m]",
            yaxis_title="Y [m]",
            zaxis_title="Z [m]",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    fig.write_html(path, include_plotlyjs="cdn")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Offline robust AprilTag map + camera trajectory from rosbag2.")
    p.add_argument("bag_path", nargs="?", default="", help="Path to rosbag2 folder (positional shortcut)")
    p.add_argument("output_dir_path", nargs="?", default="", help="Output directory (positional shortcut)")
    p.add_argument("--bag", default="", help="Path to rosbag2 folder")
    p.add_argument("--detections-topic", default="/detections")
    p.add_argument("--camera-info-topic", default="/camera_info")
    p.add_argument("--frame-stride", type=int, default=1, help="Use 1 out of N detection messages (N>=1)")
    p.add_argument("--tag-size", type=float, default=0.20, help="Tag size in meters")
    p.add_argument("--min-margin", type=float, default=20.0, help="Minimum decision margin")
    p.add_argument("--anchor-tag", default="", help="Optional anchor tag name, e.g. tag36h11:0")
    p.add_argument("--min-tags-per-pose", type=int, default=1)
    p.add_argument("--relocalization-mode", choices=["lsq", "avg"], default="lsq")
    p.add_argument("--plot-3d-png", default="", help="Output PNG path for 3D/top plot")
    p.add_argument("--plot-3d-html", default="", help="Output HTML path for interactive 3D plot")
    p.add_argument("--smooth-method", choices=["ema", "global"], default="global")
    p.add_argument("--smooth-alpha-pos", type=float, default=0.35, help="EMA alpha for position smoothing [0..1]")
    p.add_argument("--smooth-alpha-rot", type=float, default=0.25, help="EMA alpha for orientation smoothing [0..1]")
    p.add_argument("--global-data-pos", type=float, default=4.0, help="Global smoother: data weight for position")
    p.add_argument("--global-data-rot", type=float, default=4.0, help="Global smoother: data weight for rotation")
    p.add_argument("--global-smooth-pos", type=float, default=50.0, help="Global smoother: smoothness weight for position")
    p.add_argument("--global-smooth-rot", type=float, default=20.0, help="Global smoother: smoothness weight for rotation")
    p.add_argument("--show-axes", action="store_true", help="Draw TF orientation axes for tags and sampled camera poses")
    p.add_argument("--tag-axis-len", type=float, default=0.08, help="Axis length for tag frames in meters")
    p.add_argument("--cam-axis-len", type=float, default=0.06, help="Axis length for sampled camera frames in meters")
    p.add_argument("--cam-axis-step", type=int, default=8, help="Sample every N camera poses for axis drawing")
    p.add_argument("--lap-mode", choices=["all", "single_best"], default="single_best", help="Keep full trajectory or only best lap")
    p.add_argument("--lap-min-turns", type=float, default=1.5, help="Minimum estimated turns needed before lap extraction")
    p.add_argument("--lap-min-samples", type=int, default=40, help="Minimum samples for a valid lap segment")
    p.add_argument("--show-progress", action="store_true", help="Show progress percentages during long steps")
    p.add_argument("--progress-step-nfev", type=int, default=25, help="NFEV chunk size for optimization progress")
    p.add_argument("--optimizer-verbose", type=int, default=0, choices=[0, 1, 2], help="SciPy least_squares verbose level")
    p.add_argument("--output-dir", default="outputs")
    args = p.parse_args()

    if not args.bag and args.bag_path:
        args.bag = args.bag_path
    if args.output_dir_path:
        args.output_dir = args.output_dir_path
    if not args.bag:
        p.error("missing bag path. Use positional <bag_path> or --bag.")
    if args.frame_stride < 1:
        p.error("--frame-stride must be >= 1")
    return args


def estimate_turns_from_traj(traj: List[Tuple[int, np.ndarray]]) -> float:
    if len(traj) < 3:
        return 0.0
    xz = np.array([[t[0, 3], t[2, 3]] for _, t in traj], dtype=float)
    center = np.mean(xz, axis=0)
    ang = np.unwrap(np.arctan2(xz[:, 1] - center[1], xz[:, 0] - center[0]))
    ang0 = ang - ang[0]
    return abs(float(ang0[-1])) / (2.0 * math.pi)


def _print_section(title: str) -> None:
    print(f"\n=== {title} ===")


def _print_kv(key: str, value: str) -> None:
    print(f"{key:<28}: {value}")


def main() -> None:
    args = parse_args()
    total_steps = 6
    progress_bar = TerminalProgressBar()
    progress_bar.update(0.0, f"Step 1/{total_steps}: read camera info")

    k, d, raw_counts = read_camera_info_and_counts(args.bag, args.detections_topic, args.camera_info_topic)
    progress_bar.update(8.0, f"Step 1/{total_steps}: read camera info")
    frames = read_detections_with_pnp(
        args.bag,
        args.detections_topic,
        k,
        d,
        args.tag_size,
        args.min_margin,
        frame_stride=args.frame_stride,
        show_progress=args.show_progress,
        progress_callback=_stage_progress(progress_bar, 8.0, 40.0, "PnP detections", 2, total_steps),
    )
    if not frames:
        raise RuntimeError("No valid detections after filtering/PnP.")

    per_tag_count = Counter()
    for obs_list in frames.values():
        for obs in obs_list:
            per_tag_count[obs.tag] += 1

    all_tags = sorted(per_tag_count.keys())
    if not all_tags:
        raise RuntimeError("No valid tag available for mapping.")

    anchor_tag = args.anchor_tag if args.anchor_tag else per_tag_count.most_common(1)[0][0]
    edges = build_edges(frames)
    initial_map = initial_map_from_edges(anchor_tag, edges)
    opt_map, rmse_map = optimize_tag_map(
        anchor_tag,
        all_tags,
        initial_map,
        edges,
        show_progress=args.show_progress,
        progress_step_nfev=args.progress_step_nfev,
        optimizer_verbose=args.optimizer_verbose,
        progress_callback=_stage_progress(progress_bar, 40.0, 62.0, "map optimization", 3, total_steps),
    )

    traj_raw = relocalize_camera(
        frames,
        opt_map,
        args.min_tags_per_pose,
        args.relocalization_mode,
        show_progress=args.show_progress,
        progress_callback=_stage_progress(progress_bar, 62.0, 78.0, "camera relocalization", 4, total_steps),
    )
    if args.smooth_method == "ema":
        traj_smooth = smooth_trajectory_ema(traj_raw, args.smooth_alpha_pos, args.smooth_alpha_rot)
        progress_bar.update(92.0, f"Step 5/{total_steps}: EMA smoothing")
    else:
        traj_smooth = smooth_trajectory_global(
            traj_raw,
            data_weight_pos=args.global_data_pos,
            data_weight_rot=args.global_data_rot,
            smooth_weight_pos=args.global_smooth_pos,
            smooth_weight_rot=args.global_smooth_rot,
            show_progress=args.show_progress,
            progress_step_nfev=args.progress_step_nfev,
            optimizer_verbose=args.optimizer_verbose,
            progress_callback=_stage_progress(progress_bar, 78.0, 92.0, "global smoothing", 5, total_steps),
        )

    traj_raw_out = traj_raw
    traj_smooth_out = traj_smooth
    lap_rows: List[Dict[str, float]] = []
    if args.lap_mode == "single_best":
        selected_smooth, lap_rows = select_best_single_lap(
            traj_smooth,
            min_turns_required=args.lap_min_turns,
            min_samples_per_lap=args.lap_min_samples,
        )
        traj_smooth_out = selected_smooth
        if lap_rows:
            best_row = min(lap_rows, key=lambda r: r["score"])
            a = int(best_row["start_idx"])
            b = int(best_row["end_idx"])
            traj_raw_out = traj_raw[a : b + 1]
        else:
            traj_raw_out = traj_raw
    progress_bar.update(96.0, f"Step 6/{total_steps}: lap selection + export")

    out_map = os.path.join(args.output_dir, "mapa_tags.csv")
    out_map_yaml = os.path.join(args.output_dir, "tag_map.yaml")
    out_traj_raw = os.path.join(args.output_dir, "trajetoria_camera_raw.csv")
    out_traj_smooth = os.path.join(args.output_dir, "trajetoria_camera_suave.csv")
    out_traj = os.path.join(args.output_dir, "trajetoria_camera.csv")
    out_diag = os.path.join(args.output_dir, "diagnostico_erros.csv")
    out_laps = os.path.join(args.output_dir, "lap_selection.csv")
    out_plot = args.plot_3d_png
    out_html = args.plot_3d_html if args.plot_3d_html else os.path.join(args.output_dir, "mapa_trajetoria_interativo.html")

    save_map_csv(out_map, opt_map, raw_counts)
    save_map_yaml(out_map_yaml, opt_map, anchor_tag, world_frame="map")
    save_traj_csv(out_traj_raw, traj_raw_out)
    save_traj_csv(out_traj_smooth, traj_smooth_out)
    save_traj_csv(out_traj, traj_smooth_out)
    save_diagnostics_csv(
        out_diag,
        n_frames=len(frames),
        n_frames_with_pose=len(traj_raw_out),
        n_tags_detected=len(raw_counts),
        n_tags_mapped=len(opt_map),
        n_edges=len(edges),
        rmse_map=rmse_map,
        relocalization_mode=args.relocalization_mode,
    )
    save_lap_selection_csv(out_laps, lap_rows)
    if out_plot:
        save_plot_3d(out_plot, opt_map, traj_smooth_out, anchor_tag)
    save_plot_interactive_html(
        out_html,
        opt_map,
        traj_raw_out,
        anchor_tag,
        traj_smooth=traj_smooth_out,
        show_axes=args.show_axes,
        tag_axis_len=args.tag_axis_len,
        cam_axis_len=args.cam_axis_len,
        cam_axis_step=args.cam_axis_step,
    )
    progress_bar.finish(f"Step 6/{total_steps}: complete")

    turns_est = estimate_turns_from_traj(traj_smooth)
    laps_est = int(math.floor(turns_est))
    tags_observed_raw = len(raw_counts)
    tags_after_quality_filter = len(per_tag_count)
    tags_mapped_connected = len(opt_map)
    tags_used_in_trajectory = tags_mapped_connected
    tags_excluded = max(0, tags_observed_raw - tags_mapped_connected)
    best_lap_msg = "N/A"
    if args.lap_mode == "single_best":
        if lap_rows:
            best_row = min(lap_rows, key=lambda r: r["score"])
            best_lap_msg = f"Lap {int(best_row['lap_id'])} preserved"
        else:
            best_lap_msg = "No valid lap segmentation, full trajectory preserved"

    smoothing_desc = (
        f"EMA(alpha_pos={args.smooth_alpha_pos}, alpha_rot={args.smooth_alpha_rot})"
        if args.smooth_method == "ema"
        else (
            "GLOBAL least_squares(loss=soft_l1, "
            f"data=({args.global_data_pos},{args.global_data_rot}), "
            f"smooth=({args.global_smooth_pos},{args.global_smooth_rot}))"
        )
    )

    _print_section("Run Summary")
    _print_kv("Optimization profile", f"map=least_squares(loss=soft_l1), relocalization={args.relocalization_mode}, smoothing={smoothing_desc}")
    _print_kv("Frame stride", str(args.frame_stride))
    _print_kv("Tags observed (raw)", str(tags_observed_raw))
    _print_kv("Tags after quality filter", str(tags_after_quality_filter))
    _print_kv("Tags mapped (connected)", str(tags_mapped_connected))
    _print_kv("Tags used in trajectory", str(tags_used_in_trajectory))
    _print_kv("Tags excluded", str(tags_excluded))
    _print_kv("Estimated laps completed", f"{laps_est} (turns={turns_est:.2f})")
    _print_kv("Lap selection mode", args.lap_mode)
    if args.lap_mode == "single_best":
        _print_kv("Candidate laps found", str(len(lap_rows)))
        _print_kv("Best lap result", best_lap_msg)
    _print_kv("Map RMSE residual", f"{rmse_map:.6f}")
    _print_kv("Anchor tag", anchor_tag)
    if tags_excluded > 0:
        _print_kv(
            "Note",
            "Excluded tags are detections that failed quality filtering or did not form a connected map component.",
        )

    _print_section("Output Files")
    _print_kv("Map CSV", out_map)
    _print_kv("Map YAML", out_map_yaml)
    _print_kv("Trajectory CSV", out_traj)
    _print_kv("Raw trajectory CSV", out_traj_raw)
    _print_kv("Smoothed trajectory CSV", out_traj_smooth)
    _print_kv("Diagnostics CSV", out_diag)
    if lap_rows:
        _print_kv("Lap selection CSV", out_laps)
    if out_plot:
        _print_kv("3D plot PNG", out_plot)
    _print_kv("Interactive HTML", out_html)


if __name__ == "__main__":
    main()
