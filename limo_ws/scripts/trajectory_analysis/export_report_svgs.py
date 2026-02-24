#!/usr/bin/env python3
import argparse
import shutil
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x, y, z, w = qx, qy, qz, qw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def make_recording_plots(output_dir: Path, rec_dir: Path) -> None:
    traj_csv = output_dir / "trajetoria_camera.csv"
    tags_csv = output_dir / "mapa_tags.csv"
    if not traj_csv.exists() or not tags_csv.exists():
        return

    traj = pd.read_csv(traj_csv)
    tags = pd.read_csv(tags_csv)

    # Auto-select the 2D plotting plane from the two axes with largest variation.
    axes = ["x", "y", "z"]
    traj_xyz = traj[axes].to_numpy(float)
    span = np.ptp(traj_xyz, axis=0)
    idx_sorted = np.argsort(span)[::-1]
    a_idx, b_idx = int(idx_sorted[0]), int(idx_sorted[1])
    a_name, b_name = axes[a_idx], axes[b_idx]

    pa = traj[a_name].to_numpy(float)
    pb = traj[b_name].to_numpy(float)
    ta = tags[a_name].to_numpy(float)
    tb = tags[b_name].to_numpy(float)
    tag_labels = tags["tag"].astype(str).tolist()

    # 1) Basic trajectory + tags
    fig, ax = plt.subplots(figsize=(8.2, 6.0))
    ax.plot(pa, pb, color="#1f77b4", linewidth=2.0, label="camera trajectory")
    ax.scatter([pa[0]], [pb[0]], s=36, color="green", label="start", zorder=4)
    ax.scatter([pa[-1]], [pb[-1]], s=36, color="red", label="end", zorder=4)
    ax.scatter(ta, tb, s=24, color="orange", label="tags", zorder=3)
    for p1, p2, lab in zip(ta, tb, tag_labels):
        ax.text(p1, p2, lab, fontsize=8)
    ax.set_title("AprilTag Environment Reconstruction and Estimated Trajectory")
    ax.set_xlabel(f"{a_name.upper()} [m]")
    ax.set_ylabel(f"{b_name.upper()} [m]")
    ax.grid(alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(
        loc="lower left",
        bbox_to_anchor=(1.02, 0.0),
        fontsize=9,
        frameon=False,
        borderaxespad=0.0,
    )
    fig.tight_layout(rect=[0.0, 0.0, 0.82, 1.0])
    fig.savefig(rec_dir / "recording_trajectory_basic.svg", format="svg")
    plt.close(fig)

    # 2) Trajectory + sampled pose axes
    fig, ax = plt.subplots(figsize=(8.2, 6.0))
    ax.plot(pa, pb, color="#1f77b4", linewidth=2.0, label="camera trajectory")
    ax.scatter([pa[0]], [pb[0]], s=36, color="green", label="start", zorder=4)
    ax.scatter([pa[-1]], [pb[-1]], s=36, color="red", label="end", zorder=4)
    ax.scatter(ta, tb, s=24, color="orange", label="tags", zorder=3)
    for p1, p2, lab in zip(ta, tb, tag_labels):
        ax.text(p1, p2, lab, fontsize=8)

    step = max(1, len(traj) // 10)
    pose_x = traj["x"].to_numpy(float)
    pose_y = traj["y"].to_numpy(float)
    pose_z = traj["z"].to_numpy(float)
    qx = traj["qx"].to_numpy(float)
    qy = traj["qy"].to_numpy(float)
    qz = traj["qz"].to_numpy(float)
    qw = traj["qw"].to_numpy(float)

    ax_coord = {"x": pose_x, "y": pose_y, "z": pose_z}
    ax.scatter(ax_coord[a_name][::step], ax_coord[b_name][::step], s=20, color="purple", label="sampled poses", zorder=5)
    # Scale axis arrows with trajectory size so TF is visible across different maps.
    span_a = float(np.ptp(pa)) if len(pa) else 1.0
    span_b = float(np.ptp(pb)) if len(pb) else 1.0
    axis_len = max(0.12, 0.05 * max(span_a, span_b))
    head_w = max(0.02, 0.010 * max(span_a, span_b))
    head_l = max(0.03, 0.015 * max(span_a, span_b))
    first_x = True
    first_z = True
    for i in range(0, len(traj), step):
        r = quat_to_rotmat(qx[i], qy[i], qz[i], qw[i])
        # Plot camera-frame X and Z axes projected on selected 2D plane.
        vx = r[:, 0]
        vz = r[:, 2]
        origin_a = ax_coord[a_name][i]
        origin_b = ax_coord[b_name][i]
        comp = {"x": 0, "y": 1, "z": 2}
        ia = comp[a_name]
        ib = comp[b_name]
        ax.arrow(
            origin_a,
            origin_b,
            axis_len * vx[ia],
            axis_len * vx[ib],
            head_width=head_w,
            head_length=head_l,
            color="red",
            length_includes_head=True,
            alpha=0.9,
            label="pose axis X" if first_x else None,
        )
        ax.arrow(
            origin_a,
            origin_b,
            axis_len * vz[ia],
            axis_len * vz[ib],
            head_width=head_w,
            head_length=head_l,
            color="blue",
            length_includes_head=True,
            alpha=0.9,
            label="pose axis Z" if first_z else None,
        )
        first_x = False
        first_z = False

    ax.set_title("AprilTag Environment Reconstruction and Estimated Trajectory")
    ax.set_xlabel(f"{a_name.upper()} [m]")
    ax.set_ylabel(f"{b_name.upper()} [m]")
    ax.grid(alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(
        loc="lower left",
        bbox_to_anchor=(1.02, 0.0),
        fontsize=9,
        frameon=False,
        borderaxespad=0.0,
    )
    fig.tight_layout(rect=[0.0, 0.0, 0.82, 1.0])
    fig.savefig(rec_dir / "recording_trajectory_with_axes.svg", format="svg")
    plt.close(fig)


def make_relocalization_plots(reloc_source_dir: Path, reloc_dir: Path) -> None:
    # Reuse didactic style when available
    point_svg = reloc_source_dir / "distance_angle_point_plot.svg"
    point_pdf = reloc_source_dir / "distance_angle_point_plot.pdf"
    if point_svg.exists():
        shutil.copy2(point_svg, reloc_dir / "relocalization_point_didactic.svg")
    elif point_pdf.exists():
        # Keep SVG-only policy; do not convert PDF automatically.
        pass

    metrics_csv = reloc_source_dir / "distance_angle_metrics.csv"
    if not metrics_csv.exists():
        return
    df = pd.read_csv(metrics_csv)
    if df.empty:
        return

    # If trajectory metrics are available, create time-series SVGs
    if "t_rel_s" in df.columns and "distance_to_ref_m" in df.columns and len(df) > 1:
        fig, ax = plt.subplots(figsize=(8.4, 4.2))
        ax.plot(df["t_rel_s"].to_numpy(), df["distance_to_ref_m"].to_numpy(), color="#1f77b4", linewidth=1.6)
        ax.set_title("Relocalization Distance to Reference Trajectory")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Distance [m]")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(reloc_dir / "relocalization_distance_timeseries.svg", format="svg")
        plt.close(fig)

    if "t_rel_s" in df.columns and "heading_error_deg" in df.columns and len(df) > 1:
        fig, ax = plt.subplots(figsize=(8.4, 4.2))
        ax.plot(df["t_rel_s"].to_numpy(), df["heading_error_deg"].to_numpy(), color="#d62728", linewidth=1.4)
        ax.set_title("Relocalization Heading Error")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Heading Error [deg]")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(reloc_dir / "relocalization_heading_error_timeseries.svg", format="svg")
        plt.close(fig)


def process_output_dir(output_dir: Path, reloc_source_dir: Path | None) -> None:
    report_root = output_dir / "report_svgs"
    rec_dir = report_root / "recording"
    rel_dir = report_root / "relocalization"
    rec_dir.mkdir(parents=True, exist_ok=True)
    rel_dir.mkdir(parents=True, exist_ok=True)

    make_recording_plots(output_dir, rec_dir)
    make_relocalization_plots(reloc_source_dir or output_dir, rel_dir)

    print(f"[ok] {output_dir}")
    print(f"  recording:      {rec_dir}")
    print(f"  relocalization: {rel_dir}")


def main() -> None:
    p = argparse.ArgumentParser(description="Export standardized SVG report assets (recording + relocalization).")
    p.add_argument("output_dir", help="Output directory containing mapping files (trajetoria_camera.csv, mapa_tags.csv)")
    p.add_argument(
        "--relocalization-source",
        default="",
        help="Directory with relocalization files (distance_angle_point_plot.svg, distance_angle_metrics.csv). "
        "If omitted, uses output_dir.",
    )
    p.add_argument(
        "--batch-root",
        default="",
        help="Optional outputs root; if provided, process all first-level directories that contain trajetoria_camera.csv",
    )
    args = p.parse_args()

    if args.batch_root:
        root = Path(args.batch_root)
        for d in sorted(root.iterdir()):
            if not d.is_dir():
                continue
            has_recording = (d / "trajetoria_camera.csv").exists() and (d / "mapa_tags.csv").exists()
            has_relocalization = (d / "distance_angle_point_plot.svg").exists() or (d / "distance_angle_metrics.csv").exists()
            if has_recording or has_relocalization:
                process_output_dir(d, None)
        return

    out_dir = Path(args.output_dir)
    reloc_dir = Path(args.relocalization_source) if args.relocalization_source else None
    process_output_dir(out_dir, reloc_dir)


if __name__ == "__main__":
    main()
