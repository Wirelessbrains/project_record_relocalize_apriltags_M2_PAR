#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import time


def start(cmd):
    return subprocess.Popen(cmd, start_new_session=True)


def main() -> int:
    ws_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    default_calib = os.path.join(
        ws_root, "src", "limo_apriltag_tools", "config", "webcam_calibration_robot.yaml"
    )
    default_apriltag = os.path.join(
        ws_root, "src", "limo_apriltag_tools", "config", "apriltag_params.yaml"
    )

    parser = argparse.ArgumentParser(
        description="Start teleop + camera + apriltag for real robot (no Ignition)."
    )
    parser.add_argument(
        "--cmd-topic",
        default="/cmd_vel",
        help="Robot cmd_vel topic.",
    )
    parser.add_argument(
        "--cmd-raw-topic",
        default="/cmd_vel_raw",
        help="Raw cmd_vel topic before curve.",
    )
    parser.add_argument("--axis-linear", type=int, default=1, help="Linear axis index.")
    parser.add_argument("--axis-angular", type=int, default=0, help="Angular axis index.")
    parser.add_argument("--scale-linear", type=float, default=0.7, help="Linear scale.")
    parser.add_argument("--scale-angular", type=float, default=0.4, help="Angular scale.")
    parser.add_argument("--linear-gain", type=float, default=1.0, help="Cmd curve linear gain.")
    parser.add_argument("--angular-gain", type=float, default=2.5, help="Cmd curve angular gain.")
    parser.add_argument("--linear-exponent", type=float, default=1.0, help="Cmd curve linear exponent.")
    parser.add_argument("--angular-exponent", type=float, default=1.0, help="Cmd curve angular exponent.")
    parser.add_argument("--linear-deadzone", type=float, default=0.03, help="Cmd curve linear deadzone.")
    parser.add_argument("--angular-deadzone", type=float, default=0.05, help="Cmd curve angular deadzone.")
    parser.add_argument("--joy-deadzone", type=float, default=0.08, help="Joystick deadzone.")
    parser.add_argument("--autorepeat", type=float, default=50.0, help="joy_node autorepeat rate.")
    parser.add_argument("--coalesce", type=float, default=0.01, help="joy_node coalesce interval.")
    parser.add_argument(
        "--no-apriltag",
        action="store_true",
        help="Do not start the apriltag detection node.",
    )
    parser.add_argument(
        "--no-sync",
        action="store_true",
        help="Do not start camera_info sync node (uses /image_raw + /camera_info directly).",
    )
    parser.add_argument("--no-base", action="store_true", help="Do not start limo_base.")
    parser.add_argument("--port-name", default="ttyUSB1", help="LIMO serial port (e.g. ttyUSB1).")
    parser.add_argument("--video-device", default="/dev/video0", help="V4L2 device.")
    parser.add_argument(
        "--camera-info-url",
        default=f"file://{default_calib}",
        help="Camera calibration YAML (file://...).",
    )
    parser.add_argument(
        "--apriltag-params",
        default=default_apriltag,
        help="apriltag_ros params file.",
    )
    parser.add_argument(
        "--image-sync-topic",
        default="/image_raw_sync",
        help="Synced image topic for apriltag detection.",
    )
    parser.add_argument(
        "--camera-info-sync-topic",
        default="/camera_info_sync",
        help="Synced camera_info topic for apriltag detection.",
    )
    args = parser.parse_args()

    processes = []
    try:
        if not args.no_base:
            processes.append(
                start(
                    [
                        "ros2",
                        "launch",
                        "limo_base",
                        "limo_base.launch.py",
                        f"port_name:={args.port_name}",
                    ]
                )
            )

        processes.append(
            start(
                [
                    "ros2",
                    "run",
                    "v4l2_camera",
                    "v4l2_camera_node",
                    "--ros-args",
                    "-p",
                    f"video_device:={args.video_device}",
                    "-p",
                    "image_size:=[640,480]",
                    "-p",
                    f"camera_info_url:={args.camera_info_url}",
                ]
            )
        )

        if not args.no_sync:
            processes.append(
                start(
                    [
                        "ros2",
                        "run",
                        "limo_apriltag_tools",
                        "camera_info_publisher_node",
                        "--ros-args",
                        "-p",
                        f"calibration_file:={args.camera_info_url.replace('file://', '')}",
                        "-p",
                        "image_topic:=/image_raw",
                        "-p",
                        f"image_output_topic:={args.image_sync_topic}",
                        "-p",
                        f"camera_info_topic:={args.camera_info_sync_topic}",
                        "-p",
                        "relay_image:=true",
                    ]
                )
            )

        if not args.no_apriltag:
            image_topic = "/image_raw" if args.no_sync else args.image_sync_topic
            camera_info_topic = "/camera_info" if args.no_sync else args.camera_info_sync_topic
            processes.append(
                start(
                    [
                        "ros2",
                        "run",
                        "apriltag_ros",
                        "apriltag_node",
                        "--ros-args",
                        "--params-file",
                        args.apriltag_params,
                        "--remap",
                        f"image_rect:={image_topic}",
                        "--remap",
                        f"camera_info:={camera_info_topic}",
                    ]
                )
            )

        processes.append(
            start(
                [
                    "ros2",
                    "run",
                    "joy",
                    "joy_node",
                    "--ros-args",
                    "-p",
                    f"autorepeat_rate:={args.autorepeat}",
                    "-p",
                    f"coalesce_interval:={args.coalesce}",
                    "-p",
                    f"deadzone:={args.joy_deadzone}",
                ]
            )
        )

        processes.append(
            start(
                [
                    "ros2",
                    "run",
                    "teleop_twist_joy",
                    "teleop_node",
                    "--ros-args",
                    "-r",
                    f"/cmd_vel:={args.cmd_raw_topic}",
                    "-p",
                    "require_enable_button:=false",
                    "-p",
                    f"axis_linear.x:={args.axis_linear}",
                    "-p",
                    f"axis_angular.yaw:={args.axis_angular}",
                    "-p",
                    f"scale_linear.x:={args.scale_linear}",
                    "-p",
                    f"scale_angular.yaw:={args.scale_angular}",
                ]
            )
        )

        processes.append(
            start(
                [
                    "python3",
                    os.path.join(os.path.dirname(__file__), "cmd_vel_curve.py"),
                    "--input",
                    args.cmd_raw_topic,
                    "--output",
                    args.cmd_topic,
                    "--linear-gain",
                    str(args.linear_gain),
                    "--angular-gain",
                    str(args.angular_gain),
                    "--linear-exponent",
                    str(args.linear_exponent),
                    "--angular-exponent",
                    str(args.angular_exponent),
                    "--linear-deadzone",
                    str(args.linear_deadzone),
                    "--angular-deadzone",
                    str(args.angular_deadzone),
                ]
            )
        )

        while True:
            time.sleep(0.5)
            for proc in list(processes):
                if proc.poll() is not None:
                    cmd = getattr(proc, "args", ["<unknown>"])
                    print(f"Process exited: {cmd}")
                    processes.remove(proc)
            if not processes:
                return 0
    except KeyboardInterrupt:
        pass
    finally:
        for proc in processes:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        time.sleep(0.5)
        for proc in processes:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
