#!/usr/bin/env python3
import argparse
import os
import signal
import subprocess
import time


def start(cmd):
    return subprocess.Popen(cmd, start_new_session=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start teleop stack for real robot (base + joy + teleop + cmd_vel_curve)."
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
    parser.add_argument("--joy-device-id", type=int, default=0, help="Joystick device_id for joy_node.")
    parser.add_argument("--no-base", action="store_true", help="Do not start limo_base.")
    parser.add_argument("--port-name", default="ttyUSB1", help="LIMO serial port (e.g. ttyUSB1).")
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
                    "joy",
                    "joy_node",
                    "--ros-args",
                    "-p",
                    f"device_id:={args.joy_device_id}",
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
