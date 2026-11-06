#!/usr/bin/env python3
"""Drives the radar_street_demo scenario in a loop.

Each lap: the ego vehicle drives down the street at --ego-speed while the
oncoming car approaches in the other lane at --oncoming-speed; after --lap
seconds both are put back at their start poses. Optionally asks the Gazebo GUI
camera to follow the ego vehicle.

Uses the `gz` command-line tools, so it only needs a running `gz sim`.
"""

import argparse
import subprocess
import sys
import threading
import time

WORLD = "radar_street_demo"
EGO_START = "position: {x: 0, y: -2.5, z: 0.8}, orientation: {w: 1}"
ONCOMING_START = "position: {x: 75, y: 2.5, z: 0.8}, orientation: {z: 1, w: 0}"


def gz(*args: str, timeout: float = 15.0) -> bool:
    try:
        return subprocess.run(["gz", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              timeout=timeout).returncode == 0
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def cmd_vel(model: str, vx: float) -> None:
    gz("topic", "-t", f"/model/{model}/cmd_vel", "-m", "gz.msgs.Twist", "-p", f"linear: {{x: {vx}}}")


def set_pose(model: str, pose: str) -> None:
    gz("service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
       "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{model}", {pose}')


def wait_for_service(name: str, timeout: float = 120.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        out = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True).stdout
        if name in out.split():
            return True
        time.sleep(1.0)
    return False


def wait_for_world(timeout: float = 120.0) -> bool:
    return wait_for_service(f"/world/{WORLD}/set_pose", timeout)


def follow_ego() -> None:
    """Make the Gazebo GUI camera chase the ego vehicle (gz-sim 8 /gui/track topic)."""
    if not wait_for_service("/gui/follow", timeout=180.0):  # GUI camera plugin is up
        return
    time.sleep(2.0)
    msg = ('track_mode: FOLLOW_LOOK_AT, '
           'follow_target: {name: "ego_vehicle"}, track_target: {name: "ego_vehicle"}, '
           'follow_offset: {x: -12, y: -3, z: 10}, follow_pgain: 0.05, track_pgain: 0.05')
    for _ in range(3):  # topic publish is fire-and-forget; repeat to be safe
        gz("topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p", msg)
        time.sleep(1.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ego-speed", type=float, default=6.0)
    ap.add_argument("--oncoming-speed", type=float, default=7.0)
    ap.add_argument("--lap", type=float, default=11.0, help="seconds per lap")
    ap.add_argument("--laps", type=int, default=0, help="0 = loop forever")
    ap.add_argument("--follow", action="store_true", help="make the GUI camera follow the ego vehicle")
    args, _ = ap.parse_known_args()   # tolerate ROS launch arguments

    if not wait_for_world():
        print("demo_driver: Gazebo world not found", file=sys.stderr)
        return 1
    if args.follow:
        # The GUI loads later than the server; attach the camera in the background.
        threading.Thread(target=follow_ego, daemon=True).start()

    lap = 0
    while args.laps == 0 or lap < args.laps:
        set_pose("ego_vehicle", EGO_START)
        set_pose("oncoming_car", ONCOMING_START)
        # Oncoming car faces -x (yaw 180 deg), so +x in its body frame is toward the ego.
        cmd_vel("ego_vehicle", args.ego_speed)
        cmd_vel("oncoming_car", args.oncoming_speed)
        print(f"demo_driver: lap {lap + 1} started", flush=True)
        time.sleep(args.lap)
        lap += 1
    cmd_vel("ego_vehicle", 0.0)
    cmd_vel("oncoming_car", 0.0)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
