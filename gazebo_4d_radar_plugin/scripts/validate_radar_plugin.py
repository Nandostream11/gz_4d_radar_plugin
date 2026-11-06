#!/usr/bin/env python3
"""End-to-end validation of the 4D radar plugin inside Gazebo.

Implements the validation plan from the design handoff:

  1. Static platform           -> all range-rates zero within noise
  2. Translation toward a wall -> r_dot = -u . v  (cosine profile over azimuth)
  3. Pure rotation about the radar origin -> range-rates stay zero
  4. Velocity ramp through v_max -> folding happens at the right speed / direction
  5. Round trip                -> RANSAC ego-velocity from the scans matches truth
  6. Moving target             -> returns from a moving box carry its velocity
                                  (separate world: radar_moving_target_world.sdf)

Expected values are computed independently of the plugin: the platform is driven
by gz-sim VelocityControl with a BODY-frame command and the radar frame equals
the body frame, so the true radar velocity in its own frame is the command itself.

Usage (workspace sourced):
    ros2 run gazebo_4d_radar_plugin validate_radar_plugin.py            # launches gz
    ros2 run gazebo_4d_radar_plugin validate_radar_plugin.py --no-launch
Exit code 0 when every check passes.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from radar_msgs.msg import RadarScan
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from radar_ego_velocity_estimator import estimate_ego_velocity, unit_los  # noqa: E402

V_MAX = 55.55
TOL = 0.20          # [m/s] per-detection tolerance (range-rate sigma 0.05 + 0.01 quantisation)
MIN_FRACTION = 0.97  # false detections (2/cycle) + dropouts must not exceed this budget
CMD_TOPIC = "/model/ego_radar_platform/cmd_vel"


def fold(v, v_max=V_MAX):
    return np.mod(v + v_max, 2 * v_max) - v_max


@dataclass
class Scan:
    stamp: float
    az: np.ndarray
    el: np.ndarray
    vr: np.ndarray
    rng: np.ndarray


@dataclass
class PhaseResult:
    name: str
    passed: bool
    metrics: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("radar_plugin_validator")
        self.scans: list[Scan] = []
        self.truth: dict[float, np.ndarray] = {}
        self.latest_truth: tuple[np.ndarray, np.ndarray] | None = None  # (linear, angular)
        self.create_subscription(RadarScan, "/radar/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/radar/ground_truth_velocity", self._on_truth,
                                 qos_profile_sensor_data)

    @staticmethod
    def _t(h) -> float:
        return h.stamp.sec + 1e-9 * h.stamp.nanosec

    def _on_scan(self, m: RadarScan) -> None:
        r = m.returns
        self.scans.append(Scan(
            self._t(m.header),
            np.array([x.azimuth for x in r]), np.array([x.elevation for x in r]),
            np.array([x.doppler_velocity for x in r]), np.array([x.range for x in r])))

    def _on_truth(self, m: TwistStamped) -> None:
        l, a = m.twist.linear, m.twist.angular
        lin = np.array([l.x, l.y, l.z])
        self.truth[round(self._t(m.header), 4)] = lin
        self.latest_truth = (lin, np.array([a.x, a.y, a.z]))

    def latest_stamp(self) -> float | None:
        return self.scans[-1].stamp if self.scans else None


def send_cmd(node: "Collector", linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0),
             topic: str = CMD_TOPIC) -> None:
    """Publish a body-frame velocity command, servicing ROS callbacks meanwhile so
    no radar scans are dropped while `gz topic` is starting up."""
    msg = (f"linear: {{x: {linear[0]}, y: {linear[1]}, z: {linear[2]}}}, "
           f"angular: {{x: {angular[0]}, y: {angular[1]}, z: {angular[2]}}}")
    p = subprocess.Popen(["gz", "topic", "-t", topic, "-m", "gz.msgs.Twist", "-p", msg])
    end = time.monotonic() + 30
    while p.poll() is None and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    if p.poll() is None:
        p.kill()
        raise RuntimeError("gz topic did not return")
    if p.returncode != 0:
        raise RuntimeError(f"gz topic failed with exit code {p.returncode}")


def spin_until(node: Collector, pred, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


def collect_phase(node: Collector, linear, angular, settle: float, duration: float) -> list[Scan]:
    """Command a motion and return the scans from the steady-state window (sim time).

    `gz topic` takes a variable amount of wall time to deliver the command, so the
    window is anchored on the moment the plugin's ground-truth velocity shows the
    command in effect (this only gates timing; the checks use the command itself).
    """
    send_cmd(node, linear, angular)
    lin_cmd, ang_cmd = np.asarray(linear, float), np.asarray(angular, float)

    def applied() -> bool:
        if node.latest_truth is None:
            return False
        lin, ang = node.latest_truth
        return bool(np.allclose(lin, lin_cmd, atol=1e-2) and np.allclose(ang, ang_cmd, atol=1e-2))

    if not spin_until(node, applied, timeout=60.0):
        raise RuntimeError(f"command linear={linear} angular={angular} never took effect")
    t_cmd = node.latest_stamp()
    start, stop = t_cmd + settle, t_cmd + settle + duration
    ok = spin_until(node, lambda: node.latest_stamp() >= stop, timeout=60.0 + 10 * duration)
    if not ok:
        raise RuntimeError("timed out waiting for radar scans (is the simulation running?)")
    return [s for s in node.scans if start <= s.stamp <= stop]


def concat(scans: list[Scan]):
    return (np.concatenate([s.az for s in scans]), np.concatenate([s.el for s in scans]),
            np.concatenate([s.vr for s in scans]), np.concatenate([s.rng for s in scans]))


def doppler_check(name: str, scans: list[Scan], v_sensor, *, folded: bool) -> PhaseResult:
    az, el, vr, _ = concat(scans)
    true_rr = -(unit_los(az, el) @ np.asarray(v_sensor, dtype=float))
    expected = fold(true_rr) if folded else true_rr
    resid = vr - expected
    frac = float(np.mean(np.abs(resid) < TOL))
    inl = np.abs(resid) < TOL
    res = PhaseResult(name, frac >= MIN_FRACTION, {
        "scans": len(scans),
        "detections": int(vr.size),
        "fraction_within_tol": round(frac, 4),
        "residual_rms_inliers": round(float(np.sqrt(np.mean(resid[inl] ** 2))), 4) if inl.any() else None,
    })
    if not scans:
        res.passed = False
        res.notes.append("no scans collected")
    return res


def share_world(name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("gazebo_4d_radar_plugin"), "worlds", name)


def launch_gz(world: str) -> subprocess.Popen:
    return subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "2", world], start_new_session=True)


def stop_gz(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)


def moving_target_check(node: "Collector") -> PhaseResult:
    """6. A box driven at -4 m/s toward a parked, level radar (no other geometry in
    the world): every return must report r_dot = v_target . u."""
    v_t = np.array([-4.0, 0.0, 0.0])
    node.scans.clear()
    if not spin_until(node, lambda: len(node.scans) > 5, timeout=120.0):
        return PhaseResult("6_moving_target_doppler", False, notes=["no scans"])
    send_cmd(node, tuple(v_t), topic="/model/target_box/cmd_vel")
    spin_until(node, lambda: False, timeout=1.0)   # let the command take effect
    i0 = len(node.scans)
    spin_until(node, lambda: False, timeout=1.5)
    scans = [s for s in node.scans[i0:] if s.vr.size]
    res = doppler_check("6_moving_target_doppler", scans, -v_t, folded=False)
    # Sensor static, target moving: equivalent to the sensor moving at -v_t.
    res.metrics["detections_per_scan"] = round(res.metrics["detections"] / max(1, len(scans)), 1)
    if res.metrics["detections_per_scan"] < 5:
        res.passed = False
        res.notes.append("target box barely detected")
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-launch", action="store_true",
                    help="use an already running gz sim (skips the moving-target world)")
    ap.add_argument("--world", default=None, help="world file (default: radar_validation_world.sdf)")
    ap.add_argument("--report", default=None, help="write JSON report here")
    ap.add_argument("--plot", default=None, help="write a PNG of the Doppler profiles here")
    args = ap.parse_args()

    gz_proc = None
    if not args.no_launch:
        if not shutil.which("gz"):
            print("gz not found on PATH", file=sys.stderr)
            return 2
        gz_proc = launch_gz(args.world or share_world("radar_validation_world.sdf"))

    rclpy.init()
    node = Collector()
    results: list[PhaseResult] = []
    plot_data = {}
    try:
        if not spin_until(node, lambda: len(node.scans) > 5, timeout=120.0):
            raise RuntimeError("no /radar/scan messages received")
        send_cmd(node)  # make sure the platform is at rest

        settle, dur = 0.3, 1.0

        s1 = collect_phase(node, (0, 0, 0), (0, 0, 0), settle, dur)
        results.append(doppler_check("1_static_platform", s1, (0, 0, 0), folded=False))

        v2 = (5.0, 0.0, 0.0)
        s2 = collect_phase(node, v2, (0, 0, 0), settle, 1.5)
        r2 = doppler_check("2_translation_cosine_profile", s2, v2, folded=False)
        # The wall is the only thing above the radar horizon: it must be detected.
        az, el, vr, rg = concat(s2)
        above = el > 0.01
        r2.metrics["wall_detections_per_scan"] = round(float(above.sum()) / max(1, len(s2)), 1)
        if r2.metrics["wall_detections_per_scan"] < 50:
            r2.passed = False
            r2.notes.append("too few wall detections - real geometry not being ray-cast?")
        results.append(r2)
        plot_data["translation"] = (az[above], el[above], vr[above], v2)

        v4a = (-50.0, 0.0, 0.0)
        s4a = collect_phase(node, v4a, (0, 0, 0), settle, 0.7)
        r4a = doppler_check("4a_ramp_below_vmax_no_folding", s4a, v4a, folded=False)
        results.append(r4a)

        v4b = (-60.0, 0.0, 0.0)
        s4b = collect_phase(node, v4b, (0, 0, 0), settle, 0.7)
        r4b = doppler_check("4b_ramp_above_vmax_folding", s4b, v4b, folded=True)
        az, el, vr, _ = concat(s4b)
        true_rr = -(unit_los(az, el) @ np.asarray(v4b))
        should_fold = np.abs(true_rr) > V_MAX
        r4b.metrics["fraction_expected_folded"] = round(float(should_fold.mean()), 3)
        if should_fold.any():
            # Folded returns must wrap to the negative side (receding +60 -> about -51).
            r4b.metrics["mean_reported_where_folded"] = round(float(vr[should_fold].mean()), 2)
            if not (vr[should_fold].mean() < 0):
                r4b.passed = False
                r4b.notes.append("folded returns did not wrap to negative range-rate")
        else:
            r4b.passed = False
            r4b.notes.append("scenario produced no returns beyond v_max")
        results.append(r4b)
        plot_data["folding"] = (az, el, vr, v4b)

        # 5. Round trip on scans from phases 2 and 4a (unfolded, two speeds).
        errs, gt_errs = [], []
        rng = np.random.default_rng(1)
        for scans, v in ((s2, v2), (s4a, v4a)):
            for s in scans:
                est = estimate_ego_velocity(s.az, s.el, s.vr, rng=rng)
                if est is None:
                    continue
                errs.append(float(np.linalg.norm(est.velocity - np.asarray(v))))
                gt = node.truth.get(round(s.stamp, 4))
                if gt is not None:
                    gt_errs.append(float(np.linalg.norm(gt - np.asarray(v))))
        r5 = PhaseResult("5_round_trip_ransac_ego_velocity", False)
        if errs:
            e = np.array(errs)
            r5.metrics = {"estimates": len(errs), "median_error_mps": round(float(np.median(e)), 4),
                          "p95_error_mps": round(float(np.percentile(e, 95)), 4),
                          "ground_truth_topic_max_error_mps":
                              round(max(gt_errs), 5) if gt_errs else None}
            r5.passed = (np.median(e) < 0.05 and np.percentile(e, 95) < 0.10
                         and bool(gt_errs) and max(gt_errs) < 1e-3)
        else:
            r5.notes.append("RANSAC produced no estimates")
        results.append(r5)

        # 3. Pure yaw about the radar origin (last: it changes the heading).
        s3 = collect_phase(node, (0, 0, 0), (0, 0, 1.0), settle, dur)
        results.append(doppler_check("3_pure_rotation_zero_doppler", s3, (0, 0, 0), folded=False))
        send_cmd(node)

        if not args.no_launch:
            stop_gz(gz_proc)
            gz_proc = launch_gz(share_world("radar_moving_target_world.sdf"))
            results.append(moving_target_check(node))

    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_gz(gz_proc)

    results.sort(key=lambda r: r.name)
    width = max(len(r.name) for r in results)
    print("\n4D radar plugin validation")
    print("=" * (width + 10))
    for r in results:
        print(f"{'PASS' if r.passed else 'FAIL'}  {r.name:<{width}}  {json.dumps(r.metrics)}")
        for n in r.notes:
            print(f"      note: {n}")
    all_ok = all(r.passed for r in results)
    print("=" * (width + 10))
    print("ALL CHECKS PASSED" if all_ok else "SOME CHECKS FAILED")

    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump([r.__dict__ for r in results], f, indent=2)
    if args.plot and plot_data:
        make_plot(plot_data, args.plot)
    return 0 if all_ok else 1


def make_plot(data: dict, path: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    if "translation" in data:
        az, el, vr, v = data["translation"]
        ax = axes[0]
        sel = np.abs(el - np.median(el)) < 0.02
        ax.scatter(np.degrees(az[sel]), vr[sel], s=6, alpha=0.5, label="plugin detections")
        a = np.linspace(az[sel].min(), az[sel].max(), 200)
        ax.plot(np.degrees(a), -v[0] * np.cos(np.median(el)) * np.cos(a), "k", lw=1.5,
                label=r"analytic $-v\,\cos(el)\cos(az)$")
        ax.set_ylim(-v[0] - 1.0, 0.5)  # false detections fall outside; they are clutter
        ax.set_title(f"Wall returns, platform at {v[0]:.0f} m/s toward wall")
        ax.set_xlabel("azimuth [deg]")
        ax.set_ylabel("range-rate [m/s]")
        ax.legend(loc="lower center")
    if "folding" in data:
        az, el, vr, v = data["folding"]
        ax = axes[1]
        true_rr = -(unit_los(az, el) @ np.asarray(v))
        ax.scatter(true_rr, vr, s=4, alpha=0.4, label="plugin detections")
        t = np.linspace(true_rr.min(), true_rr.max(), 400)
        ax.plot(t, fold(t), "k", lw=1.2, label="analytic folding")
        ax.axvline(V_MAX, color="tab:red", ls="--", lw=1, label=r"$v_{max}$")
        ax.set_title(f"Doppler folding at {abs(v[0]):.0f} m/s (v_max = {V_MAX} m/s)")
        ax.set_xlabel("true range-rate [m/s]")
        ax.set_ylabel("reported range-rate [m/s]")
        ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"plot written to {path}")


if __name__ == "__main__":
    sys.exit(main())
