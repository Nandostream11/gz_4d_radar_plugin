#!/usr/bin/env python3
"""Instantaneous radar ego-velocity estimation (Kellner et al., ITSC 2013).

For a static scatterer seen along unit line-of-sight u (sensor frame), the radar
measures range-rate  r_dot = -u . v_s  (closing negative), where v_s is the
velocity of the radar origin. Three detections determine v_s; RANSAC rejects
moving targets, clutter and folded returns, then least squares refines on the
inliers.

Usable as a library (``estimate_ego_velocity``) or as a ROS 2 node that turns
``radar_msgs/RadarScan`` into ``geometry_msgs/TwistStamped``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class EgoVelocityResult:
    velocity: np.ndarray          # (3,) sensor-frame velocity of the radar origin [m/s]
    inliers: np.ndarray           # boolean mask over input detections
    residual_rms: float           # RMS range-rate residual over inliers [m/s]


def unit_los(azimuth: np.ndarray, elevation: np.ndarray) -> np.ndarray:
    """Unit line-of-sight vectors, sensor frame (+x boresight, +y left, +z up)."""
    ce = np.cos(elevation)
    return np.stack([ce * np.cos(azimuth), ce * np.sin(azimuth), np.sin(elevation)], axis=1)


def estimate_ego_velocity(
    azimuth,
    elevation,
    range_rate,
    *,
    closing_positive: bool = False,
    inlier_threshold: float = 0.15,
    iterations: int = 150,
    min_inliers: int = 8,
    rng: np.random.Generator | None = None,
) -> EgoVelocityResult | None:
    az = np.asarray(azimuth, dtype=float)
    el = np.asarray(elevation, dtype=float)
    vr = np.asarray(range_rate, dtype=float)
    n = az.size
    if n < max(3, min_inliers):
        return None

    rng = rng or np.random.default_rng(0)
    U = unit_los(az, el)
    # r_dot = s * (u . v) with s = -1 (closing negative) or +1.
    A = U * (1.0 if closing_positive else -1.0)

    best_mask = None
    best_count = 0
    for _ in range(iterations):
        idx = rng.choice(n, size=3, replace=False)
        A3 = A[idx]
        if abs(np.linalg.det(A3)) < 1e-3:
            continue  # near-degenerate geometry
        v = np.linalg.solve(A3, vr[idx])
        mask = np.abs(A @ v - vr) < inlier_threshold
        c = int(mask.sum())
        if c > best_count:
            best_count, best_mask = c, mask

    if best_mask is None or best_count < min_inliers:
        return None

    # Refine twice: least squares on inliers, then re-select inliers.
    mask = best_mask
    v = None
    for _ in range(2):
        v, *_ = np.linalg.lstsq(A[mask], vr[mask], rcond=None)
        new_mask = np.abs(A @ v - vr) < inlier_threshold
        if new_mask.sum() < min_inliers:
            break
        mask = new_mask
    res = A[mask] @ v - vr[mask]
    return EgoVelocityResult(velocity=v, inliers=mask, residual_rms=float(np.sqrt(np.mean(res ** 2))))


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from radar_msgs.msg import RadarScan
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class EgoVelocityNode(Node):
        def __init__(self) -> None:
            super().__init__("radar_ego_velocity_estimator")
            self.declare_parameter("scan_topic", "/radar/scan")
            self.declare_parameter("output_topic", "/radar/ego_velocity")
            self.declare_parameter("inlier_threshold", 0.15)
            self.declare_parameter("closing_velocity_positive", False)
            self._thr = float(self.get_parameter("inlier_threshold").value)
            self._closing = bool(self.get_parameter("closing_velocity_positive").value)
            self._pub = self.create_publisher(
                TwistStamped, self.get_parameter("output_topic").value, 10)
            self.create_subscription(
                RadarScan, self.get_parameter("scan_topic").value, self._on_scan,
                qos_profile_sensor_data)

        def _on_scan(self, msg: RadarScan) -> None:
            if not msg.returns:
                return
            az = [r.azimuth for r in msg.returns]
            el = [r.elevation for r in msg.returns]
            vr = [r.doppler_velocity for r in msg.returns]
            est = estimate_ego_velocity(az, el, vr, closing_positive=self._closing,
                                        inlier_threshold=self._thr)
            if est is None:
                return
            out = TwistStamped()
            out.header = msg.header
            out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = map(float, est.velocity)
            self._pub.publish(out)

    rclpy.init()
    node = EgoVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
