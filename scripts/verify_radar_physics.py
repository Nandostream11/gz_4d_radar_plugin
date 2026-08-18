#!/usr/bin/env python3
"""
Unit tests for 4D Radar physics model.
Validates Doppler kinematics, Doppler folding, rotation invariance, and RCS attenuation.
"""

import math
import unittest


def fold_doppler(v_true: float, v_max: float) -> float:
    if v_max <= 0.0:
        return v_true
    period = 2.0 * v_max
    val = math.fmod(v_true + v_max, period)
    if val < 0.0:
        val += period
    return val - v_max


def compute_range_rate(v_sensor: list, u_los: list, closing_positive: bool = False) -> float:
    dot = v_sensor[0] * u_los[0] + v_sensor[1] * u_los[1] + v_sensor[2] * u_los[2]
    return dot if closing_positive else -dot


def compute_apparent_rcs(rcs_nominal: float, range_m: float, cos_inc: float, falloff_exp: float = 4.0) -> float:
    cos_clamped = max(0.02, min(1.0, cos_inc))
    r_norm = max(1.0, range_m / 10.0)
    return rcs_nominal + 10.0 * math.log10(cos_clamped) - 10.0 * falloff_exp * math.log10(r_norm)


class TestRadarPhysics(unittest.TestCase):

    def test_static_platform_zero_doppler(self):
        v_static = [0.0, 0.0, 0.0]
        angles = [-math.pi / 4, -math.pi / 6, 0.0, math.pi / 6, math.pi / 4]
        for az in angles:
            u_los = [math.cos(az), math.sin(az), 0.0]
            r_dot = compute_range_rate(v_static, u_los)
            self.assertAlmostEqual(r_dot, 0.0, places=9)

    def test_pure_translation_cosine_profile(self):
        v_forward = [20.0, 0.0, 0.0]
        for deg in range(-50, 51, 10):
            az = math.radians(deg)
            u_los = [math.cos(az), math.sin(az), 0.0]
            r_dot = compute_range_rate(v_forward, u_los)
            expected_r_dot = -20.0 * math.cos(az)
            self.assertAlmostEqual(r_dot, expected_r_dot, places=9)

    def test_pure_rotation_invariance(self):
        v_rot_origin = [0.0, 0.0, 0.0]
        u_norm = math.sqrt(0.8**2 + 0.5**2 + 0.3**2)
        u_any = [0.8 / u_norm, 0.5 / u_norm, 0.3 / u_norm]
        r_dot = compute_range_rate(v_rot_origin, u_any)
        self.assertAlmostEqual(r_dot, 0.0, places=9)

    def test_doppler_folding_boundaries(self):
        v_max = 50.0
        self.assertAlmostEqual(fold_doppler(0.0, v_max), 0.0)
        self.assertAlmostEqual(fold_doppler(25.0, v_max), 25.0)
        self.assertAlmostEqual(fold_doppler(50.0, v_max), -50.0)
        self.assertAlmostEqual(fold_doppler(55.0, v_max), -45.0)
        self.assertAlmostEqual(fold_doppler(-55.0, v_max), 45.0)
        self.assertAlmostEqual(fold_doppler(110.0, v_max), 10.0)
        self.assertAlmostEqual(fold_doppler(-110.0, v_max), -10.0)

    def test_rcs_falloff_and_incidence_attenuation(self):
        rcs_10m = compute_apparent_rcs(10.0, 10.0, 1.0)
        rcs_20m = compute_apparent_rcs(10.0, 20.0, 1.0)
        rcs_grazing = compute_apparent_rcs(10.0, 10.0, 0.05)
        self.assertLess(rcs_20m, rcs_10m)
        self.assertLess(rcs_grazing, rcs_10m)


if __name__ == "__main__":
    unittest.main()
