#!/usr/bin/env python3
"""Testes do controle puro: não importam ROS nem acionam hardware."""
import math
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from centraliza_cone import Config, ConeController


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.c = ConeController(Config())

    def frame(self, x, t=100.0, y=120.0):
        return self.c.receive(x, y, t, t, t)

    def test_no_detection_stops(self):
        self.assertEqual(self.c.command(100., 100.).linear_x, 0.)

    def test_left_and_right(self):
        self.frame(80.)
        self.assertGreater(self.c.command(100., 100.).angular_z, 0.)
        self.frame(340., 100.1)
        self.assertLess(self.c.command(100.1, 100.1).angular_z, 0.)

    def test_ackermann_geometry_and_limit(self):
        self.frame(0.)
        cmd = self.c.command(100., 100.)
        steering = math.atan(cmd.angular_z * self.c.config.wheelbase_m / cmd.linear_x)
        self.assertAlmostEqual(steering, self.c.config.max_steering_rad)
        self.assertGreater(cmd.linear_x, 0.)

    def test_center_stops_before_confirmation(self):
        self.frame(211.5)
        cmd = self.c.command(100., 100.)
        self.assertEqual((cmd.linear_x, cmd.angular_z, cmd.centered), (0., 0., False))
        self.frame(211.5, 100.1)
        self.frame(211.5, 100.2)
        self.assertTrue(self.c.command(100.2, 100.2).centered)

    def test_hysteresis(self):
        for t in (100., 100.1, 100.2):
            self.frame(211.5, t)
        self.frame(227., 100.3)  # 15,5 px: permanece parado.
        self.assertTrue(self.c.command(100.3, 100.3).centered)
        self.frame(240., 100.4)
        self.assertFalse(self.c.command(100.4, 100.4).centered)
        self.assertGreater(self.c.command(100.4, 100.4).linear_x, 0.)

    def test_timeout_stops(self):
        self.frame(80.)
        cmd = self.c.command(100.5, 100.5)
        self.assertEqual((cmd.linear_x, cmd.angular_z, cmd.centered), (0., 0., False))

    def test_frozen_ros_clock_still_stops(self):
        self.frame(80.)
        self.assertEqual(self.c.command(100.5, 100.).linear_x, 0.)

    def test_invalid_input_clears_previous_command(self):
        for x, y in ((-1., 120.), (424., 120.), (80., 240.), (math.nan, 120.), (math.inf, 120.)):
            with self.subTest(x=x, y=y):
                self.c = ConeController(Config())
                self.frame(80.)
                self.assertFalse(self.frame(x, 100.1, y))
                self.assertEqual(self.c.command(100.1, 100.1).linear_x, 0.)

    def test_stale_missing_and_future_stamp(self):
        for stamp in (0., 99., 101.):
            self.assertFalse(self.c.receive(80., 120., stamp, 100., 100.))
            self.assertEqual(self.c.command(100., 100.).linear_x, 0.)

    def test_duplicate_does_not_confirm_or_refresh(self):
        self.frame(211.5)
        for t in (100.1, 100.2, 100.3):
            self.assertFalse(self.c.receive(211.5, 120., 100., t, t))
        self.assertFalse(self.c.command(100.3, 100.3).centered)
        self.assertEqual(self.c.command(100.5, 100.5).state, 'deteccao_expirada')

    def test_gap_resets_confirmation(self):
        self.frame(211.5)
        self.frame(211.5, 100.1)
        self.frame(211.5, 101.)
        self.assertFalse(self.c.command(101., 101.).centered)

    def test_invalid_config(self):
        for overrides in ({'wheelbase_m': 0.}, {'forward_speed_m_s': math.nan},
                          {'max_steering_rad': math.pi / 2}, {'confirmation_frames': 0},
                          {'resume_tolerance_px': 5.}, {'image_width': 1}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                Config(**overrides)


if __name__ == '__main__':
    unittest.main(verbosity=2)
