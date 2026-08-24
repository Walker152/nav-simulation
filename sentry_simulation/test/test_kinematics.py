#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[4]
KINEMATICS_PATH = (
    REPO_ROOT
    / "src"
    / "simulation"
    / "sentry_simulation"
    / "sentry_simulation"
    / "kinematics.py"
)


def load_kinematics_module():
    if not KINEMATICS_PATH.is_file():
        return None
    spec = importlib.util.spec_from_file_location("sentry_sim_kinematics", KINEMATICS_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class KinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kinematics = load_kinematics_module()

    def require_module(self):
        self.assertIsNotNone(
            self.kinematics,
            f"simulation kinematics implementation is missing: {KINEMATICS_PATH}",
        )

    def test_omni_rotates_world_velocity_into_body_frame(self):
        self.require_module()
        vx, vy, wz = self.kinematics.adapt_command(
            "omni", 1.0, 0.0, 0.4, math.pi / 2.0
        )
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, -1.0, places=6)
        self.assertAlmostEqual(wz, 0.4, places=6)

    def test_diff_never_outputs_lateral_velocity(self):
        self.require_module()
        _, vy, _ = self.kinematics.adapt_command("diff", 1.0, 1.0, 0.0, 0.0)
        self.assertEqual(vy, 0.0)

    def test_diff_aligned_command_preserves_forward_speed(self):
        self.require_module()
        vx, _, wz = self.kinematics.adapt_command("diff", 1.5, 0.0, 0.2, 0.0)
        self.assertAlmostEqual(vx, 1.5, places=6)
        self.assertAlmostEqual(wz, 0.2, places=6)

    def test_diff_lateral_command_turns_before_driving(self):
        self.require_module()
        vx, _, wz = self.kinematics.adapt_command(
            "diff", 0.0, 1.0, 0.0, 0.0, heading_gain=2.0, max_heading_rate=3.0
        )
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertGreater(wz, 0.0)
        self.assertLessEqual(wz, 3.0)

    def test_diff_backward_command_uses_reverse_without_turning(self):
        self.require_module()
        vx, _, wz = self.kinematics.adapt_command("diff", -1.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(vx, -1.0, places=6)
        self.assertAlmostEqual(wz, 0.0, places=6)

    def test_unknown_chassis_type_is_rejected(self):
        self.require_module()
        with self.assertRaises(ValueError):
            self.kinematics.adapt_command("tracked", 1.0, 0.0, 0.0, 0.0)

    def test_command_timeout_only_expires_after_deadline(self):
        self.require_module()
        self.assertFalse(self.kinematics.command_timed_out(None, 10.0, 0.25))
        self.assertFalse(self.kinematics.command_timed_out(9.8, 10.0, 0.25))
        self.assertTrue(self.kinematics.command_timed_out(9.7, 10.0, 0.25))


if __name__ == "__main__":
    unittest.main()
