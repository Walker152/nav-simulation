#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FILTER_PATH = PACKAGE_ROOT / "sentry_simulation" / "imu_filter.py"


def load_filter_module():
    if not FILTER_PATH.is_file():
        return None
    spec = importlib.util.spec_from_file_location("sentry_sim_imu_filter", FILTER_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ImuFilterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_filter_module()

    def require_module(self):
        self.assertIsNotNone(
            self.module,
            f"simulation IMU filter implementation is missing: {FILTER_PATH}",
        )

    def test_out_of_range_sample_is_saturated_per_axis(self):
        self.require_module()
        value = self.module.filter_vector((100.0, -100.0, 9.81), 28.0)
        self.assertEqual(value, (28.0, -28.0, 9.81))

    def test_collision_impulse_exposes_sensor_rail_to_point_lio(self):
        self.require_module()
        value = self.module.filter_vector(
            (284.0, 110.0, 127.0), 29.43
        )
        self.assertEqual(value, (29.43, 29.43, 29.43))

    def test_normal_motion_is_not_clipped(self):
        self.require_module()
        sample = (2.0, -1.0, 9.5)
        value = self.module.filter_vector(sample, 28.0)
        for actual, expected in zip(value, sample):
            self.assertAlmostEqual(actual, expected)

    def test_collision_impulse_does_not_leave_a_stale_value_tail(self):
        self.require_module()
        saturated = self.module.filter_vector(
            (284.0, 110.0, 127.0), 29.43
        )
        recovered = self.module.filter_vector(
            (0.2, -0.1, 9.81), 29.43
        )
        self.assertEqual(saturated, (29.43, 29.43, 29.43))
        self.assertEqual(recovered, (0.2, -0.1, 9.81))

    def test_subrail_collision_sample_is_preserved(self):
        self.require_module()
        self.assertEqual(
            self.module.filter_vector((-7.4, 10.4, 25.6), 29.43),
            (-7.4, 10.4, 25.6),
        )

    def test_rapid_gravity_direction_change_is_not_delayed(self):
        self.require_module()
        self.assertEqual(
            self.module.filter_vector((0.0, 5.0, 8.44), 29.43),
            (0.0, 5.0, 8.44),
        )

    def test_each_axis_is_saturated_independently(self):
        self.require_module()
        self.assertEqual(
            self.module.filter_vector((40.0, -2.0, -50.0), 29.43),
            (29.43, -2.0, -29.43),
        )

    def test_non_finite_sample_is_rejected(self):
        self.require_module()
        self.assertIsNone(
            self.module.filter_vector((float("nan"), 0.0, 9.81), 29.43)
        )

    def test_tilted_static_gravity_is_preserved_without_delay(self):
        self.require_module()
        half_pitch = 0.2617993877991494
        orientation = (
            0.0,
            math.sin(half_pitch),
            0.0,
            math.cos(half_pitch),
        )
        gravity = (-4.905, 0.0, 8.495709211)
        conditioned = self.module.condition_acceleration(
            gravity, orientation, sensor_limit=29.43, dynamic_limit=4.0
        )
        for actual, expected in zip(conditioned, gravity):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_normal_specific_force_is_preserved(self):
        self.require_module()
        sample = (2.0, -1.0, 9.81)
        conditioned = self.module.condition_acceleration(
            sample, (0.0, 0.0, 0.0, 1.0), sensor_limit=29.43,
            dynamic_limit=4.0,
        )
        self.assertEqual(conditioned, sample)

    def test_rigid_contact_specific_force_is_norm_limited(self):
        self.require_module()
        conditioned = self.module.condition_acceleration(
            (20.0, -20.0, 50.0),
            (0.0, 0.0, 0.0, 1.0),
            sensor_limit=29.43,
            dynamic_limit=4.0,
        )
        residual = (
            conditioned[0],
            conditioned[1],
            conditioned[2] - 9.81,
        )
        residual_norm = sum(value * value for value in residual) ** 0.5
        self.assertAlmostEqual(residual_norm, 4.0, places=6)

    def test_gravity_direction_change_does_not_reuse_previous_sample(self):
        self.require_module()
        level = self.module.condition_acceleration(
            (0.0, 0.0, 9.81),
            (0.0, 0.0, 0.0, 1.0),
            sensor_limit=29.43,
            dynamic_limit=4.0,
        )
        half_pitch = 0.2617993877991494
        tilted = self.module.condition_acceleration(
            (-4.905, 0.0, 8.495709211),
            (0.0, math.sin(half_pitch), 0.0, math.cos(half_pitch)),
            sensor_limit=29.43,
            dynamic_limit=4.0,
        )
        self.assertEqual(level, (0.0, 0.0, 9.81))
        self.assertNotEqual(tilted, level)
        self.assertAlmostEqual(tilted[0], -4.905, places=6)

    def test_world_vertical_contact_acceleration_is_removed(self):
        self.require_module()
        conditioned = self.module.condition_acceleration(
            (0.0, 0.0, 40.0),
            (0.0, 0.0, 0.0, 1.0),
            sensor_limit=29.43,
            dynamic_limit=4.0,
            vertical_dynamic_limit=0.0,
        )
        self.assertEqual(conditioned, (0.0, 0.0, 9.81))


if __name__ == "__main__":
    unittest.main()
