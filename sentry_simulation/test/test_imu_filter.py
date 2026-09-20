#!/usr/bin/env python3

import importlib.util
import time
from pathlib import Path
import unittest
from unittest.mock import patch


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

    @patch.dict("os.environ", {"ROS_DOMAIN_ID": "194"})
    def publish_sample(self, acceleration, orientation):
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        rclpy = self.module.rclpy
        rclpy.init(args=[])
        filter_node = self.module.SimImuFilter()
        observer = rclpy.create_node("imu_contract_observer")
        received = []
        subscription = observer.create_subscription(
            Imu, "/sim/imu", received.append, qos_profile_sensor_data
        )
        try:
            deadline = time.monotonic() + 3.0
            while filter_node._publisher.get_subscription_count() == 0:
                rclpy.spin_once(observer, timeout_sec=0.02)
                self.assertLess(time.monotonic(), deadline, "publisher discovery timed out")
            message = Imu()
            message.header.stamp.sec = 123
            message.header.stamp.nanosec = 456
            message.header.frame_id = "sim_lidar"
            (message.linear_acceleration.x, message.linear_acceleration.y,
             message.linear_acceleration.z) = acceleration
            (message.orientation.x, message.orientation.y,
             message.orientation.z, message.orientation.w) = orientation
            message.angular_velocity.x = 0.1
            message.angular_velocity.y = -0.2
            message.angular_velocity.z = 0.3
            filter_node._callback(message)
            deadline = time.monotonic() + 0.5
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=0.02)
            self.assertEqual(len(received), 1, "valid six-axis input was discarded")
            return received[0]
        finally:
            observer.destroy_subscription(subscription)
            observer.destroy_node()
            filter_node.destroy_node()
            rclpy.shutdown()

    def test_published_slope_force_keeps_real_vertical_dynamics(self):
        message = self.publish_sample(
            (2.47541776, 0.0, 9.66587168),
            (0.0, -0.085707, 0.0, 0.996320),
        )
        self.assertEqual(
            (message.linear_acceleration.x, message.linear_acceleration.y,
             message.linear_acceleration.z),
            (2.47541776, 0.0, 9.66587168),
        )
        self.assertEqual((message.header.stamp.sec, message.header.stamp.nanosec), (123, 456))
        self.assertEqual(message.header.frame_id, "sim_lidar")

    def test_published_six_axis_sample_does_not_require_attitude(self):
        message = self.publish_sample((6.0, -7.0, 12.0), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(
            (message.linear_acceleration.x, message.linear_acceleration.y,
             message.linear_acceleration.z), (6.0, -7.0, 12.0),
        )
        self.assertEqual(
            (message.angular_velocity.x, message.angular_velocity.y,
             message.angular_velocity.z), (0.1, -0.2, 0.3),
        )

    @patch.dict('os.environ', {'ROS_DOMAIN_ID': '194'})
    def test_runtime_errors_are_ignored_only_after_context_shutdown(self):
        from rclpy._rclpy_pybind11 import RCLError
        for error_type in (RuntimeError, RCLError):
            for context_stopped in (False, True):
                with self.subTest(error_type=error_type, context_stopped=context_stopped):
                    def failing_spin(node):
                        if context_stopped:
                            self.module.rclpy.shutdown()
                        raise error_type('injected executor failure')
                    with patch.object(self.module.rclpy, 'spin', side_effect=failing_spin):
                        if context_stopped:
                            self.module.main(args=[])
                        else:
                            with self.assertRaisesRegex(error_type, 'injected executor failure'):
                                self.module.main(args=[])
                    self.assertFalse(self.module.rclpy.ok())

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

    def test_published_horizontal_dynamics_are_not_norm_limited(self):
        message = self.publish_sample((6.0, -7.0, 9.81), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(
            (message.linear_acceleration.x, message.linear_acceleration.y,
             message.linear_acceleration.z), (6.0, -7.0, 9.81),
        )

    def test_published_sensor_rails_do_not_modify_other_axes(self):
        message = self.publish_sample((40.0, -50.0, 12.0), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(
            (message.linear_acceleration.x, message.linear_acceleration.y,
             message.linear_acceleration.z), (29.43, -29.43, 12.0),
        )


if __name__ == "__main__":
    unittest.main()
