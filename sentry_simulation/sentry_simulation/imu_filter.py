#!/usr/bin/env python3
"""Preserve Gazebo six-axis IMU measurements with per-axis sensor saturation."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def filter_vector(sample, limit):
    """Clamp finite values to each sensor axis rail without adding stale samples."""
    limit = float(limit)
    if limit <= 0.0:
        raise ValueError("IMU limit must be positive")
    values = tuple(float(value) for value in sample)
    if any(not math.isfinite(value) for value in values):
        return None
    return tuple(max(-limit, min(limit, value)) for value in values)


class SimImuFilter(Node):
    def __init__(self):
        super().__init__("sentry_sim_imu_filter")
        self.declare_parameter("input_topic", "/sim/imu_raw")
        self.declare_parameter("output_topic", "/sim/imu")
        self.declare_parameter("acceleration_limit", 29.43)
        self.declare_parameter("angular_velocity_limit", 35.0)
        self._acceleration_limit = self.get_parameter("acceleration_limit").value
        self._angular_velocity_limit = self.get_parameter(
            "angular_velocity_limit"
        ).value

        self._publisher = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("input_topic").value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message):
        filtered_acceleration = filter_vector(
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            self._acceleration_limit,
        )
        filtered_angular_velocity = filter_vector(
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            self._angular_velocity_limit,
        )
        if filtered_acceleration is None or filtered_angular_velocity is None:
            return

        output = Imu()
        output.header = message.header
        output.orientation = message.orientation
        output.orientation_covariance = message.orientation_covariance
        output.angular_velocity_covariance = message.angular_velocity_covariance
        output.linear_acceleration_covariance = message.linear_acceleration_covariance
        (
            output.linear_acceleration.x,
            output.linear_acceleration.y,
            output.linear_acceleration.z,
        ) = filtered_acceleration
        (
            output.angular_velocity.x,
            output.angular_velocity.y,
            output.angular_velocity.z,
        ) = filtered_angular_velocity
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SimImuFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A signal can invalidate the context between executor readiness and take.
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
