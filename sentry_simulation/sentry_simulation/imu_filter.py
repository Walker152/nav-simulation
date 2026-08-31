#!/usr/bin/env python3
"""Apply MID360-like per-axis measurement saturation to Gazebo's IMU."""

import math

import rclpy
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


def condition_acceleration(
    sample, orientation, sensor_limit, dynamic_limit,
    vertical_dynamic_limit=0.0, gravity=9.81
):
    """Keep planar specific force and reject Gazebo's vertical contact impulses."""
    values = tuple(float(value) for value in sample)
    quaternion = tuple(float(value) for value in orientation)
    if any(not math.isfinite(value) for value in values + quaternion):
        return None
    dynamic_limit = float(dynamic_limit)
    vertical_dynamic_limit = float(vertical_dynamic_limit)
    if dynamic_limit <= 0.0 or vertical_dynamic_limit < 0.0:
        raise ValueError("dynamic acceleration limits are invalid")

    qx, qy, qz, qw = quaternion
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if quaternion_norm <= 1.0e-9:
        return None
    qx /= quaternion_norm
    qy /= quaternion_norm
    qz /= quaternion_norm
    qw /= quaternion_norm

    gravity_body = (
        2.0 * (qx * qz - qw * qy) * gravity,
        2.0 * (qy * qz + qw * qx) * gravity,
        (1.0 - 2.0 * (qx * qx + qy * qy)) * gravity,
    )
    specific_force = tuple(
        value - gravity_axis
        for value, gravity_axis in zip(values, gravity_body)
    )
    rotation = (
        (1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),
         2.0 * (qx * qz + qy * qw)),
        (2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),
         2.0 * (qy * qz - qx * qw)),
        (2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw),
         1.0 - 2.0 * (qx * qx + qy * qy)),
    )
    world_specific = [
        sum(rotation[row][column] * specific_force[column] for column in range(3))
        for row in range(3)
    ]
    planar_norm = math.hypot(world_specific[0], world_specific[1])
    if planar_norm > dynamic_limit:
        planar_scale = dynamic_limit / planar_norm
        world_specific[0] *= planar_scale
        world_specific[1] *= planar_scale
    world_specific[2] = max(
        -vertical_dynamic_limit,
        min(vertical_dynamic_limit, world_specific[2]),
    )
    specific_force = tuple(
        sum(rotation[row][column] * world_specific[row] for row in range(3))
        for column in range(3)
    )
    conditioned = tuple(
        gravity_axis + force_axis
        for gravity_axis, force_axis in zip(gravity_body, specific_force)
    )
    return filter_vector(conditioned, sensor_limit)


class SimImuFilter(Node):
    def __init__(self):
        super().__init__("sentry_sim_imu_filter")
        self.declare_parameter("input_topic", "/sim/imu_raw")
        self.declare_parameter("output_topic", "/sim/imu")
        self.declare_parameter("acceleration_limit", 29.43)
        self.declare_parameter("angular_velocity_limit", 35.0)
        self.declare_parameter("dynamic_acceleration_limit", 4.0)
        self.declare_parameter("vertical_dynamic_acceleration_limit", 0.0)
        self._acceleration_limit = self.get_parameter("acceleration_limit").value
        self._angular_velocity_limit = self.get_parameter(
            "angular_velocity_limit"
        ).value
        self._dynamic_acceleration_limit = self.get_parameter(
            "dynamic_acceleration_limit"
        ).value
        self._vertical_dynamic_acceleration_limit = self.get_parameter(
            "vertical_dynamic_acceleration_limit"
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
        filtered_acceleration = condition_acceleration(
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ),
            sensor_limit=self._acceleration_limit,
            dynamic_limit=self._dynamic_acceleration_limit,
            vertical_dynamic_limit=self._vertical_dynamic_acceleration_limit,
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
