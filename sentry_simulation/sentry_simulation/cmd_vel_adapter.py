#!/usr/bin/env python3
"""Adapt the controller's world-frame velocity for the selected simulator chassis."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sentry_simulation.kinematics import adapt_command, command_timed_out


class SentrySimCmdAdapter(Node):
    def __init__(self) -> None:
        super().__init__("sentry_sim_cmd_adapter")
        self.declare_parameter("chassis_type", "omni")
        self.declare_parameter("input_topic", "/cmd_vel_mpc")
        self.declare_parameter("output_topic", "/sim/cmd_vel")
        self.declare_parameter("odom_topic", "/aft_mapped_to_init")
        self.declare_parameter("heading_gain", 2.0)
        self.declare_parameter("max_heading_rate", 3.0)
        self.declare_parameter("command_timeout", 0.25)

        self._chassis_type = str(self.get_parameter("chassis_type").value)
        if self._chassis_type not in ("omni", "diff"):
            raise ValueError("chassis_type must be 'omni' or 'diff'")
        self._heading_gain = float(self.get_parameter("heading_gain").value)
        self._max_heading_rate = float(self.get_parameter("max_heading_rate").value)
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        if self._command_timeout <= 0.0:
            raise ValueError("command_timeout must be positive")
        self._yaw = None
        self._warned_no_odom = False
        self._last_command_time = None
        self._sent_timeout_stop = False

        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._command_callback,
            10,
        )
        self.create_timer(0.05, self._watchdog_callback)

    def _odom_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y + orientation.z * orientation.z
        )
        self._yaw = math.atan2(sin_yaw, cos_yaw)

    def _command_callback(self, message: Twist) -> None:
        if self._yaw is None:
            if not self._warned_no_odom:
                self.get_logger().warning(
                    "waiting for /aft_mapped_to_init before forwarding velocity commands"
                )
                self._warned_no_odom = True
            return

        vx, vy, wz = adapt_command(
            self._chassis_type,
            message.linear.x,
            message.linear.y,
            message.angular.z,
            self._yaw,
            self._heading_gain,
            self._max_heading_rate,
        )
        output = Twist()
        output.linear.x = vx
        output.linear.y = vy
        output.angular.z = wz
        self._publisher.publish(output)
        self._last_command_time = self._now_seconds()
        self._sent_timeout_stop = False

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _watchdog_callback(self) -> None:
        if self._sent_timeout_stop or not command_timed_out(
            self._last_command_time, self._now_seconds(), self._command_timeout
        ):
            return
        self._publisher.publish(Twist())
        self._sent_timeout_stop = True
        self.get_logger().warning("MPC command timed out; sent a zero chassis command")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SentrySimCmdAdapter()
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
