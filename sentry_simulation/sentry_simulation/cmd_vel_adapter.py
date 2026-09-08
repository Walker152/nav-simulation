#!/usr/bin/env python3
"""Forward the controller's body-frame command to the simulator with a watchdog."""

from geometry_msgs.msg import Twist
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sentry_simulation.kinematics import command_timed_out


class SentrySimCmdAdapter(Node):
    def __init__(self) -> None:
        super().__init__("sentry_sim_cmd_adapter")
        self.declare_parameter("chassis_type", "omni")
        self.declare_parameter("input_topic", "/cmd_vel_mpc")
        self.declare_parameter("output_topic", "/sim/cmd_vel")
        self.declare_parameter("command_timeout", 0.25)

        self._chassis_type = str(self.get_parameter("chassis_type").value)
        if self._chassis_type != "omni":
            raise ValueError("navigation command forwarding currently supports only omni")
        self._command_timeout = float(self.get_parameter("command_timeout").value)
        if self._command_timeout <= 0.0:
            raise ValueError("command_timeout must be positive")
        self._last_command_time = None
        self._sent_timeout_stop = False

        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._command_callback,
            10,
        )
        self.create_timer(0.05, self._watchdog_callback)

    def _command_callback(self, message: Twist) -> None:
        self._publisher.publish(message)
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
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
