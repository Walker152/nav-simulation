"""Pure command-frame and chassis-constraint conversions."""

import math
from typing import Tuple


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def command_timed_out(last_command_time, now: float, timeout: float) -> bool:
    """Return whether an initialized command stream exceeded its deadline."""

    return last_command_time is not None and now - last_command_time > timeout


def adapt_command(
    chassis_type: str,
    vx_world: float,
    vy_world: float,
    wz: float,
    yaw: float,
    heading_gain: float = 2.0,
    max_heading_rate: float = 3.0,
) -> Tuple[float, float, float]:
    """Convert the MPC world-frame command to a Gazebo body-frame command.

    The omni chassis retains both planar degrees of freedom.  The differential
    chassis selects the shorter of forward/reverse travel and converts lateral
    demand into a bounded heading correction.
    """

    if chassis_type == "omni":
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * vx_world + sin_yaw * vy_world,
            -sin_yaw * vx_world + cos_yaw * vy_world,
            wz,
        )

    if chassis_type != "diff":
        raise ValueError(f"unsupported chassis_type: {chassis_type}")

    speed = math.hypot(vx_world, vy_world)
    if speed < 1.0e-9:
        return 0.0, 0.0, _clamp(wz, max_heading_rate)

    heading_error = _wrap_angle(math.atan2(vy_world, vx_world) - yaw)
    direction = 1.0
    if heading_error > math.pi / 2.0:
        heading_error -= math.pi
        direction = -1.0
    elif heading_error < -math.pi / 2.0:
        heading_error += math.pi
        direction = -1.0

    forward = direction * speed * max(0.0, math.cos(heading_error))
    angular = _clamp(wz + heading_gain * heading_error, max_heading_rate)
    return forward, 0.0, angular
