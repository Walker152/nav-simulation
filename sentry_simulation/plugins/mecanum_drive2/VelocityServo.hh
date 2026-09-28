#pragma once

#include "../gazebo_compat.hh"
#include <algorithm>
#include <chrono>

namespace sentry_simulation
{
// PI only (D=0). PID stores the integral contribution in force or torque units.
inline double updateVelocityServo(
    SENTRY_GZ::math::PID &pid, double target, double actual,
    const std::chrono::duration<double> &dt)
{
    const double error = actual - target;
    const double gain = pid.IGain();
    if (target == 0.0) {
        // A zero command brakes with P, without a retained drive bias.
        pid.Reset();
        pid.SetIGain(0.0);
    } else {
        double proportional, integral, derivative;
        pid.Errors(proportional, integral, derivative);
        const double nextIntegral = std::clamp(
            integral + gain * error * dt.count(), pid.IMin(), pid.IMax());
        const double requested = -pid.PGain() * error - nextIntegral + pid.CmdOffset();
        if ((requested > pid.CmdMax() && error < 0.0) ||
            (requested < pid.CmdMin() && error > 0.0)) {
            pid.SetIGain(0.0);  // Hold I while saturated; opposing error can unwind it.
        }
    }
    const double effort = pid.Update(error, dt);
    pid.SetIGain(gain);
    return effort;
}
}  // namespace sentry_simulation
