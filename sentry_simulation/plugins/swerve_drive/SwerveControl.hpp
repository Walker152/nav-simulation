// Simulation-specific command lifecycle and bounded references. No ROS dependency.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <limits>
#include <string>
#include "kinco_swerve_core/module_command_optimizer.hpp"
#include "kinco_swerve_core/swerve_kinematics.hpp"

namespace sentry_simulation::swerve {
namespace core = kinco_swerve_core;
inline constexpr double time_roundoff=0.5e-9;
inline double stamp_seconds(int64_t seconds, int32_t nanoseconds) {
  // Do not normalize malformed transport stamps into a valid command.
  if (seconds<0 || nanoseconds<0 || nanoseconds>=1000000000) return 0;
  return seconds+nanoseconds*1e-9;
}
struct Command {
  double stamp{};
  std::string frame;
  // vx, vy, vz, wx, wy, wz: unused components still participate in validation.
  std::array<double,6> values{};
  bool well_formed() const {
    return std::isfinite(stamp) && stamp>0 && (frame.empty() || frame=="base_link") &&
      std::all_of(values.begin(),values.end(),[](double x) { return std::isfinite(x); });
  }
  bool valid(double now, double timeout) const {
    // Half a nanosecond absorbs only integer-clock conversion roundoff.
    return well_formed() && std::isfinite(now) && stamp-now<=time_roundoff &&
      now-stamp<=timeout+time_roundoff;
  }
};
// Gazebo publishes this step's clock before PreUpdate. The callback cannot judge
// time against the previous update. Validate time at the first consumer instead.
// At that common now, valid stamps form an interval, so two extrema preserve all
// intermediate rejection events without a message queue or a future allowance.
struct Mailbox {
  Command latest;
  uint64_t generation{};
  void receive(const Command & command) {
    if (!command.well_formed()) { clear(); return; }
    latest=command;
    pending_min=std::min(pending_min,command.stamp);
    pending_max=std::max(pending_max,command.stamp);
  }
  void consume(double now, double timeout) {
    if (pending_max>0 && (!std::isfinite(now) || pending_max-now>time_roundoff ||
      now-pending_min>timeout+time_roundoff)) ++generation;
    // Invalid latest is discarded permanently, never held until time catches up.
    if (!latest.valid(now,timeout)) latest={};
    pending_min=std::numeric_limits<double>::infinity(); pending_max=0;
  }
  void clear() {
    latest={}; ++generation;
    pending_min=std::numeric_limits<double>::infinity(); pending_max=0;
  }
private:
  double pending_min{std::numeric_limits<double>::infinity()},pending_max{};
};
struct Parameters {
  double wheel_radius{.065}, wheelbase{.68}, track{.48};
  double steering_limit{2.7052603406}, steering_hard_limit{2.9321531434};
  double max_linear_speed{2}, max_yaw_speed{2}, max_wheel_speed{3};
  double linear_acceleration{1}, yaw_acceleration{2};
  double max_steering_velocity{6.28318530718}, max_steering_acceleration{50.2654824576};
  double command_timeout{.25}, publish_rate{100};
  double steering_kp{80}, steering_kd{4}, steering_torque{13.5};
  double wheel_kp{4}, wheel_torque{22};
  double steering_ki{0};  // Nm/(rad*s); zero preserves the original PD path.
  void validate() const {
    for (double v : {wheel_radius,wheelbase,track,steering_limit,steering_hard_limit,
      max_linear_speed,max_yaw_speed,max_wheel_speed,linear_acceleration,yaw_acceleration,
      max_steering_velocity,max_steering_acceleration,command_timeout,publish_rate,
      steering_kp,steering_kd,steering_torque,wheel_kp,wheel_torque}) {
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("swerve parameters must be finite and positive");
    }
    if (!std::isfinite(steering_ki) || steering_ki<0)
      throw std::invalid_argument("steering_ki must be finite and nonnegative");
    if (steering_limit >= steering_hard_limit || steering_hard_limit >= 3.14159265359)
      throw std::invalid_argument("require steering_limit < steering_hard_limit < pi");
  }
  core::SwerveGeometry geometry() const {
    core::SwerveGeometry g;
    for (std::size_t i=0; i<4; ++i) {
      g.modules[i].position={i<2 ? wheelbase/2 : -wheelbase/2, i%2 ? -track/2 : track/2};
      g.modules[i].steering_range={-steering_limit,steering_limit};
    }
    return g;
  }
};
struct Feedback {
  std::array<double,4> steering_position{}, steering_velocity{}, wheel_position{}, wheel_velocity{};
  bool valid(double hard_limit) const {
    for (std::size_t i=0; i<4; ++i) {
      if (!std::isfinite(steering_position[i]) || std::abs(steering_position[i])>hard_limit ||
        !std::isfinite(steering_velocity[i]) || !std::isfinite(wheel_position[i]) ||
        !std::isfinite(wheel_velocity[i])) return false;
    }
    return true;
  }
};
struct Output {
  bool feedback_valid{};
  core::ChassisTwist reference;
  std::array<double,4> steering_position{}, steering_velocity{}, wheel_velocity{};
  std::array<double,4> steering_force{};  // Raw PD+I; Drive retains the total torque clamp.
};
struct SteeringProfile {
  double position{}, velocity{};
  void step(double target, double dt, double speed, double acceleration) {
    // Reserve one full sample of braking distance. No position clamp or velocity
    // reset at arrival: both derivatives stay bounded even on a goal reversal.
    const double distance=target-position;
    const double a_dt=acceleration*dt;
    const double safe_speed=std::sqrt(a_dt*a_dt+2*acceleration*std::abs(distance))-a_dt;
    const double desired=std::copysign(std::min(speed,safe_speed),distance);
    velocity += std::clamp(desired-velocity,-a_dt,a_dt);
    position += velocity*dt;
  }
};
class Control {
public:
  explicit Control(const Parameters & parameters) : p(parameters),
    kinematics(p.geometry()), optimizer(p.geometry(),{p.max_wheel_speed,1e-5}) { p.validate(); }
  void reset() { reference={}; profiles={}; steering_integral={}; active=false; holding=false; }
  Output step(const Command & command, uint64_t generation, double now, double dt, const Feedback & feedback) {
    Output out;
    out.feedback_valid=feedback.valid(p.steering_hard_limit);
    if (!out.feedback_valid) { reset(); return finalize(out,feedback,dt); }
    if (generation!=seen_generation) {
      if (active) reset();  // Repeated invalid packets retain the first hold angle.
      seen_generation=generation;
    }
    const bool valid=command.valid(now,p.command_timeout) && std::isfinite(dt) && dt>0 && dt<=.1;
    if (!valid) {
      if (!holding) {
        for (std::size_t i=0;i<4;++i) profiles[i]={feedback.steering_position[i],0};
      }
      reference={}; active=false; holding=true;
    } else {
      if (!active) {
        for (std::size_t i=0;i<4;++i) profiles[i]={feedback.steering_position[i],0};
        active=true; holding=false;
      }
      double x=command.values[0], y=command.values[1];
      const double norm=std::hypot(x,y);
      if (norm>p.max_linear_speed) { x=x/norm*p.max_linear_speed; y=y/norm*p.max_linear_speed; }
      const double dx=x-reference.linear_x_mps, dy=y-reference.linear_y_mps;
      const double delta=std::hypot(dx,dy);
      const double scale=delta>0 ? std::min(1.0,p.linear_acceleration*dt/delta) : 1;
      reference.linear_x_mps+=dx*scale; reference.linear_y_mps+=dy*scale;
      const double yaw=std::clamp(command.values[5],-p.max_yaw_speed,p.max_yaw_speed);
      reference.angular_z_radps+=std::clamp(yaw-reference.angular_z_radps,-p.yaw_acceleration*dt,p.yaw_acceleration*dt);
      core::ModuleArray<core::SteeringReference> steering;
      for (std::size_t i=0;i<4;++i) steering[i]={feedback.steering_position[i],
        std::clamp(profiles[i].position,-p.steering_limit,p.steering_limit)};
      const auto velocities=kinematics.to_module_velocities(reference);
      const auto commands=optimizer.optimize(velocities.velocities,steering);
      if (velocities.status!=core::SwerveStatus::kOk || commands.status!=core::SwerveStatus::kOk) {
        reset(); return finalize(out,feedback,dt);
      }
      // Project each wheel onto its measured rolling direction. A module taking
      // a long turn at a steering limit must not stop the other aligned modules.
      for (std::size_t i=0;i<4;++i) {
        const auto & target=commands.setpoints[i];
        profiles[i].step(target.steering_angle_rad,dt,p.max_steering_velocity,p.max_steering_acceleration);
        const double error=std::abs(target.steering_angle_rad-feedback.steering_position[i]);
        // Keep this wheel stopped through perpendicular and longer limited turns.
        const double alignment=error<1.5707963267948966 ? std::cos(error) : 0.0;
        out.wheel_velocity[i]=target.wheel_speed_mps/p.wheel_radius*alignment;
      }
    }
    out.reference=reference;
    for (std::size_t i=0;i<4;++i) {
      out.steering_position[i]=profiles[i].position;
      out.steering_velocity[i]=profiles[i].velocity;
    }
    return finalize(out,feedback,dt);
  }
private:
  Output finalize(Output out, const Feedback & feedback, double dt) {
    if (!out.feedback_valid) return out;  // reset() already cleared I; never form PD from NaN feedback.
    // Only a successful valid step remains active and nonholding. Invalid dt,
    // timeout and optimizer failures take the original hold/reset paths above.
    const bool integrate=p.steering_ki>0 && active && !holding;
    if (!integrate) steering_integral={};
    for (std::size_t i=0;i<4;++i) {
      const double error=out.steering_position[i]-feedback.steering_position[i];
      const double pd=p.steering_kp*error+
        p.steering_kd*(out.steering_velocity[i]-feedback.steering_velocity[i]);
      if (!integrate) {
        out.steering_force[i]=pd;  // Preserve old arithmetic, including signed zero; no +0.
        continue;
      }
      const double previous=steering_integral[i];
      const double trial=std::clamp(previous+p.steering_ki*error*dt,-p.steering_torque,p.steering_torque);
      const double total=pd+trial;
      // Reject only increments farther into total saturation; reversal may unload I.
      if (!((total>p.steering_torque && trial>previous) ||
            (total<-p.steering_torque && trial<previous))) steering_integral[i]=trial;
      out.steering_force[i]=pd+steering_integral[i];
    }
    return out;
  }
  Parameters p;
  core::SwerveKinematics kinematics;
  core::ModuleCommandOptimizer optimizer;
  core::ChassisTwist reference{};
  std::array<SteeringProfile,4> profiles{};
  std::array<double,4> steering_integral{};  // One Nm state per steering joint, owned by this lifecycle.
  uint64_t seen_generation{};
  bool active{},holding{};
};
}  // namespace sentry_simulation::swerve
