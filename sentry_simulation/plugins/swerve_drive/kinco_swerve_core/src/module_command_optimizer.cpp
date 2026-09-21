#include "kinco_swerve_core/module_command_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace kinco_swerve_core {
namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kLimitTolerance = 1e-12;
constexpr double kDirectPreferenceTravelDifferenceRad = kPi / 180.0;

bool finite(const double value) noexcept { return std::isfinite(value); }

double normalize_angle(const double angle_rad) noexcept {
  double result = std::remainder(angle_rad, kTwoPi);
  if (result >= kPi) {
    result -= kTwoPi;
  }
  return result;
}

bool within_range(const double angle_rad, const SteeringRange& range) noexcept {
  return angle_rad >= range.lower_angle_rad - kLimitTolerance &&
         angle_rad <= range.upper_angle_rad + kLimitTolerance;
}

enum class EquivalentStatus { kOk, kNumericalFailure, kNoFeasibleAngle };

bool accept_and_clamp(double& candidate_angle_rad, const SteeringRange& range) noexcept {
  if (!within_range(candidate_angle_rad, range)) {
    return false;
  }
  candidate_angle_rad =
    std::max(range.lower_angle_rad, std::min(candidate_angle_rad, range.upper_angle_rad));
  return true;
}

EquivalentStatus nearest_limited_equivalent(const double base_angle_rad,
                                            const double reference_angle_rad,
                                            const SteeringRange& range,
                                            double& candidate_angle_rad) noexcept {
  const double difference_rad = base_angle_rad - reference_angle_rad;
  if (!finite(difference_rad)) {
    return EquivalentStatus::kNumericalFailure;
  }
  candidate_angle_rad = reference_angle_rad + normalize_angle(difference_rad);
  if (!finite(candidate_angle_rad)) {
    return EquivalentStatus::kNumericalFailure;
  }
  if (accept_and_clamp(candidate_angle_rad, range)) {
    return EquivalentStatus::kOk;
  }
  if (candidate_angle_rad < range.lower_angle_rad) {
    const double turns = std::ceil((range.lower_angle_rad - candidate_angle_rad) / kTwoPi);
    if (!finite(turns)) {
      return EquivalentStatus::kNumericalFailure;
    }
    candidate_angle_rad += turns * kTwoPi;
    if (!finite(candidate_angle_rad)) {
      return EquivalentStatus::kNumericalFailure;
    }
    if (accept_and_clamp(candidate_angle_rad, range)) {
      return EquivalentStatus::kOk;
    }
  }
  if (candidate_angle_rad > range.upper_angle_rad) {
    const double turns = std::ceil((candidate_angle_rad - range.upper_angle_rad) / kTwoPi);
    if (!finite(turns)) {
      return EquivalentStatus::kNumericalFailure;
    }
    candidate_angle_rad -= turns * kTwoPi;
    if (!finite(candidate_angle_rad)) {
      return EquivalentStatus::kNumericalFailure;
    }
    if (accept_and_clamp(candidate_angle_rad, range)) {
      return EquivalentStatus::kOk;
    }
  }
  return EquivalentStatus::kNoFeasibleAngle;
}

ModuleCommandResult no_feasible_steering_result() noexcept {
  ModuleCommandResult result{};
  result.status = SwerveStatus::kNoFeasibleSteeringAngle;
  return result;
}

}  // namespace

ModuleCommandOptimizer::ModuleCommandOptimizer(const SwerveGeometry& geometry,
                                               const ModuleCommandLimits& limits)
    : geometry_(geometry), limits_(limits) {
  for (const auto& module : geometry_.modules) {
    const auto& range = module.steering_range;
    if (std::isnan(range.lower_angle_rad) || std::isnan(range.upper_angle_rad) ||
        range.lower_angle_rad >= range.upper_angle_rad) {
      throw std::invalid_argument("steering lower angle must be less than upper angle");
    }
  }
  if (!finite(limits_.maximum_wheel_speed_mps) || limits_.maximum_wheel_speed_mps <= 0.0) {
    throw std::invalid_argument("maximum wheel speed must be finite and positive");
  }
  if (!finite(limits_.steering_hold_speed_mps) || limits_.steering_hold_speed_mps < 0.0) {
    throw std::invalid_argument("steering hold speed must be finite and non-negative");
  }
}

ModuleCommandResult ModuleCommandOptimizer::optimize(
  const ModuleArray<ModuleVelocity>& velocities,
  const ModuleArray<SteeringReference>& steering) const noexcept {
  ModuleArray<ModuleSetpoint> setpoints{};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const auto& velocity = velocities[i];
    const auto& reference = steering[i];
    if (!finite(velocity.x_mps) || !finite(velocity.y_mps) ||
        !finite(reference.measured_angle_rad) || !finite(reference.held_angle_rad)) {
      return ModuleCommandResult{};
    }

    const double speed_mps = std::hypot(velocity.x_mps, velocity.y_mps);
    if (!finite(speed_mps)) {
      return ModuleCommandResult{};
    }

    double selected_angle_rad = 0.0;
    double selected_speed_mps = 0.0;
    const auto& range = geometry_.modules[i].steering_range;
    if (speed_mps <= limits_.steering_hold_speed_mps) {
      const auto held_status = nearest_limited_equivalent(
        reference.held_angle_rad, reference.measured_angle_rad, range, selected_angle_rad);
      if (held_status == EquivalentStatus::kNumericalFailure) {
        return ModuleCommandResult{};
      }
      if (held_status == EquivalentStatus::kNoFeasibleAngle) {
        return no_feasible_steering_result();
      }
    } else {
      const double vector_angle_rad = std::atan2(velocity.y_mps, velocity.x_mps);
      double direct_angle_rad = 0.0;
      double reversed_angle_rad = 0.0;
      const auto direct_status = nearest_limited_equivalent(
        vector_angle_rad, reference.measured_angle_rad, range, direct_angle_rad);
      const auto reversed_status = nearest_limited_equivalent(
        vector_angle_rad + kPi, reference.measured_angle_rad, range, reversed_angle_rad);
      if (direct_status == EquivalentStatus::kNumericalFailure ||
          reversed_status == EquivalentStatus::kNumericalFailure) {
        return ModuleCommandResult{};
      }
      const bool direct_feasible = direct_status == EquivalentStatus::kOk;
      const bool reversed_feasible = reversed_status == EquivalentStatus::kOk;
      if (!direct_feasible && !reversed_feasible) {
        return no_feasible_steering_result();
      }
      if (direct_feasible &&
          (!reversed_feasible || std::abs(direct_angle_rad - reference.measured_angle_rad) <=
                                   std::abs(reversed_angle_rad - reference.measured_angle_rad) +
                                     kDirectPreferenceTravelDifferenceRad + kLimitTolerance)) {
        selected_angle_rad = direct_angle_rad;
        selected_speed_mps = speed_mps;
      } else {
        selected_angle_rad = reversed_angle_rad;
        selected_speed_mps = -speed_mps;
      }
    }
    setpoints[i] = {selected_speed_mps, selected_angle_rad};
  }

  ModuleCommandResult result{};
  result.setpoints = setpoints;
  double largest_speed_mps = 0.0;
  for (const auto& setpoint : result.setpoints) {
    largest_speed_mps = std::max(largest_speed_mps, std::abs(setpoint.wheel_speed_mps));
  }
  result.saturation_scale = largest_speed_mps > limits_.maximum_wheel_speed_mps
                              ? limits_.maximum_wheel_speed_mps / largest_speed_mps
                              : 1.0;
  for (auto& setpoint : result.setpoints) {
    setpoint.wheel_speed_mps *= result.saturation_scale;
  }
  result.status = SwerveStatus::kOk;
  return result;
}

}  // namespace kinco_swerve_core
