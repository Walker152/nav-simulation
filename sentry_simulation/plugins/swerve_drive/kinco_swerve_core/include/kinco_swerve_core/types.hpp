#pragma once

#include <array>
#include <cstddef>
#include <limits>

namespace kinco_swerve_core
{

constexpr std::size_t kModuleCount = 4U;

enum class ModuleId : std::size_t
{
  kFrontLeft = 0U,
  kFrontRight = 1U,
  kRearLeft = 2U,
  kRearRight = 3U
};

template<typename T>
using ModuleArray = std::array<T, kModuleCount>;

struct ChassisTwist
{
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  double angular_z_radps{0.0};
};

struct ModulePosition
{
  double x_m{0.0};
  double y_m{0.0};
};

struct SteeringRange
{
  double lower_angle_rad{-std::numeric_limits<double>::infinity()};
  double upper_angle_rad{std::numeric_limits<double>::infinity()};
};

struct ModuleGeometry
{
  ModulePosition position{};
  SteeringRange steering_range{};
};

struct SwerveGeometry
{
  ModuleArray<ModuleGeometry> modules{};
};

struct ModuleVelocity
{
  double x_mps{0.0};
  double y_mps{0.0};
};

struct SteeringReference
{
  double measured_angle_rad{0.0};
  double held_angle_rad{0.0};
};

struct ModuleState
{
  double wheel_speed_mps{0.0};
  double steering_angle_rad{0.0};
};

struct ModuleSetpoint
{
  double wheel_speed_mps{0.0};
  double steering_angle_rad{0.0};
};

struct ModuleCommandLimits
{
  double maximum_wheel_speed_mps{0.0};
  double steering_hold_speed_mps{0.0};
};

enum class SwerveStatus
{
  kOk,
  kNonFiniteInput,
  kNoFeasibleSteeringAngle
};

struct ModuleVelocityResult
{
  SwerveStatus status{SwerveStatus::kNonFiniteInput};
  ModuleArray<ModuleVelocity> velocities{};
};

struct ChassisTwistEstimate
{
  SwerveStatus status{SwerveStatus::kNonFiniteInput};
  ChassisTwist twist{};
  double residual_rms_mps{0.0};
};

struct ModuleCommandResult
{
  SwerveStatus status{SwerveStatus::kNonFiniteInput};
  ModuleArray<ModuleSetpoint> setpoints{};
  double saturation_scale{0.0};
};

}  // namespace kinco_swerve_core
