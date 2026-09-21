#pragma once

#include "kinco_swerve_core/types.hpp"

namespace kinco_swerve_core
{

class SwerveKinematics
{
public:
  explicit SwerveKinematics(const SwerveGeometry & geometry);

  ModuleVelocityResult to_module_velocities(const ChassisTwist & twist) const noexcept;

  ChassisTwistEstimate estimate_chassis_twist(
    const ModuleArray<ModuleState> & modules) const noexcept;

private:
  using Matrix3 = std::array<std::array<double, 3>, 3>;

  SwerveGeometry geometry_{};
  Matrix3 normal_inverse_{};
};

}  // namespace kinco_swerve_core
