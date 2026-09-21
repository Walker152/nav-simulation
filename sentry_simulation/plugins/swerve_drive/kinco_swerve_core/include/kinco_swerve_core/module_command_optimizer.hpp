#pragma once

#include "kinco_swerve_core/types.hpp"

namespace kinco_swerve_core
{

class ModuleCommandOptimizer
{
public:
  ModuleCommandOptimizer(
    const SwerveGeometry & geometry,
    const ModuleCommandLimits & limits);

  ModuleCommandResult optimize(
    const ModuleArray<ModuleVelocity> & velocities,
    const ModuleArray<SteeringReference> & steering) const noexcept;

private:
  SwerveGeometry geometry_{};
  ModuleCommandLimits limits_{};
};

}  // namespace kinco_swerve_core
