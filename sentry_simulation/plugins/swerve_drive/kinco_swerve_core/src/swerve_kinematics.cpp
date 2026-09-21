#include "kinco_swerve_core/swerve_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace kinco_swerve_core
{
namespace
{

using Matrix3 = std::array<std::array<double, 3>, 3>;

bool finite(const double value) noexcept
{
  return std::isfinite(value);
}

Matrix3 invert_3x3(const Matrix3 & matrix)
{
  std::array<std::array<double, 6>, 3> augmented{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t col = 0; col < 3U; ++col) {
      augmented[row][col] = matrix[row][col];
    }
    augmented[row][row + 3U] = 1.0;
  }

  for (std::size_t pivot = 0; pivot < 3U; ++pivot) {
    std::size_t best_row = pivot;
    for (std::size_t row = pivot + 1U; row < 3U; ++row) {
      if (std::abs(augmented[row][pivot]) > std::abs(augmented[best_row][pivot])) {
        best_row = row;
      }
    }
    if (!finite(augmented[best_row][pivot]) ||
      std::abs(augmented[best_row][pivot]) < 1e-12)
    {
      throw std::invalid_argument("swerve module geometry is singular");
    }
    if (best_row != pivot) {
      std::swap(augmented[best_row], augmented[pivot]);
    }

    const double divisor = augmented[pivot][pivot];
    for (std::size_t col = 0; col < 6U; ++col) {
      augmented[pivot][col] /= divisor;
    }
    for (std::size_t row = 0; row < 3U; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = augmented[row][pivot];
      for (std::size_t col = 0; col < 6U; ++col) {
        augmented[row][col] -= factor * augmented[pivot][col];
      }
    }
  }

  Matrix3 inverse{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t col = 0; col < 3U; ++col) {
      inverse[row][col] = augmented[row][col + 3U];
      if (!finite(inverse[row][col])) {
        throw std::invalid_argument("swerve module geometry inverse is non-finite");
      }
    }
  }
  return inverse;
}

}  // namespace

SwerveKinematics::SwerveKinematics(
  const SwerveGeometry & geometry)
: geometry_(geometry)
{
  double centroid_x_m = 0.0;
  double centroid_y_m = 0.0;
  for (const auto & module : geometry_.modules) {
    if (!finite(module.position.x_m) || !finite(module.position.y_m)) {
      throw std::invalid_argument("swerve module position must be finite");
    }
    centroid_x_m += module.position.x_m;
    centroid_y_m += module.position.y_m;
  }
  const double module_count = static_cast<double>(kModuleCount);
  centroid_x_m /= module_count;
  centroid_y_m /= module_count;
  if (!finite(centroid_x_m) || !finite(centroid_y_m)) {
    throw std::invalid_argument("swerve module centroid must be finite");
  }

  double centered_spread_m2 = 0.0;
  for (const auto & module : geometry_.modules) {
    const double offset_x_m = module.position.x_m - centroid_x_m;
    const double offset_y_m = module.position.y_m - centroid_y_m;
    centered_spread_m2 += offset_x_m * offset_x_m + offset_y_m * offset_y_m;
  }
  if (!finite(centered_spread_m2) || centered_spread_m2 < 1e-12) {
    throw std::invalid_argument(
            "swerve module positions make the forward-kinematics matrix rank deficient");
  }

  Matrix3 normal{};
  for (const auto & module : geometry_.modules) {
    const std::array<double, 3> row_x{1.0, 0.0, -module.position.y_m};
    const std::array<double, 3> row_y{0.0, 1.0, module.position.x_m};
    for (std::size_t row = 0; row < 3U; ++row) {
      for (std::size_t col = 0; col < 3U; ++col) {
        normal[row][col] += row_x[row] * row_x[col] + row_y[row] * row_y[col];
      }
    }
  }
  normal_inverse_ = invert_3x3(normal);
}

ModuleVelocityResult SwerveKinematics::to_module_velocities(
  const ChassisTwist & twist) const noexcept
{
  ModuleVelocityResult result{};
  if (!finite(twist.linear_x_mps) || !finite(twist.linear_y_mps) ||
    !finite(twist.angular_z_radps))
  {
    return result;
  }

  ModuleArray<ModuleVelocity> velocities{};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const auto & position = geometry_.modules[i].position;
    const double x_mps = twist.linear_x_mps - twist.angular_z_radps * position.y_m;
    const double y_mps = twist.linear_y_mps + twist.angular_z_radps * position.x_m;
    if (!finite(x_mps) || !finite(y_mps)) {
      return ModuleVelocityResult{};
    }
    velocities[i] = {x_mps, y_mps};
  }
  result.velocities = velocities;
  result.status = SwerveStatus::kOk;
  return result;
}

ChassisTwistEstimate SwerveKinematics::estimate_chassis_twist(
  const ModuleArray<ModuleState> & modules) const noexcept
{
  ChassisTwistEstimate result{};
  std::array<double, 3> rhs{};
  ModuleArray<ModuleVelocity> measured_velocities{};

  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const auto & state = modules[i];
    if (!finite(state.wheel_speed_mps) || !finite(state.steering_angle_rad)) {
      return result;
    }
    const double velocity_x_mps =
      state.wheel_speed_mps * std::cos(state.steering_angle_rad);
    const double velocity_y_mps =
      state.wheel_speed_mps * std::sin(state.steering_angle_rad);
    if (!finite(velocity_x_mps) || !finite(velocity_y_mps)) {
      return result;
    }
    measured_velocities[i] = {velocity_x_mps, velocity_y_mps};

    const auto & position = geometry_.modules[i].position;
    const double yaw_contribution =
      -position.y_m * velocity_x_mps + position.x_m * velocity_y_mps;
    if (!finite(yaw_contribution)) {
      return result;
    }
    rhs[0] += velocity_x_mps;
    rhs[1] += velocity_y_mps;
    rhs[2] += yaw_contribution;
    if (!finite(rhs[0]) || !finite(rhs[1]) || !finite(rhs[2])) {
      return result;
    }
  }

  std::array<double, 3> solution{};
  for (std::size_t row = 0; row < 3U; ++row) {
    for (std::size_t col = 0; col < 3U; ++col) {
      const double contribution = normal_inverse_[row][col] * rhs[col];
      if (!finite(contribution)) {
        return result;
      }
      solution[row] += contribution;
      if (!finite(solution[row])) {
        return result;
      }
    }
  }
  const ChassisTwist twist{solution[0], solution[1], solution[2]};

  double squared_error_mps2 = 0.0;
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const auto & position = geometry_.modules[i].position;
    const double predicted_x_mps = twist.linear_x_mps -
      twist.angular_z_radps * position.y_m;
    const double predicted_y_mps = twist.linear_y_mps +
      twist.angular_z_radps * position.x_m;
    const double error_x_mps = predicted_x_mps - measured_velocities[i].x_mps;
    const double error_y_mps = predicted_y_mps - measured_velocities[i].y_mps;
    if (!finite(error_x_mps) || !finite(error_y_mps)) {
      return result;
    }
    squared_error_mps2 +=
      error_x_mps * error_x_mps + error_y_mps * error_y_mps;
    if (!finite(squared_error_mps2)) {
      return result;
    }
  }

  const double residual_rms_mps = std::sqrt(
    squared_error_mps2 / (2.0 * static_cast<double>(kModuleCount)));
  if (!finite(residual_rms_mps)) {
    return result;
  }
  result.twist = twist;
  result.residual_rms_mps = residual_rms_mps;
  result.status = SwerveStatus::kOk;
  return result;
}

}  // namespace kinco_swerve_core
