#include "moon_planner/model/motion_constraints.hpp"

#include <cmath>

namespace moon_planner {

MotionConstraints::MotionConstraints(VehicleConfig vehicle_config) : vehicle_config_(vehicle_config) {}

bool MotionConstraints::IsControlValid(double v_mps, double omega_radps) const {
  if (std::abs(v_mps) > vehicle_config_.max_linear_velocity_mps) {
    return false;
  }
  if (std::abs(omega_radps) > vehicle_config_.max_angular_velocity_radps) {
    return false;
  }
  if (std::abs(v_mps) > 1e-6) {
    const double curvature = std::abs(omega_radps / v_mps);
    if (curvature > vehicle_config_.max_curvature_1pm) {
      return false;
    }
  }
  return true;
}

bool MotionConstraints::IsSlopeTraversable(double slope_rad) const {
  return std::abs(slope_rad) <= vehicle_config_.max_slope_rad;
}

}  // namespace moon_planner
