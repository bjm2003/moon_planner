#include "moon_planner/config/vehicle_config.hpp"

#include <cmath>

namespace moon_planner {

double VehicleConfig::CollisionRadius() const {
  return 0.5 * std::sqrt(length_m * length_m + width_m * width_m) + safety_margin_m;
}

bool VehicleConfig::IsValid() const {
  return length_m > 0.0 && width_m > 0.0 && height_m > 0.0 && safety_margin_m >= 0.0 &&
         max_linear_velocity_mps > 0.0 && max_angular_velocity_radps > 0.0 &&
         max_acceleration_mps2 > 0.0 && max_curvature_1pm > 0.0 && max_slope_rad > 0.0;
}

}  // namespace moon_planner
