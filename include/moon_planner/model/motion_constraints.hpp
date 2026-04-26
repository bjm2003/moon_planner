#pragma once

#include "moon_planner/config/vehicle_config.hpp"

namespace moon_planner {

class MotionConstraints {
 public:
  explicit MotionConstraints(VehicleConfig vehicle_config);

  bool IsControlValid(double v_mps, double omega_radps) const;
  bool IsSlopeTraversable(double slope_rad) const;
  const VehicleConfig& vehicle_config() const { return vehicle_config_; }

 private:
  VehicleConfig vehicle_config_;
};

}  // namespace moon_planner
