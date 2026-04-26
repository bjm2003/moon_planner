#pragma once

#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

class Footprint {
 public:
  explicit Footprint(VehicleConfig vehicle_config = {});

  double radius_m() const { return radius_m_; }
  std::vector<Point2D> Corners(const Pose2D& pose) const;

 private:
  VehicleConfig vehicle_config_;
  double radius_m_{0.0};
};

}  // namespace moon_planner
