#include "moon_planner/collision/footprint.hpp"

#include "moon_planner/core/geometry.hpp"

namespace moon_planner {

Footprint::Footprint(VehicleConfig vehicle_config)
    : vehicle_config_(vehicle_config), radius_m_(vehicle_config.CollisionRadius()) {}

std::vector<Point2D> Footprint::Corners(const Pose2D& pose) const {
  const double half_length = 0.5 * vehicle_config_.length_m + vehicle_config_.safety_margin_m;
  const double half_width = 0.5 * vehicle_config_.width_m + vehicle_config_.safety_margin_m;
  const std::vector<Point2D> local{{half_length, half_width}, {half_length, -half_width}, {-half_length, -half_width}, {-half_length, half_width}};
  return TransformPolygon(local, pose);
}

}  // namespace moon_planner
