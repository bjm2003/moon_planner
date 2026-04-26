#pragma once

namespace moon_planner {

struct VehicleConfig {
  double length_m{1.2};
  double width_m{0.9};
  double height_m{0.7};
  double wheel_base_m{0.8};
  double track_width_m{0.7};
  double ground_clearance_m{0.18};
  double safety_margin_m{0.2};
  double max_linear_velocity_mps{1.4};
  double max_angular_velocity_radps{0.8};
  double max_acceleration_mps2{0.3};
  double max_curvature_1pm{1.5};
  double max_slope_rad{0.35};

  double CollisionRadius() const;
  bool IsValid() const;
};

}  // namespace moon_planner
