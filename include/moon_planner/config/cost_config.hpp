#pragma once

namespace moon_planner {

struct CostConfig {
  double length{1.0};
  double turn{0.25};
  double slope{2.0};
  double roughness{1.0};
  double safety{4.0};
  double reverse{1.5};
  double history{0.5};

  bool IsValid() const;
};

}  // namespace moon_planner
