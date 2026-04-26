#pragma once

#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/map/cost_map.hpp"
#include "moon_planner/map/elevation_grid.hpp"

namespace moon_planner {

class TraversabilityEvaluator {
 public:
  explicit TraversabilityEvaluator(MotionConstraints constraints);

  bool IsTraversable(double x_m, double y_m, const CostMap& cost_map, const ElevationGrid* elevation = nullptr) const;

 private:
  MotionConstraints constraints_;
};

}  // namespace moon_planner
