#include "moon_planner/cost/traversability_evaluator.hpp"

namespace moon_planner {

TraversabilityEvaluator::TraversabilityEvaluator(MotionConstraints constraints) : constraints_(constraints) {}

bool TraversabilityEvaluator::IsTraversable(double x_m,
                                            double y_m,
                                            const CostMap& cost_map,
                                            const ElevationGrid* elevation) const {
  const auto cell = cost_map.index().WorldToCell(x_m, y_m);
  if (!cell || cost_map.IsLethal(cell->x, cell->y)) {
    return false;
  }
  if (elevation != nullptr && elevation->IsValid()) {
    return constraints_.IsSlopeTraversable(elevation->SlopeMagnitude(cell->x, cell->y));
  }
  return true;
}

}  // namespace moon_planner
