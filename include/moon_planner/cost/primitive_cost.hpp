#pragma once

#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/map/cost_map.hpp"
#include "moon_planner/primitives/motion_primitive.hpp"

namespace moon_planner {

class PrimitiveCost {
 public:
  explicit PrimitiveCost(CostConfig config = {});

  double Evaluate(const State& origin, const MotionPrimitive& primitive, const CostMap& cost_map) const;

 private:
  CostConfig config_;
};

}  // namespace moon_planner
