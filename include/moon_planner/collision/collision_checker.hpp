#pragma once

#include "moon_planner/collision/footprint.hpp"
#include "moon_planner/map/occupancy_grid.hpp"
#include "moon_planner/primitives/motion_primitive.hpp"

namespace moon_planner {

class CollisionChecker {
 public:
  CollisionChecker();
  explicit CollisionChecker(Footprint footprint);

  bool IsStateCollisionFree(const State& state, const OccupancyGrid& occupancy) const;
  bool IsPrimitiveCollisionFree(const State& origin, const MotionPrimitive& primitive, const OccupancyGrid& occupancy) const;

 private:
  Footprint footprint_;
};

}  // namespace moon_planner
