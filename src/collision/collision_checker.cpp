#include "moon_planner/collision/collision_checker.hpp"

#include <cmath>

namespace moon_planner {

CollisionChecker::CollisionChecker() : footprint_(Footprint()) {}

CollisionChecker::CollisionChecker(Footprint footprint) : footprint_(footprint) {}

bool CollisionChecker::IsStateCollisionFree(const State& state, const OccupancyGrid& occupancy) const {
  const auto center = occupancy.index().WorldToCell(state.x, state.y);
  if (!center) {
    return false;
  }
  const int radius_cells = static_cast<int>(std::ceil(footprint_.radius_m() / occupancy.index().resolution_m()));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int x = center->x + dx;
      const int y = center->y + dy;
      const double dist_cells = std::sqrt(static_cast<double>(dx * dx + dy * dy));
      if (dist_cells <= static_cast<double>(radius_cells) && occupancy.IsOccupiedCell(x, y)) {
        return false;
      }
    }
  }
  return true;
}

bool CollisionChecker::IsPrimitiveCollisionFree(const State& origin,
                                                const MotionPrimitive& primitive,
                                                const OccupancyGrid& occupancy) const {
  for (const State& relative : primitive.relative_states) {
    State world = relative;
    world.x = origin.x + relative.x;
    world.y = origin.y + relative.y;
    world.yaw = relative.yaw;
    if (!IsStateCollisionFree(world, occupancy)) {
      return false;
    }
  }
  return true;
}

}  // namespace moon_planner
