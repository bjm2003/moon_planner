#include "moon_planner/collision/collision_checker.hpp"

#include "moon_planner/core/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace moon_planner {

namespace {

bool CellSquareIntersectsFootprint(int cell_x,
                                   int cell_y,
                                   const GridIndex& grid_index,
                                   const std::vector<Point2D>& footprint_corners) {
  const Point2D center = grid_index.CellCenter(cell_x, cell_y);
  const double half = 0.5 * grid_index.resolution_m();
  const Point2D cell_corners[4] = {
      {center.x - half, center.y - half},
      {center.x + half, center.y - half},
      {center.x + half, center.y + half},
      {center.x - half, center.y + half},
  };

  for (const Point2D& corner : cell_corners) {
    if (IsPointInsideConvexPolygon(corner, footprint_corners)) {
      return true;
    }
  }

  const double min_x = center.x - half;
  const double max_x = center.x + half;
  const double min_y = center.y - half;
  const double max_y = center.y + half;
  for (const Point2D& corner : footprint_corners) {
    if (corner.x >= min_x && corner.x <= max_x && corner.y >= min_y && corner.y <= max_y) {
      return true;
    }
  }
  return false;
}

}  // namespace

CollisionChecker::CollisionChecker() : footprint_(Footprint()) {}

CollisionChecker::CollisionChecker(Footprint footprint) : footprint_(footprint) {}

bool CollisionChecker::IsStateCollisionFree(const State& state, const OccupancyGrid& occupancy) const {
  const auto center = occupancy.index().WorldToCell(state.x, state.y);
  if (!center) {
    return false;
  }
  const std::vector<Point2D> footprint_corners = footprint_.Corners(Pose2D{state.x, state.y, state.yaw});
  const int radius_cells = static_cast<int>(std::ceil(footprint_.radius_m() / occupancy.index().resolution_m()));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int x = center->x + dx;
      const int y = center->y + dy;
      if (!occupancy.index().IsInside(x, y)) {
        return false;
      }
      if (occupancy.IsOccupiedCell(x, y) &&
          CellSquareIntersectsFootprint(x, y, occupancy.index(), footprint_corners)) {
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
