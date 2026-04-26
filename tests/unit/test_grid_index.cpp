#include "moon_planner/map/grid_index.hpp"

#include <cassert>

int main() {
  moon_planner::GridIndex index(10, 20, 0.5, moon_planner::Point2D{1.0, 2.0});
  assert(index.IsValid());
  assert(index.CellCount() == 200);
  const auto cell = index.WorldToCell(1.25, 2.25);
  assert(cell.has_value());
  assert(cell->x == 0);
  assert(cell->y == 0);
  const auto center = index.CellCenter(0, 0);
  assert(center.x == 1.25);
  assert(center.y == 2.25);
  return 0;
}
