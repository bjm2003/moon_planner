#include "moon_planner/collision/collision_checker.hpp"
#include "moon_planner/collision/footprint.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/map/occupancy_grid.hpp"

#include <cassert>

int main() {
  using namespace moon_planner;

  VehicleConfig vehicle_config;
  vehicle_config.length_m = 1.0;
  vehicle_config.width_m = 0.6;
  vehicle_config.safety_margin_m = 0.0;
  CollisionChecker checker(Footprint(vehicle_config));
  OccupancyGrid occupancy(GridIndex(100, 100, 0.1), OccupancyGrid::kFree);

  State state;
  state.x = 5.0;
  state.y = 5.0;
  state.yaw = 0.0;
  assert(checker.IsStateCollisionFree(state, occupancy));

  occupancy.SetOccupiedWorld(5.0, 5.0);
  assert(!checker.IsStateCollisionFree(state, occupancy));

  occupancy = OccupancyGrid(GridIndex(100, 100, 0.1), OccupancyGrid::kFree);
  occupancy.SetOccupiedWorld(5.7, 5.0);
  assert(checker.IsStateCollisionFree(state, occupancy));

  occupancy.SetOccupiedWorld(5.45, 5.0);
  assert(!checker.IsStateCollisionFree(state, occupancy));
  return 0;
}
