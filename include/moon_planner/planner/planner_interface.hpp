#pragma once

#include "moon_planner/core/types.hpp"
#include "moon_planner/map/occupancy_grid.hpp"

namespace moon_planner {

class PlannerInterface {
 public:
  virtual ~PlannerInterface() = default;
  virtual PlanningResult Plan(const PlanningRequest& request, const OccupancyGrid& occupancy) = 0;
};

}  // namespace moon_planner
