#pragma once

#include "moon_planner/core/types.hpp"
#include "moon_planner/map/elevation_grid.hpp"
#include "moon_planner/map/occupancy_grid.hpp"

#include <string>

namespace moon_planner {

struct PlanningScenario {
  std::string name{"default"};
  PlanningRequest request;
  OccupancyGrid occupancy;
  ElevationGrid elevation;
};

class ScenarioReader {
 public:
  PlanningScenario Read(const std::string& path) const;
  PlanningScenario DefaultScenario() const;
  PlanningRequest DefaultRequest() const;
};

}  // namespace moon_planner
