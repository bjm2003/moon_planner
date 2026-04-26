#pragma once

#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"

namespace moon_planner {

struct PlannerContext {
  PlannerConfig planner_config;
  VehicleConfig vehicle_config;
  CostConfig cost_config;
};

}  // namespace moon_planner
