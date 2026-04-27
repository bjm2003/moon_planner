#pragma once

#include "moon_planner/config/cost_config.hpp"
#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/config/vehicle_config.hpp"
#include "moon_planner/map/elevation_grid.hpp"
#include "moon_planner/map/history_layer.hpp"
#include "moon_planner/planner/planner_interface.hpp"
#include "moon_planner/primitives/primitive_library.hpp"

namespace moon_planner {

class LatticePlanner : public PlannerInterface {
 public:
  LatticePlanner(PlannerConfig planner_config,
                 VehicleConfig vehicle_config,
                 CostConfig cost_config,
                 PrimitiveLibrary primitive_library);

  PlanningResult Plan(const PlanningRequest& request, const OccupancyGrid& occupancy) override;
  PlanningResult Plan(const PlanningRequest& request, const OccupancyGrid& occupancy, const ElevationGrid* elevation);
  PlanningResult Plan(const PlanningRequest& request,
                      const OccupancyGrid& occupancy,
                      const ElevationGrid* elevation,
                      const HistoryLayer* history);

 private:
  PlannerConfig planner_config_;
  VehicleConfig vehicle_config_;
  CostConfig cost_config_;
  PrimitiveLibrary primitive_library_;
};

}  // namespace moon_planner
