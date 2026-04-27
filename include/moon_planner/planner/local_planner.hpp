#pragma once

#include "moon_planner/planner/lattice_planner.hpp"

namespace moon_planner {

class LocalPlanner : public PlannerInterface {
 public:
  LocalPlanner(PlannerConfig planner_config,
               VehicleConfig vehicle_config,
               CostConfig cost_config,
               PrimitiveLibrary primitive_library);

  PlanningResult Plan(const PlanningRequest& request, const OccupancyGrid& occupancy) override;
  PlanningResult Plan(const PlanningRequest& request, const OccupancyGrid& occupancy, const ElevationGrid* elevation);
  PlanningResult Plan(const PlanningRequest& request,
                      const OccupancyGrid& occupancy,
                      const ElevationGrid* elevation,
                      const HistoryLayer* history);

  PlanningRequest BuildLocalRequest(const PlanningRequest& request) const;

 private:
  PlannerConfig planner_config_;
  LatticePlanner lattice_planner_;
};

}  // namespace moon_planner
