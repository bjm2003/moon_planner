#pragma once

#include "moon_planner/core/types.hpp"

namespace moon_planner {

class RecoveryPlanner {
 public:
  PlanningRequest BuildReverseRecoveryRequest(const State& current_state, double reverse_distance_m) const;
  PlanningResult BuildReverseRecovery(const State& current_state,
                                      double reverse_distance_m,
                                      double reverse_speed_mps = 0.2) const;
  PlanningResult BuildEmergencyStop(const State& current_state) const;
};

}  // namespace moon_planner
