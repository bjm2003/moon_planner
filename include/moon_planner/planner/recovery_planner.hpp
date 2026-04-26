#pragma once

#include "moon_planner/core/types.hpp"

namespace moon_planner {

class RecoveryPlanner {
 public:
  PlanningResult BuildEmergencyStop(const State& current_state) const;
};

}  // namespace moon_planner
