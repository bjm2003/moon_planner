#pragma once

#include "moon_planner/core/types.hpp"

namespace moon_planner {

class ScenarioReader {
 public:
  PlanningRequest DefaultRequest() const;
};

}  // namespace moon_planner
