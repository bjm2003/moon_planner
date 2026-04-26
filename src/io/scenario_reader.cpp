#include "moon_planner/io/scenario_reader.hpp"

namespace moon_planner {

PlanningRequest ScenarioReader::DefaultRequest() const {
  PlanningRequest request;
  request.start.x = 1.0;
  request.start.y = 1.0;
  request.goal.x = 6.0;
  request.goal.y = 6.0;
  return request;
}

}  // namespace moon_planner
