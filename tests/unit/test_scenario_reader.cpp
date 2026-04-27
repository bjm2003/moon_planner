#include "moon_planner/io/scenario_reader.hpp"

#include <cassert>
#include <cmath>

int main() {
  moon_planner::ScenarioReader reader;
  const auto scenario = reader.Read("scenarios/flat_empty.yaml");
  assert(scenario.name == "flat_empty");
  assert(scenario.occupancy.index().width() == 160);
  assert(scenario.occupancy.index().height() == 80);
  assert(std::abs(scenario.occupancy.index().resolution_m() - 0.1) < 1e-9);
  assert(std::abs(scenario.request.start.x - 2.0) < 1e-9);
  assert(std::abs(scenario.request.goal.x - 13.0) < 1e-9);

  const auto slope_scenario = reader.Read("scenarios/slope_region.yaml");
  assert(slope_scenario.elevation.IsValid());
  assert(slope_scenario.elevation.SlopeMagnitude(80, 15) > 0.35);
  return 0;
}
