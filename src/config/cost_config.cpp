#include "moon_planner/config/cost_config.hpp"

namespace moon_planner {

bool CostConfig::IsValid() const {
  return length >= 0.0 && turn >= 0.0 && slope >= 0.0 && roughness >= 0.0 &&
         safety >= 0.0 && reverse >= 0.0 && history >= 0.0;
}

}  // namespace moon_planner
