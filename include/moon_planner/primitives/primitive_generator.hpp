#pragma once

#include "moon_planner/config/planner_config.hpp"
#include "moon_planner/model/motion_constraints.hpp"
#include "moon_planner/model/skid_steer_model.hpp"
#include "moon_planner/primitives/primitive_library.hpp"

namespace moon_planner {

class PrimitiveGenerator {
 public:
  PrimitiveLibrary Generate(const PlannerConfig& planner_config,
                            const PrimitiveGenerationConfig& primitive_config,
                            const MotionConstraints& constraints) const;
};

}  // namespace moon_planner
