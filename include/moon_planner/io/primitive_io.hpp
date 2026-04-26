#pragma once

#include "moon_planner/primitives/primitive_library.hpp"

#include <string>

namespace moon_planner {

class PrimitiveIO {
 public:
  bool WriteSummary(const PrimitiveLibrary& library, const std::string& path) const;
};

}  // namespace moon_planner
