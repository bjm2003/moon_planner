#include "moon_planner/trajectory/path_extractor.hpp"

namespace moon_planner {

std::vector<State> PathExtractor::Simplify(const std::vector<State>& path) const {
  return path;
}

}  // namespace moon_planner
