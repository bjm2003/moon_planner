#include "moon_planner/runtime/diagnostics.hpp"

#include <sstream>

namespace moon_planner {

std::string FormatDiagnostics(const PlannerDiagnostics& diagnostics) {
  std::ostringstream stream;
  stream << "expanded_nodes=" << diagnostics.expanded_nodes
         << ", planning_time_ms=" << diagnostics.planning_time_ms
         << ", path_length_m=" << diagnostics.path_length_m
         << ", total_cost=" << diagnostics.total_cost
         << ", message=" << diagnostics.message;
  return stream.str();
}

}  // namespace moon_planner
