#include "moon_planner/io/primitive_io.hpp"

#include <fstream>

namespace moon_planner {

bool PrimitiveIO::WriteSummary(const PrimitiveLibrary& library, const std::string& path) const {
  std::ofstream output(path);
  if (!output.good()) {
    return false;
  }
  output << "heading_bins: " << library.heading_bins() << '\n';
  output << "primitive_count: " << library.Size() << '\n';
  return true;
}

}  // namespace moon_planner
