#include "moon_planner/io/map_writer.hpp"

#include <fstream>

namespace moon_planner {

bool MapWriter::WriteAscii(const OccupancyGrid& grid, const std::string& path) const {
  std::ofstream output(path);
  if (!output.good() || !grid.IsValid()) {
    return false;
  }
  for (int y = 0; y < grid.index().height(); ++y) {
    for (int x = 0; x < grid.index().width(); ++x) {
      output << (grid.IsOccupiedCell(x, y) ? '#' : '.');
    }
    output << '\n';
  }
  return true;
}

}  // namespace moon_planner
