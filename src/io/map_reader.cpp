#include "moon_planner/io/map_reader.hpp"

namespace moon_planner {

OccupancyGrid MapReader::ReadEmptyMap(int width, int height, double resolution_m) const {
  return OccupancyGrid(GridIndex(width, height, resolution_m), OccupancyGrid::kFree);
}

}  // namespace moon_planner
