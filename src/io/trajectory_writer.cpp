#include "moon_planner/io/trajectory_writer.hpp"

#include <fstream>

namespace moon_planner {

bool TrajectoryWriter::WriteCsv(const std::vector<TrajectoryPoint>& trajectory, const std::string& path) const {
  std::ofstream output(path);
  if (!output.good()) {
    return false;
  }
  output << "t,x,y,z,yaw,v,omega\n";
  for (const auto& point : trajectory) {
    output << point.relative_time_s << ',' << point.state.x << ',' << point.state.y << ',' << point.state.z << ','
           << point.state.yaw << ',' << point.state.v << ',' << point.state.omega << '\n';
  }
  return true;
}

}  // namespace moon_planner
