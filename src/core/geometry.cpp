#include "moon_planner/core/geometry.hpp"

#include <cmath>

namespace moon_planner {

double NormalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double Distance2D(const Point2D& lhs, const Point2D& rhs) {
  return Distance2D(lhs.x, lhs.y, rhs.x, rhs.y);
}

double Distance2D(double x0, double y0, double x1, double y1) {
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

Point2D RotatePoint(const Point2D& point, double yaw_rad) {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  return {c * point.x - s * point.y, s * point.x + c * point.y};
}

Point2D TransformPoint(const Point2D& local_point, const Pose2D& pose) {
  Point2D rotated = RotatePoint(local_point, pose.yaw);
  rotated.x += pose.x;
  rotated.y += pose.y;
  return rotated;
}

std::vector<Point2D> TransformPolygon(const std::vector<Point2D>& local_polygon, const Pose2D& pose) {
  std::vector<Point2D> transformed;
  transformed.reserve(local_polygon.size());
  for (const auto& point : local_polygon) {
    transformed.push_back(TransformPoint(point, pose));
  }
  return transformed;
}

bool IsPointInsideConvexPolygon(const Point2D& point, const std::vector<Point2D>& polygon) {
  if (polygon.size() < 3) {
    return false;
  }
  double previous_cross = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2D& a = polygon[i];
    const Point2D& b = polygon[(i + 1) % polygon.size()];
    const double cross = (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
    if (std::abs(cross) < 1e-9) {
      continue;
    }
    if (previous_cross == 0.0) {
      previous_cross = cross;
      continue;
    }
    if ((previous_cross > 0.0 && cross < 0.0) || (previous_cross < 0.0 && cross > 0.0)) {
      return false;
    }
  }
  return true;
}

}  // namespace moon_planner
