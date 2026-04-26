#pragma once

#include "moon_planner/core/types.hpp"

#include <vector>

namespace moon_planner {

constexpr double kPi = 3.14159265358979323846;

double NormalizeAngle(double angle_rad);
double Distance2D(const Point2D& lhs, const Point2D& rhs);
double Distance2D(double x0, double y0, double x1, double y1);
Point2D RotatePoint(const Point2D& point, double yaw_rad);
Point2D TransformPoint(const Point2D& local_point, const Pose2D& pose);
std::vector<Point2D> TransformPolygon(const std::vector<Point2D>& local_polygon, const Pose2D& pose);

}  // namespace moon_planner
