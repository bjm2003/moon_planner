#pragma once

#include <algorithm>
#include <cmath>

namespace moon_planner {

template <typename T>
constexpr T Square(T value) {
  return value * value;
}

template <typename T>
constexpr T Clamp(T value, T lower, T upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

inline bool NearlyEqual(double lhs, double rhs, double eps = 1e-9) {
  return std::abs(lhs - rhs) <= eps;
}

}  // namespace moon_planner
