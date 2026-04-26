#pragma once

#include <chrono>

namespace moon_planner {

class Stopwatch {
 public:
  Stopwatch() : start_(std::chrono::steady_clock::now()) {}

  void Reset() { start_ = std::chrono::steady_clock::now(); }

  double ElapsedMilliseconds() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(now - start_).count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

}  // namespace moon_planner
