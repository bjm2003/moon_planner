#pragma once

#include "moon_planner/primitives/motion_primitive.hpp"

#include <vector>

namespace moon_planner {

class PrimitiveLibrary {
 public:
  PrimitiveLibrary() = default;
  explicit PrimitiveLibrary(int heading_bins);

  int heading_bins() const { return heading_bins_; }
  bool IsValid() const { return heading_bins_ > 0 && primitives_by_heading_.size() == static_cast<std::size_t>(heading_bins_); }
  void Reset(int heading_bins);
  void Add(MotionPrimitive primitive);
  const std::vector<MotionPrimitive>& Query(int heading_bin) const;
  std::size_t Size() const;

 private:
  int NormalizeHeadingBin(int heading_bin) const;

  int heading_bins_{0};
  std::vector<std::vector<MotionPrimitive>> primitives_by_heading_;
};

}  // namespace moon_planner
