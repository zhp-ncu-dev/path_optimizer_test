#pragma once

#include "eigen3/Eigen/Core"

namespace PathOptimizationNS {

class Map {
 public:
  Map();
  double getObstacleDistance(const Eigen::Vector2d &pos) const;
  bool isInside(const Eigen::Vector2d &pos) const;
};
}  // namespace PathOptimizationNS