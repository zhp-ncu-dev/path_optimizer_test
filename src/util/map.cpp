#include "map.h"

#include "glog/logging.h"

namespace PathOptimizationNS {

Map::Map() {}

double Map::getObstacleDistance(const Eigen::Vector2d &pos) const {
  return 0.0;
}

bool Map::isInside(const Eigen::Vector2d &pos) const { return false; }

}  // namespace PathOptimizationNS