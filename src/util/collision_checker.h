#pragma once

#include "src/data_struct/data_struct.h"

namespace PathOptimizationNS {

class Config;

class CollisionChecker {
 public:
  CollisionChecker();

  bool isSingleStateCollisionFreeImproved(const State &current);

  bool isSingleStateCollisionFree(const State &current);
};

}  // namespace PathOptimizationNS