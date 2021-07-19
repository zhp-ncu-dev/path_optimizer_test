#include "collision_checker.h"
#include "src/config/planning_flags.h"

namespace PathOptimizationNS {

CollisionChecker::CollisionChecker() {}

bool CollisionChecker::isSingleStateCollisionFree(const State &current) {
  return true;
}

bool CollisionChecker::isSingleStateCollisionFreeImproved(
    const State &current) {
  return false;
}

}  // namespace PathOptimizationNS
