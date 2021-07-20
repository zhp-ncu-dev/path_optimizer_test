#include "map.h"

#include <limits>

#include "glog/logging.h"

namespace PathOptimizationNS {

Map::Map(const std::vector<State>& original_path,
         const std::vector<State>& left_boundary,
         const std::vector<State>& right_boundary)
    : original_path_(original_path),
      left_boundary_(left_boundary),
      right_boundary_(right_boundary) {}

const std::vector<State>& Map::GetOriginalPath() const {
  return original_path_;
}

const std::vector<State>& Map::GetLeftBoundary() const {
  return left_boundary_;
}

const std::vector<State>& Map::GetRightBoundary() const {
  return right_boundary_;
}

double Map::GetObstacleDistance(const State& pos) const {
  double left_min_dis = 0.0;
  int left_index = -1;
  double right_min_dis = 0.0;
  int right_index = -1;
  if (!IsInside(pos, &left_min_dis, &left_index, &right_min_dis,
                &right_index)) {
    LOG(INFO) << "pos is not in map boundary";
    return 0.0;
  }
  return std::sqrt(std::min(left_min_dis, right_min_dis));
}

bool Map::IsInside(const State& pos, double* left_min_dis, int* left_index,
                   double* right_min_dis, int* right_index) const {
  if (!ClosedPointIndex(left_boundary_, pos, left_index, left_min_dis)) {
    LOG(ERROR) << "get left min_dis error.";
    return false;
  }
  if (!ClosedPointIndex(right_boundary_, pos, right_index, right_min_dis)) {
    LOG(ERROR) << "get right min_dis error.";
    return false;
  }
  // next pos
  int left_next_pos = *left_index > left_boundary_.size() - 1 ? *left_index - 1
                                                              : *left_index + 1;
  int right_next_pos = *right_index > right_boundary_.size() - 1
                           ? *right_index - 1
                           : *right_index + 1;

  // 向量交叉相乘
  State right_state_1 = right_boundary_[std::min(*right_index, right_next_pos)];
  State right_state_2 = right_boundary_[std::max(*right_index, right_next_pos)];
  State left_state_1 = left_boundary_[std::min(*left_index, left_next_pos)];
  State left_state_2 = left_boundary_[std::max(*left_index, left_next_pos)];

  double right_value = (right_state_1.x - pos.x) * (right_state_2.y - pos.y) -
                       (right_state_2.x - pos.x) * (right_state_1.y - pos.y);
  double left_value = (left_state_1.x - pos.x) * (left_state_2.y - pos.y) -
                      (left_state_2.x - pos.x) * (left_state_1.y - pos.y);
  if (right_value > 0 && left_value < 0) return true;
  return false;
}

bool Map::ClosedPointIndex(const std::vector<State>& boundary,
                           const State& point, int* index,
                           double* min_dis) const {
  if (boundary.size() < 3) {
    LOG(ERROR) << "boundary size less 3.";
    return false;
  }
  *min_dis = std::numeric_limits<double>::max();
  for (size_t i = 0; i < boundary.size(); ++i) {
    double current_dis = (point.x - boundary[i].x) * (point.x - boundary[i].x) +
                         (point.y - boundary[i].y) * (point.y - boundary[i].y);
    if (*min_dis > current_dis) {
      *min_dis = current_dis;
      *index = i;
    }
  }
  return true;
}

}  // namespace PathOptimizationNS