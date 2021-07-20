#include <iostream>

#include "src/data_struct/data_struct.h"
#include "src/path_optimizer.h"
#include "src/util/map.h"
#include "src/util/matplotlib_cpp.h"

using namespace PathOptimizationNS;
namespace plt = matplotlibcpp;

int main() {
  // create map
  const double delta_x = 0.5;
  const double road_width = 10;
  const size_t total_boundary_size = 200;
  std::vector<State> left_boundary(total_boundary_size);
  std::vector<State> right_boundary(total_boundary_size);
  std::vector<State> original_path(total_boundary_size);
  std::vector<double> plot_x, plot_left_y, plot_right_y, plot_center_y;
  for (size_t i = 0; i < total_boundary_size; ++i) {
    const double x = delta_x * i;
    const double left_y = road_width;
    const double right_y = 0.0;
    left_boundary[i] = {x, left_y};
    right_boundary[i] = {x, right_y};
    original_path[i] = {x, road_width / 2.0};
    plot_x.emplace_back(x);
    plot_left_y.emplace_back(left_y);
    plot_right_y.emplace_back(right_y);
    plot_center_y.emplace_back(road_width / 2.0);
  }
  plt::plot(plot_x, plot_left_y, "b-");
  plt::plot(plot_x, plot_right_y, "b-");
  // plt::plot(plot_x, plot_center_y, "r--");

  Map map(original_path, left_boundary, right_boundary);

  // creat path_optimizer
  State start_state = original_path.front();
  State end_state = original_path.back();
  PathOptimizer path_optimizer(start_state, end_state, map);

  // get smoothed reference path
  std::vector<State> result_path, smoothed_reference_path;
  if (!path_optimizer.solve(original_path, &result_path)) {
    LOG(ERROR) << "smooth failed.";
    return 0.0;
  }

  // if (!path_optimizer.solve_smooth_reference_path(original_path,
  //                                                 &smoothed_reference_path))
  //                                                 {
  //   LOG(ERROR) << "smooth failed.";
  //   return 0;
  // }
  // std::vector<double> smoothed_x, smoothed_y;
  // for (const auto& point : smoothed_reference_path) {
  //   smoothed_x.emplace_back(point.x);
  //   smoothed_y.emplace_back(point.y);
  // }
  // plt::plot(smoothed_x, smoothed_y, "r-");

  plt::show();
  return 0;
}  // namespace plt=matplotlibcppintmain()