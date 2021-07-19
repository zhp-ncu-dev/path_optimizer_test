#pragma once

#include <iostream>
#include <vector>

#include "src/data_struct/data_struct.h"

namespace PathOptimizationNS {

class CarGeometry {
 public:
  CarGeometry() = default;
  CarGeometry(double width, double back_length, double front_length);
  void init(double width, double back_length, double front_length);
  std::vector<Circle> getCircles(const State &pos) const;
  Circle getBoundingCircle(const State &pos) const;

 private:
  void setCircles();
  double width_{}, length_{}, front_length_{}, back_length_{};
  // Four corners:
  // f: front, r: rear
  // l: left, r: right
  State fl_p_, fr_p_, rl_p_, rr_p_;
  std::vector<Circle> circles_;
  Circle bounding_c_;
};
}