#pragma once

#include <cassert>
#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>

namespace PathOptimizationNS {
namespace tk {
class spline;
}
}  // namespace PathOptimizationNS

namespace PathOptimizationNS {

class State;

// Set angle to -pi ~ pi
template <typename T>
T constraintAngle(T angle) {
  if (angle > M_PI) {
    angle -= 2 * M_PI;
    return constraintAngle(angle);
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
    return constraintAngle(angle);
  } else {
    return angle;
  }
}

// Time duration in seconds.
double time_s(const clock_t &begin, const clock_t &end);

// Time duration in ms.
double time_ms(const clock_t &begin, const clock_t &end);

// Output time duration.
void time_s_out(const clock_t &begin, const clock_t &end,
                const std::string &text);
void time_ms_out(const clock_t &begin, const clock_t &end,
                 const std::string &text);

// Return true if a == b.
bool isEqual(double a, double b);

// Calculate heading for spline.
double getHeading(const tk::spline &xs, const tk::spline &ys, double s);

// Calculate curvature for spline.
double getCurvature(const tk::spline &xs, const tk::spline &ys, double tmp_s);

// Calculate distance between two points.
double distance(const State &p1, const State &p2);

// Coordinate transform.
State local2Global(const State &reference, const State &target);
State global2Local(const State &reference, const State &target);

State findClosestPoint(const tk::spline &xs, const tk::spline &ys, double max_s,
                       const State &state, double grid = 0, double start_s = 0);

}  // namespace PathOptimizationNS
