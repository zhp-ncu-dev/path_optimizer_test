#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "src/data_struct/data_struct.h"
#include "src/data_struct/reference_line_impl.h"
#include "src/util/map.h"
#include "src/util/spline.h"
#include "src/util/tools.h"

namespace PathOptimizationNS {

class ReferencePath {
 public:
  ReferencePath();
  const tk::spline &getXS() const;
  const tk::spline &getYS() const;
  double getXS(double s) const;
  double getYS(double s) const;
  void setSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
  void setOriginalSpline(const tk::spline &x_s, const tk::spline &y_s,
                         double max_s);
  void clear();
  std::size_t getSize() const;
  double getLength() const;
  void setLength(double s);
  const std::vector<State> &getReferenceStates() const;
  const std::vector<CoveringCircleBounds> &getBounds() const;
  const std::vector<double> &getMaxKList() const;
  const std::vector<double> &getMaxKpList() const;
  std::vector<std::tuple<State, double, double>> display_abnormal_bounds()
      const;
  // Set reference_states_ directly, only used in solveWithoutSmoothing.
  void setReference(const std::vector<State> &reference);
  void setReference(const std::vector<State> &&reference);
  // Calculate upper and lower bounds for each covering circle.
  void updateBounds(const Map &map);
  // If the reference_states_ have speed and acceleration information, call this
  // func to calculate curvature and curvature rate bounds.
  void updateLimits();
  // Calculate reference_states_ from x_s_ and y_s_, given delta s.
  bool buildReferenceFromSpline(double delta_s_smaller, double delta_s_larger);

 private:
  std::shared_ptr<ReferencePathImpl> reference_path_impl_;
};

} 