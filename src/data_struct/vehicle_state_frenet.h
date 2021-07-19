#pragma once

#include <vector>

#include "src/data_struct/data_struct.h"

namespace PathOptimizationNS {

class VehicleState {
 public:
  VehicleState();
  VehicleState(const State &start_state, const State &end_state,
               double offset = 0, double heading_error = 0);
  ~VehicleState() = default;
  const State &getStartState() const;
  const State &getEndState() const;
  void setStartState(const State &state);
  void setEndState(const State &state);
  std::vector<double> getInitError() const;
  void setInitError(double init_offset, double init_heading_error);

 private:
  // Initial error with reference line.
  double initial_offset_{};
  double initial_heading_error_{};
  // Initial state.
  State start_state_;
  // Target state.
  State end_state_;
};

}  // namespace PathOptimizationNS