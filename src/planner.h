#ifndef PLANNER_H
#define PLANNER_H

#include <cstddef>

#include "planner_types.h"

struct PlannerConfig {
  std::size_t output_points = 50;
  double time_step_seconds = 0.02;
  double target_speed_mph = 49.5;
  double max_acceleration_mps2 = 5.0;
  double lane_width_meters = 4.0;
  int lane_count = 3;
};

class PathPlanner {
public:
  explicit PathPlanner(const PlannerConfig &config = PlannerConfig());

  PlannerOutput Plan(const PlannerInput &input, const MapData &map);
  double reference_speed_mps() const { return reference_speed_mps_; }

private:
  PlannerConfig config_;
  bool speed_initialized_ = false;
  double reference_speed_mps_ = 0.0;
};

#endif // PLANNER_H
