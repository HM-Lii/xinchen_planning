#ifndef PLANNER_H
#define PLANNER_H

#include <cstddef>
#include <vector>

#include "longitudinal_qp.h"
#include "path_stitcher.h"
#include "planner_monitor.h"
#include "planner_types.h"
#include "speed_reference.h"

struct PlannerConfig {
  std::size_t output_points = 50;
  double time_step_seconds = 0.02;
  double target_speed_mph = 49.5;
  std::size_t qp_horizon_steps = 80;
  double qp_time_step_seconds = 0.1;
  double min_acceleration_mps2 = -5.0;
  double max_acceleration_mps2 = 3.0;
  double max_jerk_mps3 = 8.0;
  double initial_jerk_continuity_weight = 0.4;
  double reference_min_acceleration_mps2 = -2.5;
  double reference_max_jerk_mps3 = 2.0;
  double time_headway_seconds = 1.5;
  double standstill_gap_meters = 2.0;
  double ego_length_meters = 4.8;
  double obstacle_length_meters = 4.8;
  double ego_width_meters = 2.0;
  double obstacle_width_meters = 2.0;
  double prediction_margin_meters = 1.0;
  double traffic_lookahead_meters = 250.0;
  double lane_boundary_margin_meters = 0.35;
  double lane_width_meters = 4.0;
  int lane_count = 3;
  PlannerMonitorConfig monitor;
};

class PathPlanner {
public:
  explicit PathPlanner(const PlannerConfig &config = PlannerConfig());

  PlannerOutput Plan(const PlannerInput &input, const MapData &map);
  double reference_speed_mps() const { return reference_speed_mps_; }
  bool last_plan_emergency() const { return last_plan_emergency_; }
  const PlannerCycleDiagnostics &last_diagnostics() const {
    return last_diagnostics_;
  }

private:
  PlannerConfig config_;
  LongitudinalQp longitudinal_qp_;
  SpeedReferenceGenerator speed_reference_generator_;
  PathStitcher path_stitcher_;
  PlannerRuntimeMonitor runtime_monitor_;
  std::vector<LongitudinalState> last_output_states_;
  std::vector<double> last_output_x_;
  std::vector<double> last_output_y_;
  std::uint64_t plan_cycle_ = 0;
  PlannerCycleDiagnostics last_diagnostics_;
  bool last_plan_emergency_ = false;
  double reference_speed_mps_ = 0.0;
  int target_lane_ = -1;
};

#endif // PLANNER_H
