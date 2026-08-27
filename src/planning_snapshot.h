#ifndef PLANNING_SNAPSHOT_H
#define PLANNING_SNAPSHOT_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "longitudinal_types.h"
#include "path_stitcher.h"
#include "planner_types.h"

// Immutable control-cycle context shared by the planner's evaluation stages.
// Keeping this data contract separate from PathPlanner prevents leaf planning
// modules from depending on the top-level orchestration interface.
struct PlanningFrontier {
  double time_from_telemetry_s = 0.0;
  double road_s_unwrapped_m = 0.0;
  double d_m = 0.0;
  LongitudinalState longitudinal;
  LateralPathState lateral;
  PlanningStateSource state_source = PlanningStateSource::kTelemetry;
};

struct PlanningSnapshot {
  PlannerInput input;
  PlanningFrontier frontier;
  std::vector<LongitudinalState> retained_longitudinal_states;
  std::vector<LateralPathState> retained_lateral_states;
  std::vector<double> kinematic_seed_x;
  std::vector<double> kinematic_seed_y;
  std::size_t original_previous_path_points = 0;
  std::size_t retained_prefix_points = 0;
  std::uint64_t cycle = 0;
  // Absolute execution time on the committed-output timeline. When control
  // history is aligned, PathPlanner advances this by the number of path points
  // actually consumed since the previous committed output.
  double telemetry_time_s = 0.0;
  int target_lane = -1;
  bool historical_plan_aligned = false;
  bool control_lane_cruise_backup_valid = false;
  bool control_lane_cruise_handoff_valid = false;
  PlanningStateResetReason reset_reason =
      PlanningStateResetReason::kNoCommittedHistory;
};

#endif // PLANNING_SNAPSHOT_H
