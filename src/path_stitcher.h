#ifndef PATH_STITCHER_H
#define PATH_STITCHER_H

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "longitudinal_types.h"
#include "planner_types.h"

// Exact lateral state associated with one generated output point.  The queue
// is shifted by the number of consumed previous_path points on every callback.
struct LateralPathState {
  bool valid = false;
  std::uint64_t transition_id = 0;
  double correction_progress_m = 0.0;
  double road_parameter_s = 0.0;
  double planned_d = 0.0;
  double expected_x = 0.0;
  double expected_y = 0.0;
};

struct LateralStitchDiagnostics {
  std::uint64_t transition_id = 0;
  bool transition_active = false;
  bool state_reset = false;
  bool state_aligned = false;
  bool rolling_replanned = false;
  std::uint64_t rolling_replan_count = 0;
  double transition_length_m = 0.0;
  double rolling_origin_m = 0.0;
  double progress_m = 0.0;
  double remaining_m = 0.0;
  double position_residual_m = 0.0;
  double frontier_d = 0.0;
};

struct StitchedRoadPathResult {
  std::vector<std::pair<double, double>> new_points;
  std::vector<LateralPathState> output_states;
  LateralStitchDiagnostics diagnostics;
};

// Stored coefficients for one finite lateral transition.  It is public so the
// stateless curve-evaluation helpers can operate on it; ownership remains in
// PathStitcher.
struct LateralCorrectionPlan {
  bool initialized = false;
  bool has_correction = false;
  std::uint64_t id = 0;
  double start_d = 0.0;
  double target_d = 0.0;
  double transition_length_m = 0.0;
  double polynomial_origin_progress_m = 0.0;
  std::array<double, 6> d_coefficients = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::uint64_t rolling_replan_count = 0;
  double position_error_x = 0.0;
  double position_error_y = 0.0;
  double tangent_error_x = 0.0;
  double tangent_error_y = 0.0;
  double curvature_error_x = 0.0;
  double curvature_error_y = 0.0;
};

class PathStitcher {
public:
  StitchedRoadPathResult Sample(
      const PlannerInput &input, double plan_start_s, double lane_d,
      const std::vector<LongitudinalState> &states, const MapData &map);

private:
  LateralCorrectionPlan plan_;
  std::vector<LateralPathState> last_output_states_;
  std::uint64_t next_transition_id_ = 1;
};

#endif // PATH_STITCHER_H
