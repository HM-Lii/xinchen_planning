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
  double residual_correction_progress_m = 0.0;
  double road_parameter_s = 0.0;
  double planned_d = 0.0;
  double expected_x = 0.0;
  double expected_y = 0.0;
  // Geometry derivatives are carried with the generated point so ownership
  // can move between ActiveBehavior and LaneCruise without reconstructing a
  // boundary from simulator-quantized Cartesian samples.
  bool exact_tangent_valid = false;
  double tangent_x = 0.0;
  double tangent_y = 0.0;
  bool exact_curvature_valid = false;
  double curvature_x_per_m = 0.0;
  double curvature_y_per_m = 0.0;
};

struct SpatialBoundaryCorrection {
  double position_x_m = 0.0;
  double position_y_m = 0.0;
  double tangent_x = 0.0;
  double tangent_y = 0.0;
  double curvature_x_per_m = 0.0;
  double curvature_y_per_m = 0.0;
  double transition_length_m = 0.0;
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

// Stored coefficients for one finite lateral transition.  It is public so the
// stateless curve-evaluation helpers can operate on it; ownership remains in
// the committed PathStitcherState.
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

struct PathStitcherState {
  LateralCorrectionPlan plan;
  SpatialBoundaryCorrection residual_correction;
  std::vector<LateralPathState> last_output_states;
  std::uint64_t next_transition_id = 1;
};

// Immutable geometry prepared before traffic is projected and before the
// longitudinal QP is solved. Candidate sampling only reads this object.
struct FixedSpatialPath {
  LateralCorrectionPlan plan;
  std::vector<LateralPathState> retained_states;
  std::uint64_t next_transition_id = 1;
  double start_progress_m = 0.0;
  double start_road_parameter_s = 0.0;
  SpatialBoundaryCorrection residual_correction;
  double residual_start_progress_m = 0.0;
  LateralStitchDiagnostics diagnostics;
};

struct StitchedRoadPathResult {
  std::vector<std::pair<double, double>> new_points;
  std::vector<LateralPathState> output_states;
  LateralStitchDiagnostics diagnostics;
  PathStitcherState next_state;
};

class PathStitcher {
public:
  FixedSpatialPath Prepare(
      const PlannerInput &input, double plan_start_s, double current_d,
      double lane_d, double maximum_planning_speed_mps, const MapData &map,
      const PathStitcherState &current_state,
      const std::vector<LateralPathState> &retained_states,
      bool historical_state_aligned) const;

  StitchedRoadPathResult
  Sample(const FixedSpatialPath &path,
         const std::vector<LongitudinalState> &states,
         const MapData &map) const;
};

double FixedPathProgressToRoadParameterDistance(
    const FixedSpatialPath &path, double target_road_s_unwrapped_m,
    const MapData &map);

#endif // PATH_STITCHER_H
