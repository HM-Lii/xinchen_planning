#ifndef SPATIAL_PATH_H
#define SPATIAL_PATH_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "behavior_planner.h"

struct MapData;
struct PlanningSnapshot;

// P2.3 builds lane-change geometry only.  The path is immutable and has no
// ownership of the control PathStitcher state.
struct SpatialPathPlannerConfig {
  double planning_horizon_s = 8.0;
  double post_maneuver_observation_s = 2.0;
  double nominal_transition_duration_s = 4.5;
  // Active P2.5 may size the transition from the current frontier speed while
  // still checking and constraining it against the independent hard speed
  // upper bound. Standalone P2.3 regression keeps the legacy false default.
  bool use_frontier_speed_for_transition_length = false;
  double minimum_transition_length_m = 40.0;
  double maximum_transition_length_m = 120.0;
  double maximum_path_extent_m = 250.0;
  double geometry_sample_step_m = 0.5;
  std::size_t maximum_geometry_samples = 640;
  // P2.6 keeps one committed geometry identity. A non-zero value appends a
  // target-lane cruise segment after transition-only early checks. Production
  // includes both a complete fresh QP horizon and a bounded settling reserve.
  double committed_continuation_horizon_s = 0.0;

  double speed_upper_bound_mps = 22.12848;
  double maximum_abs_longitudinal_acceleration_mps2 = 5.0;
  double maximum_abs_lateral_slope = 0.25;
  double maximum_abs_lateral_second_derivative_per_m = 0.03;
  double maximum_abs_curvature_per_m = 0.08;
  double maximum_lateral_acceleration_mps2 = 4.0;
  double maximum_lateral_jerk_mps3 = 10.0;

  double c2_position_tolerance_m = 1e-8;
  double c2_first_derivative_tolerance = 1e-8;
  double c2_second_derivative_tolerance_per_m = 1e-8;
  double c3_third_derivative_tolerance_per_m2 = 1e-8;
  double road_boundary_tolerance_m = 0.05;
  double minimum_boundary_sample_spacing_m = 0.05;

  double ego_length_m = 4.8;
  double ego_width_m = 2.0;
  double lane_boundary_margin_m = 0.35;
  double lane_width_m = 4.0;
  int lane_count = 3;
};

enum class SpatialPathPrecheckReason {
  kInvalidCandidate,
  kInitialBoundaryUnavailable,
  kC2Discontinuity,
  kC3Discontinuity,
  kTransitionLengthInsufficient,
  kLateralDerivative,
  kCurvature,
  kRoadBoundary,
  kLateralAcceleration,
  kLateralJerk,
  kLaneOccupancyInconsistent,
  kGeometryCoverageInsufficient
};

enum class SpatialPathCandidateStatus { kPrecheckRejected, kPrecheckPassed };

struct SpatialPathGeometrySample {
  // construction_progress_m is the monotonic RoadS delta used to evaluate
  // the septic. path_progress_m is the integrated Cartesian arc length and
  // is the only longitudinal coordinate exposed to downstream optimization.
  double construction_progress_m = 0.0;
  double path_progress_m = 0.0;
  double road_s_unwrapped_m = 0.0;
  double d_m = 0.0;
  double d_first_derivative = 0.0;
  double d_second_derivative_per_m = 0.0;
  double d_third_derivative_per_m2 = 0.0;
  double x_m = 0.0;
  double y_m = 0.0;
  double tangent_x = 0.0;
  double tangent_y = 0.0;
  double curvature_x_per_m = 0.0;
  double curvature_y_per_m = 0.0;
  double curvature_per_m = 0.0;
  double curvature_rate_per_m2 = 0.0;
  double road_margin_m = 0.0;
};

struct SpatialPathGeometryTable {
  std::size_t sample_count = 0;
  double path_extent_m = 0.0;
  double construction_extent_m = 0.0;
  double transition_completion_path_progress_m = 0.0;
  std::vector<SpatialPathGeometrySample> samples;
};

struct LaneOccupancyProfile {
  bool target_lane_coverage_started = false;
  bool source_lane_departed = false;
  bool lane_change_completed = false;
  double target_lane_coverage_start_path_progress_m = 0.0;
  double source_lane_departure_path_progress_m = 0.0;
  double lane_change_completion_path_progress_m = 0.0;
};

struct SpatialPathPrecheckResult {
  bool evaluated = false;
  bool passed = false;
  std::vector<SpatialPathPrecheckReason> rejection_reasons;
  double c2_position_residual_m = 0.0;
  double c2_first_derivative_residual = 0.0;
  double c2_second_derivative_residual_per_m = 0.0;
  double c3_third_derivative_residual_per_m2 = 0.0;
  double maximum_abs_lateral_slope = 0.0;
  double maximum_abs_lateral_second_derivative_per_m = 0.0;
  double maximum_abs_curvature_per_m = 0.0;
  double maximum_abs_curvature_rate_per_m2 = 0.0;
  double minimum_road_margin_m = 0.0;
  double speed_upper_bound_mps = 0.0;
  double maximum_estimated_lateral_acceleration_mps2 = 0.0;
  double maximum_estimated_lateral_jerk_mps3 = 0.0;
};

struct SpatialPathCandidate {
  std::uint64_t candidate_id = 0;
  int source_lane = -1;
  int target_lane = -1;
  double start_road_s_unwrapped_m = 0.0;
  double start_d_m = 0.0;
  double target_d_m = 0.0;
  double start_d_first_derivative = 0.0;
  double start_d_second_derivative_per_m = 0.0;
  double start_d_third_derivative_per_m2 = 0.0;
  double transition_length_m = 0.0;
  double requested_path_extent_m = 0.0;
  std::array<double, 8> d_coefficients = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  SpatialPathGeometryTable geometry;
  LaneOccupancyProfile occupancy;
  SpatialPathPrecheckResult precheck;
  SpatialPathCandidateStatus status =
      SpatialPathCandidateStatus::kPrecheckRejected;
};

struct SpatialPathBatchSnapshot {
  std::uint64_t cycle = 0;
  std::size_t evaluated_candidate_count = 0;
  std::size_t generated_candidate_count = 0;
  std::size_t precheck_passed_candidate_count = 0;
  std::vector<SpatialPathCandidate> candidates;
};

const char *SpatialPathPrecheckReasonName(SpatialPathPrecheckReason reason);
const char *SpatialPathCandidateStatusName(SpatialPathCandidateStatus status);
bool HasSpatialPathPrecheckReason(const SpatialPathPrecheckResult &result,
                                  SpatialPathPrecheckReason reason);
const SpatialPathCandidate *
FindSpatialPathCandidate(const SpatialPathBatchSnapshot &snapshot,
                         std::uint64_t candidate_id);
SpatialPathGeometrySample
SampleSpatialPathAtProgress(const SpatialPathGeometryTable &geometry,
                            double path_progress_m);

class SpatialPathPlanner {
public:
  explicit SpatialPathPlanner(
      const SpatialPathPlannerConfig &config = SpatialPathPlannerConfig());

  SpatialPathBatchSnapshot Generate(const PlanningSnapshot &planning,
                                    const BehaviorPlanningSnapshot &behavior,
                                    const MapData &map) const;

private:
  SpatialPathPlannerConfig config_;
};

#endif // SPATIAL_PATH_H
