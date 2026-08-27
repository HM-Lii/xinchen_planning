#ifndef PLANNER_MONITOR_H
#define PLANNER_MONITOR_H

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "longitudinal_types.h"
#include "path_stitcher.h"
#include "planner_types.h"
#include "trajectory_validator.h"

struct PlannerMonitorConfig {
  bool enabled = false;
  bool write_csv = true;
  std::string log_directory = "logs";
  std::size_t console_summary_interval_cycles = 50;
  std::size_t violation_report_interval_cycles = 10;
  std::size_t detail_csv_interval_cycles = 50;
  double maximum_cartesian_acceleration_mps2 = 10.0;
  double maximum_cartesian_jerk_mps3 = 10.0;
  double speed_tolerance_mps = 0.05;
  double acceleration_tolerance_mps2 = 0.05;
  double jerk_tolerance_mps3 = 0.05;
  double safety_tolerance_meters = 0.05;
};

struct IndexedMetric {
  bool valid = false;
  double value = 0.0;
  std::size_t index = 0;
};

struct CartesianKinematicSample {
  std::size_t output_index = 0;
  bool is_previous_path = false;
  double x = 0.0;
  double y = 0.0;
  double velocity_x_mps = 0.0;
  double velocity_y_mps = 0.0;
  double speed_mps = 0.0;
  double acceleration_x_mps2 = 0.0;
  double acceleration_y_mps2 = 0.0;
  double acceleration_mps2 = 0.0;
  double tangential_acceleration_mps2 = 0.0;
  bool jerk_valid = false;
  double jerk_x_mps3 = 0.0;
  double jerk_y_mps3 = 0.0;
  double jerk_mps3 = 0.0;
  double tangential_jerk_mps3 = 0.0;
};

struct QpNodeMonitorSample {
  std::size_t node_index = 0;
  double time_seconds = 0.0;
  LongitudinalState state;
  double reference_speed_mps = 0.0;
  bool intrusion_speed_limit_valid = false;
  double minimum_intrusion_speed_limit_mps = 0.0;
  double intrusion_limiting_obstacle_id = 0.0;
  bool headway_margin_valid = false;
  double minimum_headway_margin_meters = 0.0;
  double headway_limiting_obstacle_id = 0.0;
  bool collision_margin_valid = false;
  double minimum_collision_margin_meters = 0.0;
  double collision_limiting_obstacle_id = 0.0;
};

struct PlannerMonitorLimits {
  double output_time_step_seconds = 0.02;
  double maximum_speed_mps = 0.0;
  double minimum_acceleration_mps2 = 0.0;
  double maximum_acceleration_mps2 = 0.0;
  double maximum_jerk_mps3 = 0.0;
  double maximum_cartesian_acceleration_mps2 = 10.0;
  double maximum_cartesian_jerk_mps3 = 10.0;
  double speed_tolerance_mps = 0.05;
  double acceleration_tolerance_mps2 = 0.05;
  double jerk_tolerance_mps3 = 0.05;
  double safety_tolerance_meters = 0.05;
  double maximum_lateral_deviation_meters = 1.65;
  double time_headway_seconds = 0.0;
  double fixed_headway_gap_meters = 0.0;
  double collision_gap_meters = 0.0;
};

struct CollisionEventDiagnostics {
  CollisionEvidence evidence;
  bool qp_relevant = false;
  double qp_initial_relative_s_m = 0.0;
  double qp_predicted_speed_mps = 0.0;
  double qp_obstacle_d_m = 0.0;
};

struct ControlCandidateDiagnostics {
  std::uint64_t candidate_id = 0;
  LongitudinalSafetyPolicy safety_policy =
      LongitudinalSafetyPolicy::kNormalOperational;
  FallbackLevel fallback_level = FallbackLevel::kNormal;
  bool minimum_risk_candidate = false;
  bool selected_for_dispatch = false;
  bool has_plan = false;
  bool validation_valid = false;
  std::string failure_reason;
  std::string failure_detail;
  bool qp_success = false;
  bool qp_hard_safe = false;
  std::string qp_status;
  double qp_minimum_physical_margin_m = 0.0;
  double qp_minimum_operational_margin_m = 0.0;
  double qp_maximum_headway_slack_m = 0.0;
  double qp_maximum_collision_violation_m = 0.0;
  double validation_minimum_collision_margin_m = 0.0;
  double validation_minimum_road_margin_m = 0.0;
  std::vector<double> shielded_same_lane_rear_vehicle_ids;
  double evaluate_time_ms = 0.0;
  double validate_time_ms = 0.0;
  std::vector<CollisionEventDiagnostics> collision_events;
};

// Flattened P2.6 evidence for one behavior candidate.  The monitor owns only
// strings and scalar values so its public data format stays decoupled from the
// behavior, spatial-path and ST planner implementation types.
struct BehaviorCandidateDiagnostics {
  std::uint64_t candidate_id = 0;
  std::string behavior;
  int source_lane = -1;
  int target_lane = -1;
  std::string passing_order;
  std::string behavior_status;
  bool gap_has_front_vehicle = false;
  int gap_front_vehicle_id = 0;
  bool gap_has_rear_vehicle = false;
  int gap_rear_vehicle_id = 0;
  std::size_t stable_observations = 0;
  double stable_duration_s = 0.0;
  double estimated_progress_m = 0.0;
  double estimated_progress_gain_m = 0.0;
  double estimated_speed_gain_mps = 0.0;
  double uncertainty_cost = 0.0;
  bool admission_evaluated = false;
  bool admission_passed = false;
  std::string admission_rejection_reasons;
  bool has_front_margin = false;
  double minimum_front_margin_m = 0.0;
  bool has_rear_margin = false;
  double minimum_rear_margin_m = 0.0;
  bool has_rear_ttc = false;
  double minimum_rear_ttc_s = 0.0;
  double reachable_center_overlap_m = 0.0;
  bool has_source_front_margin = false;
  double minimum_source_front_margin_m = 0.0;
  int source_front_limiting_vehicle_id = 0;
  double source_front_limiting_time_s = 0.0;
  double source_front_limiting_ego_road_s_m = 0.0;
  double source_front_limiting_occupied_min_road_s_m = 0.0;
  double source_front_limiting_occupied_max_road_s_m = 0.0;
  double source_front_limiting_min_rate_mps = 0.0;
  double source_front_limiting_max_rate_mps = 0.0;
  double source_front_limiting_uncertainty_m = 0.0;
  std::string source_front_limiting_hypothesis;
  bool source_front_risk_observed = false;
  double first_source_front_risk_time_s = 0.0;
  int first_source_front_risk_vehicle_id = 0;
  double first_source_front_risk_margin_m = 0.0;
  std::string first_source_front_risk_hypothesis;
  bool target_kinematic_failure_observed = false;
  double first_target_kinematic_failure_time_s = 0.0;
  std::size_t first_target_propagated_state_count = 0;
  std::size_t first_target_feasible_state_count = 0;
  double first_target_best_front_margin_m = 0.0;
  double first_target_best_rear_margin_m = 0.0;
  double first_target_best_rear_ttc_margin_m = 0.0;
  bool gap_current_observation_valid = false;
  bool gap_topology_consistent = false;
  bool topology_failure_observed = false;
  std::string first_topology_failure_kind;
  double first_topology_failure_time_s = 0.0;
  bool first_topology_expected_front_found = false;
  bool first_topology_expected_rear_found = false;
  bool first_topology_actual_front_present = false;
  int first_topology_actual_front_vehicle_id = 0;
  bool first_topology_actual_rear_present = false;
  int first_topology_actual_rear_vehicle_id = 0;
  bool expected_boundaries_reversed_observed = false;
  double first_expected_boundaries_reversed_time_s = 0.0;
  bool merge_corridor_intrusion_observed = false;
  double first_merge_corridor_intrusion_time_s = 0.0;
  int first_merge_corridor_intrusion_vehicle_id = 0;
  std::string first_merge_corridor_intrusion_hypothesis;
  bool merge_corridor_blocked_observed = false;
  double first_merge_corridor_blocked_time_s = 0.0;
  std::size_t first_merge_corridor_candidate_state_count = 0;
  std::size_t first_merge_corridor_feasible_state_count = 0;
  int first_merge_corridor_blocking_vehicle_id = 0;
  std::string first_merge_corridor_blocking_hypothesis;
  double first_merge_corridor_blocking_margin_m = 0.0;
  bool prediction_evidence_complete = false;
  double minimum_gap_window_m = 0.0;

  bool spatial_evaluated = false;
  std::string spatial_status;
  bool spatial_precheck_passed = false;
  std::string spatial_rejection_reasons;
  double spatial_transition_length_m = 0.0;
  double spatial_path_extent_m = 0.0;
  double spatial_completion_progress_m = 0.0;
  double spatial_minimum_road_margin_m = 0.0;
  double spatial_speed_upper_bound_mps = 0.0;
  double spatial_maximum_lateral_acceleration_mps2 = 0.0;
  double spatial_maximum_lateral_jerk_mps3 = 0.0;

  bool st_evaluated = false;
  std::string st_status;
  bool st_prediction_evidence_complete = false;
  bool st_corridor_empty = false;
  std::size_t st_first_evidence_failure_node = 0;
  std::size_t st_first_empty_node = 0;
  double st_minimum_width_m = 0.0;
  std::size_t st_minimum_width_node = 0;
  bool st_minimum_width_lower_source_present = false;
  int st_minimum_width_lower_source_vehicle_id = 0;
  bool st_minimum_width_upper_source_present = false;
  int st_minimum_width_upper_source_vehicle_id = 0;
  bool st_first_empty_lower_source_present = false;
  int st_first_empty_lower_source_vehicle_id = 0;
  bool st_first_empty_upper_source_present = false;
  int st_first_empty_upper_source_vehicle_id = 0;
  bool st_curvature_initial_speed_feasible = false;
  double st_minimum_speed_limit_mps = 0.0;
  bool st_qp_attempted = false;
  bool st_qp_success = false;
  bool st_qp_bounds_satisfied = false;
  std::string st_qp_status;
  double st_qp_objective = 0.0;
  double st_terminal_progress_m = 0.0;
  double st_terminal_speed_mps = 0.0;
  bool st_source_lane_departed_in_time = false;
  double st_source_lane_departure_time_s = 0.0;
  double st_source_lane_departure_deadline_s = 0.0;
  bool st_lane_change_completed_in_time = false;
  double st_lane_change_completion_time_s = 0.0;
  double st_lane_change_completion_deadline_s = 0.0;
  double st_minimum_lower_margin_m = 0.0;
  double st_minimum_upper_margin_m = 0.0;
  double st_maximum_speed_excess_mps = 0.0;

  bool final_evaluated = false;
  std::string final_status;
  bool best_valid_candidate = false;
  bool selected_for_dispatch = false;
  bool tightening_attempted = false;
  bool tightening_succeeded = false;
  bool final_validation_valid = false;
  std::string final_rejection_detail;
  double final_minimum_physical_margin_m = 0.0;
  double final_minimum_operational_margin_m = 0.0;
  double final_minimum_road_margin_m = 0.0;
  double final_maximum_acceleration_mps2 = 0.0;
  double final_maximum_jerk_mps3 = 0.0;
  double final_minimum_longitudinal_acceleration_mps2 = 0.0;
  double final_cost = 0.0;
};

struct PlannerCycleDiagnostics {
  std::uint64_t cycle = 0;
  PlannerOperatingMode operating_mode = PlannerOperatingMode::kLaneCruiseOnly;
  PlannerOperatingMode requested_operating_mode =
      PlannerOperatingMode::kLaneCruiseOnly;
  std::string behavior_phase = "KeepLane";
  bool behavior_lane_change_selected = false;
  bool behavior_first_commit = false;
  bool behavior_continuation = false;
  int behavior_source_lane = -1;
  int behavior_target_lane = -1;
  std::uint64_t behavior_commit_cycle = 0;
  std::size_t behavior_stable_proposal_cycles = 0;
  std::size_t behavior_target_stable_cycles = 0;
  std::uint64_t behavior_oscillation_count = 0;
  std::uint64_t behavior_completed_maneuver_count = 0;
  std::uint64_t behavior_cancelled_proposal_count = 0;
  bool behavior_evaluation_attempted = false;
  bool behavior_evaluation_succeeded = false;
  bool behavior_transaction_committed = false;
  std::string behavior_error_stage;
  std::string behavior_error_detail;
  std::uint64_t behavior_result_cycle = 0;
  std::string behavior_tracker_reset_reason = "None";
  std::size_t behavior_observed_track_count = 0;
  std::size_t behavior_valid_track_count = 0;
  std::size_t behavior_admissible_track_count = 0;
  std::size_t behavior_stale_track_count = 0;
  std::size_t behavior_reacquired_track_count = 0;
  std::size_t behavior_reused_id_count = 0;
  std::size_t behavior_prediction_trajectory_count = 0;
  std::size_t behavior_admissible_prediction_count = 0;
  std::size_t behavior_generated_candidate_count = 0;
  std::size_t behavior_stable_gap_count = 0;
  std::size_t behavior_coarse_admitted_count = 0;
  std::size_t behavior_spatial_evaluated_count = 0;
  std::size_t behavior_spatial_generated_count = 0;
  std::size_t behavior_spatial_passed_count = 0;
  std::size_t behavior_st_evaluated_count = 0;
  std::size_t behavior_st_corridor_ready_count = 0;
  std::size_t behavior_st_corridor_empty_count = 0;
  std::size_t behavior_st_prediction_rejected_count = 0;
  std::size_t behavior_st_curvature_rejected_count = 0;
  std::size_t behavior_st_qp_attempted_count = 0;
  std::size_t behavior_st_qp_solved_count = 0;
  std::size_t behavior_st_qp_failed_count = 0;
  std::size_t behavior_st_horizon_rejected_count = 0;
  std::size_t behavior_st_deadline_skipped_count = 0;
  std::size_t behavior_final_candidate_count = 0;
  std::size_t behavior_final_valid_count = 0;
  std::size_t behavior_final_validation_rejected_count = 0;
  bool behavior_has_best_valid_candidate = false;
  std::uint64_t behavior_best_candidate_id = 0;
  int behavior_best_target_lane = -1;
  std::uint64_t behavior_full_validation_rejection_count = 0;
  std::uint64_t behavior_tightening_attempt_count = 0;
  std::uint64_t behavior_tightening_success_count = 0;
  std::uint64_t candidate_id = 0;
  PlanDisposition plan_disposition = PlanDisposition::kInfrastructureFailure;
  FallbackLevel fallback_level = FallbackLevel::kInfrastructureFailure;
  std::size_t original_previous_path_size = 0;
  std::size_t retained_prefix_points = 0;
  std::size_t previous_path_size = 0;
  std::size_t new_point_count = 0;
  double planning_frontier_delay_s = 0.0;
  PlanningStateSource state_source = PlanningStateSource::kTelemetry;
  PlanningStateResetReason state_reset_reason =
      PlanningStateResetReason::kNoCommittedHistory;
  bool validation_valid = false;
  std::string first_violation_type;
  double first_violation_time_s = 0.0;
  double minimum_physical_margin_m = 0.0;
  double minimum_operational_margin_m = 0.0;
  double maximum_headway_slack_m = 0.0;
  bool collision_unavoidable = false;
  bool infrastructure_failure = false;
  std::string infrastructure_failure_reason;
  bool minimum_risk_dispatch = false;
  std::string minimum_risk_reason;
  std::size_t predicted_collision_object_count = 0;
  double predicted_first_collision_time_s = 0.0;
  double predicted_relative_collision_speed_mps = 0.0;
  std::size_t traffic_vehicle_count = 0;
  std::vector<double> shielded_same_lane_rear_vehicle_ids;
  double validation_minimum_collision_margin_m = 0.0;
  bool has_first_collision_evidence = false;
  CollisionEventDiagnostics first_collision;
  std::size_t full_trajectory_points = 0;
  double evaluate_time_ms = 0.0;
  double validate_time_ms = 0.0;
  double ego_speed_mps = 0.0;
  double initial_speed_mps = 0.0;
  double initial_acceleration_mps2 = 0.0;
  double initial_jerk_mps3 = 0.0;
  double plan_start_s = 0.0;
  double plan_start_d = 0.0;
  double lane_center_d = 0.0;
  bool historical_plan_aligned = false;
  LateralStitchDiagnostics lateral;
  std::size_t relevant_obstacle_count = 0;
  IndexedMetric nearest_obstacle_distance;

  double reference_first_mps = 0.0;
  double reference_minimum_mps = 0.0;
  double reference_last_mps = 0.0;
  std::string qp_status;
  double qp_objective = 0.0;
  double qp_maximum_headway_slack_meters = 0.0;
  bool emergency = false;

  IndexedMetric qp_minimum_speed;
  IndexedMetric qp_maximum_speed;
  IndexedMetric qp_minimum_acceleration;
  IndexedMetric qp_maximum_acceleration;
  IndexedMetric qp_maximum_absolute_jerk;
  IndexedMetric minimum_intrusion_speed_limit;
  double intrusion_limiting_obstacle_id = 0.0;
  IndexedMetric qp_minimum_headway_margin;
  double qp_headway_limiting_obstacle_id = 0.0;
  IndexedMetric qp_minimum_collision_margin;
  double qp_collision_limiting_obstacle_id = 0.0;

  IndexedMetric cartesian_maximum_speed;
  IndexedMetric cartesian_minimum_tangential_acceleration;
  IndexedMetric cartesian_maximum_tangential_acceleration;
  IndexedMetric cartesian_maximum_acceleration;
  IndexedMetric cartesian_maximum_absolute_tangential_jerk;
  IndexedMetric cartesian_maximum_jerk;
  IndexedMetric new_path_junction_speed;
  IndexedMetric historical_path_position_residual;
  bool cartesian_metrics_new_path_only = false;

  bool qp_speed_violation = false;
  bool qp_acceleration_violation = false;
  bool qp_jerk_violation = false;
  bool qp_collision_violation = false;
  bool cartesian_speed_violation = false;
  bool cartesian_acceleration_violation = false;
  bool cartesian_jerk_violation = false;
  bool lane_deviation_violation = false;

  std::vector<CartesianKinematicSample> cartesian_samples;
  std::vector<LongitudinalState> output_longitudinal_states;
  std::vector<LateralPathState> output_lateral_states;
  std::vector<QpNodeMonitorSample> qp_samples;
  std::vector<ControlCandidateDiagnostics> control_candidates;
  std::vector<BehaviorCandidateDiagnostics> behavior_candidates;

  bool HasViolation() const;
};

std::vector<CartesianKinematicSample> ComputeCartesianKinematics(
    const PlannerInput &input, const PlannerOutput &output,
    double time_step_seconds, std::size_t previous_path_size);

PlannerCycleDiagnostics BuildPlannerDiagnostics(
    std::uint64_t cycle, const PlannerInput &input, const PlannerOutput &output,
    std::size_t previous_path_size, const LongitudinalState &initial_state,
    double plan_start_s, double plan_start_d, double lane_center_d,
    const std::vector<PredictedObstacle> &obstacles,
    const std::vector<double> &reference_speed_mps,
    const LongitudinalQpResult &qp_result,
    const std::vector<LongitudinalState> &output_longitudinal_states,
    const PlannerMonitorLimits &limits,
    const LateralStitchDiagnostics &lateral = LateralStitchDiagnostics(),
    const std::vector<LateralPathState> &output_lateral_states =
        std::vector<LateralPathState>(),
    bool historical_plan_aligned = false);

class PlannerRuntimeMonitor {
public:
  explicit PlannerRuntimeMonitor(
      const PlannerMonitorConfig &config = PlannerMonitorConfig());

  void Record(const PlannerCycleDiagnostics &diagnostics);
private:
  void EnsureCsvStreams();
  void WriteCycleCsv(const PlannerCycleDiagnostics &diagnostics);
  void WriteDetailCsv(const PlannerCycleDiagnostics &diagnostics);
  void WriteControlCandidateCsv(
      const PlannerCycleDiagnostics &diagnostics);
  void WriteBehaviorCandidateCsv(
      const PlannerCycleDiagnostics &diagnostics);
  void WriteCollisionCsv(const PlannerCycleDiagnostics &diagnostics);
  void PrintSummary(const PlannerCycleDiagnostics &diagnostics) const;
  void PrintViolation(const PlannerCycleDiagnostics &diagnostics) const;

  PlannerMonitorConfig config_;
  std::uint64_t session_id_ = 0;
  std::uint64_t last_violation_report_cycle_ = 0;
  unsigned int previous_violation_mask_ = 0;
  bool csv_initialization_attempted_ = false;
  bool csv_available_ = false;
  std::ofstream cycle_csv_;
  std::ofstream point_csv_;
  std::ofstream qp_csv_;
  std::ofstream control_candidate_csv_;
  std::ofstream behavior_candidate_csv_;
  std::ofstream collision_csv_;
  std::uint64_t last_control_violation_cycle_ = 0;
};

#endif // PLANNER_MONITOR_H
