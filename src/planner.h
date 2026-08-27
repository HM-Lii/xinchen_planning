#ifndef PLANNER_H
#define PLANNER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "active_behavior_planner.h"
#include "longitudinal_qp.h"
#include "path_stitcher.h"
#include "planner_monitor.h"
#include "planning_snapshot.h"
#include "planner_types.h"
#include "speed_reference.h"
#include "trajectory_validator.h"

struct RetainedPathConfig {
  // Production configuration may reduce this window but cannot exceed 15.
  std::size_t maximum_points = 15;
  std::size_t minimum_stitch_points = 2;
  // Phase 1 promotes bounded retention to a production invariant. `false` is
  // rejected rather than silently restoring the legacy unbounded behavior.
  bool enabled = true;
};

struct PlannerConfig {
  PlannerOperatingMode operating_mode =
      PlannerOperatingMode::kLaneCruiseOnly;
  std::size_t output_points = 50;
  double time_step_seconds = 0.02;
  double target_speed_mph = 49.5;
  std::size_t qp_horizon_steps = 80;
  double qp_time_step_seconds = 0.1;
  double min_acceleration_mps2 = -5.0;
  double max_acceleration_mps2 = 3.0;
  double max_jerk_mps3 = 8.0;
  // Cartesian jerk includes road-curvature and lateral-correction terms in
  // addition to the longitudinal jerk bound above.
  double validator_maximum_cartesian_jerk_mps3 = 10.0;
  double initial_jerk_continuity_weight = 0.4;
  double reference_min_acceleration_mps2 = -2.5;
  double reference_max_jerk_mps3 = 2.0;
  double time_headway_seconds = 1.5;
  double gap_closing_time_seconds = 6.0;
  double standstill_gap_meters = 2.0;
  double ego_length_meters = 4.8;
  double obstacle_length_meters = 4.8;
  double ego_width_meters = 2.0;
  double obstacle_width_meters = 2.0;
  double prediction_margin_meters = 1.0;
  double physical_collision_margin_meters = 0.0;
  double headway_slack_weight = 200.0;
  double collision_violation_weight = 20000.0;
  double traffic_lookahead_meters = 250.0;
  double lane_boundary_margin_meters = 0.35;
  double lane_width_meters = 4.0;
  int lane_count = 3;
  RetainedPathConfig retained_path;
  ActiveBehaviorPlannerConfig active_behavior;
  PlannerMonitorConfig monitor;
};

struct PlannerState {
  int target_lane = -1;
  std::vector<LongitudinalState> output_longitudinal;
  std::vector<LateralPathState> output_lateral;
  std::vector<double> output_x;
  std::vector<double> output_y;
  PathStitcherState path_stitcher;
  QpWarmStartState qp_warm_start;
  double reference_speed_mps = 0.0;
  double telemetry_time_s = 0.0;
  std::uint64_t committed_generation = 0;
};

struct CandidateCost {
  double progress_loss_m = 0.0;
};

struct CandidatePlan {
  std::uint64_t candidate_id = 0;
  FallbackLevel fallback_level = FallbackLevel::kNormal;
  FullTrajectory full_trajectory;
  PlannerOutput output;
  PlannerState next_state;
  LongitudinalQpResult qp;
  CandidateCost cost;
  ValidationResult validation;
  std::vector<PredictedObstacle> obstacles;
  std::vector<double> reference_speed_mps;
  LateralStitchDiagnostics lateral_diagnostics;
  double evaluate_time_ms = 0.0;
  double validate_time_ms = 0.0;
  bool behavior_lane_change = false;
  bool first_behavior_commit = false;
  bool behavior_continuation = false;
  int behavior_source_lane = -1;
  int behavior_target_lane = -1;
  GapId behavior_gap;
};

enum class CandidateFailureReason {
  kNone,
  kQpInfeasible,
  kHardValidation,
  kInfrastructure
};

struct CandidateEvaluation {
  std::uint64_t candidate_id = 0;
  LongitudinalSafetyPolicy safety_policy =
      LongitudinalSafetyPolicy::kNormalOperational;
  bool valid = false;
  bool has_plan = false;
  CandidatePlan plan;
  ValidationResult validation;
  CandidateFailureReason failure = CandidateFailureReason::kNone;
  std::string failure_detail;
  double evaluate_time_ms = 0.0;
  double validate_time_ms = 0.0;
};

struct EmergencyPlannerState {
  int target_lane = -1;
  std::vector<LongitudinalState> output_longitudinal;
  std::vector<LateralPathState> output_lateral;
  std::vector<double> output_x;
  std::vector<double> output_y;
  PathStitcherState path_stitcher;
  QpWarmStartState emergency_qp_warm_start;
  double reference_speed_mps = 0.0;
  double telemetry_time_s = 0.0;
};

struct RiskEvaluation {
  std::size_t predicted_collision_object_count = 0;
  double predicted_first_collision_time_s = 0.0;
  double predicted_relative_collision_speed_mps = 0.0;
  double maximum_overlap_m = 0.0;
  std::string selected_reason;
};

struct MinimumRiskDispatch {
  std::uint64_t candidate_id = 0;
  FullTrajectory full_trajectory;
  PlannerOutput output;
  EmergencyPlannerState next_emergency_state;
  LongitudinalQpResult qp;
  ValidationResult validation;
  RiskEvaluation risk;
  std::vector<PredictedObstacle> obstacles;
  std::vector<double> reference_speed_mps;
  LateralStitchDiagnostics lateral_diagnostics;
  double exclusive_evaluate_time_ms = 0.0;
  double exclusive_validate_time_ms = 0.0;
  double evaluate_time_ms = 0.0;
  double validate_time_ms = 0.0;
};

struct InfrastructureFailureDetails {
  std::string reason;
};

struct PlanningCycleDecision {
  PlanDisposition disposition = PlanDisposition::kInfrastructureFailure;
  bool has_validated_candidate = false;
  CandidatePlan validated_candidate;
  bool has_minimum_risk = false;
  MinimumRiskDispatch minimum_risk;
  bool has_infrastructure_failure = false;
  InfrastructureFailureDetails infrastructure_failure;
  std::vector<CandidateEvaluation> ordinary_evaluations;
};

class PathPlanner {
public:
  explicit PathPlanner(const PlannerConfig &config = PlannerConfig());
  ~PathPlanner();

  PlannerOutput Plan(const PlannerInput &input, const MapData &map);
  PlanningCycleDecision PlanCycle(const PlannerInput &input,
                                  const MapData &map);
  double reference_speed_mps() const { return state_.reference_speed_mps; }
  bool last_plan_emergency() const {
    return last_decision_.disposition == PlanDisposition::kMinimumRiskDispatch;
  }
  const PlannerCycleDiagnostics &last_diagnostics() const {
    return last_diagnostics_;
  }
  const PlanningCycleDecision &last_decision() const { return last_decision_; }
  std::uint64_t committed_generation() const {
    return state_.committed_generation;
  }
  // operating_mode() is the requested mode. During a committed maneuver a
  // LaneCruiseOnly request disables new maneuvers immediately while the
  // effective active mode remains responsible until target-lane settling.
  PlannerOperatingMode operating_mode() const { return requested_mode_; }
  PlannerOperatingMode effective_operating_mode() const {
    return effective_mode_;
  }
  void SetOperatingMode(PlannerOperatingMode mode);
  ActiveBehaviorDiagnostics behavior_diagnostics() const;

private:
  struct LaneCruiseCycleContext {
    FixedSpatialPath fixed_path;
    std::vector<PredictedObstacle> obstacles;
    double evaluate_time_ms = 0.0;
    double validate_time_ms = 0.0;
  };

  struct ActiveBehaviorTransaction {
    bool open = false;
    bool candidate_dispatched = false;
  };

  PlanningSnapshot BuildSnapshot(const PlannerInput &input,
                                 const MapData &map) const;
  LaneCruiseCycleContext
  EvaluateLaneCruiseCandidates(PlanningSnapshot *snapshot, const MapData &map,
                               PlanningCycleDecision *decision) const;
  void EvaluateActiveBehaviorCandidate(const PlanningSnapshot &snapshot,
                                       const MapData &map,
                                       PlanningCycleDecision *decision,
                                       ActiveBehaviorTransaction *transaction);
  void CommitSelectedDecision(PlanningCycleDecision *decision,
                              ActiveBehaviorTransaction *transaction);
  void ResolveLaneCruiseFailure(const PlanningSnapshot &snapshot,
                                const MapData &map,
                                const LaneCruiseCycleContext &lane_cruise,
                                PlanningCycleDecision *decision);
  CandidateEvaluation EvaluateLaneCruise(
      const PlanningSnapshot &snapshot, const MapData &map,
      const FixedSpatialPath &fixed_path,
      const std::vector<PredictedObstacle> &obstacles,
      const std::vector<double> &normal_reference_speed_mps,
      LongitudinalSafetyPolicy policy, std::uint64_t candidate_id) const;
  MinimumRiskDispatch EvaluateMinimumRisk(
      const PlanningSnapshot &snapshot, const MapData &map,
      const FixedSpatialPath &fixed_path,
      const std::vector<PredictedObstacle> &obstacles,
      std::uint64_t candidate_id) const;
  CandidatePlan BuildActiveCandidatePlan(
      const PlanningSnapshot &snapshot,
      const FinalBehaviorCandidate &candidate,
      bool first_commit, bool continuation) const;
  MinimumRiskDispatch BuildActiveMinimumRiskDispatch(
      const PlanningSnapshot &snapshot,
      const FinalBehaviorCandidate &candidate) const;
  void CommitCandidate(const CandidatePlan &winner);
  void DispatchEmergency(const MinimumRiskDispatch &dispatch);
  void RecordDecisionDiagnostics(const PlanningSnapshot &snapshot,
                                 const PlanningCycleDecision &decision);

  PlannerConfig config_;
  LongitudinalQp longitudinal_qp_;
  SpeedReferenceGenerator speed_reference_generator_;
  PathStitcher path_stitcher_;
  TrajectoryValidator trajectory_validator_;
  PlannerRuntimeMonitor runtime_monitor_;
  PlannerState state_;
  PlannerOperatingMode requested_mode_ =
      PlannerOperatingMode::kLaneCruiseOnly;
  PlannerOperatingMode effective_mode_ =
      PlannerOperatingMode::kLaneCruiseOnly;
  std::unique_ptr<ActiveBehaviorPlanner> active_behavior_planner_;
  bool active_behavior_evaluation_attempted_ = false;
  bool active_behavior_evaluation_succeeded_ = false;
  std::string active_behavior_error_stage_;
  std::string active_behavior_error_detail_;
  std::uint64_t plan_cycle_ = 0;
  PlannerCycleDiagnostics last_diagnostics_;
  PlanningCycleDecision last_decision_;
};

#endif // PLANNER_H
