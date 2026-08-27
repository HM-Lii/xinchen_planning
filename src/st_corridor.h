#ifndef ST_CORRIDOR_H
#define ST_CORRIDOR_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "behavior_planner.h"
#include "longitudinal_types.h"
#include "spatial_path.h"
#include "traffic_prediction.h"

struct PlanningSnapshot;

// The horizon and time step must match ActiveBehaviorPlanner's private
// candidate LongitudinalQp.
struct STCorridorPlannerConfig {
  std::size_t qp_horizon_steps = 80;
  double qp_time_step_s = 0.10;
  double post_maneuver_observation_s = 2.0;
  double maximum_speed_mps = 22.12848;
  double minimum_longitudinal_acceleration_mps2 = -5.0;
  double maximum_longitudinal_acceleration_mps2 = 3.0;
  double lateral_acceleration_budget_mps2 = 3.5;
  double curvature_epsilon_per_m = 1e-6;
  double standstill_clearance_m = 2.0;
  double physical_collision_margin_m = 0.0;
  double projection_tightening_m = 0.05;
  double ego_length_m = 4.8;
  double ego_width_m = 2.0;
  int lane_count = 3;
  std::size_t maximum_candidates = 8;
  std::size_t maximum_corridor_nodes = 128;
  double qp_bound_tolerance_m = 2e-3;
  double qp_speed_tolerance_mps = 2e-3;
};

enum class STConstraintSide {
  kNone,
  kLower,
  kUpper
};

struct STConstraintSource {
  bool present = false;
  int vehicle_id = 0;
  // One bit per TrafficPredictionHypothesis enum value. Multiple hypotheses
  // are intentionally collapsed into one conservative vehicle envelope.
  std::uint64_t hypothesis_mask = 0;
};

struct STCorridorNode {
  double time_from_frontier_s = 0.0;
  double prediction_time_s = 0.0;
  double reachable_minimum_progress_m = 0.0;
  double reachable_maximum_progress_m = 0.0;
  std::uint64_t ego_lane_mask = 0;
  double lower_path_progress_m = 0.0;
  double upper_path_progress_m = 0.0;
  STConstraintSource lower_source;
  STConstraintSource upper_source;
  std::size_t relevant_vehicle_count = 0;
  double available_width_m = 0.0;
};

struct STCorridor {
  double time_step_s = 0.0;
  PassingOrder order = PassingOrder::kStayBehindFront;
  std::size_t node_count = 0;
  bool prediction_evidence_complete = false;
  bool empty = false;
  std::size_t first_evidence_failure_node = 0;
  std::size_t first_empty_node = 0;
  double minimum_width_m = 0.0;
  std::size_t minimum_width_node = 0;
  std::size_t dual_lane_node_count = 0;
  std::size_t lower_constrained_node_count = 0;
  std::size_t upper_constrained_node_count = 0;
  STConstraintSource minimum_width_lower_source;
  STConstraintSource minimum_width_upper_source;
  STConstraintSource first_empty_lower_source;
  STConstraintSource first_empty_upper_source;
  std::vector<STCorridorNode> nodes;
};

struct CurvatureSpeedBudget {
  std::size_t node_count = 0;
  bool initial_speed_feasible = false;
  double minimum_speed_limit_mps = 0.0;
  std::size_t limiting_node = 0;
  double limiting_abs_curvature_per_m = 0.0;
  std::vector<double> maximum_speed_mps;
};

enum class STCandidateStatus {
  kInvalidInput,
  kPredictionEvidenceIncomplete,
  kCorridorEmpty,
  kCurvatureSpeedInfeasible,
  kCorridorReady,
  kQpInfeasible,
  kQpTimeout,
  kQpResultInvalid,
  kHorizonInsufficient,
  kQpSolved,
  kDeadlineSkipped
};

struct STCandidateEvaluation {
  std::uint64_t candidate_id = 0;
  int source_lane = -1;
  int target_lane = -1;
  PassingOrder order = PassingOrder::kStayBehindFront;
  STCandidateStatus status = STCandidateStatus::kInvalidInput;
  STCorridor corridor;
  CurvatureSpeedBudget speed_budget;
  bool qp_attempted = false;
  bool qp_success = false;
  bool qp_bounds_satisfied = false;
  std::string qp_status;
  double qp_objective = 0.0;
  double terminal_progress_m = 0.0;
  double terminal_speed_mps = 0.0;
  double source_lane_departure_progress_m = 0.0;
  bool source_lane_departure_observed = false;
  bool source_lane_departed_in_time = false;
  double source_lane_departure_time_s = 0.0;
  double latest_allowed_source_lane_departure_time_s = 0.0;
  double lane_change_completion_progress_m = 0.0;
  bool lane_change_completion_observed = false;
  bool lane_change_completed_in_time = false;
  double lane_change_completion_time_s = 0.0;
  double latest_allowed_completion_time_s = 0.0;
  double minimum_lower_margin_m = 0.0;
  double minimum_upper_margin_m = 0.0;
  double maximum_speed_excess_mps = 0.0;
  // The complete worker result is retained for P2.4 evidence. Control-side
  // compact snapshots deliberately omit its trajectory and warm start.
  LongitudinalQpResult qp_result;
};

// A committed maneuver carries the timing evidence from its last dispatched
// QP into the next control cycle.  Deadlines are relative to the current
// planning frontier.  Fresh candidates use the default P2.4 deadline (the
// horizon minus the post-maneuver observation window).
struct STManeuverSchedule {
  // Fresh candidates require fully aged safety-admissible traffic evidence
  // and their declared Gap boundaries to remain in the target lane.  A
  // committed continuation may use a currently valid replacement track and
  // conservatively reclassify a boundary that has physically left that lane.
  bool committed_continuation = false;
  bool source_lane_released = false;
  bool has_source_lane_departure_deadline = false;
  double source_lane_departure_deadline_s = 0.0;
  bool has_completion_deadline = false;
  double completion_deadline_s = 0.0;
};

struct STCandidateBatchSnapshot {
  std::uint64_t cycle = 0;
  std::size_t evaluated_candidate_count = 0;
  std::size_t invalid_candidate_count = 0;
  std::size_t corridor_ready_candidate_count = 0;
  std::size_t corridor_empty_candidate_count = 0;
  std::size_t prediction_rejected_candidate_count = 0;
  std::size_t curvature_rejected_candidate_count = 0;
  std::size_t qp_attempted_candidate_count = 0;
  std::size_t qp_solved_candidate_count = 0;
  std::size_t qp_failed_candidate_count = 0;
  std::size_t horizon_rejected_candidate_count = 0;
  std::size_t deadline_skipped_candidate_count = 0;
  std::vector<STCandidateEvaluation> candidates;
};

const char *STConstraintSideName(STConstraintSide side);
const char *STCandidateStatusName(STCandidateStatus status);
const STCandidateEvaluation *FindSTCandidate(
    const STCandidateBatchSnapshot &snapshot, std::uint64_t candidate_id);

// Builds a candidate-specific convex PathProgress/time corridor. Every
// vehicle receives one fixed side before any QP is formed; a QP can therefore
// never choose its own passing topology.
class STCorridorPlanner {
public:
  explicit STCorridorPlanner(
      const STCorridorPlannerConfig &config = STCorridorPlannerConfig());

  STCandidateBatchSnapshot Build(
      const PlanningSnapshot &planning,
      const BehaviorPlanningSnapshot &behavior,
      const SpatialPathBatchSnapshot &spatial_paths,
      const FullLaneTrafficPredictionSnapshot &prediction,
      const STManeuverSchedule &schedule = STManeuverSchedule()) const;

  LongitudinalQpInput MakeQpInput(
      const PlanningSnapshot &planning,
      const STCandidateEvaluation &candidate,
      double target_speed_mps) const;

  void AttachQpResult(const LongitudinalQpResult &result,
                      STCandidateEvaluation *candidate) const;
  void MarkDeadlineSkipped(STCandidateEvaluation *candidate) const;
  void RefreshCounts(STCandidateBatchSnapshot *snapshot) const;

private:
  STCorridorPlannerConfig config_;
};

#endif // ST_CORRIDOR_H
