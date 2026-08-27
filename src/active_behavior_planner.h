#ifndef ACTIVE_BEHAVIOR_PLANNER_H
#define ACTIVE_BEHAVIOR_PLANNER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "behavior_planner.h"
#include "longitudinal_types.h"
#include "planner_types.h"
#include "spatial_path.h"
#include "st_corridor.h"
#include "traffic_prediction.h"
#include "traffic_tracker.h"
#include "trajectory_validator.h"

struct PlanningSnapshot;

// P2.5/P2.6 now run synchronously on the control cycle.  The configuration is
// immutable after PathPlanner construction; PathPlanner overwrites dimensions,
// timing and vehicle geometry with its authoritative control configuration.
struct ActiveBehaviorPlannerConfig {
  LongitudinalQpConfig longitudinal_qp;
  TrafficTrackerConfig traffic_tracker;
  FullLaneTrafficPredictionConfig traffic_prediction;
  BehaviorPlannerConfig behavior_planner;
  SpatialPathPlannerConfig spatial_path;
  STCorridorPlannerConfig st_corridor;
  TrajectoryValidatorConfig trajectory_validator;

  double output_time_step_s = 0.02;
  std::size_t output_points = 50;
  std::size_t full_trajectory_sample_count = 400;
  double target_speed_mps = 22.12848;

  // A Gap must already satisfy the P2.2 observation window.  This additional
  // count suppresses a valid-candidate flip at the final validation boundary.
  std::size_t minimum_valid_proposal_cycles = 2;
  // A lane change is discretionary while the independently validated
  // lane-cruise comparator remains available.  Do not commit a fresh
  // maneuver whose exact QP requires braking beyond this comfort floor.
  // Committed continuations are governed only by their hard safety gates.
  double minimum_discretionary_acceleration_mps2 = -2.5;
  std::size_t target_stable_cycles = 3;
  double target_center_tolerance_m = 0.30;
  double target_lateral_rate_tolerance_mps = 0.35;

  // P2.5 permits one deterministic speed tightening after a Cartesian
  // dynamics rejection. Collision, road-boundary and evidence failures are
  // never relaxed or retried.
  double dynamics_tightening_factor = 0.85;
};

enum class FinalBehaviorCandidateStatus {
  kGenerated,
  kAdmissionRejected,
  kLateralRejected,
  kCorridorEmpty,
  kQpInfeasible,
  kHorizonInsufficient,
  kComfortRejected,
  kValidationRejected,
  kValid
};

enum class BehaviorManeuverPhase {
  kKeepLane,
  kPrepareCandidate,
  kCommitted,
  kSettling
};

struct FinalBehaviorCandidate {
  std::uint64_t candidate_id = 0;
  BehaviorType behavior = BehaviorType::kKeepLane;
  int source_lane = -1;
  int target_lane = -1;
  GapId gap;
  PassingOrder order = PassingOrder::kStayBehindFront;
  FinalBehaviorCandidateStatus status =
      FinalBehaviorCandidateStatus::kGenerated;

  bool qp_attempted = false;
  bool tightening_attempted = false;
  bool tightening_succeeded = false;
  LongitudinalQpResult qp;
  ValidationResult validation;
  FullTrajectory full_trajectory;
  PlannerOutput output;
  std::vector<LongitudinalState> output_longitudinal;
  std::vector<LateralPathState> output_lateral;
  SpatialPathCandidate spatial_path;
  STCandidateEvaluation st_candidate;
  std::vector<double> reference_speed_mps;

  double terminal_progress_m = 0.0;
  double terminal_speed_mps = 0.0;
  double progress_benefit_m = 0.0;
  double speed_benefit_mps = 0.0;
  double minimum_physical_margin_m = 0.0;
  double minimum_operational_margin_m = 0.0;
  double minimum_road_margin_m = 0.0;
  double maximum_acceleration_mps2 = 0.0;
  double maximum_jerk_mps3 = 0.0;
  double minimum_longitudinal_acceleration_mps2 = 0.0;
  double cost = 0.0;
  bool emergency_continuation = false;
  std::string rejection_detail;
};

struct ActiveBehaviorCycleResult {
  std::uint64_t cycle = 0;
  BehaviorManeuverPhase phase_before = BehaviorManeuverPhase::kKeepLane;
  BehaviorManeuverPhase phase_after = BehaviorManeuverPhase::kKeepLane;
  bool allow_new_maneuver = false;
  bool has_best_valid_candidate = false;
  std::uint64_t best_candidate_id = 0;
  int best_target_lane = -1;
  GapId best_gap;
  std::size_t stable_proposal_cycles = 0;
  bool would_commit = false;
  bool first_commit = false;
  bool continuation = false;
  bool target_only = false;
  bool target_stable = false;
  bool has_control_candidate = false;
  FinalBehaviorCandidate control_candidate;
  bool has_minimum_risk_candidate = false;
  FinalBehaviorCandidate minimum_risk_candidate;

  TrafficTrackingSnapshot tracking;
  FullLaneTrafficPredictionSnapshot prediction;
  BehaviorPlanningSnapshot behavior;
  SpatialPathBatchSnapshot spatial_paths;
  STCandidateBatchSnapshot st_candidates;
  std::vector<FinalBehaviorCandidate> final_candidates;
};

struct ActiveBehaviorDiagnostics {
  BehaviorManeuverPhase phase = BehaviorManeuverPhase::kKeepLane;
  std::uint64_t last_cycle = 0;
  std::uint64_t commit_cycle = 0;
  std::uint64_t committed_candidate_id = 0;
  int source_lane = -1;
  int target_lane = -1;
  GapId committed_gap;
  std::size_t stable_proposal_cycles = 0;
  std::size_t target_stable_cycles = 0;
  std::uint64_t oscillation_count = 0;
  std::uint64_t completed_maneuver_count = 0;
  std::uint64_t cancelled_proposal_count = 0;
  std::uint64_t full_validation_rejection_count = 0;
  std::uint64_t tightening_attempt_count = 0;
  std::uint64_t tightening_success_count = 0;
  bool latest_result_committed = false;
  bool has_latest_result = false;
  ActiveBehaviorCycleResult latest_result;
};

const char *FinalBehaviorCandidateStatusName(
    FinalBehaviorCandidateStatus status);
const char *BehaviorManeuverPhaseName(BehaviorManeuverPhase phase);

// Evaluate is transactional: tracker history, Gap stability, proposal state,
// committed identity and QP warm start remain unchanged until Commit succeeds
// after the selected control candidate has passed the planner commit gate.
class ActiveBehaviorPlanner {
public:
  explicit ActiveBehaviorPlanner(
      const ActiveBehaviorPlannerConfig &config);
  ~ActiveBehaviorPlanner();

  ActiveBehaviorPlanner(const ActiveBehaviorPlanner &) = delete;
  ActiveBehaviorPlanner &operator=(const ActiveBehaviorPlanner &) = delete;

  ActiveBehaviorCycleResult Evaluate(
      const PlanningSnapshot &planning, const MapData &map,
      bool allow_new_maneuver);
  void Commit(std::uint64_t cycle, bool active_candidate_dispatched);
  // Clears only the uncommitted Evaluate transaction. Persistent maneuver and
  // tracking state stay intact so a control-side fallback cannot erase an
  // already committed lane change.
  void Discard(std::uint64_t cycle) noexcept;
  // Cancels proposal/commit state after the control path history is no longer
  // aligned. Tracker history is retained long enough for its next Evaluate to
  // report ControlHistoryReset and rebuild tracks from the current frame.
  void HandleControlHistoryReset();
  void Reset();

  bool ManeuverInProgress() const;
  BehaviorManeuverPhase phase() const;
  ActiveBehaviorDiagnostics diagnostics() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

#endif // ACTIVE_BEHAVIOR_PLANNER_H
