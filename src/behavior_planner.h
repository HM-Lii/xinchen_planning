#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "traffic_prediction.h"

struct PlanningSnapshot;

struct GapId {
  int target_lane = -1;
  bool has_front_vehicle = false;
  int front_vehicle_id = 0;
  bool has_rear_vehicle = false;
  int rear_vehicle_id = 0;
};

bool operator==(const GapId &left, const GapId &right);
bool operator!=(const GapId &left, const GapId &right);
bool operator<(const GapId &left, const GapId &right);

enum class PassingOrder {
  kStayBehindFront,
  kMergeAheadOfRear,
  kWaitBehindRear
};

enum class BehaviorType {
  kKeepLane,
  kChangeLeft,
  kChangeRight
};

enum class CoarseAdmissionRejectionReason {
  kTargetLaneUnavailable,
  kGapTopologyChanged,
  kMergeCorridorBlocked,
  kPredictionEvidenceIncomplete,
  kGapNotStable,
  kFrontTrackNotAdmissible,
  kRearTrackNotAdmissible,
  kRetainedPrefixEvidenceInvalid,
  kRetainedPrefixConflict,
  kInsufficientFrontHeadway,
  kInsufficientRearHeadway,
  kRearTtcTooSmall,
  kSourceFrontTrackNotAdmissible,
  kSourceLaneFrontRisk,
  kNoCurrentLaneBrakingBackup,
  kInsufficientProgressBenefit,
  kGapUnreachable,
  kGapTooSmall,
  kPostObservationInsufficient
};

enum class BehaviorCandidateStatus {
  kKeepLaneBaseline,
  kCoarseAdmissionRejected,
  kCoarseAdmissionPassed
};

enum class GapTopologyFailureKind {
  kNone,
  kExpectedFrontMissing,
  kExpectedRearMissing,
  kExpectedFrontAndRearMissing,
  kBoundariesNotAdjacent,
  kUnexpectedRearBoundary,
  kUnexpectedFrontBoundary,
  kUnexpectedOccupiedGap
};

struct BehaviorPlannerConfig {
  double simulator_time_step_s = 0.02;
  double gap_evaluation_time_step_s = 0.50;
  double planning_horizon_s = 8.0;
  double maximum_retained_prefix_s = 0.30;
  double coarse_lane_change_duration_s = 4.0;
  double post_maneuver_observation_s = 2.0;
  double gap_stability_window_s = 0.30;
  std::size_t minimum_stable_observations = 3;
  double target_front_time_headway_s = 2.0;
  double target_rear_time_headway_s = 1.75;
  double minimum_rear_ttc_s = 7.0;
  double standstill_clearance_m = 2.0;
  double physical_collision_margin_m = 0.0;
  // Minimum extra longitudinal distance accumulated over the traffic
  // prediction horizon before a lane change has sufficient utility.
  double minimum_progress_benefit_m = 1.0;
  double minimum_gap_center_window_m = 1.0;
  double maximum_longitudinal_acceleration_mps2 = 3.0;
  double minimum_longitudinal_acceleration_mps2 = -5.0;
  double target_speed_mps = 22.12848;
  double rear_search_distance_m = 150.0;
  double front_search_distance_m = 300.0;
  double ego_length_m = 4.8;
  double ego_width_m = 2.0;
  double lane_width_m = 4.0;
  int lane_count = 3;
  std::size_t maximum_candidates = 8;
  std::size_t maximum_gap_samples = 32;
  std::size_t coarse_acceleration_samples = 9;
  std::size_t maximum_coarse_reachable_states = 512;
  double coarse_position_resolution_m = 0.50;
  double coarse_speed_resolution_mps = 0.25;
};

struct TrafficGapSample {
  double prediction_time_s = 0.0;
  bool front_boundary_present = false;
  bool rear_boundary_present = false;
  bool boundaries_adjacent = false;
  bool front_track_admissible = true;
  bool rear_track_admissible = true;
  double front_occupied_min_road_s_m = 0.0;
  double rear_occupied_max_road_s_m = 0.0;
  double front_road_s_rate_mps = 0.0;
  double rear_road_s_rate_mps = 0.0;
  double front_uncertainty_m = 0.0;
  double rear_uncertainty_m = 0.0;
  double available_ego_center_min_road_s_m = 0.0;
  double available_ego_center_max_road_s_m = 0.0;
  double available_ego_center_window_m = 0.0;
};

struct TrafficGap {
  GapId id;
  bool current_observation_valid = false;
  bool topology_consistent = false;
  bool topology_failure_observed = false;
  GapTopologyFailureKind first_topology_failure_kind =
      GapTopologyFailureKind::kNone;
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
  TrafficPredictionHypothesis first_merge_corridor_intrusion_hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  bool prediction_evidence_complete = false;
  double minimum_ego_center_window_m = 0.0;
  double maximum_boundary_uncertainty_m = 0.0;
  std::vector<TrafficGapSample> samples;
};

struct CoarseAdmissionResult {
  bool evaluated = false;
  bool passed = false;
  std::vector<CoarseAdmissionRejectionReason> rejection_reasons;
  bool has_front_margin = false;
  bool has_rear_margin = false;
  bool has_rear_ttc = false;
  double minimum_front_margin_m = 0.0;
  double minimum_rear_margin_m = 0.0;
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
  TrafficPredictionHypothesis source_front_limiting_hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  bool source_front_risk_observed = false;
  double first_source_front_risk_time_s = 0.0;
  int first_source_front_risk_vehicle_id = 0;
  double first_source_front_risk_margin_m = 0.0;
  TrafficPredictionHypothesis first_source_front_risk_hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  bool target_kinematic_failure_observed = false;
  double first_target_kinematic_failure_time_s = 0.0;
  std::size_t first_target_propagated_state_count = 0;
  std::size_t first_target_feasible_state_count = 0;
  double first_target_best_front_margin_m = 0.0;
  double first_target_best_rear_margin_m = 0.0;
  double first_target_best_rear_ttc_margin_m = 0.0;
  bool merge_corridor_blocked_observed = false;
  double first_merge_corridor_blocked_time_s = 0.0;
  std::size_t first_merge_corridor_candidate_state_count = 0;
  std::size_t first_merge_corridor_feasible_state_count = 0;
  int first_merge_corridor_blocking_vehicle_id = 0;
  TrafficPredictionHypothesis first_merge_corridor_blocking_hypothesis =
      TrafficPredictionHypothesis::kNominalConstantVelocity;
  double first_merge_corridor_blocking_margin_m = 0.0;
};

struct BehaviorCandidate {
  std::uint64_t candidate_id = 0;
  BehaviorType behavior = BehaviorType::kKeepLane;
  int source_lane = -1;
  int target_lane = -1;
  GapId gap;
  PassingOrder order = PassingOrder::kStayBehindFront;
  double estimated_progress_m = 0.0;
  double estimated_progress_gain_m = 0.0;
  // Equivalent average-speed gain retained for diagnostics and legacy
  // consumers. Admission and candidate ranking use progress gain.
  double estimated_speed_gain_mps = 0.0;
  double uncertainty_cost = 0.0;
  std::size_t stable_observations = 0;
  double stable_duration_s = 0.0;
  TrafficGap traffic_gap;
  CoarseAdmissionResult coarse_admission;
  BehaviorCandidateStatus status =
      BehaviorCandidateStatus::kCoarseAdmissionRejected;
};

struct GapStabilityRecord {
  GapId gap;
  std::size_t consecutive_observations = 0;
  double observed_duration_s = 0.0;
};

struct BehaviorPlannerState {
  std::uint64_t cycle = 0;
  std::vector<GapStabilityRecord> gap_stability;
};

struct BehaviorPlanningSnapshot {
  std::uint64_t cycle = 0;
  int source_lane = -1;
  std::size_t generated_candidate_count = 0;
  std::size_t stable_gap_count = 0;
  std::size_t coarse_admitted_candidate_count = 0;
  bool has_best_coarse_candidate = false;
  std::uint64_t best_coarse_candidate_id = 0;
  std::vector<BehaviorCandidate> candidates;
};

struct BehaviorPlanningUpdate {
  BehaviorPlannerState next_state;
  BehaviorPlanningSnapshot snapshot;
};

const char *PassingOrderName(PassingOrder order);
const char *BehaviorTypeName(BehaviorType behavior);
const char *CoarseAdmissionRejectionReasonName(
    CoarseAdmissionRejectionReason reason);
const char *BehaviorCandidateStatusName(
    BehaviorCandidateStatus status);
const char *GapTopologyFailureKindName(
    GapTopologyFailureKind kind);
bool HasCoarseAdmissionRejectionReason(
    const CoarseAdmissionResult &result,
    CoarseAdmissionRejectionReason reason);

// P2.2 is intentionally stateless during Evaluate. Gap stability becomes
// visible to a later cycle only if ActiveBehaviorPlanner commits next_state.
class BehaviorPlanner {
public:
  explicit BehaviorPlanner(
      const BehaviorPlannerConfig &config);

  BehaviorPlanningUpdate Evaluate(
      const PlanningSnapshot &planning,
      const TrafficTrackingSnapshot &tracking,
      const FullLaneTrafficPredictionSnapshot &prediction,
      double track_length_m,
      const BehaviorPlannerState &state) const;

private:
  BehaviorPlannerConfig config_;
};

#endif // BEHAVIOR_PLANNER_H
