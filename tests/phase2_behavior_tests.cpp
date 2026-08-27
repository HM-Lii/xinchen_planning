#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behavior_planner.h"
#include "planner.h"
#include "traffic_prediction.h"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

void RunTest(void (*test)(), const std::string &name) {
  try {
    test();
  } catch (const std::exception &error) {
    Expect(false, name + " threw: " + error.what());
  }
}

TrackedVehicleState Track(int id, double road_s_m, double d_m,
                          double speed_mps,
                          bool safety_admissible = true) {
  TrackedVehicleState result;
  result.id = id;
  result.road_s_wrapped_m = road_s_m;
  result.road_s_unwrapped_m = road_s_m;
  result.relative_road_s_m = road_s_m - 100.0;
  result.d_m = d_m;
  result.road_s_rate_mps = speed_mps;
  result.length_m = 4.8;
  result.width_m = 2.0;
  result.longitudinal_uncertainty_m = 0.25;
  result.observation_count = 10;
  result.valid = safety_admissible;
  result.safety_admissible = safety_admissible;
  return result;
}

TrafficTrackingSnapshot Tracking(
    std::uint64_t cycle,
    const std::vector<TrackedVehicleState> &tracks) {
  TrafficTrackingSnapshot result;
  result.cycle = cycle;
  result.ego_road_s_unwrapped_m = 100.0;
  result.tracks = tracks;
  result.observed_track_count = tracks.size();
  for (const TrackedVehicleState &track : tracks) {
    if (track.valid) {
      ++result.valid_track_count;
    }
    if (track.safety_admissible) {
      ++result.safety_admissible_track_count;
    }
  }
  return result;
}

PlanningSnapshot Planning(std::uint64_t cycle,
                          bool braking_backup = true) {
  PlanningSnapshot result;
  result.cycle = cycle;
  result.target_lane = 1;
  result.input.ego.s = 100.0;
  result.input.ego.d = 6.0;
  result.frontier.road_s_unwrapped_m = 100.0;
  result.frontier.d_m = 6.0;
  result.frontier.longitudinal.v = 15.0;
  result.frontier.lateral.valid = true;
  result.frontier.lateral.road_parameter_s = 100.0;
  result.frontier.lateral.planned_d = 6.0;
  result.frontier.state_source = PlanningStateSource::kTelemetry;
  result.control_lane_cruise_backup_valid = braking_backup;
  result.reset_reason = PlanningStateResetReason::kNone;
  return result;
}

FullLaneTrafficPredictionConfig PredictionConfig() {
  FullLaneTrafficPredictionConfig result;
  result.front_conservative_deceleration_mps2 = -1.0;
  result.rear_conservative_acceleration_mps2 = 0.01;
  result.longitudinal_uncertainty_growth_mps = 0.05;
  return result;
}

BehaviorPlannerConfig BehaviorConfig() {
  BehaviorPlannerConfig result;
  result.gap_stability_window_s = 0.04;
  result.minimum_stable_observations = 3;
  result.target_front_time_headway_s = 1.5;
  result.target_rear_time_headway_s = 1.0;
  result.minimum_rear_ttc_s = 5.0;
  result.minimum_progress_benefit_m = 0.5;
  return result;
}

const BehaviorCandidate *FindCandidate(
    const BehaviorPlanningSnapshot &snapshot, int target_lane,
    PassingOrder order) {
  for (const BehaviorCandidate &candidate : snapshot.candidates) {
    if (candidate.target_lane == target_lane &&
        candidate.order == order &&
        candidate.behavior != BehaviorType::kKeepLane) {
      return &candidate;
    }
  }
  return nullptr;
}

const BehaviorCandidate *FindKeepCandidate(
    const BehaviorPlanningSnapshot &snapshot) {
  for (const BehaviorCandidate &candidate : snapshot.candidates) {
    if (candidate.behavior == BehaviorType::kKeepLane) {
      return &candidate;
    }
  }
  return nullptr;
}

BehaviorPlanningUpdate EvaluateCycle(
    const BehaviorPlanner &planner,
    const FullLaneTrafficPredictor &predictor,
    std::uint64_t cycle,
    const std::vector<TrackedVehicleState> &tracks,
    const BehaviorPlannerState &state, bool braking_backup = true,
    PlanningSnapshot *planning_override = nullptr) {
  const TrafficTrackingSnapshot tracking = Tracking(cycle, tracks);
  PlanningSnapshot planning = planning_override == nullptr
                                  ? Planning(cycle, braking_backup)
                                  : *planning_override;
  planning.cycle = cycle;
  const FullLaneTrafficPredictionSnapshot prediction =
      predictor.Predict(tracking,
                        planning.frontier.time_from_telemetry_s);
  return planner.Evaluate(planning, tracking, prediction, 6945.554,
                          state);
}

std::vector<TrackedVehicleState> SafePassingTraffic() {
  return {Track(10, 130.0, 6.0, 8.0),
          Track(20, 260.0, 2.0, 20.0),
          Track(21, 30.0, 2.0, 14.0)};
}

BehaviorPlanningUpdate StableEvaluation(
    const std::vector<TrackedVehicleState> &tracks,
    const BehaviorPlannerConfig &behavior_config,
    const FullLaneTrafficPredictionConfig &prediction_config,
    bool braking_backup = true) {
  BehaviorPlanner planner(behavior_config);
  FullLaneTrafficPredictor predictor(prediction_config);
  BehaviorPlannerState state;
  BehaviorPlanningUpdate update;
  for (std::uint64_t cycle = 1; cycle <= 3; ++cycle) {
    update = EvaluateCycle(planner, predictor, cycle, tracks, state,
                           braking_backup);
    state = update.next_state;
  }
  return update;
}

void TestGapIdentityPassingOrdersAndBoundedTopK() {
  const BehaviorPlannerConfig behavior_config = BehaviorConfig();
  BehaviorPlanner planner(behavior_config);
  FullLaneTrafficPredictor predictor(PredictionConfig());
  const BehaviorPlanningUpdate update = EvaluateCycle(
      planner, predictor, 1, SafePassingTraffic(),
      BehaviorPlannerState());

  const BehaviorCandidate *keep = FindKeepCandidate(update.snapshot);
  const BehaviorCandidate *immediate = FindCandidate(
      update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  const BehaviorCandidate *wait = FindCandidate(
      update.snapshot, 0, PassingOrder::kWaitBehindRear);
  Expect(keep != nullptr &&
             keep->status ==
                 BehaviorCandidateStatus::kKeepLaneBaseline,
         "current-lane cruise remains an explicit baseline candidate");
  Expect(immediate != nullptr && immediate->gap.has_front_vehicle &&
             immediate->gap.front_vehicle_id == 20 &&
             immediate->gap.has_rear_vehicle &&
             immediate->gap.rear_vehicle_id == 21,
         "the immediate target GapId preserves its front/rear IDs");
  Expect(wait != nullptr && wait->gap.has_front_vehicle &&
             wait->gap.front_vehicle_id == 21 &&
             !wait->gap.has_rear_vehicle,
         "WaitBehindRear fixes the passed rear vehicle as the front bound");
  Expect(update.snapshot.generated_candidate_count ==
             update.snapshot.candidates.size() &&
             update.snapshot.candidates.size() <=
                 behavior_config.maximum_candidates,
         "finite enumeration is retained through a bounded deterministic Top-K");
  for (const BehaviorCandidate &candidate : update.snapshot.candidates) {
    Expect(candidate.traffic_gap.samples.size() <=
               behavior_config.maximum_gap_samples,
           "every multi-time gap uses bounded sample storage");
  }

  BehaviorPlannerConfig bounded_config = behavior_config;
  bounded_config.maximum_candidates = 2;
  bounded_config.maximum_gap_samples = 13;
  BehaviorPlanner bounded_planner(bounded_config);
  const BehaviorPlanningUpdate bounded = EvaluateCycle(
      bounded_planner, predictor, 1, SafePassingTraffic(),
      BehaviorPlannerState());
  Expect(bounded.snapshot.candidates.size() == 2 &&
             bounded.next_state.gap_stability.size() <= 1,
         "Top-K also bounds the committed stability records");
}

void TestTransactionalStabilityAndGapIdentityReset() {
  const BehaviorPlannerConfig behavior_config = BehaviorConfig();
  BehaviorPlanner planner(behavior_config);
  FullLaneTrafficPredictor predictor(PredictionConfig());
  BehaviorPlannerState committed;

  const BehaviorPlanningUpdate first = EvaluateCycle(
      planner, predictor, 1, SafePassingTraffic(), committed);
  const BehaviorCandidate *first_left = FindCandidate(
      first.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(first_left != nullptr && first_left->stable_observations == 1 &&
             HasCoarseAdmissionRejectionReason(
                 first_left->coarse_admission,
                 CoarseAdmissionRejectionReason::kGapNotStable),
         "a new GapId is rejected until it has committed stability evidence");

  const BehaviorPlanningUpdate discarded_a = EvaluateCycle(
      planner, predictor, 2, SafePassingTraffic(), first.next_state);
  const BehaviorPlanningUpdate discarded_b = EvaluateCycle(
      planner, predictor, 2, SafePassingTraffic(), first.next_state);
  const BehaviorCandidate *second_a = FindCandidate(
      discarded_a.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  const BehaviorCandidate *second_b = FindCandidate(
      discarded_b.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(second_a != nullptr && second_b != nullptr &&
             second_a->stable_observations == 2 &&
             second_b->stable_observations == 2 &&
             second_a->candidate_id == second_b->candidate_id,
         "discarded Evaluate calls cannot advance committed stability");

  const BehaviorPlanningUpdate skipped = EvaluateCycle(
      planner, predictor, 3, SafePassingTraffic(), first.next_state);
  const BehaviorCandidate *skipped_candidate = FindCandidate(
      skipped.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(skipped_candidate != nullptr &&
             skipped_candidate->stable_observations == 1,
         "a missing committed control cycle breaks consecutive GapId evidence");

  committed = discarded_a.next_state;
  const BehaviorPlanningUpdate third = EvaluateCycle(
      planner, predictor, 3, SafePassingTraffic(), committed);
  const BehaviorCandidate *stable = FindCandidate(
      third.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(stable != nullptr && stable->stable_observations == 3 &&
             std::fabs(stable->stable_duration_s - 0.04) < 1e-12 &&
             stable->status ==
                 BehaviorCandidateStatus::kCoarseAdmissionPassed,
         "three committed observations satisfy the configured stability window");

  std::vector<TrackedVehicleState> changed = SafePassingTraffic();
  changed[1].id = 22;
  const BehaviorPlanningUpdate fourth = EvaluateCycle(
      planner, predictor, 4, changed, third.next_state);
  const BehaviorCandidate *changed_gap = FindCandidate(
      fourth.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(changed_gap != nullptr &&
             changed_gap->gap.front_vehicle_id == 22 &&
             changed_gap->stable_observations == 1 &&
             changed_gap->status ==
                 BehaviorCandidateStatus::kCoarseAdmissionRejected,
         "an unexplained boundary-ID change resets GapId stability");
}

void TestGapIdentityAcrossRoadSLoop() {
  const double track_length_m = 6945.554;
  const double ego_unwrapped_s_m = 6950.0;
  std::vector<TrackedVehicleState> tracks = {
      Track(10, 6960.0, 6.0, 12.0),
      Track(20, 6970.0, 2.0, 18.0),
      Track(21, 6930.0, 2.0, 14.0)};
  for (TrackedVehicleState &track : tracks) {
    track.relative_road_s_m =
        track.road_s_unwrapped_m - ego_unwrapped_s_m;
  }
  TrafficTrackingSnapshot tracking = Tracking(1, tracks);
  tracking.ego_road_s_unwrapped_m = ego_unwrapped_s_m;
  PlanningSnapshot planning = Planning(1);
  planning.input.ego.s = ego_unwrapped_s_m - track_length_m;
  planning.frontier.road_s_unwrapped_m =
      ego_unwrapped_s_m - track_length_m;
  FullLaneTrafficPredictor predictor(PredictionConfig());
  const FullLaneTrafficPredictionSnapshot prediction =
      predictor.Predict(tracking, 0.0);
  BehaviorPlanner planner(BehaviorConfig());
  const BehaviorPlanningUpdate update = planner.Evaluate(
      planning, tracking, prediction, track_length_m,
      BehaviorPlannerState());
  const BehaviorCandidate *candidate = FindCandidate(
      update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(candidate != nullptr && candidate->gap.has_front_vehicle &&
             candidate->gap.front_vehicle_id == 20 &&
             candidate->gap.has_rear_vehicle &&
             candidate->gap.rear_vehicle_id == 21,
         "wrapped planning RoadS aligns with unwrapped front/rear GapId bounds");
}

void TestDangerousRearAndBrakingFrontRejections() {
  std::vector<TrackedVehicleState> fast_rear = SafePassingTraffic();
  fast_rear[2] = Track(21, 80.0, 2.0, 32.0);
  const BehaviorPlanningUpdate rear_update = StableEvaluation(
      fast_rear, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *rear = FindCandidate(
      rear_update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(rear != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 rear->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kInsufficientRearHeadway) &&
             HasCoarseAdmissionRejectionReason(
                 rear->coarse_admission,
                 CoarseAdmissionRejectionReason::kRearTtcTooSmall),
         "a fast target-lane rear vehicle is rejected by headway and TTC");
  Expect(rear != nullptr &&
             rear->coarse_admission
                 .target_kinematic_failure_observed &&
             rear->coarse_admission
                     .first_target_kinematic_failure_time_s >=
                 BehaviorConfig().coarse_lane_change_duration_s &&
             rear->coarse_admission
                     .first_target_propagated_state_count > 0 &&
             rear->coarse_admission
                     .first_target_feasible_state_count == 0 &&
             rear->coarse_admission
                     .first_target_best_rear_ttc_margin_m < 0.0,
         "rear rejection exposes the first coherent reachability failure");

  std::vector<TrackedVehicleState> close_front = SafePassingTraffic();
  close_front[1] = Track(20, 106.0, 2.0, 0.0);
  const BehaviorPlanningUpdate front_update = StableEvaluation(
      close_front, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *front = FindCandidate(
      front_update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(front != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 front->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kInsufficientFrontHeadway),
         "the conservative front-braking envelope rejects a close merge");
  Expect(front != nullptr &&
             front->coarse_admission
                 .target_kinematic_failure_observed &&
             front->coarse_admission
                     .first_target_best_front_margin_m < 0.0,
         "front rejection is based on one propagated position/speed state");
}

void TestEvidenceBackupBenefitAndTrackFreshnessRejections() {
  std::vector<TrackedVehicleState> stale_front = SafePassingTraffic();
  stale_front[1] = Track(20, 260.0, 2.0, 20.0, false);
  const BehaviorPlanningUpdate stale_update = StableEvaluation(
      stale_front, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *stale = FindCandidate(
      stale_update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(stale != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 stale->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kFrontTrackNotAdmissible),
         "a stale target boundary is never safety-admissible");

  const BehaviorPlanningUpdate backup_update = StableEvaluation(
      SafePassingTraffic(), BehaviorConfig(), PredictionConfig(), false);
  const BehaviorCandidate *no_backup = FindCandidate(
      backup_update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(no_backup != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 no_backup->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kNoCurrentLaneBrakingBackup),
         "lane change admission requires a current-lane braking backup");

  const std::vector<TrackedVehicleState> no_front_traffic = {
      Track(20, 260.0, 2.0, 20.0), Track(21, 30.0, 2.0, 14.0)};
  const BehaviorPlanningUpdate benefit_update = StableEvaluation(
      no_front_traffic, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *no_benefit = FindCandidate(
      benefit_update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(no_benefit != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 no_benefit->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kInsufficientProgressBenefit),
         "an otherwise open lane is rejected when it has no speed benefit");

  FullLaneTrafficPredictionConfig short_prediction = PredictionConfig();
  short_prediction.planning_horizon_s = 3.0;
  short_prediction.post_maneuver_observation_s = 0.0;
  const BehaviorPlanningUpdate coverage_update = StableEvaluation(
      SafePassingTraffic(), BehaviorConfig(), short_prediction);
  const BehaviorCandidate *coverage = FindCandidate(
      coverage_update.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(coverage != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 coverage->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kPredictionEvidenceIncomplete) &&
             HasCoarseAdmissionRejectionReason(
                 coverage->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kPostObservationInsufficient),
         "missing completion/post coverage is recorded as explicit evidence failure");
}

void TestProgressBenefitAccumulatesOverPredictionHorizon() {
  BehaviorPlannerConfig behavior_config = BehaviorConfig();
  behavior_config.minimum_progress_benefit_m = 1.0;
  const std::vector<TrackedVehicleState> traffic = {
      Track(10, 160.0, 6.0, 14.0),
      Track(20, 160.0, 2.0, 14.25),
      Track(21, 30.0, 2.0, 14.0)};

  const BehaviorPlanningUpdate update = StableEvaluation(
      traffic, behavior_config, PredictionConfig());
  const BehaviorCandidate *candidate = FindCandidate(
      update.snapshot, 0, PassingOrder::kMergeAheadOfRear);

  Expect(candidate != nullptr &&
             candidate->estimated_progress_gain_m > 1.9 &&
             candidate->estimated_progress_gain_m < 2.1,
         "a persistent 0.25 m/s target-lane advantage accumulates to about 2 m over the 8 s prediction horizon");
  Expect(candidate != nullptr &&
             candidate->estimated_speed_gain_mps > 0.24 &&
             candidate->estimated_speed_gain_mps < 0.26,
         "the retained speed-gain diagnostic is the horizon-average of accumulated progress");
  Expect(candidate != nullptr &&
             !HasCoarseAdmissionRejectionReason(
                 candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kInsufficientProgressBenefit),
         "accumulated progress, rather than a 1 m/s instantaneous threshold, admits the useful lane change");
}

void TestTopologySourceLaneAndSmallGapRejections() {
  std::vector<TrackedVehicleState> cutting_in = SafePassingTraffic();
  TrackedVehicleState intruder = Track(30, 104.0, 10.0, 10.0);
  intruder.d_rate_mps = -3.0;
  cutting_in.push_back(intruder);
  const BehaviorPlanningUpdate topology = StableEvaluation(
      cutting_in, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *topology_candidate = FindCandidate(
      topology.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(topology_candidate != nullptr &&
             !HasCoarseAdmissionRejectionReason(
                 topology_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kGapTopologyChanged) &&
             topology_candidate->traffic_gap.topology_consistent &&
             !HasCoarseAdmissionRejectionReason(
                 topology_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kMergeCorridorBlocked) &&
             !topology_candidate->coarse_admission
                  .merge_corridor_blocked_observed &&
             topology_candidate->coarse_admission.passed,
         "an intrusion remains diagnostic and advances when at least one coherent post-merge state avoids it");
  Expect(topology_candidate != nullptr &&
             topology_candidate->traffic_gap.topology_failure_observed &&
             topology_candidate->traffic_gap
                     .first_topology_failure_kind ==
                 GapTopologyFailureKind::kBoundariesNotAdjacent &&
             topology_candidate->traffic_gap
                 .first_topology_expected_front_found &&
             topology_candidate->traffic_gap
                 .first_topology_expected_rear_found &&
             topology_candidate->traffic_gap
                 .first_topology_actual_front_present &&
             topology_candidate->traffic_gap
                     .first_topology_actual_front_vehicle_id == 30 &&
             topology_candidate->traffic_gap
                 .first_topology_actual_rear_present &&
             topology_candidate->traffic_gap
                     .first_topology_actual_rear_vehicle_id == 21 &&
             topology_candidate->traffic_gap
                     .first_topology_failure_time_s > 0.0 &&
             topology_candidate->traffic_gap
                 .merge_corridor_intrusion_observed &&
             topology_candidate->traffic_gap
                     .first_merge_corridor_intrusion_vehicle_id == 30 &&
             topology_candidate->traffic_gap
                     .first_merge_corridor_intrusion_hypothesis ==
                 TrafficPredictionHypothesis::kLateralContinuation,
         "topology diagnostics record the first failure time, actual boundaries and intrusion hypothesis");

  std::vector<TrackedVehicleState> centered_without_lateral_motion =
      SafePassingTraffic();
  TrackedVehicleState centered_source_rear =
      Track(40, 90.0, 6.0, 15.0);
  centered_without_lateral_motion.push_back(centered_source_rear);
  const BehaviorPlanningUpdate centered_ignored = StableEvaluation(
      centered_without_lateral_motion, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *centered_candidate = FindCandidate(
      centered_ignored.snapshot, 0,
      PassingOrder::kMergeAheadOfRear);
  Expect(centered_candidate != nullptr &&
             centered_candidate->coarse_admission.passed &&
             !centered_candidate->traffic_gap
                  .merge_corridor_intrusion_observed &&
             !HasCoarseAdmissionRejectionReason(
                 centered_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kMergeCorridorBlocked),
         "a lane-centered vehicle without lateral motion does not intrude into the adjacent lane");

  std::vector<TrackedVehicleState> blocking_cut_in =
      SafePassingTraffic();
  TrackedVehicleState blocker = Track(31, 100.0, 6.0, 15.0);
  blocker.d_rate_mps = -2.0;
  blocker.length_m = 100.0;
  blocking_cut_in.push_back(blocker);
  const BehaviorPlanningUpdate blocked = StableEvaluation(
      blocking_cut_in, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *blocked_candidate = FindCandidate(
      blocked.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(blocked_candidate != nullptr &&
             blocked_candidate->traffic_gap.topology_consistent &&
             HasCoarseAdmissionRejectionReason(
                 blocked_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kMergeCorridorBlocked) &&
             blocked_candidate->coarse_admission
                 .merge_corridor_blocked_observed &&
             blocked_candidate->coarse_admission
                     .first_merge_corridor_candidate_state_count > 0 &&
             blocked_candidate->coarse_admission
                     .first_merge_corridor_feasible_state_count == 0 &&
             blocked_candidate->coarse_admission
                     .first_merge_corridor_blocking_vehicle_id == 31 &&
             blocked_candidate->coarse_admission
                     .first_merge_corridor_blocking_hypothesis ==
                 TrafficPredictionHypothesis::kLateralContinuation &&
             blocked_candidate->coarse_admission
                     .first_merge_corridor_blocking_margin_m < 0.0,
         "an intruder that removes every post-merge coherent state receives the dedicated hard rejection");

  std::vector<TrackedVehicleState> reversing = SafePassingTraffic();
  reversing[1] = Track(20, 115.0, 2.0, 0.0);
  reversing[2] = Track(21, 90.0, 2.0, 35.0);
  const BehaviorPlanningUpdate reversed = StableEvaluation(
      reversing, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *reversed_candidate = FindCandidate(
      reversed.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(reversed_candidate != nullptr &&
             reversed_candidate->traffic_gap
                 .expected_boundaries_reversed_observed &&
             reversed_candidate->traffic_gap
                     .first_expected_boundaries_reversed_time_s > 0.0 &&
             !reversed_candidate->traffic_gap.topology_consistent &&
             HasCoarseAdmissionRejectionReason(
                 reversed_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kGapTopologyChanged),
         "a true expected-boundary reversal remains a topology hard rejection");

  std::vector<TrackedVehicleState> distant_cutting_in =
      SafePassingTraffic();
  TrackedVehicleState distant_intruder =
      Track(30, 150.0, 6.0, 15.0);
  distant_intruder.d_rate_mps = -2.0;
  distant_cutting_in.push_back(distant_intruder);
  const BehaviorPlanningUpdate distant_topology = StableEvaluation(
      distant_cutting_in, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *distant_candidate = FindCandidate(
      distant_topology.snapshot, 0,
      PassingOrder::kMergeAheadOfRear);
  Expect(distant_candidate != nullptr &&
             distant_candidate->traffic_gap.topology_failure_observed &&
             distant_candidate->traffic_gap.topology_consistent &&
             !distant_candidate->traffic_gap
                  .merge_corridor_intrusion_observed &&
             distant_candidate->stable_observations == 3 &&
             !HasCoarseAdmissionRejectionReason(
                 distant_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kGapTopologyChanged) &&
             !HasCoarseAdmissionRejectionReason(
                 distant_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::kGapNotStable),
         "a distant future reordering remains observable without invalidating the current stable GapId");

  std::vector<TrackedVehicleState> source_rear = {
      Track(40, 90.0, 6.0, 25.0),
      Track(20, 260.0, 2.0, 20.0),
      Track(21, 30.0, 2.0, 14.0)};
  const BehaviorPlanningUpdate rear_source = StableEvaluation(
      source_rear, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *rear_source_candidate = FindCandidate(
      rear_source.snapshot, 0,
      PassingOrder::kMergeAheadOfRear);
  Expect(rear_source_candidate != nullptr &&
             !rear_source_candidate->coarse_admission
                  .has_source_front_margin &&
             !HasCoarseAdmissionRejectionReason(
                 rear_source_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kSourceLaneFrontRisk),
         "a source-lane rear acceleration hypothesis is not relabeled as a front vehicle");

  std::vector<TrackedVehicleState> source_risk = SafePassingTraffic();
  source_risk[0] = Track(10, 106.0, 6.0, 5.0);
  const BehaviorPlanningUpdate source = StableEvaluation(
      source_risk, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *source_candidate = FindCandidate(
      source.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(source_candidate != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 source_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kSourceLaneFrontRisk),
         "the source-lane front bound remains active until lane departure");
  Expect(source_candidate != nullptr &&
             source_candidate->coarse_admission
                 .has_source_front_margin &&
             source_candidate->coarse_admission
                     .minimum_source_front_margin_m < 0.0 &&
             source_candidate->coarse_admission
                     .source_front_limiting_vehicle_id == 10 &&
             source_candidate->coarse_admission
                     .source_front_limiting_hypothesis ==
                 TrafficPredictionHypothesis::
                     kFrontConservativeBraking &&
             source_candidate->coarse_admission
                 .source_front_risk_observed &&
             source_candidate->coarse_admission
                     .first_source_front_risk_vehicle_id == 10 &&
             source_candidate->coarse_admission
                     .first_source_front_risk_margin_m < 0.0 &&
             source_candidate->coarse_admission
                     .source_front_limiting_occupied_min_road_s_m <
                 source_candidate->coarse_admission
                         .source_front_limiting_ego_road_s_m +
                     0.5 * BehaviorConfig().ego_length_m +
                     BehaviorConfig().standstill_clearance_m,
         "source-lane risk records the limiting vehicle, time, envelope and signed clearance");

  source_risk[0] = Track(10, 130.0, 6.0, 8.0, false);
  const BehaviorPlanningUpdate source_stale = StableEvaluation(
      source_risk, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *source_stale_candidate = FindCandidate(
      source_stale.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(source_stale_candidate != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 source_stale_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kSourceFrontTrackNotAdmissible),
         "an inadmissible source-lane front track fails closed");

  std::vector<TrackedVehicleState> small_gap = SafePassingTraffic();
  small_gap[1] = Track(20, 106.0, 2.0, 15.0);
  small_gap[2] = Track(21, 94.0, 2.0, 15.0);
  const BehaviorPlanningUpdate small = StableEvaluation(
      small_gap, BehaviorConfig(), PredictionConfig());
  const BehaviorCandidate *small_candidate = FindCandidate(
      small.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(small_candidate != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 small_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::kGapTooSmall) &&
             HasCoarseAdmissionRejectionReason(
                 small_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::kGapUnreachable),
         "vehicle dimensions and uncertainty reject a physically small gap");
}

void TestRetainedPrefixConflictAndInvalidEvidence() {
  const BehaviorPlannerConfig behavior_config = BehaviorConfig();
  BehaviorPlanner planner(behavior_config);
  FullLaneTrafficPredictor predictor(PredictionConfig());
  std::vector<TrackedVehicleState> traffic = SafePassingTraffic();
  traffic.push_back(Track(99, 100.3, 10.0, 15.0));

  PlanningSnapshot conflict_planning = Planning(1);
  conflict_planning.frontier.time_from_telemetry_s = 0.02;
  conflict_planning.frontier.road_s_unwrapped_m = 100.3;
  conflict_planning.retained_prefix_points = 1;
  LongitudinalState longitudinal;
  longitudinal.s = 0.3;
  longitudinal.v = 15.0;
  conflict_planning.retained_longitudinal_states.push_back(longitudinal);
  LateralPathState lateral;
  lateral.valid = true;
  lateral.road_parameter_s = 100.3;
  lateral.planned_d = 10.0;
  conflict_planning.retained_lateral_states.push_back(lateral);
  const BehaviorPlanningUpdate conflict = EvaluateCycle(
      planner, predictor, 1, traffic, BehaviorPlannerState(), true,
      &conflict_planning);
  const BehaviorCandidate *conflict_candidate = FindCandidate(
      conflict.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(conflict_candidate != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 conflict_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kRetainedPrefixConflict),
         "predicted collision in the immutable retained prefix rejects admission");

  PlanningSnapshot invalid_planning = conflict_planning;
  invalid_planning.retained_longitudinal_states.clear();
  const BehaviorPlanningUpdate invalid = EvaluateCycle(
      planner, predictor, 1, SafePassingTraffic(),
      BehaviorPlannerState(), true, &invalid_planning);
  const BehaviorCandidate *invalid_candidate = FindCandidate(
      invalid.snapshot, 0, PassingOrder::kMergeAheadOfRear);
  Expect(invalid_candidate != nullptr &&
             HasCoarseAdmissionRejectionReason(
                 invalid_candidate->coarse_admission,
                 CoarseAdmissionRejectionReason::
                     kRetainedPrefixEvidenceInvalid),
         "malformed retained-prefix evidence fails closed");
}

} // namespace

int main() {
  RunTest(TestGapIdentityPassingOrdersAndBoundedTopK,
          "TestGapIdentityPassingOrdersAndBoundedTopK");
  RunTest(TestTransactionalStabilityAndGapIdentityReset,
          "TestTransactionalStabilityAndGapIdentityReset");
  RunTest(TestGapIdentityAcrossRoadSLoop,
          "TestGapIdentityAcrossRoadSLoop");
  RunTest(TestDangerousRearAndBrakingFrontRejections,
          "TestDangerousRearAndBrakingFrontRejections");
  RunTest(TestEvidenceBackupBenefitAndTrackFreshnessRejections,
          "TestEvidenceBackupBenefitAndTrackFreshnessRejections");
  RunTest(TestProgressBenefitAccumulatesOverPredictionHorizon,
          "TestProgressBenefitAccumulatesOverPredictionHorizon");
  RunTest(TestTopologySourceLaneAndSmallGapRejections,
          "TestTopologySourceLaneAndSmallGapRejections");
  RunTest(TestRetainedPrefixConflictAndInvalidEvidence,
          "TestRetainedPrefixConflictAndInvalidEvidence");
  if (failures != 0) {
    std::cerr << failures << " P2.2 behavior assertion(s) failed"
              << std::endl;
    return 1;
  }
  std::cout << "All P2.2 behavior tests passed" << std::endl;
  return 0;
}
