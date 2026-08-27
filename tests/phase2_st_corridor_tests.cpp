#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

#include "longitudinal_qp.h"
#include "planner.h"
#include "st_corridor.h"

namespace {

int failures = 0;
const std::uint64_t kCycle = 17;
const double kPathStartRoadS = 100.0;

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

PlanningSnapshot Planning(double speed_mps = 20.0) {
  PlanningSnapshot result;
  result.cycle = kCycle;
  result.target_lane = 1;
  result.frontier.road_s_unwrapped_m = kPathStartRoadS;
  result.frontier.d_m = 6.0;
  result.frontier.longitudinal.v = speed_mps;
  result.frontier.longitudinal.a = 0.0;
  result.frontier.state_source = PlanningStateSource::kTelemetry;
  return result;
}

BehaviorCandidate Candidate(PassingOrder order, bool has_front,
                            int front_id, bool has_rear, int rear_id,
                            std::uint64_t candidate_id = 101) {
  BehaviorCandidate result;
  result.candidate_id = candidate_id;
  result.behavior = BehaviorType::kChangeLeft;
  result.source_lane = 1;
  result.target_lane = 0;
  result.order = order;
  result.gap.target_lane = 0;
  result.gap.has_front_vehicle = has_front;
  result.gap.front_vehicle_id = front_id;
  result.gap.has_rear_vehicle = has_rear;
  result.gap.rear_vehicle_id = rear_id;
  result.coarse_admission.evaluated = true;
  result.coarse_admission.passed = true;
  result.status = BehaviorCandidateStatus::kCoarseAdmissionPassed;
  return result;
}

BehaviorPlanningSnapshot Behavior(const BehaviorCandidate &candidate) {
  BehaviorPlanningSnapshot result;
  result.cycle = kCycle;
  result.source_lane = 1;
  result.generated_candidate_count = 1;
  result.coarse_admitted_candidate_count = 1;
  result.candidates.push_back(candidate);
  return result;
}

SpatialPathCandidate Path(std::uint64_t candidate_id = 101,
                          double curvature_per_m = 0.0) {
  SpatialPathCandidate result;
  result.candidate_id = candidate_id;
  result.source_lane = 1;
  result.target_lane = 0;
  result.start_road_s_unwrapped_m = kPathStartRoadS;
  result.precheck.evaluated = true;
  result.precheck.passed = true;
  result.status = SpatialPathCandidateStatus::kPrecheckPassed;
  result.occupancy.target_lane_coverage_started = true;
  result.occupancy.target_lane_coverage_start_path_progress_m = 10.0;
  result.occupancy.source_lane_departed = true;
  result.occupancy.source_lane_departure_path_progress_m = 30.0;
  result.occupancy.lane_change_completed = true;
  result.occupancy.lane_change_completion_path_progress_m = 40.0;
  for (int index = 0; index <= 220; ++index) {
    SpatialPathGeometrySample sample;
    sample.construction_progress_m = static_cast<double>(index);
    sample.path_progress_m = static_cast<double>(index);
    sample.road_s_unwrapped_m =
        kPathStartRoadS + static_cast<double>(index);
    sample.d_m = index < 40 ? 6.0 - 0.1 * index : 2.0;
    sample.tangent_x = 1.0;
    sample.curvature_per_m = curvature_per_m;
    result.geometry.samples.push_back(sample);
  }
  result.geometry.sample_count = result.geometry.samples.size();
  result.geometry.path_extent_m = 220.0;
  result.geometry.construction_extent_m = 220.0;
  result.geometry.transition_completion_path_progress_m = 40.0;
  return result;
}

SpatialPathBatchSnapshot Paths(const SpatialPathCandidate &path) {
  SpatialPathBatchSnapshot result;
  result.cycle = kCycle;
  result.evaluated_candidate_count = 1;
  result.generated_candidate_count = 1;
  result.precheck_passed_candidate_count = 1;
  result.candidates.push_back(path);
  return result;
}

TrafficPredictionTrajectory Trajectory(
    int id, int lane, double initial_center_road_s_m,
    double speed_mps, double retained_prefix_s = 0.2,
    std::size_t last_node = 80) {
  TrafficPredictionTrajectory result;
  result.vehicle_id = id;
  result.source_track_valid = true;
  result.safety_admissible = true;
  result.vehicle_length_m = 4.8;
  result.vehicle_width_m = 2.0;
  for (std::size_t index = 0; index <= last_node; ++index) {
    PredictedTrafficOccupancy occupancy;
    const double relative_time_s = static_cast<double>(index) * 0.1;
    occupancy.prediction_time_s = retained_prefix_s + relative_time_s;
    occupancy.road_s_unwrapped_m =
        initial_center_road_s_m + speed_mps * relative_time_s;
    occupancy.road_s_rate_mps = speed_mps;
    occupancy.d_m = 2.0 + 4.0 * lane;
    occupancy.occupied_road_s_min_m =
        occupancy.road_s_unwrapped_m - 2.4;
    occupancy.occupied_road_s_max_m =
        occupancy.road_s_unwrapped_m + 2.4;
    occupancy.occupied_d_min_m = occupancy.d_m - 1.0;
    occupancy.occupied_d_max_m = occupancy.d_m + 1.0;
    occupancy.occupied_lane_mask =
        std::uint64_t(1) << static_cast<unsigned>(lane);
    result.occupancies.push_back(occupancy);
  }
  return result;
}

FullLaneTrafficPredictionSnapshot Prediction() {
  FullLaneTrafficPredictionSnapshot result;
  result.cycle = kCycle;
  result.time_step_s = 0.1;
  result.retained_prefix_duration_s = 0.2;
  result.planning_horizon_s = 8.0;
  result.required_coverage_s = 8.2;
  result.grid_coverage_s = 8.2;
  return result;
}

const STCorridorNode *FirstTargetLaneNode(
    const STCandidateEvaluation &candidate) {
  for (const STCorridorNode &node : candidate.corridor.nodes) {
    if ((node.ego_lane_mask & std::uint64_t(1)) != 0) {
      return &node;
    }
  }
  return nullptr;
}

void TestEmptyTrafficCorridorAndCandidateQp() {
  STCorridorPlanner planner;
  const BehaviorCandidate behavior_candidate =
      Candidate(PassingOrder::kStayBehindFront, false, 0, false, 0);
  STCandidateBatchSnapshot batch = planner.Build(
      Planning(), Behavior(behavior_candidate), Paths(Path()), Prediction());
  Expect(batch.candidates.size() == 1 &&
             batch.candidates.front().status ==
                 STCandidateStatus::kCorridorReady &&
             batch.candidates.front().corridor.nodes.size() == 81,
         "a prechecked path receives a complete empty-traffic corridor");
  if (batch.candidates.empty()) {
    return;
  }
  const STCandidateEvaluation &candidate = batch.candidates.front();
  Expect(candidate.corridor.nodes.front().ego_lane_mask ==
             (std::uint64_t(1) << 1) &&
             FirstTargetLaneNode(candidate) != nullptr,
         "reachable progress produces source-only then target occupancy");

  LongitudinalQpConfig qp_config;
  LongitudinalQp qp(qp_config);
  const LongitudinalQpInput input =
      planner.MakeQpInput(Planning(), candidate, 20.0);
  const LongitudinalQpResult qp_result = qp.Evaluate(input, qp.warm_start());
  planner.AttachQpResult(qp_result, &batch.candidates.front());
  planner.RefreshCounts(&batch);
  Expect(batch.candidates.front().status == STCandidateStatus::kQpSolved &&
             batch.candidates.front().qp_bounds_satisfied &&
             batch.qp_solved_candidate_count == 1,
         "candidate longitudinal QP respects corridor and curvature bounds");
}

void TestDualLaneIntersectionAndConstraintSources() {
  FullLaneTrafficPredictionSnapshot prediction = Prediction();
  prediction.trajectories.push_back(Trajectory(11, 1, 150.0, 0.0));
  prediction.trajectories.push_back(Trajectory(12, 0, 130.0, 0.0));
  prediction.trajectories.push_back(Trajectory(13, 0, 95.0, 0.0));
  const BehaviorCandidate behavior_candidate = Candidate(
      PassingOrder::kMergeAheadOfRear, true, 12, true, 13);
  STCorridorPlanner planner;
  const STCandidateBatchSnapshot batch = planner.Build(
      Planning(), Behavior(behavior_candidate), Paths(Path()), prediction);
  Expect(batch.candidates.size() == 1,
         "dual-lane candidate is evaluated");
  if (batch.candidates.empty()) {
    return;
  }
  const STCorridorNode *node = FirstTargetLaneNode(batch.candidates.front());
  Expect(node != nullptr &&
             node->ego_lane_mask ==
                 ((std::uint64_t(1) << 1) | std::uint64_t(1)) &&
             node->upper_source.present &&
             node->upper_source.vehicle_id == 12 &&
             node->lower_source.present &&
             node->lower_source.vehicle_id == 13 &&
             node->relevant_vehicle_count == 3,
         "dual occupancy intersects source/target traffic and records tight sources");
}

void TestThreePassingOrdersAreFixedBeforeQp() {
  FullLaneTrafficPredictionSnapshot prediction = Prediction();
  prediction.trajectories.push_back(Trajectory(21, 0, 130.0, 0.0));
  prediction.trajectories.push_back(Trajectory(22, 0, 95.0, 0.0));
  STCorridorPlanner planner;

  const STCandidateBatchSnapshot stay = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, true, 21,
                         false, 0)),
      Paths(Path()), prediction);
  const STCorridorNode *stay_node =
      FirstTargetLaneNode(stay.candidates.front());

  const STCandidateBatchSnapshot merge = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kMergeAheadOfRear, false, 0,
                         true, 22)),
      Paths(Path()), prediction);
  const STCorridorNode *merge_node =
      FirstTargetLaneNode(merge.candidates.front());

  const STCandidateBatchSnapshot wait = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kWaitBehindRear, true, 22,
                         false, 0)),
      Paths(Path()), prediction);
  const STCorridorNode *wait_node =
      FirstTargetLaneNode(wait.candidates.front());
  Expect(stay_node != nullptr && stay_node->upper_source.present &&
             stay_node->upper_source.vehicle_id == 21 &&
             merge_node != nullptr && merge_node->lower_source.present &&
             merge_node->lower_source.vehicle_id == 22 &&
             wait_node != nullptr && wait_node->upper_source.present &&
             wait_node->upper_source.vehicle_id == 22,
         "StayBehind, MergeAhead and WaitBehind become fixed upper/lower constraints");
}

void TestCorridorEmptyBeforeQp() {
  FullLaneTrafficPredictionSnapshot prediction = Prediction();
  prediction.trajectories.push_back(Trajectory(31, 0, 125.0, 0.0));
  prediction.trajectories.push_back(Trajectory(32, 0, 120.0, 0.0));
  STCorridorPlanner planner;
  const STCandidateBatchSnapshot batch = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kMergeAheadOfRear, true, 31,
                         true, 32)),
      Paths(Path()), prediction);
  Expect(batch.candidates.front().status ==
             STCandidateStatus::kCorridorEmpty &&
             batch.candidates.front().corridor.empty &&
             batch.corridor_empty_candidate_count == 1,
         "crossed fixed bounds reject CorridorEmpty without invoking QP");
}

void TestNonBoundaryIntruderStillConstrainsCorridor() {
  FullLaneTrafficPredictionSnapshot prediction = Prediction();
  prediction.trajectories.push_back(Trajectory(33, 0, 95.0, 0.0));
  prediction.trajectories.push_back(Trajectory(34, 0, 99.0, 0.0));
  STCorridorPlanner planner;
  const STCandidateBatchSnapshot batch = planner.Build(
      Planning(5.0),
      Behavior(Candidate(PassingOrder::kWaitBehindRear, true, 33,
                         false, 0)),
      Paths(Path()), prediction);
  bool observed_intruder_constraint = false;
  for (const STCorridorNode &node :
       batch.candidates.front().corridor.nodes) {
    observed_intruder_constraint =
        observed_intruder_constraint ||
        (node.upper_source.present &&
         node.upper_source.vehicle_id == 33 &&
         node.lower_source.present &&
         node.lower_source.vehicle_id == 34);
  }
  Expect(batch.candidates.front().status ==
             STCandidateStatus::kCorridorEmpty &&
             observed_intruder_constraint,
         "a non-boundary target vehicle remains a hard ST constraint after topology diagnostics are downgraded");
}

void TestCurvatureSpeedBudget() {
  STCorridorPlannerConfig config;
  config.lateral_acceleration_budget_mps2 = 3.2;
  STCorridorPlanner planner(config);
  const STCandidateBatchSnapshot batch = planner.Build(
      Planning(5.0),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path(101, 0.04)), Prediction());
  const double expected_limit = std::sqrt(3.2 / 0.04);
  Expect(batch.candidates.front().status ==
             STCandidateStatus::kCorridorReady &&
             std::fabs(batch.candidates.front()
                           .speed_budget.minimum_speed_limit_mps -
                       expected_limit) < 1e-9 &&
             batch.candidates.front()
                     .speed_budget.limiting_abs_curvature_per_m == 0.04,
         "per-node speed budget implements sqrt(a_lat/abs(kappa))");

  const STCandidateBatchSnapshot too_fast = planner.Build(
      Planning(10.0),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path(101, 0.04)), Prediction());
  Expect(too_fast.candidates.front().status ==
             STCandidateStatus::kCurvatureSpeedInfeasible,
         "frontier curvature-speed violation fails before QP");
}

void TestCompletionObservationHorizonGate() {
  STCorridorPlanner planner;
  STCandidateBatchSnapshot batch = planner.Build(
      Planning(5.0),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path()), Prediction());
  Expect(batch.candidates.size() == 1 &&
             batch.candidates.front().status ==
                 STCandidateStatus::kCorridorReady,
         "slow candidate reaches QP before the completion-time gate");
  if (batch.candidates.empty()) {
    return;
  }
  batch.candidates.front().lane_change_completion_progress_m = 200.0;
  LongitudinalQp qp;
  const LongitudinalQpInput input = planner.MakeQpInput(
      Planning(5.0), batch.candidates.front(), 5.0);
  const LongitudinalQpResult qp_result =
      qp.Evaluate(input, qp.warm_start());
  planner.AttachQpResult(qp_result, &batch.candidates.front());
  planner.RefreshCounts(&batch);
  Expect(qp_result.success &&
             batch.candidates.front().status ==
                 STCandidateStatus::kHorizonInsufficient &&
             !batch.candidates.front().lane_change_completion_observed &&
             !batch.candidates.front().lane_change_completed_in_time &&
             std::fabs(batch.candidates.front()
                           .latest_allowed_completion_time_s -
                       6.0) < 1e-12 &&
             batch.horizon_rejected_candidate_count == 1,
         "QP success is rejected when lane-change completion leaves less than the post-observation window");
}

void TestPredictionEvidenceAndFixedSideFailClosed() {
  FullLaneTrafficPredictionSnapshot incomplete = Prediction();
  incomplete.trajectories.push_back(
      Trajectory(41, 1, 140.0, 0.0, 0.2, 5));
  STCorridorPlanner planner;
  const STCandidateBatchSnapshot missing = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path()), incomplete);
  Expect(missing.candidates.front().status ==
             STCandidateStatus::kPredictionEvidenceIncomplete,
         "missing per-trajectory time coverage fails closed");

  FullLaneTrafficPredictionSnapshot wrong_lane = Prediction();
  wrong_lane.trajectories.push_back(Trajectory(43, 1, 130.0, 0.0));
  const STCandidateBatchSnapshot absent_target_boundary = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, true, 43,
                         false, 0)),
      Paths(Path()), wrong_lane);
  Expect(absent_target_boundary.candidates.front().status ==
             STCandidateStatus::kPredictionEvidenceIncomplete,
         "a declared target boundary present only in the source lane does not satisfy target-lane evidence");

  STManeuverSchedule committed_schedule;
  committed_schedule.committed_continuation = true;
  const STCandidateBatchSnapshot committed_departed_boundary = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, true, 43,
                         false, 0)),
      Paths(Path()), wrong_lane, committed_schedule);
  Expect(committed_departed_boundary.candidates.front().status ==
             STCandidateStatus::kCorridorReady,
         "a valid frozen boundary that left the target lane is conservatively "
         "reclassified during committed continuation");

  const STCandidateBatchSnapshot committed_missing_boundary = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, true, 43,
                         false, 0)),
      Paths(Path()), Prediction(), committed_schedule);
  Expect(committed_missing_boundary.candidates.front().status ==
             STCandidateStatus::kPredictionEvidenceIncomplete,
         "a frozen boundary missing from all lanes still fails a committed "
         "continuation closed");

  FullLaneTrafficPredictionSnapshot unsafe = Prediction();
  TrafficPredictionTrajectory unsafe_trajectory =
      Trajectory(44, 1, 140.0, 0.0);
  unsafe_trajectory.safety_admissible = false;
  unsafe.trajectories.push_back(unsafe_trajectory);
  const STCandidateBatchSnapshot unsafe_evidence = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path()), unsafe);
  Expect(unsafe_evidence.candidates.front().status ==
             STCandidateStatus::kPredictionEvidenceIncomplete,
         "a relevant non-admissible source track fails closed");

  FullLaneTrafficPredictionSnapshot fresh_replacement = Prediction();
  TrafficPredictionTrajectory replacement_trajectory =
      Trajectory(45, 1, 300.0, 20.0);
  replacement_trajectory.safety_admissible = false;
  fresh_replacement.trajectories.push_back(replacement_trajectory);
  const STCandidateBatchSnapshot committed_replacement = planner.Build(
      Planning(),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path()), fresh_replacement, committed_schedule);
  Expect(committed_replacement.candidates.front().status ==
             STCandidateStatus::kCorridorReady,
         "a currently valid replacement track remains conservative evidence "
         "for committed control while its admission age rebuilds");

  FullLaneTrafficPredictionSnapshot reused_ahead = Prediction();
  TrafficPredictionTrajectory reused_ahead_trajectory =
      Trajectory(46, 0, 150.0, 20.0);
  reused_ahead_trajectory.safety_admissible = false;
  reused_ahead.trajectories.push_back(reused_ahead_trajectory);
  const STCandidateBatchSnapshot reclassified_reused_boundary =
      planner.Build(
          Planning(),
          Behavior(Candidate(PassingOrder::kMergeAheadOfRear, false, 0,
                             true, 46)),
          Paths(Path()), reused_ahead, committed_schedule);
  bool reused_boundary_is_upper = false;
  if (!reclassified_reused_boundary.candidates.empty()) {
    for (const STCorridorNode &node :
         reclassified_reused_boundary.candidates.front().corridor.nodes) {
      if (node.upper_source.present &&
          node.upper_source.vehicle_id == 46) {
        reused_boundary_is_upper =
            reused_boundary_is_upper || !node.lower_source.present;
      }
    }
  }
  Expect(reclassified_reused_boundary.candidates.front().status ==
                 STCandidateStatus::kCorridorReady &&
             reused_boundary_is_upper,
         "a reused frozen rear ID still in the target lane but now observed "
         "ahead is constrained on its current safe side instead of retaining "
         "stale Gap topology");

  FullLaneTrafficPredictionSnapshot crossing = Prediction();
  crossing.trajectories.push_back(Trajectory(42, 1, 110.0, -5.0));
  const STCandidateBatchSnapshot fixed = planner.Build(
      Planning(5.0),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path()), crossing);
  bool always_upper = true;
  for (const STCorridorNode &node : fixed.candidates.front().corridor.nodes) {
    if (node.upper_source.present && node.upper_source.vehicle_id == 42) {
      always_upper = always_upper && !node.lower_source.present;
    }
  }
  Expect(always_upper && fixed.candidates.front().corridor.empty,
         "a crossing vehicle keeps its frontier side and cannot induce a QP topology switch");
}

void TestInitialProgressExclusionIsCorridorEmpty() {
  FullLaneTrafficPredictionSnapshot prediction = Prediction();
  prediction.trajectories.push_back(Trajectory(51, 1, 101.0, 0.0));
  STCorridorPlanner planner;
  const STCandidateBatchSnapshot batch = planner.Build(
      Planning(5.0),
      Behavior(Candidate(PassingOrder::kStayBehindFront, false, 0,
                         false, 0)),
      Paths(Path()), prediction);
  Expect(batch.candidates.front().status ==
             STCandidateStatus::kCorridorEmpty &&
             batch.candidates.front().corridor.first_empty_node == 0,
         "a front upper bound excluding p=0 is rejected before QP validation");

}

void TestSourceLaneConstraintsEndAtHardCompletionDeadline() {
  FullLaneTrafficPredictionSnapshot prediction = Prediction();
  // These source-lane vehicles form a valid corridor through t=5.9 s but
  // their conservative fixed-side envelopes cross at t=6.0 s.  Once the QP
  // is forced to complete the lane change at t=6.0 s, they are no longer
  // relevant during the target-lane observation window.
  prediction.trajectories.push_back(Trajectory(61, 1, 140.0, 5.0));
  prediction.trajectories.push_back(Trajectory(62, 1, 90.0, 11.0));
  prediction.trajectories.push_back(Trajectory(63, 0, 150.0, 10.0));
  STCorridorPlanner planner;
  STCandidateBatchSnapshot batch = planner.Build(
      Planning(10.0),
      Behavior(Candidate(PassingOrder::kStayBehindFront, true, 63,
                         false, 0)),
      Paths(Path()), prediction);
  Expect(batch.candidates.size() == 1 &&
             batch.candidates.front().status ==
                 STCandidateStatus::kCorridorReady,
         "source-lane envelope crossing after mandatory completion does not "
         "empty the target-only observation corridor");
  if (batch.candidates.empty() ||
      batch.candidates.front().corridor.nodes.size() != 81) {
    return;
  }
  const STCandidateEvaluation &candidate = batch.candidates.front();
  const STCorridorNode &before_release = candidate.corridor.nodes[59];
  const STCorridorNode &at_release = candidate.corridor.nodes[60];
  const STCorridorNode &terminal = candidate.corridor.nodes[80];
  Expect(before_release.upper_source.present &&
             before_release.upper_source.vehicle_id == 61 &&
             before_release.lower_source.present &&
             before_release.lower_source.vehicle_id == 62,
         "source-front and source-rear constraints remain hard before the "
         "completion deadline");
  const bool released_to_target_only =
      at_release.ego_lane_mask == std::uint64_t(1) &&
      at_release.lower_path_progress_m >= 40.0 &&
      !at_release.lower_source.present &&
      at_release.upper_source.present &&
      at_release.upper_source.vehicle_id == 63 &&
      terminal.ego_lane_mask == std::uint64_t(1);
  if (!released_to_target_only) {
    std::cerr << "release node mask=" << at_release.ego_lane_mask
              << " lower=" << at_release.lower_path_progress_m
              << " lower_source=" << at_release.lower_source.present << ':'
              << at_release.lower_source.vehicle_id
              << " upper_source=" << at_release.upper_source.present << ':'
              << at_release.upper_source.vehicle_id
              << " terminal_mask=" << terminal.ego_lane_mask << std::endl;
  }
  Expect(released_to_target_only,
         "the deadline imposes completion progress and switches the ST mask "
         "to target lane only");

  const LongitudinalQpInput input =
      planner.MakeQpInput(Planning(10.0), candidate, 10.0);
  Expect(input.minimum_progress_m[60] >= 40.0,
         "lane-change completion is a QP hard lower bound, not a post-check");
  LongitudinalQp qp;
  const LongitudinalQpResult qp_result = qp.Evaluate(input, qp.warm_start());
  planner.AttachQpResult(qp_result, &batch.candidates.front());
  Expect(batch.candidates.front().status == STCandidateStatus::kQpSolved &&
             batch.candidates.front().source_lane_departed_in_time &&
             batch.candidates.front().lane_change_completed_in_time,
         "the phase-aware hard bounds remain solvable and retain completion "
         "evidence");
}

void TestInvalidConfigurationRejected() {
  BehaviorCandidate malformed =
      Candidate(PassingOrder::kStayBehindFront, false, 0, false, 0);
  malformed.gap.target_lane = 2;
  STCorridorPlanner nominal;
  const STCandidateBatchSnapshot invalid_candidate = nominal.Build(
      Planning(), Behavior(malformed), Paths(Path()), Prediction());
  Expect(invalid_candidate.candidates.size() == 1 &&
             invalid_candidate.candidates.front().status ==
                 STCandidateStatus::kInvalidInput &&
             invalid_candidate.invalid_candidate_count == 1,
         "a behavior/path/gap identity mismatch is an explicit invalid candidate");

  STCorridorPlannerConfig config;
  config.maximum_corridor_nodes = 10;
  bool rejected = false;
  try {
    STCorridorPlanner planner(config);
    (void)planner;
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Expect(rejected, "corridor node capacity must cover the QP horizon");
}

} // namespace

int main() {
  RunTest(TestEmptyTrafficCorridorAndCandidateQp,
          "TestEmptyTrafficCorridorAndCandidateQp");
  RunTest(TestDualLaneIntersectionAndConstraintSources,
          "TestDualLaneIntersectionAndConstraintSources");
  RunTest(TestThreePassingOrdersAreFixedBeforeQp,
          "TestThreePassingOrdersAreFixedBeforeQp");
  RunTest(TestCorridorEmptyBeforeQp, "TestCorridorEmptyBeforeQp");
  RunTest(TestNonBoundaryIntruderStillConstrainsCorridor,
          "TestNonBoundaryIntruderStillConstrainsCorridor");
  RunTest(TestCurvatureSpeedBudget, "TestCurvatureSpeedBudget");
  RunTest(TestCompletionObservationHorizonGate,
          "TestCompletionObservationHorizonGate");
  RunTest(TestPredictionEvidenceAndFixedSideFailClosed,
          "TestPredictionEvidenceAndFixedSideFailClosed");
  RunTest(TestInitialProgressExclusionIsCorridorEmpty,
          "TestInitialProgressExclusionIsCorridorEmpty");
  RunTest(TestSourceLaneConstraintsEndAtHardCompletionDeadline,
          "TestSourceLaneConstraintsEndAtHardCompletionDeadline");
  RunTest(TestInvalidConfigurationRejected,
          "TestInvalidConfigurationRejected");
  if (failures != 0) {
    std::cerr << failures << " P2.4 ST-corridor assertion(s) failed"
              << std::endl;
    return 1;
  }
  std::cout << "All P2.4 ST-corridor tests passed" << std::endl;
  return 0;
}
