#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "map.h"
#include "planner.h"
#include "simulator_protocol.h"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

PlannerInput HighwayInput(double speed_mps, const MapData &map,
                          double road_s_m = 100.0) {
  PlannerInput input;
  input.ego.s = road_s_m;
  input.ego.d = 6.0;
  input.ego.speed_mph = speed_mps / 0.44704;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = geometry.x;
  input.ego.y = geometry.y;
  input.ego.yaw_deg =
      std::atan2(geometry.first_derivative_y, geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;
  return input;
}

DetectedVehicle TrafficVehicle(int id, double road_s, double d,
                               double speed_mps, const MapData &map) {
  const RoadGeometrySample geometry = EvaluateRoadGeometry(road_s, d, map);
  const double metric =
      std::hypot(geometry.first_derivative_x, geometry.first_derivative_y);
  DetectedVehicle vehicle;
  vehicle.id = static_cast<double>(id);
  vehicle.x = geometry.x;
  vehicle.y = geometry.y;
  vehicle.s = road_s;
  vehicle.d = d;
  vehicle.vx_mps = geometry.first_derivative_x / metric * speed_mps;
  vehicle.vy_mps = geometry.first_derivative_y / metric * speed_mps;
  return vehicle;
}

const PlannerOutput &DecisionOutput(const PlanningCycleDecision &decision) {
  if (decision.disposition == PlanDisposition::kValidatedCandidate &&
      decision.has_validated_candidate) {
    return decision.validated_candidate.output;
  }
  if (decision.disposition == PlanDisposition::kMinimumRiskDispatch &&
      decision.has_minimum_risk) {
    return decision.minimum_risk.output;
  }
  throw std::runtime_error("decision has no output");
}

void AdvanceInputBy(const PlannerOutput &output, std::size_t consumed_points,
                    const MapData &map, PlannerInput *input) {
  if (consumed_points == 0 || consumed_points >= output.next_x.size() ||
      output.next_x.size() != output.next_y.size()) {
    throw std::invalid_argument("invalid simulated consumption count");
  }
  const std::size_t ego_index = consumed_points - 1;
  input->ego.x = output.next_x[ego_index];
  input->ego.y = output.next_y[ego_index];
  const RoadProjection ego =
      ProjectCartesianToRoad(input->ego.x, input->ego.y, input->ego.s, map);
  input->ego.s = ego.road_s_unwrapped_m;
  input->ego.d = ego.d_m;
  input->ego.yaw_deg =
      std::atan2(output.next_y[ego_index + 1] - output.next_y[ego_index],
                 output.next_x[ego_index + 1] - output.next_x[ego_index]) *
      180.0 / 3.14159265358979323846;
  input->ego.speed_mph =
      std::hypot(output.next_x[ego_index + 1] - output.next_x[ego_index],
                 output.next_y[ego_index + 1] - output.next_y[ego_index]) /
      0.02 / 0.44704;
  input->previous_path_x.assign(output.next_x.begin() + consumed_points,
                                output.next_x.end());
  input->previous_path_y.assign(output.next_y.begin() + consumed_points,
                                output.next_y.end());
  const RoadProjection end = ProjectCartesianToRoad(
      input->previous_path_x.back(), input->previous_path_y.back(),
      input->ego.s + 10.0, map);
  input->end_path_s = end.road_s_unwrapped_m;
  input->end_path_d = end.d_m;
}

void AdvanceInput(const PlannerOutput &output, const MapData &map,
                  PlannerInput *input) {
  AdvanceInputBy(output, 1, map, input);
}

double RoundToMillimeter(double value) {
  return std::round(value * 1000.0) / 1000.0;
}

void AdvanceSimulatorLikeBy(const PlannerOutput &output,
                            std::size_t consumed_points,
                            const MapData &map, PlannerInput *input) {
  AdvanceInputBy(output, consumed_points, map, input);
  for (double &x : input->previous_path_x) {
    x = RoundToMillimeter(x);
  }
  for (double &y : input->previous_path_y) {
    y = RoundToMillimeter(y);
  }
  const RoadProjection end = ProjectCartesianToRoad(
      input->previous_path_x.back(), input->previous_path_y.back(),
      input->ego.s + 10.0, map);
  input->end_path_s = end.road_s_unwrapped_m;
  input->end_path_d = end.d_m;

  for (DetectedVehicle &vehicle : input->traffic) {
    const double speed_mps = std::hypot(vehicle.vx_mps, vehicle.vy_mps);
    vehicle.s += speed_mps * 0.02 *
                 static_cast<double>(consumed_points);
    const RoadGeometrySample geometry =
        EvaluateRoadGeometry(vehicle.s, vehicle.d, map);
    const double metric =
        std::hypot(geometry.first_derivative_x, geometry.first_derivative_y);
    vehicle.x = geometry.x;
    vehicle.y = geometry.y;
    vehicle.vx_mps = geometry.first_derivative_x / metric * speed_mps;
    vehicle.vy_mps = geometry.first_derivative_y / metric * speed_mps;
  }
}

void AdvanceSimulatorLike(const PlannerOutput &output, const MapData &map,
                          PlannerInput *input) {
  AdvanceSimulatorLikeBy(output, 1, map, input);
}

PlannerConfig ActiveTestConfig() {
  PlannerConfig config;
  config.operating_mode = PlannerOperatingMode::kBehaviorActive;
  config.active_behavior.behavior_planner.minimum_stable_observations = 1;
  config.active_behavior.behavior_planner.gap_stability_window_s = 0.0;
  config.active_behavior.behavior_planner.minimum_progress_benefit_m = 0.1;
  config.active_behavior.minimum_valid_proposal_cycles = 2;
  config.active_behavior.traffic_prediction
      .front_conservative_deceleration_mps2 = -1.0;
  config.active_behavior.traffic_prediction
      .rear_conservative_acceleration_mps2 = 0.01;
  config.active_behavior.traffic_prediction
      .longitudinal_uncertainty_growth_mps = 0.05;
  return config;
}

PlannerConfig ProductionActiveConfig() {
  PlannerConfig config;
  config.operating_mode = PlannerOperatingMode::kBehaviorActive;
  return config;
}

void MoveTrafficToLane(int vehicle_id, double d_m, const MapData &map,
                       PlannerInput *input) {
  for (DetectedVehicle &vehicle : input->traffic) {
    if (static_cast<int>(vehicle.id) != vehicle_id) {
      continue;
    }
    const double speed_mps = std::hypot(vehicle.vx_mps, vehicle.vy_mps);
    vehicle.d = d_m;
    const RoadGeometrySample geometry =
        EvaluateRoadGeometry(vehicle.s, vehicle.d, map);
    const double metric =
        std::hypot(geometry.first_derivative_x,
                   geometry.first_derivative_y);
    vehicle.x = geometry.x;
    vehicle.y = geometry.y;
    vehicle.vx_mps =
        geometry.first_derivative_x / metric * speed_mps;
    vehicle.vy_mps =
        geometry.first_derivative_y / metric * speed_mps;
    return;
  }
  throw std::runtime_error("missing traffic vehicle for lane move");
}

void RemoveTrafficVehicles(const std::vector<int> &vehicle_ids,
                           PlannerInput *input) {
  input->traffic.erase(
      std::remove_if(
          input->traffic.begin(), input->traffic.end(),
          [&vehicle_ids](const DetectedVehicle &vehicle) {
            return std::find(vehicle_ids.begin(), vehicle_ids.end(),
                             static_cast<int>(vehicle.id)) !=
                   vehicle_ids.end();
          }),
      input->traffic.end());
}

int LaneFromD(double d_m) {
  return std::max(0, std::min(2, static_cast<int>(d_m / 4.0)));
}

void ConfigureAlternatingTraffic(int current_lane, int target_lane,
                                 int generation, const MapData &map,
                                 PlannerInput *input) {
  const int blocked_lane = 3 - current_lane - target_lane;
  const double lane_d[3] = {2.0, 6.0, 10.0};
  const double ego_s = input->ego.s;
  const int id_base = 500 + generation * 20;
  input->traffic.clear();
  input->traffic.push_back(TrafficVehicle(
      id_base, NormalizeS(ego_s + 70.0, map.track_length),
      lane_d[current_lane], 14.0, map));
  input->traffic.push_back(TrafficVehicle(
      id_base + 1, NormalizeS(ego_s + 4.0, map.track_length),
      lane_d[blocked_lane], 18.0, map));
  for (int lane = 0; lane < 3; ++lane) {
    input->traffic.push_back(TrafficVehicle(
        id_base + 2 + lane * 3,
        NormalizeS(ego_s + 500.0 + 40.0 * lane, map.track_length),
        lane_d[lane], 22.0, map));
    input->traffic.push_back(TrafficVehicle(
        id_base + 3 + lane * 3,
        NormalizeS(ego_s - 500.0 - 40.0 * lane, map.track_length),
        lane_d[lane], 0.0, map));
    input->traffic.push_back(TrafficVehicle(
        id_base + 4 + lane * 3,
        NormalizeS(ego_s + 800.0 + 40.0 * lane, map.track_length),
        lane_d[lane], 22.0, map));
  }
  input->traffic.push_back(TrafficVehicle(
      id_base + 11, NormalizeS(ego_s - 800.0, map.track_length),
      lane_d[target_lane], 0.0, map));
}

void ConfigureNeutralTwelveVehicleTraffic(const MapData &map,
                                          PlannerInput *input) {
  const double lane_d[3] = {2.0, 6.0, 10.0};
  const double ego_s = input->ego.s;
  input->traffic.clear();
  for (int lane = 0; lane < 3; ++lane) {
    const double offsets[4] = {500.0, -500.0, 700.0, -700.0};
    for (int index = 0; index < 4; ++index) {
      input->traffic.push_back(TrafficVehicle(
          800 + lane * 4 + index,
          NormalizeS(ego_s + offsets[index], map.track_length),
          lane_d[lane], index % 2 == 0 ? 22.0 : 0.0, map));
    }
  }
}

void PrintCandidateEvidence(const ActiveBehaviorDiagnostics &diagnostics) {
  if (!diagnostics.has_latest_result) {
    std::cerr << "no active result" << std::endl;
    return;
  }
  const ActiveBehaviorCycleResult &cycle = diagnostics.latest_result;
  std::cerr << "phase=" << BehaviorManeuverPhaseName(cycle.phase_after)
            << " behavior=" << cycle.behavior.generated_candidate_count
            << " coarse=" << cycle.behavior.coarse_admitted_candidate_count
            << " paths=" << cycle.spatial_paths.precheck_passed_candidate_count
            << " st=" << cycle.st_candidates.qp_solved_candidate_count
            << " final=" << cycle.final_candidates.size() << std::endl;
  for (const BehaviorCandidate &candidate : cycle.behavior.candidates) {
    std::cerr << " behavior candidate " << candidate.candidate_id << ' '
              << BehaviorCandidateStatusName(candidate.status)
              << " lane=" << candidate.target_lane << " reasons=";
    for (CoarseAdmissionRejectionReason reason :
         candidate.coarse_admission.rejection_reasons) {
      std::cerr << CoarseAdmissionRejectionReasonName(reason) << '|';
    }
    std::cerr << std::endl;
  }
  for (const FinalBehaviorCandidate &candidate : cycle.final_candidates) {
    std::cerr << " final candidate " << candidate.candidate_id << ' '
              << FinalBehaviorCandidateStatusName(candidate.status)
              << " lane=" << candidate.target_lane
              << " detail=" << candidate.rejection_detail
              << " completion_progress="
              << candidate.st_candidate.lane_change_completion_progress_m
              << " completion_time="
              << candidate.st_candidate.lane_change_completion_time_s
              << " completion_deadline="
              << candidate.st_candidate.latest_allowed_completion_time_s
              << " departure_progress="
              << candidate.st_candidate.source_lane_departure_progress_m
              << " departure_time="
              << candidate.st_candidate.source_lane_departure_time_s
              << " departure_deadline="
              << candidate.st_candidate
                     .latest_allowed_source_lane_departure_time_s
              << " min_acceleration="
              << candidate.minimum_longitudinal_acceleration_mps2
              << " terminal_progress="
              << candidate.st_candidate.terminal_progress_m
              << " terminal_speed=" << candidate.st_candidate.terminal_speed_mps
              << std::endl;
  }
  for (const SpatialPathCandidate &candidate : cycle.spatial_paths.candidates) {
    std::cerr << " spatial candidate " << candidate.candidate_id << ' '
              << SpatialPathCandidateStatusName(candidate.status) << " a_lat="
              << candidate.precheck.maximum_estimated_lateral_acceleration_mps2
              << " kappa=" << candidate.precheck.maximum_abs_curvature_per_m
              << " speed=" << candidate.precheck.speed_upper_bound_mps
              << " jerk="
              << candidate.precheck.maximum_estimated_lateral_jerk_mps3
              << " reasons=";
    for (SpatialPathPrecheckReason reason :
         candidate.precheck.rejection_reasons) {
      std::cerr << SpatialPathPrecheckReasonName(reason) << '|';
    }
    std::cerr << std::endl;
  }
}

PlanningCycleDecision RunUntilCommit(PathPlanner *planner, PlannerInput *input,
                                     const MapData &map,
                                     std::size_t maximum_cycles) {
  PlanningCycleDecision decision;
  for (std::size_t cycle = 0; cycle < maximum_cycles; ++cycle) {
    decision = planner->PlanCycle(*input, map);
    if (decision.disposition != PlanDisposition::kValidatedCandidate) {
      return decision;
    }
    if (decision.validated_candidate.behavior_lane_change) {
      return decision;
    }
    AdvanceInput(DecisionOutput(decision), map, input);
  }
  return decision;
}

void TestStableBoundedGapCommitsValidatedLaneChange() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ActiveTestConfig());
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(100, 165.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(101, 260.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(102, 25.0, 2.0, 8.0, map));
  // Block the right lane so the safe bounded left Gap is deterministic.
  input.traffic.push_back(TrafficVehicle(103, 104.0, 10.0, 10.0, map));

  const PlanningCycleDecision decision =
      RunUntilCommit(&planner, &input, map, 6);
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.behavior_lane_change) {
    PrintCandidateEvidence(planner.behavior_diagnostics());
  }
  Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
             decision.has_validated_candidate &&
             decision.validated_candidate.behavior_lane_change,
         "a stable conservative target Gap reaches active control");
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.behavior_lane_change) {
    return;
  }
  const CandidatePlan &plan = decision.validated_candidate;
  Expect(plan.first_behavior_commit && plan.behavior_target_lane == 0,
         "the bounded left Gap commits exactly once to lane 0");
  Expect(plan.validation.valid && plan.qp.hard_safe,
         "active output passed the full Cartesian hard gate");
  const PlannerCycleDiagnostics &commit_diagnostics =
      planner.last_diagnostics();
  Expect(commit_diagnostics.behavior_evaluation_attempted &&
             commit_diagnostics.behavior_evaluation_succeeded &&
             commit_diagnostics.behavior_transaction_committed &&
             commit_diagnostics.behavior_final_valid_count >= 1 &&
             commit_diagnostics.behavior_has_best_valid_candidate &&
             !commit_diagnostics.behavior_candidates.empty(),
         "active commit exposes stage counts and candidate evidence");
  Expect(plan.full_trajectory.points.back().lateral.planned_d < 2.25,
         "the complete trajectory reaches the target-lane center");
  Expect(plan.full_trajectory.points.back().time_from_telemetry_s + 1e-9 >=
             plan.full_trajectory.planning_frontier_delay_s + 8.0,
         "the validated trajectory covers the full maneuver horizon");

  const std::uint64_t transition_id = plan.lateral_diagnostics.transition_id;
  AdvanceInput(plan.output, map, &input);
  const PlanningCycleDecision continuation = planner.PlanCycle(input, map);
  Expect(continuation.disposition == PlanDisposition::kValidatedCandidate &&
             continuation.has_validated_candidate &&
             continuation.validated_candidate.behavior_lane_change &&
             continuation.validated_candidate.behavior_continuation &&
             !continuation.validated_candidate.first_behavior_commit,
         "the cycle after commit continues the same active maneuver");
  if (continuation.has_validated_candidate &&
      continuation.validated_candidate.behavior_lane_change) {
    Expect(continuation.validated_candidate.lateral_diagnostics.transition_id ==
               transition_id,
           "Committed continuation preserves its path identity");
  }
}

void TestActiveAndLaneCruiseWarmStartsRemainIsolated() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ActiveTestConfig());
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(120, 165.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(121, 260.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(122, 25.0, 2.0, 8.0, map));
  input.traffic.push_back(TrafficVehicle(123, 104.0, 10.0, 10.0, map));

  QpWarmStartState committed_lane_cruise_warm_start;
  bool observed_commit = false;
  for (std::size_t cycle = 0; cycle < 8; ++cycle) {
    const PlanningCycleDecision decision = planner.PlanCycle(input, map);
    Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
               decision.has_validated_candidate,
           "warm-start isolation setup retains validated control");
    if (!decision.has_validated_candidate) {
      return;
    }
    const CandidatePlan &plan = decision.validated_candidate;
    if (plan.first_behavior_commit) {
      observed_commit = true;
      Expect(plan.next_state.qp_warm_start.primal ==
                 committed_lane_cruise_warm_start.primal,
             "an Active QP result cannot overwrite the independently "
             "committed LaneCruise warm start");
      break;
    }
    committed_lane_cruise_warm_start =
        plan.next_state.qp_warm_start;
    AdvanceInput(plan.output, map, &input);
  }
  Expect(observed_commit,
         "warm-start isolation regression reaches an Active commit");
}

void TestSimulatorLikeThreeLaneC3StitchKeepsCartesianJerkBounded() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ActiveTestConfig());
  PlannerInput input = HighwayInput(18.0, map, 986.0);
  input.traffic.push_back(TrafficVehicle(150, 1056.0, 6.0, 14.0, map));
  input.traffic.push_back(TrafficVehicle(151, 1146.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(152, 906.0, 2.0, 12.0, map));
  input.traffic.push_back(TrafficVehicle(153, 990.0, 10.0, 18.0, map));

  PlanningCycleDecision decision;
  std::size_t validation_rejected_count = 0;
  for (std::size_t cycle = 0; cycle < 12; ++cycle) {
    decision = planner.PlanCycle(input, map);
    validation_rejected_count +=
        planner.last_diagnostics().behavior_final_validation_rejected_count;
    if (decision.disposition != PlanDisposition::kValidatedCandidate ||
        (decision.has_validated_candidate &&
         decision.validated_candidate.behavior_lane_change)) {
      break;
    }
    AdvanceSimulatorLike(DecisionOutput(decision), map, &input);
  }
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.behavior_lane_change) {
    PrintCandidateEvidence(planner.behavior_diagnostics());
  }
  Expect(
      decision.disposition == PlanDisposition::kValidatedCandidate &&
          decision.has_validated_candidate &&
          decision.validated_candidate.behavior_lane_change,
      "a simulator-like three-lane candidate survives the inherited C3 stitch");
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.behavior_lane_change) {
    return;
  }
  const CandidatePlan &plan = decision.validated_candidate;
  Expect(plan.full_trajectory.retained_prefix_points == 15 &&
             std::fabs(plan.full_trajectory.planning_frontier_delay_s - 0.30) <
                 1e-12 &&
             plan.full_trajectory.points.size() > 15 &&
             std::fabs(plan.full_trajectory.points[15].time_from_telemetry_s -
                       0.32) < 1e-12 &&
             !plan.full_trajectory.points[15].retained_prefix,
         "the simulator-like regression checks the exact 15-point/0.32-second "
         "stitch boundary");
  Expect(validation_rejected_count == 0,
         "no candidate is rejected by final Cartesian validation before the "
         "simulator-like commit");
  Expect(plan.validation.maximum_jerk_mps3 <= 10.05,
         "the simulator-like C3 boundary keeps Cartesian jerk within the hard "
         "limit: " +
             std::to_string(plan.validation.maximum_jerk_mps3));
  const std::uint64_t transition_id =
      plan.lateral_diagnostics.transition_id;
  AdvanceSimulatorLike(plan.output, map, &input);
  const PlanningCycleDecision continuation = planner.PlanCycle(input, map);
  if (continuation.disposition != PlanDisposition::kValidatedCandidate ||
      !continuation.has_validated_candidate ||
      !continuation.validated_candidate.behavior_lane_change) {
    std::cerr << "high-speed continuation disposition="
              << static_cast<int>(continuation.disposition)
              << " ordinary=" << continuation.ordinary_evaluations.size()
              << " active_attempted="
              << planner.last_diagnostics().behavior_evaluation_attempted
              << " active_succeeded="
              << planner.last_diagnostics().behavior_evaluation_succeeded
              << " active_error="
              << planner.last_diagnostics().behavior_error_stage << ':'
              << planner.last_diagnostics().behavior_error_detail
              << " reset_reason="
              << static_cast<int>(
                     planner.last_diagnostics().state_reset_reason)
              << " phase="
              << planner.last_diagnostics().behavior_phase
              << std::endl;
    for (const CandidateEvaluation &evaluation :
         continuation.ordinary_evaluations) {
      std::cerr << " continuation candidate " << evaluation.candidate_id
                << " valid=" << evaluation.valid
                << " detail=" << evaluation.failure_detail << std::endl;
    }
    PrintCandidateEvidence(planner.behavior_diagnostics());
  }
  Expect(continuation.disposition == PlanDisposition::kValidatedCandidate &&
             continuation.has_validated_candidate &&
             continuation.validated_candidate.behavior_lane_change &&
             continuation.validated_candidate.behavior_continuation,
         "the high-speed cycle after commit keeps the validated active path");
  bool valid_lane_cruise_handoff = false;
  for (const CandidateEvaluation &evaluation :
       continuation.ordinary_evaluations) {
    valid_lane_cruise_handoff =
        valid_lane_cruise_handoff ||
        (evaluation.valid && evaluation.has_plan &&
         !evaluation.plan.behavior_lane_change &&
         evaluation.plan.validation.maximum_jerk_mps3 <= 10.05);
  }
  Expect(valid_lane_cruise_handoff,
         "the exact Active-to-LaneCruise boundary produces a validated, "
         "Cartesian-jerk-bounded same-cycle handoff candidate");
  if (continuation.has_validated_candidate &&
      continuation.validated_candidate.behavior_lane_change) {
    Expect(continuation.validated_candidate.lateral_diagnostics.transition_id ==
                   transition_id &&
               continuation.validated_candidate.validation.valid &&
               continuation.validated_candidate.validation.maximum_jerk_mps3 <=
                   10.05,
           "the committed high-speed continuation preserves identity and the "
           "Cartesian jerk gate");
  }
}

void TestProductionConfigurationMultiCycleClosedLoopLaneChange() {
  const MapData map = LoadMap("data/highway_map.csv");
  const PlannerConfig config = ProductionActiveConfig();
  Expect(config.active_behavior.traffic_prediction
                     .rear_conservative_acceleration_mps2 == 0.5 &&
             config.active_behavior.traffic_prediction
                     .longitudinal_uncertainty_growth_mps == 0.25 &&
             config.active_behavior.traffic_prediction
                     .longitudinal_acceleration_duration_s == 2.0 &&
             config.active_behavior.behavior_planner
                     .minimum_stable_observations == 3 &&
             config.active_behavior.behavior_planner
                     .gap_stability_window_s == 0.30,
         "production closed-loop regression uses unmodified traffic and "
         "behavior defaults");
  PathPlanner planner(config);
  PlannerInput input = HighwayInput(18.0, map, 986.0);
  input.traffic.push_back(TrafficVehicle(350, 1056.0, 6.0, 14.0, map));
  // Keep the target lane empty so this regression isolates lifecycle and
  // control ownership rather than introducing a longitudinal handoff brake.
  input.traffic.push_back(TrafficVehicle(353, 990.0, 10.0, 18.0, map));

  bool committed = false;
  bool target_only_observed = false;
  bool target_stable_observed = false;
  bool handoff_observed = false;
  bool post_handoff_control_observed = false;
  bool settling_with_lane_cruise_backup_observed = false;
  std::uint64_t transition_id = 0;
  std::size_t committed_control_cycles = 0;
  std::size_t target_only_cycles = 0;
  for (std::size_t cycle = 0; cycle < 80; ++cycle) {
    const PlanningCycleDecision decision = planner.PlanCycle(input, map);
    if (decision.disposition != PlanDisposition::kValidatedCandidate ||
        !decision.has_validated_candidate) {
      std::cerr << "production closed loop failed at cycle " << cycle
                << " disposition="
                << static_cast<int>(decision.disposition)
                << " behavior_error="
                << planner.last_diagnostics().behavior_error_stage << ':'
                << planner.last_diagnostics().behavior_error_detail
                << " phase="
                << BehaviorManeuverPhaseName(
                       planner.behavior_diagnostics().phase)
                << " completed="
                << planner.behavior_diagnostics().completed_maneuver_count
                << " cancelled="
                << planner.behavior_diagnostics().cancelled_proposal_count
                << " reset="
                << static_cast<int>(
                       planner.last_diagnostics().state_reset_reason)
                << std::endl;
      for (const CandidateEvaluation &evaluation :
           decision.ordinary_evaluations) {
        std::cerr << " production candidate " << evaluation.candidate_id
                  << " valid=" << evaluation.valid
                  << " behavior="
                  << (evaluation.has_plan &&
                              evaluation.plan.behavior_lane_change
                          ? "active"
                          : "cruise")
                  << " detail=" << evaluation.failure_detail << std::endl;
      }
      Expect(false,
             "production closed loop always retains validated control");
      return;
    }
    const CandidatePlan &plan = decision.validated_candidate;
    const ActiveBehaviorDiagnostics behavior =
        planner.behavior_diagnostics();
    const bool handoff_was_already_observed = handoff_observed;
    if (plan.first_behavior_commit) {
      committed = true;
      transition_id = plan.lateral_diagnostics.transition_id;
    }
    const bool handoff_this_cycle =
        committed && !handoff_observed &&
        behavior.phase == BehaviorManeuverPhase::kKeepLane &&
        behavior.completed_maneuver_count == 1;
    if (committed && !handoff_observed && !handoff_this_cycle) {
      Expect(plan.behavior_lane_change &&
                 plan.lateral_diagnostics.transition_id == transition_id,
             "production closed loop preserves committed path identity");
      ++committed_control_cycles;
    }
    if (behavior.has_latest_result &&
        behavior.latest_result.target_only) {
      target_only_observed = true;
      ++target_only_cycles;
      if (!handoff_this_cycle) {
        bool lane_cruise_backup_valid = false;
        for (const CandidateEvaluation &evaluation :
             decision.ordinary_evaluations) {
          lane_cruise_backup_valid =
              lane_cruise_backup_valid ||
              (evaluation.valid && evaluation.has_plan &&
               !evaluation.plan.behavior_lane_change);
        }
        settling_with_lane_cruise_backup_observed =
            settling_with_lane_cruise_backup_observed ||
            lane_cruise_backup_valid;
        Expect(plan.behavior_lane_change && plan.behavior_continuation &&
                   plan.lateral_diagnostics.transition_id == transition_id &&
                   behavior.latest_result.has_control_candidate &&
                   behavior.latest_result.final_candidates.size() == 1 &&
                   behavior.latest_result.final_candidates.front().status ==
                       FinalBehaviorCandidateStatus::kValid,
               "target-only settling keeps one validated committed control "
               "candidate");
      }
    }
    if (handoff_this_cycle) {
      handoff_observed = true;
      target_stable_observed = true;
      Expect(behavior.has_latest_result &&
                 behavior.latest_result.target_stable &&
                 !behavior.latest_result.has_control_candidate &&
                 !plan.behavior_lane_change && plan.validation.valid &&
                 plan.fallback_level == FallbackLevel::kNormal &&
                 plan.validation.maximum_jerk_mps3 <= 10.05,
             "target stability hands off only to a same-cycle validated "
             "normal-operational, jerk-bounded lane-cruise candidate");
    } else if (handoff_was_already_observed) {
      post_handoff_control_observed = true;
      Expect(!plan.behavior_lane_change && plan.validation.valid &&
                 plan.fallback_level == FallbackLevel::kNormal,
             "the cycle after handoff retains validated normal lane-cruise "
             "control");
      break;
    }
    // Match the production runtime's 15-point retained prefix. This reaches
    // the target-only boundary in a bounded number of control cycles and
    // exercises the same Cartesian seam as the logged failure.
    AdvanceSimulatorLikeBy(plan.output, committed ? 35 : 1, map, &input);
  }
  const bool full_lifecycle_observed =
      committed && committed_control_cycles >= 3 && target_only_observed &&
      target_only_cycles >=
          config.active_behavior.target_stable_cycles &&
      target_stable_observed && handoff_observed &&
      post_handoff_control_observed &&
      settling_with_lane_cruise_backup_observed;
  if (!full_lifecycle_observed) {
    std::cerr << "production lifecycle committed=" << committed
              << " committed_cycles=" << committed_control_cycles
              << " target_only=" << target_only_observed
              << " target_only_cycles=" << target_only_cycles
              << " target_stable=" << target_stable_observed
              << " handoff=" << handoff_observed
              << " post_handoff=" << post_handoff_control_observed
              << " validated_backup="
              << settling_with_lane_cruise_backup_observed
              << " final_phase="
              << BehaviorManeuverPhaseName(planner.behavior_diagnostics().phase)
              << std::endl;
  }
  Expect(full_lifecycle_observed,
         "production defaults retain validated control through commit, "
         "target-only settling, completion, and lane-cruise handoff");
}

void TestProductionCommittedLaneChangeSurvivesTargetRearIdReuse() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ProductionActiveConfig());
  PlannerInput input = HighwayInput(18.0, map, 986.0);
  input.traffic.push_back(TrafficVehicle(360, 1056.0, 6.0, 14.0, map));
  input.traffic.push_back(TrafficVehicle(361, 1146.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(362, 906.0, 2.0, 12.0, map));
  input.traffic.push_back(TrafficVehicle(363, 990.0, 10.0, 18.0, map));

  PlanningCycleDecision decision;
  std::uint64_t transition_id = 0;
  int target_rear_vehicle_id = -1;
  std::size_t continuation_cycles = 0;
  for (std::size_t cycle = 0; cycle < 80; ++cycle) {
    decision = planner.PlanCycle(input, map);
    if (decision.disposition != PlanDisposition::kValidatedCandidate ||
        !decision.has_validated_candidate) {
      Expect(false,
             "production ID-reuse setup reaches a committed lane change");
      return;
    }
    const CandidatePlan &plan = decision.validated_candidate;
    if (plan.first_behavior_commit) {
      transition_id = plan.lateral_diagnostics.transition_id;
      target_rear_vehicle_id = plan.behavior_gap.rear_vehicle_id;
    } else if (transition_id != 0 && plan.behavior_continuation) {
      ++continuation_cycles;
    }
    if (transition_id != 0 && continuation_cycles >= 3) {
      AdvanceSimulatorLike(plan.output, map, &input);
      break;
    }
    AdvanceSimulatorLike(plan.output, map, &input);
  }
  Expect(transition_id != 0 && continuation_cycles >= 3 &&
             target_rear_vehicle_id >= 0,
         "production ID-reuse regression establishes the same committed "
         "window as the runtime failure");
  if (transition_id == 0 || continuation_cycles < 3 ||
      target_rear_vehicle_id < 0) {
    return;
  }

  // Moving the frozen target rear boundary across a full lane exceeds the
  // production tracker's ID-reuse lateral tolerance.  The committed planner
  // must retain a safe continuous control path while the new track rebuilds
  // its minimum evidence age.
  MoveTrafficToLane(target_rear_vehicle_id, 6.0, map, &input);
  decision = planner.PlanCycle(input, map);
  if (decision.disposition != PlanDisposition::kValidatedCandidate ||
      !decision.has_validated_candidate ||
      !decision.validated_candidate.behavior_lane_change) {
    std::cerr << "production ID reuse disposition="
              << static_cast<int>(decision.disposition)
              << " behavior_error="
              << planner.last_diagnostics().behavior_error_stage << ':'
              << planner.last_diagnostics().behavior_error_detail
              << std::endl;
  }
  Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
             decision.has_validated_candidate &&
             decision.validated_candidate.behavior_lane_change &&
             decision.validated_candidate.behavior_continuation &&
             decision.validated_candidate.lateral_diagnostics.transition_id ==
                 transition_id,
         "temporary target-rear ID reuse cannot drop committed control or "
         "enter infrastructure failure");
}

void TestCommittedPredictionDropoutBuildsValidatedEmergencyContinuation() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig config = ActiveTestConfig();
  config.active_behavior.target_stable_cycles = 100;
  config.active_behavior.traffic_tracker.minimum_safety_track_age_s = 0.02;
  config.active_behavior.traffic_tracker.stale_after_s = 0.04;
  config.active_behavior.traffic_tracker.drop_after_s = 0.06;
  PathPlanner planner(config);
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(380, 165.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(381, 260.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(382, 25.0, 2.0, 8.0, map));
  input.traffic.push_back(TrafficVehicle(383, 104.0, 10.0, 10.0, map));

  PlanningCycleDecision decision = RunUntilCommit(&planner, &input, map, 8);
  Expect(decision.has_validated_candidate &&
             decision.validated_candidate.first_behavior_commit,
         "prediction-dropout regression establishes a committed maneuver");
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.first_behavior_commit) {
    return;
  }
  const GapId committed_gap = decision.validated_candidate.behavior_gap;
  std::vector<int> removed_ids;
  if (committed_gap.has_front_vehicle) {
    removed_ids.push_back(committed_gap.front_vehicle_id);
  }
  if (committed_gap.has_rear_vehicle) {
    removed_ids.push_back(committed_gap.rear_vehicle_id);
  }
  AdvanceSimulatorLikeBy(decision.validated_candidate.output, 35, map,
                         &input);
  RemoveTrafficVehicles(removed_ids, &input);

  bool observed_prediction_rejection = false;
  bool observed_valid_emergency = false;
  for (std::size_t cycle = 0; cycle < 10; ++cycle) {
    decision = planner.PlanCycle(input, map);
    Expect(decision.disposition != PlanDisposition::kInfrastructureFailure,
           "committed prediction dropout never loses control output");
    const ActiveBehaviorDiagnostics diagnostics =
        planner.behavior_diagnostics();
    if (diagnostics.has_latest_result) {
      observed_prediction_rejection =
          observed_prediction_rejection ||
          diagnostics.latest_result.st_candidates
                  .prediction_rejected_candidate_count > 0;
      for (const FinalBehaviorCandidate &candidate :
           diagnostics.latest_result.final_candidates) {
        observed_valid_emergency =
            observed_valid_emergency ||
            (candidate.emergency_continuation &&
             candidate.status == FinalBehaviorCandidateStatus::kValid &&
             candidate.validation.valid && candidate.qp.success &&
             candidate.qp.safety_policy ==
                 LongitudinalSafetyPolicy::kMaximumBraking);
      }
    }
    if (observed_prediction_rejection && observed_valid_emergency) {
      break;
    }
    if (decision.disposition == PlanDisposition::kInfrastructureFailure) {
      break;
    }
    AdvanceSimulatorLikeBy(DecisionOutput(decision), 35, map, &input);
  }
  Expect(observed_prediction_rejection,
         "removing a committed Gap boundary exposes prediction evidence "
         "loss in the refreshed ST corridor");
  Expect(observed_valid_emergency,
         "prediction evidence loss produces a fully validated maximum-"
         "braking continuation on the immutable committed geometry");
}

void TestTrackerUsesConsumedPointsThroughPlanner() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig config = ActiveTestConfig();
  config.active_behavior.traffic_tracker.minimum_safety_track_age_s = 0.0;
  config.active_behavior.traffic_tracker.stale_after_s = 0.30;
  config.active_behavior.traffic_tracker.drop_after_s = 1.0;
  PathPlanner planner(config);
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(390, 76.0, 2.0, 30.0, map));

  PlanningCycleDecision decision = planner.PlanCycle(input, map);
  Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
             decision.has_validated_candidate,
         "tracker elapsed-time integration establishes a committed output");
  if (!decision.has_validated_candidate) {
    return;
  }

  const PlannerOutput output = DecisionOutput(decision);
  AdvanceSimulatorLikeBy(output, 35, map, &input);
  input.traffic.clear();
  decision = planner.PlanCycle(input, map);

  const ActiveBehaviorDiagnostics diagnostics = planner.behavior_diagnostics();
  const TrackedVehicleState *missing = nullptr;
  if (diagnostics.has_latest_result) {
    for (const TrackedVehicleState &track :
         diagnostics.latest_result.tracking.tracks) {
      if (track.id == 390) {
        missing = &track;
        break;
      }
    }
  }
  Expect(missing != nullptr,
         "a 0.7 s missing track remains available until its drop deadline");
  if (missing != nullptr) {
    Expect(std::fabs(missing->time_since_update_s - 0.70) < 1e-12 &&
               std::fabs(missing->longitudinal_uncertainty_m - 1.20) < 1e-12 &&
               !missing->valid && !missing->safety_admissible &&
               diagnostics.latest_result.tracking.stale_track_count == 1,
           "PathPlanner passes 35 consumed points as 0.7 s to the tracker");
  }
}

void TestCommittedGeometryMinimumRiskPreventsManualOutput() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig config = ActiveTestConfig();
  config.active_behavior.target_stable_cycles = 100;
  PathPlanner planner(config);
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(390, 165.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(391, 260.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(392, 25.0, 2.0, 8.0, map));
  input.traffic.push_back(TrafficVehicle(393, 104.0, 10.0, 10.0, map));

  PlanningCycleDecision decision = RunUntilCommit(&planner, &input, map, 8);
  Expect(decision.has_validated_candidate &&
             decision.validated_candidate.first_behavior_commit,
         "committed minimum-risk regression establishes active ownership");
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.first_behavior_commit) {
    return;
  }
  AdvanceSimulatorLike(decision.validated_candidate.output, map, &input);
  const std::size_t collision_index =
      std::min<std::size_t>(10, input.previous_path_x.size() - 1);
  DetectedVehicle unavoidable;
  unavoidable.id = 399.0;
  unavoidable.x = input.previous_path_x[collision_index];
  unavoidable.y = input.previous_path_y[collision_index];
  const RoadProjection projection = ProjectCartesianToRoad(
      unavoidable.x, unavoidable.y, input.ego.s + 5.0, map);
  unavoidable.s = NormalizeS(projection.road_s_unwrapped_m,
                             map.track_length);
  unavoidable.d = projection.d_m;
  input.traffic.push_back(unavoidable);

  decision = planner.PlanCycle(input, map);
  if (decision.disposition != PlanDisposition::kMinimumRiskDispatch) {
    std::cerr << "committed minimum-risk disposition="
              << static_cast<int>(decision.disposition)
              << " infrastructure="
              << decision.infrastructure_failure.reason << std::endl;
    PrintCandidateEvidence(planner.behavior_diagnostics());
  }
  Expect(decision.disposition == PlanDisposition::kMinimumRiskDispatch &&
             decision.has_minimum_risk &&
             decision.minimum_risk.output.next_x.size() == 50 &&
             decision.minimum_risk.output.next_y.size() == 50 &&
             decision.minimum_risk.validation.HasOnlyCollisionViolations() &&
             decision.minimum_risk.risk.selected_reason.find(
                 "immutable committed geometry") != std::string::npos,
         "an unavoidable retained-prefix collision dispatches an explicit "
         "committed-geometry MinimumRisk trajectory instead of manual mode");
  const ActiveBehaviorDiagnostics diagnostics =
      planner.behavior_diagnostics();
  Expect(diagnostics.has_latest_result &&
             diagnostics.latest_result_committed &&
             diagnostics.latest_result.has_minimum_risk_candidate,
         "the committed MinimumRisk dispatch preserves its Active evidence");
}

void TestTargetOnlySettlingDoesNotBrakeForCommittedPathEnd() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig config = ProductionActiveConfig();
  // Hold active ownership after reaching the target so the regression cannot
  // pass merely because lane-cruise handoff happens before the finite path
  // tail becomes visible to the receding horizon.
  config.active_behavior.target_stable_cycles = 100;
  PathPlanner planner(config);
  PlannerInput input = HighwayInput(18.0, map, 986.0);
  input.traffic.push_back(TrafficVehicle(370, 1056.0, 6.0, 14.0, map));
  // Reproduce the logged spacious target gap: a faster front is far ahead
  // and a slower rear is far behind. Block the opposite adjacent lane so the
  // committed topology is deterministic.
  input.traffic.push_back(TrafficVehicle(371, 1146.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(372, 906.0, 2.0, 12.0, map));
  input.traffic.push_back(TrafficVehicle(373, 990.0, 10.0, 18.0, map));

  bool committed = false;
  std::size_t target_only_cycles = 0;
  double minimum_target_only_acceleration_mps2 =
      std::numeric_limits<double>::infinity();
  double minimum_terminal_path_reserve_m =
      std::numeric_limits<double>::infinity();
  for (std::size_t cycle = 0; cycle < 100 && target_only_cycles < 5;
       ++cycle) {
    const PlanningCycleDecision decision = planner.PlanCycle(input, map);
    if (decision.disposition != PlanDisposition::kValidatedCandidate ||
        !decision.has_validated_candidate) {
      Expect(false,
             "target-only path-end regression retains validated control");
      return;
    }
    const CandidatePlan &plan = decision.validated_candidate;
    committed = committed || plan.first_behavior_commit;
    const ActiveBehaviorDiagnostics behavior =
        planner.behavior_diagnostics();
    if (committed && behavior.has_latest_result &&
        behavior.latest_result.target_only &&
        plan.behavior_lane_change) {
      ++target_only_cycles;
      for (const LongitudinalState &state : plan.qp.trajectory.states) {
        minimum_target_only_acceleration_mps2 = std::min(
            minimum_target_only_acceleration_mps2, state.a);
      }
      if (!behavior.latest_result.final_candidates.empty()) {
        const FinalBehaviorCandidate &final =
            behavior.latest_result.final_candidates.front();
        minimum_terminal_path_reserve_m = std::min(
            minimum_terminal_path_reserve_m,
            final.spatial_path.geometry.path_extent_m -
                final.st_candidate.terminal_progress_m);
      }
    }
    AdvanceSimulatorLikeBy(plan.output, committed ? 15 : 1, map, &input);
  }

  Expect(committed && target_only_cycles == 5,
         "production closed loop exercises five target-only settling cycles");
  Expect(minimum_target_only_acceleration_mps2 > -0.75,
         "a spacious target gap cannot brake for the committed path end: " +
             std::to_string(minimum_target_only_acceleration_mps2));
  Expect(minimum_terminal_path_reserve_m > 5.0,
         "the committed target-lane tail stays beyond the fresh QP horizon: " +
             std::to_string(minimum_terminal_path_reserve_m));
}

void TestControlHistoryResetCancelsCommittedManeuver() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ActiveTestConfig());
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(180, 165.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(181, 260.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(182, 25.0, 2.0, 8.0, map));
  input.traffic.push_back(TrafficVehicle(183, 104.0, 10.0, 10.0, map));

  const PlanningCycleDecision committed =
      RunUntilCommit(&planner, &input, map, 7);
  Expect(committed.has_validated_candidate &&
             committed.validated_candidate.behavior_lane_change,
         "control-reset regression first establishes a committed maneuver");
  if (!committed.has_validated_candidate ||
      !committed.validated_candidate.behavior_lane_change) {
    return;
  }

  PlannerInput reset_input = HighwayInput(10.0, map, 400.0);
  const PlanningCycleDecision reset = planner.PlanCycle(reset_input, map);
  const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
  Expect(reset.disposition == PlanDisposition::kValidatedCandidate &&
             reset.has_validated_candidate &&
             !reset.validated_candidate.behavior_lane_change,
         "a discontinuous control history returns to validated lane cruise");
  if (reset.has_validated_candidate) {
    Expect(reset.validated_candidate.next_state.target_lane == 1,
           "history reset derives the control lane from current telemetry");
  }
  Expect(diagnostics.state_reset_reason ==
                 PlanningStateResetReason::kHistoryPositionMismatch &&
             diagnostics.behavior_tracker_reset_reason ==
                 "ControlHistoryReset" &&
             planner.behavior_diagnostics().phase ==
                 BehaviorManeuverPhase::kKeepLane &&
             planner.behavior_diagnostics().commit_cycle == 0,
         "history reset clears the old behavior commit while preserving the "
         "tracker's explicit reset evidence");
}

void TestOuterLaneZeroSpeedQuantizedFeedbackStaysInsideRoad() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ProductionActiveConfig());
  PlannerInput input = HighwayInput(0.0, map, 400.0);
  input.ego.d = 9.934;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample pose =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = pose.x;
  input.ego.y = pose.y;
  input.ego.yaw_deg =
      std::atan2(pose.first_derivative_y, pose.first_derivative_x) *
      180.0 / 3.14159265358979323846;

  PlanningCycleDecision decision = planner.PlanCycle(input, map);
  Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
             decision.has_validated_candidate &&
             decision.validated_candidate.validation.valid,
         "outer-lane zero-speed cold start produces validated control");
  if (!decision.has_validated_candidate) {
    return;
  }
  AdvanceSimulatorLike(decision.validated_candidate.output, map, &input);
  decision = planner.PlanCycle(input, map);
  if (decision.disposition != PlanDisposition::kValidatedCandidate) {
    for (const CandidateEvaluation &evaluation :
         decision.ordinary_evaluations) {
      std::cerr << "outer-lane candidate " << evaluation.candidate_id
                << " valid=" << evaluation.valid
                << " detail=" << evaluation.failure_detail << std::endl;
    }
  }
  Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
             decision.has_validated_candidate &&
             decision.validated_candidate.validation.valid &&
             decision.validated_candidate.validation.minimum_road_margin_m >=
                 -0.05,
         "millimeter-quantized low-speed feedback uses the exact tangent and "
         "does not create a false outer-road-boundary crossing");
}

void TestEmptyTargetLaneCommitsAndModeFallbackRemainsAvailable() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ActiveTestConfig());
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(200, 165.0, 6.0, 7.0, map));
  // No target-lane front or rear objects. Block only the right lane.
  input.traffic.push_back(TrafficVehicle(201, 103.0, 10.0, 10.0, map));

  const PlanningCycleDecision decision =
      RunUntilCommit(&planner, &input, map, 6);
  if (!decision.has_validated_candidate ||
      !decision.validated_candidate.behavior_lane_change) {
    PrintCandidateEvidence(planner.behavior_diagnostics());
  }
  Expect(decision.has_validated_candidate &&
             decision.validated_candidate.behavior_lane_change &&
             !decision.validated_candidate.behavior_gap.has_front_vehicle &&
             !decision.validated_candidate.behavior_gap.has_rear_vehicle,
         "an empty adjacent lane is represented by an unbounded safe Gap");

  planner.SetOperatingMode(PlannerOperatingMode::kLaneCruiseOnly);
  Expect(planner.operating_mode() == PlannerOperatingMode::kLaneCruiseOnly,
         "LaneCruiseOnly remains a public requested-mode fallback");
  if (planner.behavior_diagnostics().phase ==
      BehaviorManeuverPhase::kCommitted) {
    Expect(planner.effective_operating_mode() ==
               PlannerOperatingMode::kBehaviorActive,
           "a committed lane change delays effective fallback until settling");
  }

  PlanningCycleDecision continuation = decision;
  for (std::size_t cycle = 0;
       cycle < 24 && planner.effective_operating_mode() ==
                         PlannerOperatingMode::kBehaviorActive;
       ++cycle) {
    AdvanceInputBy(DecisionOutput(continuation), 25, map, &input);
    continuation = planner.PlanCycle(input, map);
    Expect(continuation.disposition == PlanDisposition::kValidatedCandidate,
           "requested fallback keeps a validated target-lane trajectory");
    if (continuation.disposition != PlanDisposition::kValidatedCandidate) {
      break;
    }
  }
  Expect(planner.effective_operating_mode() ==
                 PlannerOperatingMode::kLaneCruiseOnly &&
             planner.behavior_diagnostics().completed_maneuver_count == 1,
         "LaneCruiseOnly becomes effective after target-lane settling");
  if (continuation.has_validated_candidate) {
    Expect(!continuation.validated_candidate.behavior_lane_change,
           "post-settling output is owned by the lane-cruise fallback");
  }
}

void TestLaneCruiseComparisonInterfaceBeforeCommit() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig active_config = ActiveTestConfig();
  PlannerConfig cruise_config = active_config;
  cruise_config.operating_mode = PlannerOperatingMode::kLaneCruiseOnly;
  PathPlanner active(active_config);
  PathPlanner cruise(cruise_config);
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(250, 165.0, 6.0, 7.0, map));

  const PlanningCycleDecision active_decision = active.PlanCycle(input, map);
  const PlanningCycleDecision cruise_decision = cruise.PlanCycle(input, map);
  Expect(
      active_decision.has_validated_candidate &&
          cruise_decision.has_validated_candidate &&
          !active_decision.validated_candidate.behavior_lane_change &&
          active_decision.validated_candidate.output.next_x ==
              cruise_decision.validated_candidate.output.next_x &&
          active_decision.validated_candidate.output.next_y ==
              cruise_decision.validated_candidate.output.next_y,
      "before commitment active mode retains an exact lane-cruise comparator");

  active.SetOperatingMode(PlannerOperatingMode::kLaneCruiseOnly);
  Expect(active.operating_mode() == PlannerOperatingMode::kLaneCruiseOnly &&
             active.effective_operating_mode() ==
                 PlannerOperatingMode::kLaneCruiseOnly,
         "a pre-commit fallback request takes effect immediately");
}

void TestDiscretionaryComfortGateRejectsHardBrakingCommit() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig config = ActiveTestConfig();
  config.active_behavior.minimum_discretionary_acceleration_mps2 = -2.5;
  PathPlanner planner(config);
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(270, 128.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(271, 260.0, 2.0, 20.0, map));
  input.traffic.push_back(TrafficVehicle(272, 25.0, 2.0, 8.0, map));
  input.traffic.push_back(TrafficVehicle(273, 104.0, 10.0, 10.0, map));

  bool observed_comfort_rejection = false;
  for (std::size_t cycle = 0; cycle < 6; ++cycle) {
    const PlanningCycleDecision decision = planner.PlanCycle(input, map);
    const ActiveBehaviorDiagnostics diagnostics =
        planner.behavior_diagnostics();
    if (diagnostics.has_latest_result) {
      for (const FinalBehaviorCandidate &candidate :
           diagnostics.latest_result.final_candidates) {
        if (candidate.status ==
            FinalBehaviorCandidateStatus::kComfortRejected) {
          observed_comfort_rejection = true;
          Expect(candidate.minimum_longitudinal_acceleration_mps2 < -2.5 &&
                     candidate.rejection_detail.find(
                         "discretionary floor") != std::string::npos,
                 "comfort rejection retains the exact braking evidence");
        }
      }
    }
    Expect(decision.disposition == PlanDisposition::kValidatedCandidate,
           "a rejected discretionary maneuver preserves lane-cruise control");
    if (decision.disposition != PlanDisposition::kValidatedCandidate) {
      break;
    }
    if (decision.validated_candidate.behavior_lane_change) {
      double minimum_acceleration =
          decision.validated_candidate.qp.trajectory.states.front().a;
      for (const LongitudinalState &state :
           decision.validated_candidate.qp.trajectory.states) {
        minimum_acceleration = std::min(minimum_acceleration, state.a);
      }
      Expect(minimum_acceleration >= -2.5 - 1e-6,
             "a fresh lane change can commit only after satisfying the "
             "configured comfort floor");
      break;
    }
    AdvanceInput(DecisionOutput(decision), map, &input);
  }
  if (!observed_comfort_rejection) {
    PrintCandidateEvidence(planner.behavior_diagnostics());
  }
  Expect(observed_comfort_rejection,
         "the exact active QP blocks a hard-braking discretionary commit");
}

void TestUnsafeRearDoesNotTakeControl() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ActiveTestConfig());
  PlannerInput input = HighwayInput(10.0, map);
  input.traffic.push_back(TrafficVehicle(300, 135.0, 6.0, 7.0, map));
  input.traffic.push_back(TrafficVehicle(301, 92.0, 2.0, 25.0, map));
  input.traffic.push_back(TrafficVehicle(302, 103.0, 10.0, 10.0, map));
  for (std::size_t cycle = 0; cycle < 4; ++cycle) {
    const PlanningCycleDecision decision = planner.PlanCycle(input, map);
    Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
               !decision.validated_candidate.behavior_lane_change,
           "a fast close rear vehicle keeps control on lane cruise");
    AdvanceInput(DecisionOutput(decision), map, &input);
  }
  const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
  bool has_admission_rejection = false;
  bool has_source_front_evidence = false;
  for (const BehaviorCandidateDiagnostics &candidate :
       diagnostics.behavior_candidates) {
    has_admission_rejection = has_admission_rejection ||
                              !candidate.admission_rejection_reasons.empty();
    has_source_front_evidence =
        has_source_front_evidence ||
        (candidate.behavior != "KeepLane" &&
         candidate.has_source_front_margin &&
         candidate.source_front_limiting_vehicle_id == 300 &&
         !candidate.source_front_limiting_hypothesis.empty() &&
         candidate.gap_current_observation_valid &&
         candidate.source_front_limiting_occupied_min_road_s_m >
             candidate.source_front_limiting_ego_road_s_m);
  }
  Expect(diagnostics.behavior_evaluation_attempted &&
             diagnostics.behavior_evaluation_succeeded &&
             diagnostics.behavior_transaction_committed &&
             diagnostics.behavior_generated_candidate_count > 1 &&
             has_admission_rejection && has_source_front_evidence,
         "failed-closed behavior exposes rejection and source-front limiter "
         "evidence");
}

void TestThousandCycleTwelveVehicleAlternatingClosedLoop() {
  const MapData map = LoadMap("data/highway_map.csv");
  PathPlanner planner(ProductionActiveConfig());
  PlannerInput input = HighwayInput(18.0, map, 986.0);
  ConfigureAlternatingTraffic(1, 0, 0, map, &input);

  std::size_t commit_count = 0;
  std::uint64_t completed_count = 0;
  std::uint64_t previous_completed_count = 0;
  std::size_t minimum_risk_count = 0;
  bool infrastructure_failure = false;
  int scenario_current_lane = 1;
  int scenario_target_lane = 0;
  for (std::size_t cycle = 0; cycle < 1000; ++cycle) {
    PlannerOutput output;
    try {
      output = planner.Plan(input, map);
    } catch (const std::exception &error) {
      infrastructure_failure = true;
      std::cerr << "1000-cycle closed loop failed at cycle " << cycle
                << " detail=" << error.what()
                << " active="
                << planner.last_diagnostics().behavior_error_stage << ':'
                << planner.last_diagnostics().behavior_error_detail
                << std::endl;
      break;
    }
    const PlanningCycleDecision &decision = planner.last_decision();
    if (decision.disposition == PlanDisposition::kMinimumRiskDispatch) {
      ++minimum_risk_count;
    }
    Expect(output.next_x.size() == 50 && output.next_y.size() == 50,
           "every long closed-loop cycle satisfies the 50-point output "
           "contract");
    const std::string response = MakeControlMessage(output);
    Expect(response.find("42[\"control\",") == 0 &&
               response.find("42[\"manual\",") == std::string::npos,
           "valid long closed-loop telemetry always serializes a control "
           "response instead of manual mode");
    if (decision.has_validated_candidate) {
      const CandidatePlan &plan = decision.validated_candidate;
      commit_count += plan.first_behavior_commit ? 1 : 0;
      Expect(plan.validation.valid &&
                 plan.validation.maximum_jerk_mps3 <= 10.05,
             "every selected long closed-loop candidate remains inside the "
             "complete Cartesian hard gate");
    }

    const ActiveBehaviorDiagnostics behavior =
        planner.behavior_diagnostics();
    completed_count = behavior.completed_maneuver_count;
    if (completed_count > previous_completed_count) {
      previous_completed_count = completed_count;
      if (completed_count < 5) {
        scenario_current_lane =
            decision.has_validated_candidate
                ? decision.validated_candidate.next_state.target_lane
                : LaneFromD(input.end_path_d);
        scenario_target_lane = scenario_current_lane == 0 ? 1 : 0;
        ConfigureAlternatingTraffic(scenario_current_lane,
                                    scenario_target_lane,
                                    static_cast<int>(completed_count), map,
                                    &input);
      } else if (completed_count == 5) {
        ConfigureNeutralTwelveVehicleTraffic(map, &input);
      }
    }
    if (completed_count >= 5 && cycle % 50 == 0) {
      ConfigureNeutralTwelveVehicleTraffic(map, &input);
    } else if (completed_count < 5 && cycle > 0 && cycle % 50 == 0) {
      ConfigureAlternatingTraffic(scenario_current_lane,
                                  scenario_target_lane,
                                  static_cast<int>(completed_count), map,
                                  &input);
    }
    Expect(input.traffic.size() == 12,
           "long closed loop continuously carries twelve traffic vehicles");
    const bool maneuver_in_progress =
        behavior.phase == BehaviorManeuverPhase::kCommitted ||
        behavior.phase == BehaviorManeuverPhase::kSettling;
    AdvanceSimulatorLikeBy(output, maneuver_in_progress ? 35 : 1, map,
                           &input);
  }

  Expect(!infrastructure_failure,
         "1000-cycle closed loop never enters InfrastructureFailure/manual");
  if (commit_count < 5 || completed_count < 5 || minimum_risk_count != 0) {
    std::cerr << "1000-cycle summary commits=" << commit_count
              << " completed=" << completed_count
              << " minimum_risk=" << minimum_risk_count << std::endl;
  }
  Expect(commit_count >= 5 && completed_count >= 5,
         "1000-cycle closed loop completes at least five alternating left/"
         "right maneuvers");
  Expect(minimum_risk_count == 0,
         "nominal twelve-vehicle long closed loop needs no MinimumRisk "
         "dispatch");
}

} // namespace

int main(int argc, char **argv) {
  try {
    const bool long_only =
        argc == 2 && std::string(argv[1]) == "--long-only";
    if (long_only) {
      TestThousandCycleTwelveVehicleAlternatingClosedLoop();
    } else {
      TestStableBoundedGapCommitsValidatedLaneChange();
      TestActiveAndLaneCruiseWarmStartsRemainIsolated();
      TestSimulatorLikeThreeLaneC3StitchKeepsCartesianJerkBounded();
      TestProductionConfigurationMultiCycleClosedLoopLaneChange();
      TestProductionCommittedLaneChangeSurvivesTargetRearIdReuse();
      TestCommittedPredictionDropoutBuildsValidatedEmergencyContinuation();
      TestTrackerUsesConsumedPointsThroughPlanner();
      TestCommittedGeometryMinimumRiskPreventsManualOutput();
      TestTargetOnlySettlingDoesNotBrakeForCommittedPathEnd();
      TestControlHistoryResetCancelsCommittedManeuver();
      TestOuterLaneZeroSpeedQuantizedFeedbackStaysInsideRoad();
      TestEmptyTargetLaneCommitsAndModeFallbackRemainsAvailable();
      TestLaneCruiseComparisonInterfaceBeforeCommit();
      TestDiscretionaryComfortGateRejectsHardBrakingCommit();
      TestUnsafeRearDoesNotTakeControl();
    }
  } catch (const std::exception &error) {
    std::cerr << "FAILED: uncaught exception: " << error.what() << std::endl;
    ++failures;
  }
  if (failures != 0) {
    std::cerr << failures << " active behavior tests failed" << std::endl;
    return 1;
  }
  std::cout << "all active behavior tests passed" << std::endl;
  return 0;
}
