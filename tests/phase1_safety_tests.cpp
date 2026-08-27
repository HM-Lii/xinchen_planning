#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "longitudinal_qp.h"
#include "map.h"
#include "planner.h"
#include "trajectory_validator.h"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string &message) {
  Expect(std::fabs(actual - expected) <= tolerance, message);
}

void RunTest(void (*test)(), const std::string &name) {
  try {
    test();
  } catch (const std::exception &error) {
    Expect(false, name + " threw: " + error.what());
  }
}

void SetFrenetVelocity(DetectedVehicle *vehicle, double speed_mps,
                       const MapData &map) {
  const RoadGeometrySample road =
      EvaluateRoadGeometry(vehicle->s, vehicle->d, map);
  const double metric =
      std::hypot(road.first_derivative_x, road.first_derivative_y);
  vehicle->vx_mps = road.first_derivative_x * speed_mps / metric;
  vehicle->vy_mps = road.first_derivative_y * speed_mps / metric;
}

void SetFrenetMotion(DetectedVehicle *vehicle, double speed_mps,
                     double d_rate_mps, const MapData &map) {
  const RoadGeometrySample road =
      EvaluateRoadGeometry(vehicle->s, vehicle->d, map);
  const RoadGeometrySample center = EvaluateRoadGeometry(vehicle->s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometry(vehicle->s, 1.0, map);
  const double metric =
      std::hypot(road.first_derivative_x, road.first_derivative_y);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  vehicle->vx_mps =
      road.first_derivative_x * speed_mps / metric + normal_x * d_rate_mps;
  vehicle->vy_mps =
      road.first_derivative_y * speed_mps / metric + normal_y * d_rate_mps;
}

PlannerInput HighwayInput(double speed_mps, const MapData &map) {
  PlannerInput input;
  input.ego.s = 100.0;
  input.ego.d = 6.0;
  input.ego.speed_mph = speed_mps / 0.44704;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = geometry.x;
  input.ego.y = geometry.y;
  input.ego.yaw_deg =
      std::atan2(geometry.first_derivative_y,
                 geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;
  return input;
}

PlannerInput HighwayInputWithHistory(std::size_t history_points,
                                     const MapData &map) {
  PlannerInput input = HighwayInput(10.0, map);
  double road_s = input.ego.s;
  for (std::size_t index = 0; index < history_points; ++index) {
    road_s = AdvanceRoadParameter(road_s, 0.2, input.ego.d, map);
    const RoadGeometrySample point =
        EvaluateRoadGeometry(road_s, input.ego.d, map);
    input.previous_path_x.push_back(point.x);
    input.previous_path_y.push_back(point.y);
  }
  input.end_path_s = road_s;
  input.end_path_d = input.ego.d;
  return input;
}

DetectedVehicle VehicleAt(double id, double road_s, double d,
                          double speed_mps, const MapData &map) {
  DetectedVehicle vehicle;
  vehicle.id = id;
  vehicle.s = road_s;
  vehicle.d = d;
  const RoadGeometrySample geometry = EvaluateRoadGeometry(road_s, d, map);
  vehicle.x = geometry.x;
  vehicle.y = geometry.y;
  SetFrenetVelocity(&vehicle, speed_mps, map);
  return vehicle;
}

void TestQpPoliciesAndTransactionalWarmStart() {
  LongitudinalQpConfig config;
  config.maximum_speed_mps = 20.0;
  LongitudinalQp qp(config);

  LongitudinalQpInput input;
  input.initial_speed_mps = 20.0;
  input.reference_speed_mps.assign(config.horizon_steps + 1, 20.0);
  PredictedObstacle lead;
  lead.id = 7.0;
  lead.relative_s = 10.0;
  lead.speed_mps = 20.0;
  input.obstacles.push_back(lead);

  const LongitudinalQpResult normal =
      qp.Evaluate(input, QpWarmStartState());
  Expect(!normal.success,
         "normal policy does not soften the operational headway boundary");
  Expect(qp.warm_start().primal.empty(),
         "failed evaluation cannot mutate the committed QP warm start");

  input.safety_policy = LongitudinalSafetyPolicy::kDegradedBraking;
  const LongitudinalQpResult degraded =
      qp.Evaluate(input, QpWarmStartState());
  Expect(degraded.success && degraded.hard_safe,
         "degraded policy softens headway while preserving physical safety");
  Expect(degraded.maximum_headway_slack_meters > 30.0,
         "degraded result reports its operational-margin slack");
  Expect(qp.warm_start().primal.empty(),
         "successful evaluation is still side-effect free");
  qp.CommitWarmStart(degraded);
  const QpWarmStartState committed = qp.warm_start();
  Expect(!committed.primal.empty(),
         "the selected QP result can be committed explicitly");

  input.obstacles.front().relative_s = 4.0;
  const LongitudinalQpResult overlapping = qp.Evaluate(input, committed);
  Expect(!overlapping.success,
         "physical body overlap remains infeasible in degraded mode");
  Expect(qp.warm_start().primal == committed.primal,
         "an infeasible later evaluation preserves the committed warm start");

  input.safety_policy = LongitudinalSafetyPolicy::kMaximumBraking;
  input.reference_speed_mps.assign(config.horizon_steps + 1, 0.0);
  input.allow_physical_collision_violation = true;
  const LongitudinalQpResult minimum_risk =
      qp.Evaluate(input, QpWarmStartState());
  Expect(minimum_risk.success && !minimum_risk.hard_safe &&
             minimum_risk.maximum_collision_violation_meters > 0.0,
         "minimum-risk QP keeps collision separation as an explicit "
         "violation variable");
}

void TestFixedPathRoadProjectionRoundTrip() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput input = HighwayInput(10.0, map);
  input.ego.d = 2.0;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = geometry.x;
  input.ego.y = geometry.y;
  input.ego.yaw_deg =
      std::atan2(geometry.first_derivative_y,
                 geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;

  PathStitcher stitcher;
  const FixedSpatialPath fixed = stitcher.Prepare(
      input, input.ego.s, input.ego.d, 6.0, 22.0, map,
      PathStitcherState(), {}, false);
  const double target_road_s =
      AdvanceRoadParameter(input.ego.s, 30.0, 6.0, map);
  const double path_progress =
      FixedPathProgressToRoadParameterDistance(fixed, target_road_s, map);
  LongitudinalState state;
  state.s = path_progress;
  const StitchedRoadPathResult sampled =
      stitcher.Sample(fixed, {state}, map);
  Expect(sampled.output_states.size() == 1,
         "fixed-path projection round trip returns one sampled state");
  if (!sampled.output_states.empty()) {
    ExpectNear(sampled.output_states.back().road_parameter_s, target_road_s,
               1e-5,
               "RoadS to PathProgress mapping integrates to the target "
               "road parameter");
  }
}

void TestBoundedPrefixAndFullHorizon() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerConfig disabled_config;
  disabled_config.retained_path.enabled = false;
  bool disabled_rejected = false;
  try {
    PathPlanner disabled(disabled_config);
    (void)disabled;
  } catch (const std::invalid_argument &) {
    disabled_rejected = true;
  }
  Expect(disabled_rejected,
         "production bounded-prefix retention cannot be disabled");

  PlannerConfig oversized_config;
  oversized_config.retained_path.maximum_points = 16;
  bool oversized_rejected = false;
  try {
    PathPlanner oversized(oversized_config);
    (void)oversized;
  } catch (const std::invalid_argument &) {
    oversized_rejected = true;
  }
  Expect(oversized_rejected,
         "production retained-prefix configuration cannot exceed 15 points");

  for (std::size_t boundary = 0; boundary <= 50; ++boundary) {
    const PlannerInput input = HighwayInputWithHistory(boundary, map);
    PathPlanner planner;
    const PlanningCycleDecision decision = planner.PlanCycle(input, map);
    Expect(decision.disposition == PlanDisposition::kValidatedCandidate &&
               decision.has_validated_candidate,
           "bounded-prefix boundary produces a validated lane-cruise plan");
    if (!decision.has_validated_candidate) {
      continue;
    }
    const CandidatePlan &plan = decision.validated_candidate;
    const std::size_t retained = std::min<std::size_t>(boundary, 15);
    Expect(plan.full_trajectory.retained_prefix_points == retained,
           "retained prefix is capped at the production 15-point limit");
    Expect(plan.full_trajectory.points.size() == retained + 400,
           "full candidate contains the retained prefix plus eight seconds");
    ExpectNear(plan.full_trajectory.points.back().time_from_telemetry_s,
               static_cast<double>(retained) * 0.02 + 8.0, 1e-9,
               "full candidate reaches the complete QP horizon");
    Expect(plan.validation.valid && plan.qp.hard_safe,
           "ordinary output passes the hard validator before commit");
    for (std::size_t index = 0; index < retained; ++index) {
      ExpectNear(plan.output.next_x[index], input.previous_path_x[index],
                 1e-12, "bounded retention preserves historical x exactly");
      ExpectNear(plan.output.next_y[index], input.previous_path_y[index],
                 1e-12, "bounded retention preserves historical y exactly");
    }
    Expect(planner.last_diagnostics().original_previous_path_size == boundary &&
               planner.last_diagnostics().retained_prefix_points == retained &&
               planner.last_diagnostics().full_trajectory_points ==
                   retained + 400,
           "phase-1 prefix and full-horizon evidence is observable");
  }

  PathPlanner exact_planner;
  PlannerInput first_input = HighwayInput(10.0, map);
  const PlanningCycleDecision first = exact_planner.PlanCycle(first_input, map);
  Expect(first.has_validated_candidate,
         "exact-inheritance setup produces its first plan");
  if (!first.has_validated_candidate) {
    return;
  }
  PlannerInput next_input = first_input;
  next_input.previous_path_x = first.validated_candidate.output.next_x;
  next_input.previous_path_y = first.validated_candidate.output.next_y;
  next_input.end_path_s =
      first.validated_candidate.next_state.output_lateral.back()
          .road_parameter_s;
  next_input.end_path_d =
      first.validated_candidate.next_state.output_lateral.back().planned_d;
  const PlanningCycleDecision next =
      exact_planner.PlanCycle(next_input, map);
  Expect(next.has_validated_candidate &&
             next.validated_candidate.full_trajectory.exact_retained_state &&
             next.validated_candidate.full_trajectory.retained_prefix_points ==
                 15,
         "aligned 50-point history restores the exact 15-point frontier");
  if (!next.has_validated_candidate) {
    return;
  }

  PlannerInput unchanged_input = next_input;
  unchanged_input.previous_path_x = next.validated_candidate.output.next_x;
  unchanged_input.previous_path_y = next.validated_candidate.output.next_y;
  unchanged_input.end_path_s =
      next.validated_candidate.next_state.output_lateral.back()
          .road_parameter_s;
  unchanged_input.end_path_d =
      next.validated_candidate.next_state.output_lateral.back().planned_d;
  const PlanningCycleDecision unchanged =
      exact_planner.PlanCycle(unchanged_input, map);
  Expect(unchanged.has_validated_candidate &&
             unchanged.validated_candidate.full_trajectory
                 .exact_retained_state &&
             !unchanged.validated_candidate.lateral_diagnostics.state_reset,
         "a second zero-consumption callback preserves exact stitching state");
  if (unchanged.has_validated_candidate) {
    for (std::size_t index = 0; index < 15; ++index) {
      ExpectNear(unchanged.validated_candidate.output.next_x[index],
                 unchanged_input.previous_path_x[index], 1e-12,
                 "zero-consumption callback preserves retained x exactly");
      ExpectNear(unchanged.validated_candidate.output.next_y[index],
                 unchanged_input.previous_path_y[index], 1e-12,
                 "zero-consumption callback preserves retained y exactly");
    }
  }
}

void TestFallbackDispositionsAndStateIsolation() {
  const MapData map = LoadMap("data/highway_map.csv");

  PlannerInput short_headway = HighwayInput(10.0, map);
  const double short_lead_s = AdvanceRoadParameter(
      short_headway.ego.s, 12.0, short_headway.ego.d, map);
  short_headway.traffic.push_back(
      VehicleAt(11.0, short_lead_s, 6.0, 10.0, map));
  PathPlanner degraded_planner;
  const PlanningCycleDecision degraded =
      degraded_planner.PlanCycle(short_headway, map);
  Expect(degraded.has_validated_candidate &&
             degraded.validated_candidate.fallback_level ==
                 FallbackLevel::kDegradedBraking,
         "short operational headway selects a validated degraded candidate");
  Expect(degraded.ordinary_evaluations.size() == 2 &&
             !degraded.ordinary_evaluations.front().valid &&
             degraded.validated_candidate.qp.maximum_headway_slack_meters >
                 0.0,
         "normal rejection and degraded slack remain explicit evidence");

  PlannerInput blocked = HighwayInput(20.0, map);
  const double blocked_lead_s =
      AdvanceRoadParameter(blocked.ego.s, 20.0, blocked.ego.d, map);
  blocked.traffic.push_back(
      VehicleAt(12.0, blocked_lead_s, 6.0, 0.0, map));
  PathPlanner emergency_planner;
  const PlanningCycleDecision emergency =
      emergency_planner.PlanCycle(blocked, map);
  Expect(emergency.disposition == PlanDisposition::kMinimumRiskDispatch &&
             emergency.has_minimum_risk &&
             !emergency.has_validated_candidate,
         "physically unavoidable collision uses the independent dispatch");
  if (emergency.has_minimum_risk) {
    Expect(emergency.ordinary_evaluations.size() == 3 &&
               !emergency.minimum_risk.obstacles.empty() &&
               emergency.minimum_risk.qp
                       .maximum_collision_violation_meters >
                   0.0 &&
               emergency.minimum_risk.validation
                   .HasOnlyCollisionViolations(),
           "minimum-risk evidence retains obstacles and stays distinct from "
           "hard-safe success");
    Expect(emergency.minimum_risk.output.next_x.size() == 50,
           "minimum-risk dispatch preserves the external output contract");
  }

  PlannerInput outside_road = HighwayInput(10.0, map);
  outside_road.ego.d = -0.5;
  outside_road.end_path_d = outside_road.ego.d;
  const RoadGeometrySample outside_geometry = EvaluateRoadGeometry(
      outside_road.ego.s, outside_road.ego.d, map);
  outside_road.ego.x = outside_geometry.x;
  outside_road.ego.y = outside_geometry.y;
  PathPlanner isolated_planner;
  const std::uint64_t generation_before =
      isolated_planner.committed_generation();
  const PlanningCycleDecision rejected =
      isolated_planner.PlanCycle(outside_road, map);
  Expect(rejected.disposition == PlanDisposition::kInfrastructureFailure &&
             rejected.ordinary_evaluations.size() == 3 &&
             isolated_planner.committed_generation() == generation_before,
         "hard-validator candidate rejection cannot commit control state");

  PlannerInput invalid = HighwayInput(10.0, map);
  invalid.ego.x = std::numeric_limits<double>::quiet_NaN();
  const PlanningCycleDecision infrastructure =
      isolated_planner.PlanCycle(invalid, map);
  Expect(infrastructure.disposition ==
             PlanDisposition::kInfrastructureFailure &&
             infrastructure.has_infrastructure_failure &&
             isolated_planner.committed_generation() == generation_before,
         "invalid evidence is classified as infrastructure failure without "
         "state mutation");

  const PlannerInput valid = HighwayInput(10.0, map);
  PathPlanner fresh_planner;
  const PlanningCycleDecision after_rejection =
      isolated_planner.PlanCycle(valid, map);
  const PlanningCycleDecision fresh = fresh_planner.PlanCycle(valid, map);
  Expect(after_rejection.has_validated_candidate &&
             fresh.has_validated_candidate,
         "planner recovers after side-effect-free rejected evaluations");
  if (after_rejection.has_validated_candidate && fresh.has_validated_candidate) {
    for (std::size_t index = 0; index < 50; ++index) {
      ExpectNear(after_rejection.validated_candidate.output.next_x[index],
                 fresh.validated_candidate.output.next_x[index], 1e-9,
                 "rejected evaluation leaves deterministic x output");
      ExpectNear(after_rejection.validated_candidate.output.next_y[index],
                 fresh.validated_candidate.output.next_y[index], 1e-9,
                 "rejected evaluation leaves deterministic y output");
    }
  }
}

MapData SquareMap() {
  MapData map;
  map.x = {0.0, 10.0, 10.0, 0.0};
  map.y = {0.0, 0.0, 10.0, 10.0};
  map.s = {0.0, 10.0, 20.0, 30.0};
  map.dx = {0.0, 1.0, 0.0, -1.0};
  map.dy = {-1.0, 0.0, 1.0, 0.0};
  map.track_length = 40.0;
  return map;
}

FullTrajectory StationaryTrajectory(const PlannerInput &input,
                                    const MapData &map,
                                    double horizon_s = 1.0) {
  FullTrajectory trajectory;
  trajectory.time_step_s = 0.02;
  trajectory.planning_horizon_s = horizon_s;
  const RoadGeometrySample geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  const std::size_t point_count =
      static_cast<std::size_t>(std::llround(horizon_s / 0.02));
  for (std::size_t index = 0; index < point_count; ++index) {
    TrajectoryPoint point;
    point.time_from_telemetry_s = 0.02 * static_cast<double>(index + 1);
    point.x = geometry.x;
    point.y = geometry.y;
    point.lateral.valid = true;
    point.lateral.road_parameter_s = input.ego.s;
    point.lateral.planned_d = input.ego.d;
    point.lateral.expected_x = point.x;
    point.lateral.expected_y = point.y;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

void TestSameLaneRearCollisionShield() {
  const MapData map = LoadMap("data/highway_map.csv");
  const PlannerInput stationary = HighwayInput(0.0, map);
  const FullTrajectory trajectory = StationaryTrajectory(stationary, map);

  TrajectoryValidationContext context;
  context.map = &map;
  context.trajectory = &trajectory;
  context.prediction_coverage_s = trajectory.planning_horizon_s;

  PlannerInput approaching_rear = stationary;
  approaching_rear.traffic.push_back(
      VehicleAt(31.0, 90.0, 6.0, 10.0, map));
  context.input = &approaching_rear;
  const ValidationResult shielded = TrajectoryValidator().Validate(context);
  Expect(shielded.valid &&
             !shielded.HasViolation(ViolationType::kCollision) &&
             shielded.collision_evidence.empty(),
         "a fully-behind same-lane vehicle is excluded from the complete "
         "future collision timeline");
  Expect(shielded.shielded_same_lane_rear_vehicle_ids.size() == 1 &&
             shielded.shielded_same_lane_rear_vehicle_ids.front() == 31.0,
         "rear shielding remains explicit in validator diagnostics");

  PlannerInput overlapping_rear = stationary;
  overlapping_rear.traffic.push_back(
      VehicleAt(32.0, 96.0, 6.0, 0.0, map));
  context.input = &overlapping_rear;
  const ValidationResult rear_overlap =
      TrajectoryValidator().Validate(context);
  Expect(rear_overlap.HasViolation(ViolationType::kCollision) &&
             rear_overlap.shielded_same_lane_rear_vehicle_ids.empty(),
         "a same-lane rear vehicle already overlapping the ego body is not "
         "shielded");

  PlannerInput overlapping_front = stationary;
  overlapping_front.traffic.push_back(
      VehicleAt(33.0, 104.0, 6.0, 0.0, map));
  context.input = &overlapping_front;
  const ValidationResult front_overlap =
      TrajectoryValidator().Validate(context);
  Expect(front_overlap.HasViolation(ViolationType::kCollision) &&
             front_overlap.shielded_same_lane_rear_vehicle_ids.empty(),
         "a same-lane front collision remains a hard violation");

  TrajectoryValidatorConfig adjacent_config;
  adjacent_config.physical_collision_margin_m = 0.4;
  PlannerInput adjacent_intrusion = stationary;
  adjacent_intrusion.traffic.push_back(
      VehicleAt(34.0, 100.0, 8.05, 0.0, map));
  context.input = &adjacent_intrusion;
  const ValidationResult adjacent_overlap =
      TrajectoryValidator(adjacent_config).Validate(context);
  Expect(adjacent_overlap.HasViolation(ViolationType::kCollision) &&
             adjacent_overlap.shielded_same_lane_rear_vehicle_ids.empty(),
         "an adjacent-lane body intrusion remains a hard violation");
}

void TestLateralCutInCollisionTimeline() {
  const MapData map = LoadMap("data/highway_map.csv");
  const PlannerInput stationary = HighwayInput(0.0, map);
  const FullTrajectory trajectory = StationaryTrajectory(stationary, map, 3.0);

  TrajectoryValidationContext context;
  context.map = &map;
  context.trajectory = &trajectory;
  context.prediction_coverage_s = trajectory.planning_horizon_s;

  const double longitudinal_speeds[] = {0.0, 0.25};
  for (std::size_t index = 0; index < 2; ++index) {
    PlannerInput cut_in = stationary;
    DetectedVehicle vehicle =
        VehicleAt(40.0 + static_cast<double>(index), stationary.ego.s, 10.0,
                  longitudinal_speeds[index], map);
    SetFrenetMotion(&vehicle, longitudinal_speeds[index], -2.0, map);
    cut_in.traffic.push_back(vehicle);
    context.input = &cut_in;

    const ValidationResult result = TrajectoryValidator().Validate(context);
    Expect(!result.valid && result.HasViolation(ViolationType::kCollision) &&
               !result.collision_evidence.empty(),
           longitudinal_speeds[index] == 0.0
               ? "pure lateral traffic is propagated into the ego lane"
               : "lateral traffic motion is preserved with longitudinal speed");
    if (!result.collision_evidence.empty()) {
      const CollisionEvidence &evidence = result.collision_evidence.front();
      ExpectNear(evidence.observed_d_rate_mps, -2.0, 1e-8,
                 "cut-in evidence preserves the observed lateral rate");
      Expect(std::fabs(evidence.relative_d_m) < 2.0,
             "the collision pose uses the time-aligned obstacle d");
    }
  }
}

void TestSweptCollisionAndBodyRoadBoundary() {
  const MapData map = SquareMap();
  PlannerInput input;
  input.ego.x = 0.0;
  input.ego.y = -6.0;
  input.ego.s = 0.0;
  input.ego.d = 6.0;
  input.ego.yaw_deg = 0.0;
  input.ego.speed_mph = 500.0 / 0.44704;
  input.traffic.push_back(VehicleAt(21.0, 5.0, 6.0, 0.0, map));
  input.traffic.push_back(VehicleAt(22.0, 5.0, 6.0, 0.0, map));

  FullTrajectory trajectory;
  trajectory.time_step_s = 0.02;
  trajectory.planning_horizon_s = 0.02;
  TrajectoryPoint endpoint;
  endpoint.time_from_telemetry_s = 0.02;
  endpoint.x = 10.0;
  endpoint.y = -6.0;
  endpoint.longitudinal.v = 500.0;
  endpoint.lateral.valid = true;
  endpoint.lateral.road_parameter_s = 10.0;
  endpoint.lateral.planned_d = 6.0;
  endpoint.lateral.expected_x = endpoint.x;
  endpoint.lateral.expected_y = endpoint.y;
  trajectory.points.push_back(endpoint);

  TrajectoryValidatorConfig config;
  config.maximum_speed_mps = 1000.0;
  config.minimum_acceleration_mps2 = -1000000.0;
  config.maximum_acceleration_mps2 = 1000000.0;
  config.maximum_jerk_mps3 = 1000000.0;
  config.maximum_cartesian_acceleration_mps2 = 1000000.0;
  config.maximum_cartesian_jerk_mps3 = 1000000.0;
  config.ego_length_m = 2.0;
  config.ego_width_m = 1.0;
  config.obstacle_length_m = 2.0;
  config.obstacle_width_m = 1.0;
  TrajectoryValidator validator(config);
  TrajectoryValidationContext context;
  context.input = &input;
  context.map = &map;
  context.trajectory = &trajectory;
  context.prediction_coverage_s = 0.02;
  const ValidationResult swept = validator.Validate(context);
  Expect(swept.HasViolation(ViolationType::kCollision),
         "swept validation detects an interval collision missed by endpoints");
  Expect(swept.collision_evidence.size() == 2,
         "validator retains first-overlap evidence for every collision object");
  if (swept.collision_evidence.size() == 2) {
    const CollisionEvidence &first = swept.collision_evidence.front();
    Expect(first.object_id == 21.0 &&
               first.check_kind == CollisionCheckKind::kSweptInterval &&
               first.point_index == 0,
           "collision evidence identifies the object and swept interval");
    ExpectNear(first.time_from_telemetry_s, 0.01, 1e-12,
               "swept evidence records midpoint time");
    ExpectNear(first.relative_road_s_m, 0.0, 1e-9,
               "collision evidence records longitudinal alignment");
    ExpectNear(first.relative_d_m, 0.0, 1e-9,
               "collision evidence records lateral alignment");
    Expect(first.overlap_m > 0.0 &&
               std::isfinite(first.center_distance_m),
           "collision evidence records overlap and predicted centers");
  }

  PlannerInput road_input = input;
  road_input.traffic.clear();
  road_input.ego.d = 0.5;
  FullTrajectory road_trajectory = trajectory;
  const RoadGeometrySample outside_body =
      EvaluateRoadGeometry(10.0, 0.5, map);
  road_trajectory.points.front().x = outside_body.x;
  road_trajectory.points.front().y = outside_body.y;
  road_trajectory.points.front().lateral.planned_d = 6.0;
  road_trajectory.points.front().lateral.expected_x = outside_body.x;
  road_trajectory.points.front().lateral.expected_y = outside_body.y;
  context.input = &road_input;
  context.trajectory = &road_trajectory;
  const ValidationResult road = validator.Validate(context);
  Expect(road.HasViolation(ViolationType::kRoadBoundary),
         "road validation checks the ego body extent, not only its center");

  context.prediction_coverage_s = 0.01;
  const ValidationResult short_prediction = validator.Validate(context);
  Expect(short_prediction.HasViolation(ViolationType::kPredictionCoverage),
         "validator rejects prediction that does not cover the full horizon");
}

} // namespace

int main() {
  RunTest(TestQpPoliciesAndTransactionalWarmStart,
          "TestQpPoliciesAndTransactionalWarmStart");
  RunTest(TestFixedPathRoadProjectionRoundTrip,
          "TestFixedPathRoadProjectionRoundTrip");
  RunTest(TestBoundedPrefixAndFullHorizon,
          "TestBoundedPrefixAndFullHorizon");
  RunTest(TestFallbackDispositionsAndStateIsolation,
          "TestFallbackDispositionsAndStateIsolation");
  RunTest(TestSameLaneRearCollisionShield,
          "TestSameLaneRearCollisionShield");
  RunTest(TestLateralCutInCollisionTimeline,
          "TestLateralCutInCollisionTimeline");
  RunTest(TestSweptCollisionAndBodyRoadBoundary,
          "TestSweptCollisionAndBodyRoadBoundary");
  if (failures != 0) {
    std::cerr << failures << " phase-1 assertion(s) failed" << std::endl;
    return 1;
  }
  std::cout << "All phase-1 safety tests passed" << std::endl;
  return 0;
}
