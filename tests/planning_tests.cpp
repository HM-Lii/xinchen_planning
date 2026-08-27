#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "helpers.h"
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

void ExpectNear(double actual, double expected, double tolerance,
                const std::string &message) {
  Expect(std::fabs(actual - expected) <= tolerance, message);
}

void RunTest(void (*test)(), const std::string &name) {
  const char *filter = std::getenv("PLANNING_TEST_FILTER");
  if (filter != nullptr && name != filter) {
    return;
  }
  try {
    test();
  } catch (const std::exception &error) {
    Expect(false, name + " threw: " + error.what());
  }
}

std::vector<std::string> ReadLines(const std::string &path) {
  std::ifstream stream(path.c_str());
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

std::size_t CsvFieldCount(const std::string &line) {
  return static_cast<std::size_t>(std::count(line.begin(), line.end(), ',')) +
         1;
}

void SetFrenetVelocity(DetectedVehicle *vehicle, double longitudinal_speed_mps,
                       double lateral_speed_mps, const MapData &map) {
  const RoadGeometrySample road =
      EvaluateRoadGeometry(vehicle->s, vehicle->d, map);
  const RoadGeometrySample center = EvaluateRoadGeometry(vehicle->s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometry(vehicle->s, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  const double metric =
      std::hypot(road.first_derivative_x, road.first_derivative_y);
  const double parameter_rate = longitudinal_speed_mps / metric;
  vehicle->vx_mps =
      road.first_derivative_x * parameter_rate + normal_x * lateral_speed_mps;
  vehicle->vy_mps =
      road.first_derivative_y * parameter_rate + normal_y * lateral_speed_mps;
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

struct FrenetProjection {
  FrenetProjection(double s_value = 0.0, double d_value = 0.0)
      : s(s_value), d(d_value) {}

  double s = 0.0;
  double d = 0.0;
};

FrenetProjection ProjectToRoad(double x, double y, double initial_s,
                               const MapData &map) {
  double s = NormalizeS(initial_s, map.track_length);
  double d = 0.0;
  for (int iteration = 0; iteration < 12; ++iteration) {
    const RoadGeometrySample center = EvaluateRoadGeometry(s, 0.0, map);
    const RoadGeometrySample unit_offset = EvaluateRoadGeometry(s, 1.0, map);
    const double normal_x = unit_offset.x - center.x;
    const double normal_y = unit_offset.y - center.y;
    const double normal_squared = normal_x * normal_x + normal_y * normal_y;
    d = ((x - center.x) * normal_x + (y - center.y) * normal_y) /
        normal_squared;

    const RoadGeometrySample road = EvaluateRoadGeometry(s, d, map);
    const double residual_x = road.x - x;
    const double residual_y = road.y - y;
    const double objective_derivative = residual_x * road.first_derivative_x +
                                        residual_y * road.first_derivative_y;
    const double objective_second_derivative =
        road.first_derivative_x * road.first_derivative_x +
        road.first_derivative_y * road.first_derivative_y +
        residual_x * road.second_derivative_x +
        residual_y * road.second_derivative_y;
    if (std::fabs(objective_second_derivative) < 1e-10) {
      break;
    }
    const double step = objective_derivative / objective_second_derivative;
    s = NormalizeS(s - step, map.track_length);
    if (std::fabs(step) < 1e-10) {
      break;
    }
  }

  const RoadGeometrySample center = EvaluateRoadGeometry(s, 0.0, map);
  const RoadGeometrySample unit_offset = EvaluateRoadGeometry(s, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  d = ((x - center.x) * normal_x + (y - center.y) * normal_y) /
      (normal_x * normal_x + normal_y * normal_y);
  return {s, d};
}

double RoundToMillimeter(double value) {
  return std::round(value * 1000.0) / 1000.0;
}

void TestMapBoundaries() {
  const MapData map = SquareMap();
  std::string error;
  Expect(ValidateMap(map, &error), "square map should be valid");

  MapData shifted_s_map = map;
  shifted_s_map.s.front() = 1.0;
  Expect(!ValidateMap(shifted_s_map, &error),
         "map must use zero as the Frenet origin");

  const auto at_zero = FrenetToCartesian(0.0, 2.0, map);
  const auto at_wrap = FrenetToCartesian(40.0, 2.0, map);
  const auto at_negative_wrap = FrenetToCartesian(-40.0, 2.0, map);
  ExpectNear(at_zero.first, 0.0, 1e-9, "s=0 x coordinate");
  ExpectNear(at_zero.second, -2.0, 1e-9, "s=0 d offset");
  ExpectNear(at_wrap.first, at_zero.first, 1e-9, "track end wraps x");
  ExpectNear(at_wrap.second, at_zero.second, 1e-9, "track end wraps y");
  ExpectNear(at_negative_wrap.first, at_zero.first, 1e-9,
             "negative full lap wraps x");

  const std::vector<double> legacy = getXY(0.0, 0.0, map.s, map.x, map.y);
  ExpectNear(legacy[0], 0.0, 1e-9, "legacy getXY handles s=0");
  ExpectNear(legacy[1], 0.0, 1e-9, "legacy getXY handles s=0 y");
}

void TestHighwayMapLoading() {
  const MapData map = LoadMap("data/highway_map.csv");
  Expect(map.x.size() > 100, "highway map loads all waypoints");
  Expect(map.spline_x_second.size() == map.x.size() &&
             map.spline_y_second.size() == map.y.size() &&
             map.spline_dx_second.size() == map.dx.size() &&
             map.spline_dy_second.size() == map.dy.size(),
         "highway map caches all periodic spline coefficients");
  ExpectNear(map.track_length, 6945.554, 0.1,
             "highway map closing segment defines the full lap");
  const auto before_wrap = FrenetToCartesian(map.track_length - 0.01, 6.0, map);
  const auto after_wrap = FrenetToCartesian(map.track_length + 0.01, 6.0, map);
  Expect(
      std::isfinite(before_wrap.first) && std::isfinite(before_wrap.second) &&
          std::isfinite(after_wrap.first) && std::isfinite(after_wrap.second),
      "highway map conversion remains finite across the lap boundary");
}

void TestHighwayMapSplineContinuity() {
  const MapData map = LoadMap("data/highway_map.csv");
  const double epsilon = 1e-5;
  double maximum_interpolation_error = 0.0;
  double maximum_first_derivative_jump = 0.0;
  double maximum_second_derivative_jump = 0.0;
  for (std::size_t index = 0; index < map.s.size(); ++index) {
    const double waypoint_s = map.s[index];
    const RoadGeometrySample waypoint =
        EvaluateRoadGeometry(waypoint_s, 0.0, map);
    maximum_interpolation_error = std::max(
        maximum_interpolation_error,
        std::hypot(waypoint.x - map.x[index], waypoint.y - map.y[index]));
    const RoadGeometrySample before =
        EvaluateRoadGeometry(waypoint_s - epsilon, 6.0, map);
    const RoadGeometrySample after =
        EvaluateRoadGeometry(waypoint_s + epsilon, 6.0, map);
    maximum_first_derivative_jump = std::max(
        maximum_first_derivative_jump,
        std::hypot(after.first_derivative_x - before.first_derivative_x,
                   after.first_derivative_y - before.first_derivative_y));
    maximum_second_derivative_jump = std::max(
        maximum_second_derivative_jump,
        std::hypot(after.second_derivative_x - before.second_derivative_x,
                   after.second_derivative_y - before.second_derivative_y));
  }
  Expect(maximum_interpolation_error < 1e-9,
         "periodic road spline interpolates every source waypoint");
  Expect(maximum_first_derivative_jump < 1e-4,
         "periodic road spline has continuous waypoint tangents");
  Expect(maximum_second_derivative_jump < 1e-4,
         "periodic road spline has continuous waypoint curvature");
}

void TestRoadArcLengthIndexRoundTrip() {
  const MapData map = LoadMap("data/highway_map.csv");
  const double starts[] = {100.0, map.track_length - 20.0};
  const double offsets[] = {2.0, 6.0, 10.0};
  const double distances[] = {0.0, 0.1, 1.0, 12.0, 80.0, 160.0};
  double maximum_error = 0.0;
  for (double start_s : starts) {
    for (double d : offsets) {
      const RoadArcLengthIndex index =
          BuildRoadArcLengthIndex(start_s, 160.0, d, map);
      for (double distance : distances) {
        const double road_s = RoadParameterAtArcLength(index, distance);
        const double recovered =
            RoadArcLength(start_s, road_s - start_s, d, map);
        maximum_error =
            std::max(maximum_error, std::fabs(recovered - distance));
      }
    }
  }
  Expect(maximum_error < 1e-5,
         "road arc-length inverse stays within 10 micrometers across lanes "
         "and wrap: error=" +
             std::to_string(maximum_error));

  const RoadArcLengthIndex bounded =
      BuildRoadArcLengthIndex(100.0, 10.0, 6.0, map);
  bool rejected_out_of_range = false;
  try {
    (void)RoadParameterAtArcLength(
        bounded, bounded.cumulative_arc_length_m.back() + 0.1);
  } catch (const std::out_of_range &) {
    rejected_out_of_range = true;
  }
  Expect(rejected_out_of_range,
         "road arc-length inverse rejects distances beyond its coverage");
}

std::string ValidTelemetryMessage() {
  return "42[\"telemetry\",{\"x\":0,\"y\":-6,\"s\":0,\"d\":6,"
         "\"yaw\":0,\"speed\":0,\"previous_path_x\":[],"
         "\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":6,"
         "\"sensor_fusion\":[[7,1,2,3,4,5,6]]}]";
}

void TestProtocolContract() {
  const SimulatorMessage valid = ParseSimulatorMessage(ValidTelemetryMessage());
  Expect(valid.kind == SimulatorMessageKind::kTelemetry,
         "valid telemetry should parse");
  Expect(valid.input.traffic.size() == 1,
         "sensor fusion row should be converted");
  ExpectNear(valid.input.traffic[0].vx_mps, 3.0, 1e-9,
             "sensor fusion velocity mapping");

  const SimulatorMessage ignored = ParseSimulatorMessage("42[\"ping\",{}]");
  Expect(ignored.kind == SimulatorMessageKind::kIgnore,
         "unknown event should be ignored");

  const SimulatorMessage malformed =
      ParseSimulatorMessage("42[\"telemetry\",{\"x\":0}]");
  Expect(malformed.kind == SimulatorMessageKind::kManual,
         "missing telemetry fields should select manual mode");

  bool rejected_bad_output = false;
  try {
    MakeControlMessage(PlannerOutput{});
  } catch (const std::invalid_argument &) {
    rejected_bad_output = true;
  }
  Expect(
      rejected_bad_output,
      "control serializer rejects output that violates the 50-point contract");
}

void TestCartesianRuntimeMonitor() {
  PlannerInput input;
  input.ego.x = 0.0;
  input.ego.y = 0.0;
  input.ego.yaw_deg = 0.0;
  input.ego.speed_mph = 10.0 / 0.44704;

  PlannerOutput smooth;
  smooth.next_x = {0.2, 0.4, 0.6, 0.8};
  smooth.next_y = {0.0, 0.0, 0.0, 0.0};
  const std::vector<CartesianKinematicSample> smooth_samples =
      ComputeCartesianKinematics(input, smooth, 0.02, 2);
  Expect(smooth_samples.size() == 4,
         "Cartesian monitor returns one sample per path point");
  for (const CartesianKinematicSample &sample : smooth_samples) {
    ExpectNear(sample.speed_mps, 10.0, 1e-9,
               "Cartesian monitor measures constant speed");
    ExpectNear(sample.acceleration_mps2, 0.0, 1e-7,
               "Cartesian monitor measures zero acceleration");
    if (sample.jerk_valid) {
      ExpectNear(sample.jerk_mps3, 0.0, 1e-5,
                 "Cartesian monitor measures zero jerk");
    }
  }
  Expect(smooth_samples[0].is_previous_path &&
             smooth_samples[1].is_previous_path &&
             !smooth_samples[2].is_previous_path,
         "Cartesian monitor marks the old/new path boundary");

  PlannerOutput discontinuous = smooth;
  discontinuous.next_x = {0.2, 0.4, 1.4, 1.6};
  LongitudinalQpResult qp_result;
  qp_result.success = true;
  qp_result.status = "solved";
  qp_result.trajectory.time_step_seconds = 0.1;
  qp_result.trajectory.states.resize(2);
  qp_result.trajectory.states[0].v = 10.0;
  qp_result.trajectory.states[1].s = 1.0;
  qp_result.trajectory.states[1].v = 10.0;
  std::vector<LongitudinalState> output_states(4);
  for (LongitudinalState &state : output_states) {
    state.v = 10.0;
  }
  PlannerMonitorLimits limits;
  limits.output_time_step_seconds = 0.02;
  limits.maximum_speed_mps = 22.0;
  limits.minimum_acceleration_mps2 = -5.0;
  limits.maximum_acceleration_mps2 = 3.0;
  limits.maximum_jerk_mps3 = 8.0;
  limits.maximum_cartesian_acceleration_mps2 = 10.0;
  limits.maximum_cartesian_jerk_mps3 = 10.0;
  const PlannerCycleDiagnostics diagnostics = BuildPlannerDiagnostics(
      7, input, discontinuous, 2, qp_result.trajectory.states[0], 0.0, 6.0, 6.0,
      {}, {10.0, 10.0}, qp_result, output_states, limits);
  Expect(!diagnostics.qp_speed_violation &&
             !diagnostics.qp_acceleration_violation &&
             !diagnostics.qp_jerk_violation &&
             !diagnostics.qp_collision_violation,
         "monitor keeps a compliant QP classified as compliant");
  Expect(diagnostics.cartesian_speed_violation &&
             diagnostics.cartesian_acceleration_violation &&
             diagnostics.cartesian_jerk_violation,
         "monitor detects an output-space path discontinuity");
  Expect(diagnostics.cartesian_maximum_speed.valid &&
             diagnostics.cartesian_maximum_speed.index == 2,
         "monitor reports the exact speed-spike index");
  Expect(diagnostics.new_path_junction_speed.valid &&
             diagnostics.new_path_junction_speed.index == 2,
         "monitor reports the old/new path junction index");

  const PlannerCycleDiagnostics lane_departure = BuildPlannerDiagnostics(
      8, input, smooth, 2, qp_result.trajectory.states[0], 0.0, 8.0, 6.0, {},
      {10.0, 10.0}, qp_result, output_states, limits);
  Expect(lane_departure.lane_deviation_violation &&
             lane_departure.HasViolation(),
         "monitor flags departure from the locked lane center");

  limits.time_headway_seconds = 1.5;
  limits.fixed_headway_gap_meters = 7.8;
  limits.collision_gap_meters = 4.8;
  PredictedObstacle short_headway_lead;
  short_headway_lead.id = 21.0;
  short_headway_lead.relative_s = 20.0;
  short_headway_lead.speed_mps = 10.0;
  const PlannerCycleDiagnostics short_headway = BuildPlannerDiagnostics(
      9, input, smooth, 2, qp_result.trajectory.states[0], 0.0, 6.0, 6.0,
      {short_headway_lead}, {10.0, 10.0}, qp_result, output_states, limits);
  Expect(short_headway.qp_minimum_headway_margin.valid &&
             short_headway.qp_minimum_headway_margin.value < 0.0,
         "monitor records a negative soft headway margin");
  Expect(short_headway.qp_minimum_collision_margin.valid &&
             short_headway.qp_minimum_collision_margin.value > 0.0 &&
             !short_headway.qp_collision_violation &&
             !short_headway.HasViolation(),
         "soft headway deficit is not classified as a hard violation");

  PredictedObstacle overlapping_lead = short_headway_lead;
  overlapping_lead.relative_s = 4.0;
  const PlannerCycleDiagnostics overlap = BuildPlannerDiagnostics(
      10, input, smooth, 2, qp_result.trajectory.states[0], 0.0, 6.0, 6.0,
      {overlapping_lead}, {10.0, 10.0}, qp_result, output_states, limits);
  Expect(overlap.qp_minimum_collision_margin.valid &&
             overlap.qp_minimum_collision_margin.value < 0.0 &&
             overlap.qp_collision_violation && overlap.HasViolation(),
         "monitor classifies body overlap as a hard collision violation");
}

void TestRuntimeMonitorStartsWithCleanCsvLogs() {
  const std::string directory = "monitor_test_logs";
  const std::string cycle_path = directory + "/planner_cycle.csv";
  const std::string point_path = directory + "/planner_points.csv";
  const std::string qp_path = directory + "/planner_qp.csv";
  const std::string control_candidate_path =
      directory + "/planner_control_candidates.csv";
  const std::string behavior_candidate_path =
      directory + "/planner_behavior_candidates.csv";
  const std::string collision_path =
      directory + "/planner_collision_events.csv";

  PlannerMonitorConfig config;
  config.enabled = true;
  config.write_csv = true;
  config.log_directory = directory;
  config.detail_csv_interval_cycles = 1;

  {
    PlannerRuntimeMonitor first_monitor(config);
    PlannerCycleDiagnostics first;
    first.cycle = 1;
    first.cartesian_samples.push_back(CartesianKinematicSample());
    first.qp_samples.push_back(QpNodeMonitorSample());
    first_monitor.Record(first);
  }
  {
    PlannerRuntimeMonitor second_monitor(config);
    PlannerCycleDiagnostics second;
    second.cycle = 2;
    second.cartesian_samples.push_back(CartesianKinematicSample());
    second.qp_samples.push_back(QpNodeMonitorSample());
    ControlCandidateDiagnostics control_candidate;
    control_candidate.candidate_id = 21;
    control_candidate.has_plan = true;
    control_candidate.failure_reason = "HardValidation";
    control_candidate.shielded_same_lane_rear_vehicle_ids = {7.0, 8.0};
    CollisionEventDiagnostics collision;
    collision.evidence.object_id = 99.0;
    collision.evidence.check_kind = CollisionCheckKind::kSweptInterval;
    collision.evidence.point_index = 7;
    collision.evidence.time_from_telemetry_s = 0.15;
    collision.evidence.box_separation_m = -0.2;
    collision.evidence.overlap_m = 0.2;
    collision.evidence.relative_road_s_m = 0.1;
    collision.evidence.relative_d_m = -0.1;
    collision.qp_relevant = false;
    control_candidate.collision_events.push_back(collision);
    second.control_candidates.push_back(control_candidate);
    BehaviorCandidateDiagnostics behavior_candidate;
    behavior_candidate.candidate_id = 31;
    behavior_candidate.behavior = "ChangeLeft";
    behavior_candidate.source_lane = 1;
    behavior_candidate.target_lane = 0;
    behavior_candidate.behavior_status = "CoarseAdmissionRejected";
    behavior_candidate.admission_evaluated = true;
    behavior_candidate.admission_rejection_reasons =
        "GapNotStable|RearTtcTooSmall|MergeCorridorBlocked";
    behavior_candidate.has_source_front_margin = true;
    behavior_candidate.minimum_source_front_margin_m = -1.25;
    behavior_candidate.source_front_limiting_vehicle_id = 10;
    behavior_candidate.source_front_limiting_hypothesis =
        "FrontConservativeBraking";
    behavior_candidate.source_front_risk_observed = true;
    behavior_candidate.first_source_front_risk_vehicle_id = 10;
    behavior_candidate.first_source_front_risk_hypothesis =
        "FrontConservativeBraking";
    behavior_candidate.target_kinematic_failure_observed = true;
    behavior_candidate.first_target_kinematic_failure_time_s = 4.3;
    behavior_candidate.first_target_propagated_state_count = 128;
    behavior_candidate.first_target_best_front_margin_m = -2.5;
    behavior_candidate.gap_current_observation_valid = true;
    behavior_candidate.topology_failure_observed = true;
    behavior_candidate.first_topology_failure_kind =
        "BoundariesNotAdjacent";
    behavior_candidate.first_topology_actual_front_present = true;
    behavior_candidate.first_topology_actual_front_vehicle_id = 30;
    behavior_candidate.expected_boundaries_reversed_observed = true;
    behavior_candidate.first_expected_boundaries_reversed_time_s = 3.8;
    behavior_candidate.merge_corridor_intrusion_observed = true;
    behavior_candidate.first_merge_corridor_intrusion_vehicle_id = 30;
    behavior_candidate.first_merge_corridor_intrusion_hypothesis =
        "LateralContinuation";
    behavior_candidate.merge_corridor_blocked_observed = true;
    behavior_candidate.first_merge_corridor_blocked_time_s = 4.3;
    behavior_candidate.first_merge_corridor_candidate_state_count = 128;
    behavior_candidate.first_merge_corridor_feasible_state_count = 0;
    behavior_candidate.first_merge_corridor_blocking_vehicle_id = 30;
    behavior_candidate.first_merge_corridor_blocking_hypothesis =
        "LateralContinuation";
    behavior_candidate.first_merge_corridor_blocking_margin_m = -3.5;
    behavior_candidate.st_status = "CorridorEmpty";
    behavior_candidate.final_status = "ValidationRejected";
    behavior_candidate.final_rejection_detail = "Collision";
    second.behavior_candidates.push_back(behavior_candidate);
    second.behavior_evaluation_attempted = true;
    second.behavior_evaluation_succeeded = true;
    second.behavior_transaction_committed = true;
    second.behavior_generated_candidate_count = 2;
    second.behavior_coarse_admitted_count = 0;
    second.shielded_same_lane_rear_vehicle_ids = {7.0, 8.0};
    second_monitor.Record(second);

  }

  const std::vector<std::string> cycle_lines = ReadLines(cycle_path);
  const std::vector<std::string> point_lines = ReadLines(point_path);
  const std::vector<std::string> qp_lines = ReadLines(qp_path);
  const std::vector<std::string> control_candidate_lines =
      ReadLines(control_candidate_path);
  const std::vector<std::string> behavior_candidate_lines =
      ReadLines(behavior_candidate_path);
  const std::vector<std::string> collision_lines = ReadLines(collision_path);
  Expect(cycle_lines.size() == 2,
         "a new monitor run replaces the previous cycle log");
  Expect(cycle_lines.size() == 2 &&
             cycle_lines[1].find(",2,") != std::string::npos,
         "the clean cycle log contains only the new run");
  Expect(point_lines.size() == 2 &&
             point_lines[1].find(",2,") != std::string::npos,
         "the clean point log contains only the new run");
  Expect(qp_lines.size() == 2 && qp_lines[1].find(",2,") != std::string::npos,
         "the clean QP log contains only the new run");
  Expect(control_candidate_lines.size() == 2 &&
             control_candidate_lines[1].find(",2,21,") != std::string::npos,
         "the control-candidate log records each evaluated policy");
  Expect(behavior_candidate_lines.size() == 2 &&
             behavior_candidate_lines[1].find(
                 ",2,31,ChangeLeft,1,0,") != std::string::npos &&
             behavior_candidate_lines[1].find(
                 "GapNotStable|RearTtcTooSmall|MergeCorridorBlocked") !=
                 std::string::npos &&
             behavior_candidate_lines[1].find(
                 "BoundariesNotAdjacent") != std::string::npos,
         "the behavior-candidate log records per-gate rejection evidence");
  Expect(behavior_candidate_lines.front().find(
             "minimum_source_front_margin_m") != std::string::npos &&
             behavior_candidate_lines.front().find(
                 "source_front_limiting_vehicle_id") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "first_topology_failure_time_s") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "first_topology_actual_front_vehicle_id") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "source_front_limiting_hypothesis") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "first_target_kinematic_failure_time_s") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "first_merge_corridor_intrusion_vehicle_id") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "expected_boundaries_reversed_observed") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "first_merge_corridor_blocked_time_s") !=
                 std::string::npos &&
             behavior_candidate_lines.front().find(
                 "first_merge_corridor_blocking_margin_m") !=
                 std::string::npos,
         "behavior-candidate CSV exposes per-hypothesis source, coherent reachability and topology intrusion evidence");
  Expect(cycle_lines.front().find("behavior_evaluation_attempted") !=
             std::string::npos &&
             cycle_lines.front().find("behavior_coarse_admitted_count") !=
                 std::string::npos,
         "cycle CSV exposes active evaluation health and stage counts");
  Expect(cycle_lines.front().find("shielded_same_lane_rear_count") !=
             std::string::npos &&
             cycle_lines[1].find(",2,7|8,") != std::string::npos &&
             control_candidate_lines.front().find(
                 "shielded_same_lane_rear_object_ids") !=
                 std::string::npos &&
             control_candidate_lines[1].find(",2,7|8,") !=
                 std::string::npos,
         "cycle and candidate CSVs expose shielded rear counts and IDs");
  Expect(collision_lines.size() == 2 &&
             collision_lines[1].find(",99,") != std::string::npos,
         "the collision-event log records per-object evidence");
  Expect(cycle_lines.size() == 2 &&
             CsvFieldCount(cycle_lines[0]) == CsvFieldCount(cycle_lines[1]),
         "cycle CSV header and data keep the same field count");
  Expect(point_lines.size() == 2 &&
             CsvFieldCount(point_lines[0]) == CsvFieldCount(point_lines[1]),
         "point CSV header and data keep the same field count");
  Expect(qp_lines.size() == 2 &&
             CsvFieldCount(qp_lines[0]) == CsvFieldCount(qp_lines[1]),
         "QP CSV header and data keep the same field count");
  Expect(control_candidate_lines.size() == 2 &&
             CsvFieldCount(control_candidate_lines[0]) ==
                 CsvFieldCount(control_candidate_lines[1]),
         "control-candidate CSV header and data keep the same field count");
  Expect(behavior_candidate_lines.size() == 2 &&
             CsvFieldCount(behavior_candidate_lines[0]) ==
                 CsvFieldCount(behavior_candidate_lines[1]),
         "behavior-candidate CSV header and data keep the same field count");
  Expect(collision_lines.size() == 2 &&
             CsvFieldCount(collision_lines[0]) ==
                 CsvFieldCount(collision_lines[1]),
         "collision CSV header and data keep the same field count");

  std::remove(cycle_path.c_str());
  std::remove(point_path.c_str());
  std::remove(qp_path.c_str());
  std::remove(control_candidate_path.c_str());
  std::remove(behavior_candidate_path.c_str());
  std::remove(collision_path.c_str());
  std::remove(directory.c_str());
}

void TestCartesianMonitorUsesAlignedStitchingFrame() {
  PlannerInput input;
  input.ego.x = 0.001;
  input.ego.y = 0.0;
  input.ego.yaw_deg = 0.0;
  input.ego.speed_mph = 10.0 / 0.44704;

  PlannerOutput output;
  output.next_x = {0.201, 0.401, 0.601, 0.801, 1.001, 1.201, 1.401};
  output.next_y.assign(output.next_x.size(), 0.0);
  input.previous_path_x.assign(output.next_x.begin(),
                               output.next_x.begin() + 4);
  input.previous_path_y.assign(4, 0.0);

  LongitudinalQpResult qp_result;
  qp_result.success = true;
  qp_result.status = "solved";
  qp_result.trajectory.time_step_seconds = 0.1;
  qp_result.trajectory.states.resize(2);
  qp_result.trajectory.states[0].v = 10.0;
  qp_result.trajectory.states[1].s = 1.0;
  qp_result.trajectory.states[1].v = 10.0;

  std::vector<LongitudinalState> output_states(output.next_x.size());
  for (LongitudinalState &state : output_states) {
    state.v = 10.0;
  }
  std::vector<LateralPathState> lateral_states(output.next_x.size());
  for (std::size_t index = 0; index < lateral_states.size(); ++index) {
    lateral_states[index].valid = true;
    lateral_states[index].expected_x =
        index < 4 ? 0.2 * static_cast<double>(index + 1) : output.next_x[index];
  }
  LateralStitchDiagnostics lateral;
  lateral.state_aligned = true;

  PlannerMonitorLimits limits;
  limits.output_time_step_seconds = 0.02;
  limits.maximum_speed_mps = 22.0;
  limits.minimum_acceleration_mps2 = -5.0;
  limits.maximum_acceleration_mps2 = 3.0;
  limits.maximum_jerk_mps3 = 8.0;
  limits.maximum_cartesian_acceleration_mps2 = 10.0;
  limits.maximum_cartesian_jerk_mps3 = 10.0;

  const PlannerCycleDiagnostics diagnostics = BuildPlannerDiagnostics(
      9, input, output, 4, qp_result.trajectory.states[0], 0.0, 6.0, 6.0, {},
      {10.0, 10.0}, qp_result, output_states, limits, lateral, lateral_states,
      true);
  Expect(diagnostics.historical_path_position_residual.valid,
         "monitor retains the raw history quantization residual");
  ExpectNear(diagnostics.historical_path_position_residual.value, 0.001, 1e-12,
             "monitor measures the history endpoint offset");
  Expect(diagnostics.cartesian_maximum_acceleration.valid &&
             diagnostics.cartesian_maximum_acceleration.value < 1e-6,
         "a shared stitching frame removes false Cartesian acceleration");
  Expect(diagnostics.cartesian_maximum_jerk.valid &&
             diagnostics.cartesian_maximum_jerk.value < 1e-5,
         "a shared stitching frame removes false Cartesian jerk");
  Expect(!diagnostics.cartesian_acceleration_violation &&
             !diagnostics.cartesian_jerk_violation,
         "history quantization alone does not trigger a violation");
}

void TestBaselinePlanner() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput empty_road;
  empty_road.ego.s = 100.0;
  empty_road.ego.d = 6.0;
  empty_road.end_path_s = empty_road.ego.s;
  empty_road.end_path_d = empty_road.ego.d;
  const RoadGeometrySample initial_geometry =
      EvaluateRoadGeometry(empty_road.ego.s, empty_road.ego.d, map);
  empty_road.ego.x = initial_geometry.x;
  empty_road.ego.y = initial_geometry.y;
  empty_road.ego.yaw_deg =
      std::atan2(initial_geometry.first_derivative_y,
                 initial_geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;
  PathPlanner planner;
  const PlannerOutput first = planner.Plan(empty_road, map);
  Expect(first.next_x.size() == 50, "planner returns 50 x points");
  Expect(first.next_y.size() == 50, "planner returns 50 y points");
  Expect(planner.reference_speed_mps() > 0.0 &&
             planner.reference_speed_mps() <= 3.0 + 1e-3,
         "one-second output respects the acceleration-limited QP ramp");
  for (std::size_t i = 0; i < first.next_x.size(); ++i) {
    Expect(std::isfinite(first.next_x[i]) && std::isfinite(first.next_y[i]),
           "planned coordinates are finite");
  }
  const PlannerCycleDiagnostics &first_diagnostics = planner.last_diagnostics();
  Expect(first_diagnostics.cycle == 1,
         "planner diagnostics use a monotonic cycle counter");
  Expect(first_diagnostics.cartesian_samples.size() == 50 &&
             first_diagnostics.qp_samples.size() == 81,
         "planner retains output-space and QP-space diagnostic samples");
  Expect(!first_diagnostics.qp_speed_violation &&
             !first_diagnostics.qp_acceleration_violation &&
             !first_diagnostics.qp_jerk_violation,
         "baseline QP diagnostics remain inside hard bounds");

  const PlannerCycleDiagnostics first_cycle = planner.last_diagnostics();
  PlannerInput with_history = empty_road;
  with_history.previous_path_x.assign(first.next_x.end() - 2,
                                      first.next_x.end());
  with_history.previous_path_y.assign(first.next_y.end() - 2,
                                      first.next_y.end());
  with_history.ego.x = first.next_x[47];
  with_history.ego.y = first.next_y[47];
  with_history.ego.s =
      first_cycle.output_lateral_states[47].road_parameter_s;
  with_history.ego.d = first_cycle.output_lateral_states[47].planned_d;
  with_history.ego.speed_mph =
      first_cycle.output_longitudinal_states[47].v / 0.44704;
  with_history.ego.yaw_deg =
      std::atan2(first.next_y[47] - first.next_y[46],
                 first.next_x[47] - first.next_x[46]) *
      180.0 / 3.14159265358979323846;
  with_history.end_path_s =
      first_cycle.output_lateral_states.back().road_parameter_s;
  with_history.end_path_d =
      first_cycle.output_lateral_states.back().planned_d;
  const PlannerOutput continued = planner.Plan(with_history, map);
  Expect(continued.next_x.size() == 50,
         "continued path is replenished to 50 points");
  ExpectNear(continued.next_x[0], with_history.previous_path_x[0], 1e-9,
             "first historical x point is preserved");
  ExpectNear(continued.next_y[1], with_history.previous_path_y[1], 1e-9,
             "historical y points are preserved");

  const std::string control = MakeControlMessage(continued);
  Expect(control.find("42[\"control\"") == 0,
         "planner output uses the control event contract");
}

void TestPlannerStartsWithHistory() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput input;
  input.ego.s = 100.0;
  input.ego.d = 6.0;
  input.ego.speed_mph = 10.0 / 0.44704;
  const RoadGeometrySample ego_geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = ego_geometry.x;
  input.ego.y = ego_geometry.y;
  input.ego.yaw_deg =
      std::atan2(ego_geometry.first_derivative_y,
                 ego_geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;
  double road_s = input.ego.s;
  for (int index = 0; index < 2; ++index) {
    road_s = AdvanceRoadParameter(road_s, 0.2, input.ego.d, map);
    const RoadGeometrySample point =
        EvaluateRoadGeometry(road_s, input.ego.d, map);
    input.previous_path_x.push_back(point.x);
    input.previous_path_y.push_back(point.y);
  }
  input.end_path_s = road_s;
  input.end_path_d = input.ego.d;

  PathPlanner planner;
  const PlannerOutput output = planner.Plan(input, map);
  Expect(output.next_x.size() == 50 && output.next_y.size() == 50,
         "a fresh planner accepts an existing previous path");
  ExpectNear(output.next_x[0], input.previous_path_x[0], 1e-12,
             "fresh-planner history remains unchanged");
  ExpectNear(output.next_y[1], input.previous_path_y[1], 1e-12,
             "fresh-planner history y remains unchanged");
}

void TestColdStartUsesHistoricalEndpointState() {
  const MapData map = LoadMap("data/highway_map.csv");
  constexpr double time_step = 0.02;
  constexpr double lane_d = 6.0;

  PlannerInput input;
  input.ego.s = 100.0;
  input.ego.d = lane_d;
  input.ego.speed_mph = 10.0 / 0.44704;
  const RoadGeometrySample ego_geometry =
      EvaluateRoadGeometry(input.ego.s, lane_d, map);
  input.ego.x = ego_geometry.x;
  input.ego.y = ego_geometry.y;
  input.ego.yaw_deg = std::atan2(ego_geometry.first_derivative_y,
                                 ego_geometry.first_derivative_x) *
                      180.0 / 3.14159265358979323846;

  double road_s = input.ego.s;
  for (std::size_t index = 0; index < 40; ++index) {
    const double speed = 10.0 + 2.0 * static_cast<double>(index + 1) / 40.0;
    road_s = AdvanceRoadParameter(road_s, speed * time_step, lane_d, map);
    const RoadGeometrySample point = EvaluateRoadGeometry(road_s, lane_d, map);
    input.previous_path_x.push_back(point.x);
    input.previous_path_y.push_back(point.y);
  }
  input.end_path_s = road_s;
  input.end_path_d = lane_d;

  PathPlanner planner;
  const PlannerOutput output = planner.Plan(input, map);
  const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
  const std::size_t junction = diagnostics.previous_path_size;
  const double incoming_speed =
      std::hypot(output.next_x[junction - 1] - output.next_x[junction - 2],
                 output.next_y[junction - 1] - output.next_y[junction - 2]) /
      time_step;
  const double junction_speed =
      std::hypot(output.next_x[junction] - output.next_x[junction - 1],
                 output.next_y[junction] - output.next_y[junction - 1]) /
      time_step;

  Expect(std::fabs(diagnostics.initial_speed_mps - incoming_speed) < 0.1,
         "cold start uses the historical endpoint speed");
  Expect(std::fabs(junction_speed - incoming_speed) < 0.15,
         "cold-start trajectory is speed-continuous at the history junction");
  Expect(std::fabs(diagnostics.cartesian_samples[junction]
                       .tangential_acceleration_mps2) < 5.0,
         "cold-start junction acceleration remains within the planner bound");
}

void TestPlannerStitchesFromHistoricalEndpoint() {
  const MapData map = LoadMap("data/highway_map.csv");
  const double start_s = 205.2936;
  const double lane_d = 6.0;
  const double parameter_step = 0.4;
  const double offset_x = 0.35;
  const double offset_y = -0.2;

  const auto point_at = [&](double s) {
    const auto road_point = FrenetToCartesian(s, lane_d, map);
    return std::make_pair(road_point.first + offset_x,
                          road_point.second + offset_y);
  };
  const std::pair<double, double> ego_point =
      point_at(start_s - 3.0 * parameter_step);
  const std::pair<double, double> first_history =
      point_at(start_s - 2.0 * parameter_step);
  const std::pair<double, double> second_history =
      point_at(start_s - parameter_step);
  const std::pair<double, double> history_endpoint = point_at(start_s);
  const double incoming_speed =
      std::hypot(history_endpoint.first - second_history.first,
                 history_endpoint.second - second_history.second) /
      0.02;

  PlannerInput input;
  input.ego.x = ego_point.first;
  input.ego.y = ego_point.second;
  input.ego.s = start_s - 3.0 * parameter_step;
  input.ego.d = lane_d;
  input.ego.yaw_deg = std::atan2(first_history.second - ego_point.second,
                                 first_history.first - ego_point.first) *
                      180.0 / 3.14159265358979323846;
  input.ego.speed_mph = incoming_speed / 0.44704;
  input.previous_path_x = {first_history.first, second_history.first,
                           history_endpoint.first};
  input.previous_path_y = {first_history.second, second_history.second,
                           history_endpoint.second};
  input.end_path_s = start_s;
  input.end_path_d = lane_d;

  const auto reconstructed = FrenetToCartesian(start_s, lane_d, map);
  Expect(std::hypot(history_endpoint.first - reconstructed.first,
                    history_endpoint.second - reconstructed.second) > 0.3,
         "stitch regression includes a substantial Frenet anchor mismatch");

  PathPlanner planner;
  const PlannerOutput output = planner.Plan(input, map);
  const std::size_t junction = input.previous_path_x.size();
  const double junction_speed =
      std::hypot(output.next_x[junction] - output.next_x[junction - 1],
                 output.next_y[junction] - output.next_y[junction - 1]) /
      0.02;
  Expect(std::fabs(junction_speed - incoming_speed) < 0.2,
         "new path speed is continuous with the historical endpoint");
  Expect(planner.last_diagnostics().new_path_junction_speed.valid &&
             planner.last_diagnostics().new_path_junction_speed.index ==
                 junction,
         "monitor identifies the stitched junction");
  Expect(
      planner.last_diagnostics().cartesian_samples[junction].acceleration_mps2 <
          10.0,
      "historical endpoint and tangent keep junction acceleration bounded");
  Expect(
      planner.last_diagnostics().cartesian_maximum_acceleration.value < 10.0,
      "lane-return transition keeps total acceleration bounded: " +
          std::to_string(
              planner.last_diagnostics().cartesian_maximum_acceleration.value));
  Expect(planner.last_diagnostics().cartesian_maximum_jerk.value < 10.0,
         "lane-return transition keeps total jerk bounded: " +
             std::to_string(
                 planner.last_diagnostics().cartesian_maximum_jerk.value) +
             " at index " +
             std::to_string(
                 planner.last_diagnostics().cartesian_maximum_jerk.index));
  const FrenetProjection returned_to_lane =
      ProjectToRoad(output.next_x.back(), output.next_y.back(),
                    start_s + incoming_speed, map);
  Expect(std::fabs(returned_to_lane.d - lane_d) < 0.16,
         "jerk-limited stitch transition moves toward the nominal lane: " +
             std::to_string(returned_to_lane.d));
}

void TestRepeatedReplanningContinuity() {
  const MapData map = LoadMap("data/highway_map.csv");
  const double time_step = 0.02;
  const std::size_t consumed_points = 4;
  PlannerInput input;
  input.ego.s = 145.0;
  input.ego.d = 6.0;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample initial_geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = initial_geometry.x;
  input.ego.y = initial_geometry.y;
  input.ego.yaw_deg = std::atan2(initial_geometry.first_derivative_y,
                                 initial_geometry.first_derivative_x) *
                      180.0 / 3.14159265358979323846;
  input.ego.speed_mph = 20.0 / 0.44704;

  PathPlanner planner;
  double maximum_speed = 0.0;
  double maximum_acceleration = 0.0;
  double maximum_jerk = 0.0;
  int maximum_acceleration_cycle = 0;
  std::size_t maximum_acceleration_index = 0;
  int maximum_jerk_cycle = 0;
  std::size_t maximum_jerk_index = 0;
  CartesianKinematicSample maximum_jerk_sample;
  CartesianKinematicSample before_maximum_jerk_sample;
  for (int cycle = 0; cycle < 30; ++cycle) {
    const PlannerOutput output = planner.Plan(input, map);
    const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
    maximum_speed =
        std::max(maximum_speed, diagnostics.cartesian_maximum_speed.value);
    if (diagnostics.cartesian_maximum_acceleration.value >
        maximum_acceleration) {
      maximum_acceleration = diagnostics.cartesian_maximum_acceleration.value;
      maximum_acceleration_cycle = cycle + 1;
      maximum_acceleration_index =
          diagnostics.cartesian_maximum_acceleration.index;
    }
    if (diagnostics.cartesian_maximum_jerk.value > maximum_jerk) {
      maximum_jerk = diagnostics.cartesian_maximum_jerk.value;
      maximum_jerk_cycle = cycle + 1;
      maximum_jerk_index = diagnostics.cartesian_maximum_jerk.index;
      maximum_jerk_sample = diagnostics.cartesian_samples[maximum_jerk_index];
      if (maximum_jerk_index > 0) {
        before_maximum_jerk_sample =
            diagnostics.cartesian_samples[maximum_jerk_index - 1];
      }
    }

    double consumed_distance = 0.0;
    double previous_x = input.ego.x;
    double previous_y = input.ego.y;
    for (std::size_t index = 0; index < consumed_points; ++index) {
      consumed_distance += std::hypot(output.next_x[index] - previous_x,
                                      output.next_y[index] - previous_y);
      previous_x = output.next_x[index];
      previous_y = output.next_y[index];
    }
    input.ego.x = output.next_x[consumed_points - 1];
    input.ego.y = output.next_y[consumed_points - 1];
    input.ego.s = NormalizeS(input.ego.s + consumed_distance, map.track_length);
    input.ego.speed_mph = std::hypot(output.next_x[consumed_points - 1] -
                                         output.next_x[consumed_points - 2],
                                     output.next_y[consumed_points - 1] -
                                         output.next_y[consumed_points - 2]) /
                          time_step / 0.44704;
    input.ego.yaw_deg = std::atan2(output.next_y[consumed_points - 1] -
                                       output.next_y[consumed_points - 2],
                                   output.next_x[consumed_points - 1] -
                                       output.next_x[consumed_points - 2]) *
                        180.0 / 3.14159265358979323846;
    input.previous_path_x.assign(output.next_x.begin() + consumed_points,
                                 output.next_x.end());
    input.previous_path_y.assign(output.next_y.begin() + consumed_points,
                                 output.next_y.end());

    double remaining_distance = 0.0;
    previous_x = input.ego.x;
    previous_y = input.ego.y;
    for (std::size_t index = consumed_points; index < output.next_x.size();
         ++index) {
      remaining_distance += std::hypot(output.next_x[index] - previous_x,
                                       output.next_y[index] - previous_y);
      previous_x = output.next_x[index];
      previous_y = output.next_y[index];
    }
    input.end_path_s =
        NormalizeS(input.ego.s + remaining_distance, map.track_length);
    input.end_path_d = input.ego.d;
  }

  Expect(maximum_speed < 22.3,
         "repeated stitching keeps Cartesian speed below the limit margin: " +
             std::to_string(maximum_speed));
  Expect(maximum_acceleration < 10.0,
         "repeated stitching keeps total acceleration bounded: " +
             std::to_string(maximum_acceleration) + " at cycle/index " +
             std::to_string(maximum_acceleration_cycle) + "/" +
             std::to_string(maximum_acceleration_index));
  Expect(
      maximum_jerk < 10.0,
      "repeated stitching keeps total jerk bounded: " +
          std::to_string(maximum_jerk) + " at cycle/index " +
          std::to_string(maximum_jerk_cycle) + "/" +
          std::to_string(maximum_jerk_index) + " acceleration before/after " +
          std::to_string(before_maximum_jerk_sample.acceleration_x_mps2) + "," +
          std::to_string(before_maximum_jerk_sample.acceleration_y_mps2) + "/" +
          std::to_string(maximum_jerk_sample.acceleration_x_mps2) + "," +
          std::to_string(maximum_jerk_sample.acceleration_y_mps2));
}

void TestRepeatedReplanningReturnsToLockedLane() {
  const MapData map = LoadMap("data/highway_map.csv");
  const double lane_center_d = 6.0;
  const double time_step = 0.02;
  const std::size_t consumed_points = 4;

  PlannerInput input;
  input.ego.s = 124.8336;
  input.ego.d = 6.164833;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample initial_geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = initial_geometry.x;
  input.ego.y = initial_geometry.y;
  input.ego.yaw_deg = std::atan2(initial_geometry.first_derivative_y,
                                 initial_geometry.first_derivative_x) *
                          180.0 / 3.14159265358979323846 +
                      0.5;
  input.ego.speed_mph = 0.0;

  PathPlanner planner;
  double maximum_lateral_error = 0.0;
  double maximum_acceleration = 0.0;
  double maximum_jerk = 0.0;
  for (int cycle = 0; cycle < 220; ++cycle) {
    const PlannerOutput output = planner.Plan(input, map);
    const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
    ExpectNear(diagnostics.lane_center_d, lane_center_d, 1e-12,
               "no-lane-change planner keeps its initial target lane");
    maximum_lateral_error =
        std::max(maximum_lateral_error,
                 std::fabs(diagnostics.plan_start_d - lane_center_d));
    maximum_acceleration = std::max(
        maximum_acceleration, diagnostics.cartesian_maximum_acceleration.value);
    maximum_jerk =
        std::max(maximum_jerk, diagnostics.cartesian_maximum_jerk.value);

    double consumed_distance = 0.0;
    double previous_x = input.ego.x;
    double previous_y = input.ego.y;
    for (std::size_t index = 0; index < consumed_points; ++index) {
      consumed_distance += std::hypot(output.next_x[index] - previous_x,
                                      output.next_y[index] - previous_y);
      previous_x = output.next_x[index];
      previous_y = output.next_y[index];
    }
    const FrenetProjection ego_projection = ProjectToRoad(
        output.next_x[consumed_points - 1], output.next_y[consumed_points - 1],
        input.ego.s + consumed_distance, map);
    input.ego.x = output.next_x[consumed_points - 1];
    input.ego.y = output.next_y[consumed_points - 1];
    input.ego.s = ego_projection.s;
    input.ego.d = ego_projection.d;
    input.ego.speed_mph = std::hypot(output.next_x[consumed_points - 1] -
                                         output.next_x[consumed_points - 2],
                                     output.next_y[consumed_points - 1] -
                                         output.next_y[consumed_points - 2]) /
                          time_step / 0.44704;
    input.ego.yaw_deg = std::atan2(output.next_y[consumed_points - 1] -
                                       output.next_y[consumed_points - 2],
                                   output.next_x[consumed_points - 1] -
                                       output.next_x[consumed_points - 2]) *
                        180.0 / 3.14159265358979323846;

    input.previous_path_x.assign(output.next_x.begin() + consumed_points,
                                 output.next_x.end());
    input.previous_path_y.assign(output.next_y.begin() + consumed_points,
                                 output.next_y.end());
    double remaining_distance = 0.0;
    previous_x = input.ego.x;
    previous_y = input.ego.y;
    for (std::size_t index = consumed_points; index < output.next_x.size();
         ++index) {
      remaining_distance += std::hypot(output.next_x[index] - previous_x,
                                       output.next_y[index] - previous_y);
      previous_x = output.next_x[index];
      previous_y = output.next_y[index];
    }
    const FrenetProjection end_projection =
        ProjectToRoad(output.next_x.back(), output.next_y.back(),
                      input.ego.s + remaining_distance, map);
    input.end_path_s = end_projection.s;
    input.end_path_d = end_projection.d;
  }

  Expect(maximum_lateral_error < 0.3,
         "finite stitch correction prevents cumulative lane drift: " +
             std::to_string(maximum_lateral_error));
  Expect(std::fabs(input.end_path_d - lane_center_d) < 0.02,
         "finite stitch correction converges to the lane center: " +
             std::to_string(input.end_path_d));
  Expect(maximum_acceleration < 10.0,
         "lane-return replanning keeps total acceleration bounded: " +
             std::to_string(maximum_acceleration));
  Expect(maximum_jerk < 10.0,
         "lane-return replanning keeps total jerk bounded: " +
             std::to_string(maximum_jerk));
}

void TestLowSpeedLaneReturnUsesFutureSpeedBound() {
  const MapData map = LoadMap("data/highway_map.csv");
  constexpr double lane_center_d = 6.0;
  constexpr std::size_t consumed_points = 4;

  PlannerInput input;
  input.ego.s = 100.0;
  input.ego.d = 4.4;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample initial_geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = initial_geometry.x;
  input.ego.y = initial_geometry.y;
  input.ego.yaw_deg = std::atan2(initial_geometry.first_derivative_y,
                                 initial_geometry.first_derivative_x) *
                      180.0 / 3.14159265358979323846;
  input.ego.speed_mph = 0.0;

  PathPlanner planner;
  double maximum_jerk = 0.0;
  double transition_length = 0.0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    const PlannerOutput output = planner.Plan(input, map);
    const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
    if (cycle == 0) {
      transition_length = diagnostics.lateral.transition_length_m;
      ExpectNear(diagnostics.lane_center_d, lane_center_d, 1e-12,
                 "low-speed return keeps the selected middle lane");
      Expect(!diagnostics.lane_deviation_violation,
             "lane-return regression starts inside the allowed lane envelope");
    }
    if (diagnostics.cartesian_maximum_jerk.valid) {
      maximum_jerk =
          std::max(maximum_jerk, diagnostics.cartesian_maximum_jerk.value);
    }

    const std::vector<LateralPathState> &lateral_states =
        diagnostics.output_lateral_states;
    const std::vector<LongitudinalState> &longitudinal_states =
        diagnostics.output_longitudinal_states;
    input.ego.x = output.next_x[consumed_points - 1];
    input.ego.y = output.next_y[consumed_points - 1];
    input.ego.s = lateral_states[consumed_points - 1].road_parameter_s;
    input.ego.d = lateral_states[consumed_points - 1].planned_d;
    input.ego.speed_mph = longitudinal_states[consumed_points - 1].v / 0.44704;
    input.ego.yaw_deg = std::atan2(output.next_y[consumed_points - 1] -
                                       output.next_y[consumed_points - 2],
                                   output.next_x[consumed_points - 1] -
                                       output.next_x[consumed_points - 2]) *
                        180.0 / 3.14159265358979323846;
    input.previous_path_x.assign(output.next_x.begin() + consumed_points,
                                 output.next_x.end());
    input.previous_path_y.assign(output.next_y.begin() + consumed_points,
                                 output.next_y.end());
    input.end_path_s = lateral_states.back().road_parameter_s;
    input.end_path_d = lateral_states.back().planned_d;
  }

  Expect(transition_length > 15.0,
         "stationary lane return is sized for its future cruise speed");
  Expect(planner.last_diagnostics().lateral.remaining_m < 1e-3,
         "future-speed-sized lateral transition still reaches its endpoint");
  Expect(maximum_jerk < 10.0,
         "future-speed sizing keeps full-transition Cartesian jerk bounded: " +
             std::to_string(maximum_jerk));
}

void TestFixedTerminalRollingTransitionWithQuantizedHistory() {
  const MapData map = LoadMap("data/highway_map.csv");
  const double lane_center_d = 6.0;

  PlannerInput input;
  input.ego.s = 124.8336;
  input.ego.d = 6.4;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample initial_geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = initial_geometry.x;
  input.ego.y = initial_geometry.y;
  input.ego.yaw_deg = std::atan2(initial_geometry.first_derivative_y,
                                 initial_geometry.first_derivative_x) *
                          180.0 / 3.14159265358979323846 +
                      0.3;
  input.ego.speed_mph = 16.0 / 0.44704;

  PathPlanner planner;
  std::uint64_t transition_id = 0;
  double previous_progress = -1.0;
  double fixed_terminal_progress = -1.0;
  std::uint64_t previous_rolling_replan_count = 0;
  double maximum_end_lateral_error = 0.0;
  int reset_count = 0;
  LongitudinalState expected_frontier_state;
  double expected_frontier_progress = 0.0;
  bool expected_frontier_valid = false;
  for (int cycle = 0; cycle < 700; ++cycle) {
    const PlannerOutput output = planner.Plan(input, map);
    const PlannerCycleDiagnostics &diagnostics = planner.last_diagnostics();
    if (cycle == 0) {
      transition_id = diagnostics.lateral.transition_id;
      fixed_terminal_progress = diagnostics.lateral.transition_length_m;
      Expect(diagnostics.lateral.state_reset,
             "first lateral cycle initializes one transition");
      Expect(!diagnostics.lateral.rolling_replanned,
             "initial transition is not counted as a rolling replan");
    } else {
      Expect(diagnostics.historical_plan_aligned,
             "quantized previous_path keeps its saved longitudinal states");
      Expect(diagnostics.lateral.state_aligned,
             "quantized previous_path keeps its saved lateral states");
      Expect(!diagnostics.lateral.state_reset,
             "aligned rolling plan does not restart the lateral polynomial");
      Expect(diagnostics.lateral.transition_id == transition_id,
             "one lateral transition id survives all rolling callbacks");
      ExpectNear(diagnostics.lateral.transition_length_m,
                 fixed_terminal_progress, 1e-12,
                 "rolling quintic keeps one absolute terminal progress");
      const bool should_roll =
          expected_frontier_progress + 1e-6 < fixed_terminal_progress;
      Expect(diagnostics.lateral.rolling_replanned == should_roll,
             "active transition refits once at each rolling frontier");
      if (should_roll) {
        ExpectNear(diagnostics.lateral.rolling_origin_m,
                   expected_frontier_progress,
                   1e-8, "new quintic starts at the previous planned frontier");
        Expect(diagnostics.lateral.rolling_replan_count ==
                   previous_rolling_replan_count + 1,
               "rolling replan counter advances exactly once");
      } else {
        Expect(diagnostics.lateral.rolling_replan_count ==
                   previous_rolling_replan_count,
               "completed transition no longer refits coefficients");
      }
      Expect(diagnostics.lateral.position_residual_m < 0.01,
             "millimeter history error remains a small position residual");
      if (expected_frontier_valid) {
        ExpectNear(diagnostics.initial_speed_mps, expected_frontier_state.v,
                   1e-9, "rolling QP inherits the saved frontier speed");
        ExpectNear(diagnostics.initial_acceleration_mps2,
                   expected_frontier_state.a, 1e-9,
                   "rolling QP inherits the saved frontier acceleration");
        ExpectNear(diagnostics.initial_jerk_mps3, expected_frontier_state.j,
                   1e-9,
                   "rolling QP inherits the saved frontier jerk reference");
      }
    }
    if (diagnostics.lateral.state_reset) {
      ++reset_count;
    }
    Expect(diagnostics.lateral.progress_m + 1e-9 >= previous_progress,
           "lateral polynomial progress never moves backward");
    previous_progress = diagnostics.lateral.progress_m;
    previous_rolling_replan_count = diagnostics.lateral.rolling_replan_count;
    Expect(!diagnostics.qp_speed_violation &&
               !diagnostics.qp_acceleration_violation &&
               !diagnostics.qp_jerk_violation,
           "quantized history cannot corrupt QP hard constraints");
    Expect(diagnostics.output_lateral_states.size() == output.next_x.size(),
           "monitor retains one lateral state per output point");
    for (const LateralPathState &state : diagnostics.output_lateral_states) {
      Expect(state.valid, "generated path keeps valid lateral states");
      Expect(state.planned_d >= 4.0 && state.planned_d <= 8.0,
             "quintic lateral profile remains inside the locked lane");
    }
    const std::size_t consumed_points =
        2U + static_cast<std::size_t>(cycle % 6);
    const std::size_t next_frontier_index = consumed_points + 15U - 1U;
    expected_frontier_state =
        diagnostics.output_longitudinal_states[next_frontier_index];
    expected_frontier_progress =
        diagnostics.output_lateral_states[next_frontier_index]
            .correction_progress_m;
    expected_frontier_valid = true;
    const LateralPathState &ego_lateral_state =
        diagnostics.output_lateral_states[consumed_points - 1];
    const double ego_x = RoundToMillimeter(output.next_x[consumed_points - 1]);
    const double ego_y = RoundToMillimeter(output.next_y[consumed_points - 1]);
    const FrenetProjection ego_projection =
        ProjectToRoad(ego_x, ego_y, ego_lateral_state.road_parameter_s, map);
    input.ego.x = ego_x;
    input.ego.y = ego_y;
    input.ego.s = ego_projection.s;
    input.ego.d = ego_projection.d;
    input.ego.speed_mph =
        diagnostics.output_longitudinal_states[consumed_points - 1].v / 0.44704;
    input.ego.yaw_deg = std::atan2(output.next_y[consumed_points - 1] -
                                       output.next_y[consumed_points - 2],
                                   output.next_x[consumed_points - 1] -
                                       output.next_x[consumed_points - 2]) *
                        180.0 / 3.14159265358979323846;

    input.previous_path_x.clear();
    input.previous_path_y.clear();
    for (std::size_t index = consumed_points; index < output.next_x.size();
         ++index) {
      input.previous_path_x.push_back(RoundToMillimeter(output.next_x[index]));
      input.previous_path_y.push_back(RoundToMillimeter(output.next_y[index]));
    }
    const LateralPathState &end_lateral_state =
        diagnostics.output_lateral_states.back();
    const FrenetProjection end_projection = ProjectToRoad(
        input.previous_path_x.back(), input.previous_path_y.back(),
        end_lateral_state.road_parameter_s, map);
    input.end_path_s = end_projection.s;
    input.end_path_d = end_projection.d;
    maximum_end_lateral_error = std::max(
        maximum_end_lateral_error, std::fabs(input.end_path_d - lane_center_d));
    Expect(input.end_path_d >= 4.0 && input.end_path_d <= 8.0,
           "quantized Cartesian path stays inside the locked lane");
  }

  Expect(reset_count == 1,
         "fixed-terminal transition initializes once across 700 callbacks");
  Expect(previous_progress >=
             planner.last_diagnostics().lateral.transition_length_m - 1e-6,
         "rolling quintic transition reaches its fixed terminal phase");
  Expect(maximum_end_lateral_error < 0.45,
         "rolling transition never amplifies the initial lateral error");
  Expect(std::fabs(input.end_path_d - lane_center_d) < 0.01,
         "rolling quintic transition finishes at the lane center");
}

void TestEndPathDriftCannotChangeLockedLane() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput input;
  input.ego.s = 200.0;
  input.ego.d = 6.0;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = geometry.x;
  input.ego.y = geometry.y;
  input.ego.yaw_deg =
      std::atan2(geometry.first_derivative_y, geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;
  input.ego.speed_mph = 20.0 / 0.44704;

  PathPlanner planner;
  const PlannerOutput first = planner.Plan(input, map);
  const std::size_t consumed_points = 4;
  input.ego.x = first.next_x[consumed_points - 1];
  input.ego.y = first.next_y[consumed_points - 1];
  input.ego.speed_mph = std::hypot(first.next_x[consumed_points - 1] -
                                       first.next_x[consumed_points - 2],
                                   first.next_y[consumed_points - 1] -
                                       first.next_y[consumed_points - 2]) /
                        0.02 / 0.44704;
  input.ego.yaw_deg = std::atan2(first.next_y[consumed_points - 1] -
                                     first.next_y[consumed_points - 2],
                                 first.next_x[consumed_points - 1] -
                                     first.next_x[consumed_points - 2]) *
                      180.0 / 3.14159265358979323846;
  const FrenetProjection ego_projection =
      ProjectToRoad(input.ego.x, input.ego.y, input.ego.s + 1.0, map);
  input.ego.s = ego_projection.s;
  input.ego.d = ego_projection.d;
  input.previous_path_x.assign(first.next_x.begin() + consumed_points,
                               first.next_x.end());
  input.previous_path_y.assign(first.next_y.begin() + consumed_points,
                               first.next_y.end());
  const FrenetProjection end_projection = ProjectToRoad(
      first.next_x.back(), first.next_y.back(), input.ego.s + 20.0, map);
  input.end_path_s = end_projection.s;
  input.end_path_d = 8.2;

  planner.Plan(input, map);
  ExpectNear(planner.last_diagnostics().lane_center_d, 6.0, 1e-12,
             "drifting end_path_d cannot change the locked target lane");
  Expect(!planner.last_diagnostics().lane_deviation_violation,
         "exact retained state prevents end_path_d drift from moving the "
         "planning frontier");
}

PlannerInput HighwayInput(double speed_mph, const MapData &map) {
  PlannerInput input;
  input.ego.s = 100.0;
  input.ego.d = 6.0;
  input.ego.speed_mph = speed_mph;
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

void TestPlannerTrafficResponse() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput free_input = HighwayInput(45.0, map);

  PathPlanner free_planner;
  free_planner.Plan(free_input, map);
  const double free_speed = free_planner.reference_speed_mps();
  Expect(!free_planner.last_plan_emergency(),
         "free-road plan does not use emergency fallback");

  PlannerInput fast_rear_input = free_input;
  DetectedVehicle fast_rear;
  fast_rear.id = 0.0;
  fast_rear.s = 70.0;
  fast_rear.d = 6.0;
  const RoadGeometrySample fast_rear_geometry =
      EvaluateRoadGeometry(fast_rear.s, fast_rear.d, map);
  fast_rear.x = fast_rear_geometry.x;
  fast_rear.y = fast_rear_geometry.y;
  SetFrenetVelocity(&fast_rear, 30.0, 0.0, map);
  fast_rear_input.traffic.push_back(fast_rear);

  PathPlanner fast_rear_planner;
  const PlanningCycleDecision fast_rear_decision =
      fast_rear_planner.PlanCycle(fast_rear_input, map);
  const PlannerCycleDiagnostics &fast_rear_diagnostics =
      fast_rear_planner.last_diagnostics();
  Expect(fast_rear_decision.disposition ==
             PlanDisposition::kValidatedCandidate &&
             fast_rear_decision.has_validated_candidate &&
             !fast_rear_planner.last_plan_emergency(),
         "a faster same-lane rear vehicle cannot trigger emergency braking");
  Expect(fast_rear_diagnostics
                 .shielded_same_lane_rear_vehicle_ids.size() == 1 &&
             fast_rear_diagnostics
                     .shielded_same_lane_rear_vehicle_ids.front() ==
                 fast_rear.id &&
             !fast_rear_diagnostics.has_first_collision_evidence,
         "planner diagnostics expose the shielded rear object without a hard "
         "collision event");
  Expect(fast_rear_diagnostics.control_candidates.size() == 1 &&
             fast_rear_diagnostics.control_candidates.front()
                     .shielded_same_lane_rear_vehicle_ids.size() == 1,
         "the selected normal candidate records rear shielding");

  PlannerInput following_input = free_input;
  DetectedVehicle slow_lead;
  slow_lead.id = 11.0;
  slow_lead.s = 180.0;
  slow_lead.d = 6.0;
  SetFrenetVelocity(&slow_lead, 10.0, 0.0, map);
  following_input.traffic.push_back(slow_lead);

  PathPlanner following_planner;
  following_planner.Plan(following_input, map);
  Expect(!following_planner.last_plan_emergency(),
         "reachable lead vehicle is handled by the normal safety QP");
  Expect(following_planner.reference_speed_mps() + 0.2 < free_speed,
         "slow lead vehicle lowers the first-second speed plan");

  const double matched_speed_mps = 15.0;
  const double desired_gap_meters = 2.0 + 1.0 + 4.8 + 1.5 * matched_speed_mps;
  const double gap_surplus_meters = 12.0;
  PlannerInput gap_closing_input =
      HighwayInput(matched_speed_mps / 0.44704, map);
  DetectedVehicle matched_lead;
  matched_lead.id = 15.0;
  matched_lead.s =
      AdvanceRoadParameter(gap_closing_input.ego.s,
                           desired_gap_meters + gap_surplus_meters, 6.0, map);
  matched_lead.d = 6.0;
  SetFrenetVelocity(&matched_lead, matched_speed_mps, 0.0, map);
  gap_closing_input.traffic.push_back(matched_lead);

  PathPlanner gap_closing_planner;
  gap_closing_planner.Plan(gap_closing_input, map);
  const PlannerCycleDiagnostics &gap_closing_diagnostics =
      gap_closing_planner.last_diagnostics();
  ExpectNear(gap_closing_diagnostics.reference_first_mps,
             matched_speed_mps + gap_surplus_meters / 6.0, 1e-3,
             "planner uses actual ego speed and distance surplus for gradual "
             "gap closing: actual=" +
                 std::to_string(
                     gap_closing_diagnostics.reference_first_mps));

  PlannerInput intrusion_input = free_input;
  DetectedVehicle intruding_neighbor;
  intruding_neighbor.id = 14.0;
  intruding_neighbor.s = 190.0;
  intruding_neighbor.d = 9.2;
  SetFrenetVelocity(&intruding_neighbor, 10.0, -2.0, map);
  intrusion_input.traffic.push_back(intruding_neighbor);

  PathPlanner unintruded_planner;
  unintruded_planner.Plan(intrusion_input, map);
  const PlannerCycleDiagnostics &unintruded_diagnostics =
      unintruded_planner.last_diagnostics();
  Expect(unintruded_diagnostics.relevant_obstacle_count == 0 &&
             !unintruded_diagnostics.minimum_intrusion_speed_limit.valid,
         "a future-only lateral crossing does not affect current planning");

  intrusion_input.traffic.front().d = 8.35;
  SetFrenetVelocity(&intrusion_input.traffic.front(), 10.0, -2.0, map);
  PathPlanner intrusion_planner;
  intrusion_planner.Plan(intrusion_input, map);
  const PlannerCycleDiagnostics &intrusion_diagnostics =
      intrusion_planner.last_diagnostics();
  Expect(!intrusion_planner.last_plan_emergency(),
         "a current adjacent intrusion is handled by the normal QP");
  ExpectNear(intrusion_diagnostics.reference_minimum_mps, 10.0, 1e-3,
             "hard intrusion drives the reference to neighbor speed");
  Expect(
      intrusion_diagnostics.minimum_intrusion_speed_limit.valid &&
          std::fabs(intrusion_diagnostics.minimum_intrusion_speed_limit.value -
                    10.0) < 1e-9 &&
          intrusion_diagnostics.intrusion_limiting_obstacle_id == 14.0,
      "monitor identifies the intrusion speed cap and limiting vehicle");
  Expect(!intrusion_diagnostics.qp_samples.empty() &&
             intrusion_diagnostics.qp_samples.front().collision_margin_valid &&
             intrusion_diagnostics.qp_samples.back().collision_margin_valid,
         "current hard intrusion remains a body-collision constraint");

  PlannerInput short_headway_input = free_input;
  DetectedVehicle short_headway_lead = slow_lead;
  short_headway_lead.id = 13.0;
  short_headway_lead.s = 112.0;
  SetFrenetVelocity(&short_headway_lead, free_input.ego.speed_mph * 0.44704,
                    0.0, map);
  short_headway_input.traffic.push_back(short_headway_lead);

  PathPlanner short_headway_planner;
  short_headway_planner.Plan(short_headway_input, map);
  const PlannerCycleDiagnostics &short_headway_diagnostics =
      short_headway_planner.last_diagnostics();
  Expect(!short_headway_planner.last_plan_emergency(),
         "short time headway alone does not trigger emergency fallback");
  Expect(short_headway_diagnostics.qp_minimum_headway_margin.valid &&
             short_headway_diagnostics.qp_minimum_headway_margin.value < 0.0,
         "planner exposes active soft headway deficit");
  Expect(short_headway_diagnostics.qp_minimum_collision_margin.valid &&
             short_headway_diagnostics.qp_minimum_collision_margin.value >
                 0.0 &&
             !short_headway_diagnostics.qp_collision_violation,
         "short-headway plan retains positive hard body clearance");

  PlannerInput blocked_input = free_input;
  DetectedVehicle blocked_lead = slow_lead;
  blocked_lead.id = 12.0;
  blocked_lead.s = 120.0;
  SetFrenetVelocity(&blocked_lead, 0.0, 0.0, map);
  blocked_input.traffic.push_back(blocked_lead);

  PathPlanner emergency_planner;
  emergency_planner.Plan(blocked_input, map);
  Expect(emergency_planner.last_plan_emergency(),
         "predicted unavoidable body collision triggers emergency fallback");
  Expect(emergency_planner.reference_speed_mps() <
             blocked_input.ego.speed_mph * 0.44704,
         "emergency fallback commands a lower speed");
  const PlannerCycleDiagnostics &emergency_diagnostics =
      emergency_planner.last_diagnostics();
  Expect(emergency_diagnostics.control_candidates.size() == 4,
         "emergency diagnostics retain all three ordinary attempts and MRM");
  Expect(emergency_diagnostics.has_first_collision_evidence &&
             emergency_diagnostics.first_collision.evidence.object_id ==
                 blocked_lead.id,
         "emergency diagnostics identify the first collision object");
  if (emergency_diagnostics.control_candidates.size() == 4) {
    const ControlCandidateDiagnostics &mrm =
        emergency_diagnostics.control_candidates.back();
    Expect(mrm.minimum_risk_candidate && mrm.selected_for_dispatch &&
               !mrm.validation_valid && !mrm.collision_events.empty(),
           "MRM attempt remains explicitly invalid with collision evidence");
    Expect(mrm.collision_events.front().qp_relevant,
           "collision diagnostics identify a same-lane QP obstacle");
  }
}

} // namespace

int main() {
  RunTest(TestMapBoundaries, "TestMapBoundaries");
  RunTest(TestHighwayMapLoading, "TestHighwayMapLoading");
  RunTest(TestHighwayMapSplineContinuity, "TestHighwayMapSplineContinuity");
  RunTest(TestRoadArcLengthIndexRoundTrip,
          "TestRoadArcLengthIndexRoundTrip");
  RunTest(TestProtocolContract, "TestProtocolContract");
  RunTest(TestCartesianRuntimeMonitor, "TestCartesianRuntimeMonitor");
  RunTest(TestRuntimeMonitorStartsWithCleanCsvLogs,
          "TestRuntimeMonitorStartsWithCleanCsvLogs");
  RunTest(TestCartesianMonitorUsesAlignedStitchingFrame,
          "TestCartesianMonitorUsesAlignedStitchingFrame");
  RunTest(TestBaselinePlanner, "TestBaselinePlanner");
  RunTest(TestPlannerStartsWithHistory, "TestPlannerStartsWithHistory");
  RunTest(TestColdStartUsesHistoricalEndpointState,
          "TestColdStartUsesHistoricalEndpointState");
  RunTest(TestPlannerStitchesFromHistoricalEndpoint,
          "TestPlannerStitchesFromHistoricalEndpoint");
  RunTest(TestRepeatedReplanningContinuity, "TestRepeatedReplanningContinuity");
  RunTest(TestRepeatedReplanningReturnsToLockedLane,
          "TestRepeatedReplanningReturnsToLockedLane");
  RunTest(TestLowSpeedLaneReturnUsesFutureSpeedBound,
          "TestLowSpeedLaneReturnUsesFutureSpeedBound");
  RunTest(TestFixedTerminalRollingTransitionWithQuantizedHistory,
          "TestFixedTerminalRollingTransitionWithQuantizedHistory");
  RunTest(TestEndPathDriftCannotChangeLockedLane,
          "TestEndPathDriftCannotChangeLockedLane");
  RunTest(TestPlannerTrafficResponse, "TestPlannerTrafficResponse");
  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed" << std::endl;
    return 1;
  }
  std::cout << "All planning tests passed" << std::endl;
  return 0;
}
