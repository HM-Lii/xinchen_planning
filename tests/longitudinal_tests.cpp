#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/SparseCore>

#include "longitudinal_qp.h"
#include "map.h"
#include "path_stitcher.h"
#include "qp_solver.h"
#include "speed_reference.h"
#include "traffic_predictor.h"
#include "trajectory_assembler.h"
#include "trajectory_sampler.h"

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

void TestQpSolverSmoke() {
  QuadraticProgram problem;
  problem.hessian.resize(1, 1);
  problem.hessian.insert(0, 0) = 2.0;
  problem.gradient = Eigen::VectorXd::Zero(1);
  problem.constraint_matrix.resize(1, 1);
  problem.constraint_matrix.insert(0, 0) = 1.0;
  problem.lower_bound = Eigen::VectorXd::Constant(1, 1.0);
  problem.upper_bound = Eigen::VectorXd::Constant(1, 2.0);

  const QpSolverResult result = QpSolver().Solve(problem);
  Expect(result.success, "OSQP smoke problem should solve");
  if (result.success) {
    Expect(std::isfinite(result.objective),
           "successful OSQP result has a finite objective");
    Expect(result.primal.allFinite(),
           "successful OSQP result has a finite primal vector");
    ExpectNear(result.primal[0], 1.0, 1e-5,
               "OSQP smoke problem has the expected solution");
  }

  Eigen::VectorXd invalid_warm_start(1);
  invalid_warm_start[0] = std::numeric_limits<double>::quiet_NaN();
  bool rejected_non_finite_warm_start = false;
  try {
    QpSolver().Solve(problem, &invalid_warm_start);
  } catch (const std::invalid_argument &) {
    rejected_non_finite_warm_start = true;
  }
  Expect(rejected_non_finite_warm_start,
         "QP solver rejects a non-finite warm start before OSQP");
}

void TestCanonicalTrajectoryRejectsNonFiniteQpOutput() {
  LongitudinalQpResult qp;
  qp.success = true;
  qp.hard_safe = true;
  qp.trajectory.time_step_seconds = 0.1;
  qp.trajectory.states.resize(2);
  qp.trajectory.states[0].v = std::numeric_limits<double>::quiet_NaN();

  TrajectoryCanonicalizationConfig config;
  config.sample_time_step_s = 0.02;
  config.sample_count = 1;
  config.minimum_acceleration_mps2 = -5.0;
  config.maximum_acceleration_mps2 = 3.0;
  config.maximum_jerk_mps3 = 8.0;
  config.invalid_trajectory_message = "non-finite test trajectory";

  bool rejected = false;
  try {
    SampleCanonicalTrajectory(qp, config);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Expect(rejected,
         "canonical trajectory assembly rejects non-finite QP output");
}

void CheckTrajectory(const LongitudinalTrajectory &trajectory,
                     const LongitudinalQpConfig &config) {
  Expect(trajectory.states.size() == config.horizon_steps + 1,
         "trajectory has one state per QP node");
  const double dt = config.time_step_seconds;
  for (std::size_t k = 0; k + 1 < trajectory.states.size(); ++k) {
    const LongitudinalState &current = trajectory.states[k];
    const LongitudinalState &next = trajectory.states[k + 1];
    ExpectNear(next.a, current.a + current.j * dt, 1e-3,
               "acceleration dynamics hold");
    ExpectNear(next.v, current.v + current.a * dt + 0.5 * current.j * dt * dt,
               1e-3, "velocity dynamics hold");
    ExpectNear(next.s,
               current.s + current.v * dt + 0.5 * current.a * dt * dt +
                   current.j * dt * dt * dt / 6.0,
               1e-3, "position dynamics hold");
    Expect(next.s + 5e-4 >= current.s, "longitudinal progress is nonnegative");
    Expect(std::fabs(current.j) <= config.maximum_jerk_mps3 + 1e-3,
           "jerk stays within its hard bound");
  }
  for (std::size_t k = 1; k < trajectory.states.size(); ++k) {
    const LongitudinalState &state = trajectory.states[k];
    Expect(state.v >= -1e-3 && state.v <= config.maximum_speed_mps + 1e-3,
           "velocity stays within its hard bounds");
    Expect(state.a >= config.minimum_acceleration_mps2 - 1e-3 &&
               state.a <= config.maximum_acceleration_mps2 + 1e-3,
           "acceleration stays within its hard bounds");
  }
}

void TestLongitudinalAcceleration() {
  LongitudinalQpConfig config;
  config.horizon_steps = 50;
  config.time_step_seconds = 0.1;
  LongitudinalQp optimizer(config);

  LongitudinalQpInput input;
  input.reference_speed_mps.assign(config.horizon_steps + 1, 10.0);
  const LongitudinalQpResult result = optimizer.Solve(input);
  Expect(result.success, "unconstrained acceleration QP should solve");
  if (!result.success) {
    return;
  }
  CheckTrajectory(result.trajectory, config);
  Expect(result.trajectory.states.back().v > 9.5,
         "trajectory approaches the requested speed");
}

void TestLongitudinalDeceleration() {
  LongitudinalQpConfig config;
  config.horizon_steps = 50;
  LongitudinalQp optimizer(config);

  LongitudinalQpInput input;
  input.initial_speed_mps = 15.0;
  input.reference_speed_mps.assign(config.horizon_steps + 1, 5.0);
  const LongitudinalQpResult result = optimizer.Solve(input);
  Expect(result.success, "unconstrained deceleration QP should solve");
  if (!result.success) {
    return;
  }
  CheckTrajectory(result.trajectory, config);
  Expect(result.trajectory.states.back().v < 5.5,
         "trajectory approaches the lower requested speed");
}

void TestInitialJerkContinuityReference() {
  LongitudinalQpConfig config;
  config.horizon_steps = 30;
  config.initial_jerk_continuity_weight = 100.0;

  LongitudinalQpInput without_history;
  without_history.initial_speed_mps = 10.0;
  without_history.reference_speed_mps.assign(config.horizon_steps + 1, 10.0);
  const LongitudinalQpResult baseline =
      LongitudinalQp(config).Solve(without_history);

  LongitudinalQpInput with_history = without_history;
  with_history.initial_jerk_valid = true;
  with_history.initial_jerk_mps3 = 4.0;
  const LongitudinalQpResult continued =
      LongitudinalQp(config).Solve(with_history);
  Expect(baseline.success && continued.success,
         "QP solves with and without a saved jerk reference");
  if (!baseline.success || !continued.success) {
    return;
  }
  Expect(std::fabs(continued.trajectory.states.front().j - 4.0) <
             std::fabs(baseline.trajectory.states.front().j - 4.0),
         "saved historical jerk pulls the first new jerk toward continuity");
  CheckTrajectory(continued.trajectory, config);
}

void TestCandidateNodeBounds() {
  LongitudinalQpConfig config;
  config.horizon_steps = 50;
  config.maximum_speed_mps = 20.0;
  LongitudinalQp optimizer(config);

  LongitudinalQpInput input;
  const std::size_t nodes = config.horizon_steps + 1;
  input.reference_speed_mps.assign(nodes, 20.0);
  input.minimum_progress_m.assign(nodes, 0.0);
  input.maximum_progress_m.assign(nodes, 25.0);
  input.maximum_speed_mps.assign(nodes, 8.0);
  input.maximum_progress_m.front() = 0.0;
  input.minimum_progress_m.back() = 5.0;
  const LongitudinalQpResult bounded = optimizer.Solve(input);
  Expect(bounded.success,
         "candidate QP solves with per-node progress and speed bounds");
  if (bounded.success) {
    for (std::size_t k = 0; k < nodes; ++k) {
      Expect(bounded.trajectory.states[k].s + 1e-3 >=
                 input.minimum_progress_m[k] &&
                 bounded.trajectory.states[k].s <=
                     input.maximum_progress_m[k] + 1e-3 &&
                 bounded.trajectory.states[k].v <=
                     input.maximum_speed_mps[k] + 1e-3,
             "public trajectory respects every candidate node bound");
    }
  }

  LongitudinalQpInput malformed = input;
  malformed.maximum_progress_m.pop_back();
  bool rejected = false;
  try {
    (void)optimizer.Solve(malformed);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  Expect(rejected,
         "candidate bound vectors must match the complete QP horizon");
}

void TestLongitudinalHeadwayAndCollisionConstraints() {
  LongitudinalQpConfig config;
  config.horizon_steps = 80;
  config.maximum_speed_mps = 20.0;
  LongitudinalQp optimizer(config);

  LongitudinalQpInput input;
  input.initial_speed_mps = 20.0;
  input.reference_speed_mps.assign(config.horizon_steps + 1, 20.0);
  PredictedObstacle obstacle;
  obstacle.id = 9.0;
  obstacle.relative_s = 100.0;
  obstacle.speed_mps = 10.0;
  input.obstacles.push_back(obstacle);

  const LongitudinalQpResult result = optimizer.Solve(input);
  Expect(result.success, "QP with a reachable lead vehicle should solve");
  if (!result.success) {
    return;
  }
  CheckTrajectory(result.trajectory, config);
  const double fixed_headway_gap =
      config.standstill_gap_meters + config.prediction_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  const double collision_gap =
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  for (std::size_t k = 0; k < result.trajectory.states.size(); ++k) {
    const double time = static_cast<double>(k) * config.time_step_seconds;
    const LongitudinalState &state = result.trajectory.states[k];
    const double collision_residual = obstacle.relative_s +
                                      obstacle.speed_mps * time -
                                      collision_gap - state.s;
    Expect(collision_residual >= -1e-3,
           "lead-vehicle body-collision boundary remains satisfied");
  }
  Expect(result.trajectory.states.back().s < 155.0,
         "soft headway penalty reduces free-road longitudinal progress");

  LongitudinalQpInput short_headway = input;
  short_headway.obstacles[0].relative_s = 10.0;
  short_headway.obstacles[0].speed_mps = 20.0;
  const LongitudinalQpResult normal_short_headway =
      optimizer.Solve(short_headway);
  Expect(!normal_short_headway.success,
         "normal policy keeps the operational headway boundary hard");
  short_headway.safety_policy =
      LongitudinalSafetyPolicy::kDegradedBraking;
  const LongitudinalQpResult short_headway_result =
      optimizer.Solve(short_headway);
  Expect(short_headway_result.success,
         "an initially short headway remains feasible through soft slack");
  if (short_headway_result.success) {
    CheckTrajectory(short_headway_result.trajectory, config);
    const double initial_headway_margin =
        short_headway.obstacles[0].relative_s - fixed_headway_gap -
        config.time_headway_seconds * input.initial_speed_mps;
    Expect(initial_headway_margin < -30.0,
           "short-headway regression starts outside the desired gap");
    Expect(short_headway_result.maximum_headway_slack_meters > 30.0,
           "QP reports use of the headway slack variable");
    for (std::size_t k = 0; k < short_headway_result.trajectory.states.size();
         ++k) {
      const double time = static_cast<double>(k) * config.time_step_seconds;
      const double collision_residual =
          short_headway.obstacles[0].relative_s +
          short_headway.obstacles[0].speed_mps * time - collision_gap -
          short_headway_result.trajectory.states[k].s;
      Expect(collision_residual >= -1e-3,
             "soft headway never relaxes the body-collision boundary");
    }
  }

  LongitudinalQpInput overlapping = short_headway;
  overlapping.obstacles[0].relative_s = 4.0;
  const LongitudinalQpResult overlapping_result = optimizer.Solve(overlapping);
  Expect(!overlapping_result.success,
         "an immutable body overlap remains hard-infeasible");
}

void TestEmergencyStopObjective() {
  LongitudinalQpConfig config;
  config.horizon_steps = 80;
  config.maximum_speed_mps = 20.0;
  LongitudinalQp optimizer(config);

  LongitudinalQpInput input;
  input.initial_speed_mps = 20.0;
  input.reference_speed_mps.assign(config.horizon_steps + 1, 0.0);
  input.emergency_stop = true;
  const LongitudinalQpResult result = optimizer.Solve(input);
  Expect(result.success, "emergency stop QP should solve");
  if (!result.success) {
    return;
  }
  CheckTrajectory(result.trajectory, config);
  Expect(result.trajectory.states.back().v < 0.1,
         "emergency objective brings the vehicle to a stop");
  Expect(result.trajectory.states.front().j < -0.5,
         "emergency objective begins braking immediately");
  const std::vector<LongitudinalState> controller_samples =
      SampleTrajectory(result.trajectory, 0.02, 400);
  for (const LongitudinalState &state : controller_samples) {
    Expect(state.v >= -1e-4,
           "emergency trajectory stays nonnegative at controller samples");
    Expect(state.v + config.stop_viability_time_seconds * state.a >= -1e-4,
           "emergency trajectory recovers braking acceleration before stop");
  }
}

void TestTrajectorySampler() {
  LongitudinalTrajectory trajectory;
  trajectory.time_step_seconds = 0.1;
  trajectory.states.resize(2);
  trajectory.states[0].s = 0.0;
  trajectory.states[0].v = 2.0;
  trajectory.states[0].a = 1.0;
  trajectory.states[0].j = -2.0;
  trajectory.states[1].s = 0.2046666666666667;
  trajectory.states[1].v = 2.09;
  trajectory.states[1].a = 0.8;

  const std::vector<LongitudinalState> samples =
      SampleTrajectory(trajectory, 0.02, 5);
  Expect(samples.size() == 5, "sampler returns the requested point count");
  ExpectNear(samples.back().s, trajectory.states[1].s, 1e-12,
             "sampler reaches the next QP position node");
  ExpectNear(samples.back().v, trajectory.states[1].v, 1e-12,
             "sampler reaches the next QP velocity node");
  ExpectNear(samples[0].a, 0.96, 1e-12,
             "sampler evaluates first-order-held acceleration");
}

void TestTrafficPrediction() {
  ExpectNear(ForwardTrackDistance(98.0, 3.0, 100.0), 5.0, 1e-12,
             "forward distance wraps at the track boundary");

  const MapData map = LoadMap("data/highway_map.csv");

  PlannerInput input;
  input.previous_path_x.assign(10, 0.0);
  input.previous_path_y.assign(10, 0.0);

  DetectedVehicle ahead;
  ahead.id = 1.0;
  ahead.s = 2.0;
  ahead.d = 6.0;
  SetFrenetVelocity(&ahead, 10.0, 0.0, map);
  input.traffic.push_back(ahead);

  DetectedVehicle adjacent = ahead;
  adjacent.id = 2.0;
  adjacent.s = 5.0;
  adjacent.d = 10.0;
  SetFrenetVelocity(&adjacent, 10.0, 0.0, map);
  input.traffic.push_back(adjacent);

  DetectedVehicle boundary = ahead;
  boundary.id = 3.0;
  boundary.s = 8.0;
  boundary.d = 8.0;
  boundary.vx_mps = 0.0;
  input.traffic.push_back(boundary);

  TrafficPredictionConfig config;
  config.lookahead_distance_meters = 50.0;
  const std::vector<PredictedObstacle> predicted =
      PredictRelevantTraffic(input, 1.0, 6.0, 6.0, map, config);
  Expect(predicted.size() == 2,
         "traffic predictor keeps the lane and boundary vehicles only");
  if (predicted.size() == 2) {
    const double projected_s = AdvanceRoadParameter(
        ahead.s,
        10.0 * static_cast<double>(input.previous_path_x.size()) *
            config.simulator_time_step_seconds,
        6.0, map);
    const double expected_distance = RoadArcLength(
        1.0, ForwardTrackDistance(1.0, projected_s, map.track_length), 6.0,
        map);
    ExpectNear(predicted[0].relative_s, expected_distance, 1e-9,
               "traffic projection uses target-lane physical arc length");
    Expect(predicted[1].id == 3.0,
           "vehicle on the lane boundary is conservatively retained");
  }
}

void TestTrafficCorridorAndCurvedRoadDistance() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput input;

  DetectedVehicle overlapping_corridor;
  overlapping_corridor.id = 41.0;
  overlapping_corridor.s = 340.82;
  overlapping_corridor.d = 3.64;
  SetFrenetVelocity(&overlapping_corridor, 20.0, 0.0, map);
  input.traffic.push_back(overlapping_corridor);

  DetectedVehicle separated_adjacent = overlapping_corridor;
  separated_adjacent.id = 42.0;
  separated_adjacent.s = 320.0;
  separated_adjacent.d = 2.0;
  SetFrenetVelocity(&separated_adjacent, 20.0, 0.0, map);
  input.traffic.push_back(separated_adjacent);

  TrafficPredictionConfig prediction_config;
  const std::vector<PredictedObstacle> predicted =
      PredictRelevantTraffic(input, 300.0, 4.4, 6.0, map, prediction_config);
  Expect(predicted.size() == 1 && predicted.front().id == 41.0,
         "traffic filter covers the complete lateral return corridor");
  if (predicted.size() != 1) {
    return;
  }

  const double raw_parameter_gap = 40.82;
  const double physical_gap = RoadArcLength(300.0, raw_parameter_gap, 6.0, map);
  ExpectNear(predicted.front().relative_s, physical_gap, 1e-9,
             "predicted obstacle distance is measured on the target lane");
  Expect(physical_gap + 1.0 < raw_parameter_gap,
         "curved-road regression has a material Frenet/arc-length difference");

  LongitudinalQpConfig qp_config;
  qp_config.maximum_speed_mps = 20.0;
  LongitudinalQpInput qp_input;
  qp_input.initial_speed_mps = 20.0;
  qp_input.reference_speed_mps.assign(qp_config.horizon_steps + 1, 20.0);
  qp_input.obstacles = predicted;
  qp_input.safety_policy = LongitudinalSafetyPolicy::kDegradedBraking;
  const LongitudinalQpResult result = LongitudinalQp(qp_config).Solve(qp_input);
  Expect(result.success,
         "physical gap below the desired headway remains QP-feasible");
  if (result.success) {
    Expect(result.maximum_headway_slack_meters > 0.9,
           "curved-road headway deficit is represented by soft slack");
    const double collision_gap =
        0.5 * (qp_config.ego_length_meters + qp_config.obstacle_length_meters);
    for (std::size_t k = 0; k < result.trajectory.states.size(); ++k) {
      const double time = static_cast<double>(k) * qp_config.time_step_seconds;
      const double collision_margin =
          predicted.front().relative_s + predicted.front().speed_mps * time -
          collision_gap - result.trajectory.states[k].s;
      Expect(collision_margin >= -1e-3,
             "curved-road body-collision boundary stays hard");
    }
  }
}

void TestFixedPathDefersTargetLaneHardCollisionActivation() {
  const MapData map = LoadMap("data/highway_map.csv");
  PlannerInput input;
  input.ego.s = 100.0;
  input.ego.d = 6.0;
  input.ego.speed_mph = 20.0 / 0.44704;
  input.end_path_s = input.ego.s;
  input.end_path_d = input.ego.d;
  const RoadGeometrySample ego_geometry =
      EvaluateRoadGeometry(input.ego.s, input.ego.d, map);
  input.ego.x = ego_geometry.x;
  input.ego.y = ego_geometry.y;
  input.ego.yaw_deg =
      std::atan2(ego_geometry.first_derivative_y,
                 ego_geometry.first_derivative_x) *
      180.0 / 3.14159265358979323846;

  DetectedVehicle target_lane_vehicle;
  target_lane_vehicle.id = 43.0;
  target_lane_vehicle.s = 120.0;
  target_lane_vehicle.d = 2.0;
  SetFrenetVelocity(&target_lane_vehicle, 20.0, 0.0, map);
  input.traffic.push_back(target_lane_vehicle);

  TrafficPredictionConfig prediction_config;
  prediction_config.maximum_speed_mps = 20.0;
  const FixedSpatialPath fixed_path = PathStitcher().Prepare(
      input, input.ego.s, input.ego.d, 2.0,
      prediction_config.maximum_speed_mps, map, PathStitcherState(), {},
      false);
  const std::vector<PredictedObstacle> predicted = PredictRelevantTraffic(
      input, input.ego.s, input.ego.d, 2.0, map, prediction_config,
      &fixed_path);
  Expect(predicted.size() == 1 &&
             predicted.front().hard_collision_active.size() == 81,
         "fixed-path target-lane traffic remains available at every QP node");
  if (predicted.size() != 1 ||
      predicted.front().hard_collision_active.size() != 81) {
    return;
  }
  const std::vector<unsigned char> &active =
      predicted.front().hard_collision_active;
  Expect(active.front() == 0U,
         "a laterally separated target-lane object is not hard at node zero");
  Expect(active.back() != 0U,
         "the same object becomes hard when the fixed path reaches its lane");
  const auto first_active =
      std::find(active.begin(), active.end(), static_cast<unsigned char>(1U));
  Expect(first_active != active.begin() && first_active != active.end(),
         "hard activation has a nontrivial fixed-path handoff node");
}

void TestAdjacentIntrusionSpeedLimits() {
  const MapData map = LoadMap("data/highway_map.csv");
  TrafficPredictionConfig prediction_config;
  prediction_config.horizon_steps = 80;
  prediction_config.maximum_speed_mps = 20.0;

  PlannerInput unintruded_input;
  unintruded_input.previous_path_x.assign(10, 0.0);
  unintruded_input.previous_path_y.assign(10, 0.0);
  DetectedVehicle cut_in;
  cut_in.id = 51.0;
  cut_in.s = 200.0;
  cut_in.d = 9.2;
  SetFrenetVelocity(&cut_in, 10.0, -2.0, map);
  unintruded_input.traffic.push_back(cut_in);

  const std::vector<PredictedObstacle> unintruded = PredictRelevantTraffic(
      unintruded_input, 100.0, 6.0, 6.0, map, prediction_config);
  Expect(unintruded.empty(),
         "an unintruded current contour is not retained through lateral "
         "velocity extrapolation");

  PlannerInput input = unintruded_input;
  input.traffic.front().d = 8.68;
  SetFrenetVelocity(&input.traffic.front(), 10.0, -2.0, map);

  const std::vector<PredictedObstacle> predicted =
      PredictRelevantTraffic(input, 100.0, 6.0, 6.0, map, prediction_config);
  Expect(predicted.size() == 1,
         "a currently intruding adjacent contour remains relevant");
  if (predicted.size() != 1) {
    return;
  }
  const PredictedObstacle &obstacle = predicted.front();
  Expect(obstacle.hard_collision_active.size() == 81 &&
             obstacle.intrusion_speed_limit_mps.size() == 81,
         "current intrusion state covers every QP node");
  if (obstacle.hard_collision_active.size() != 81 ||
      obstacle.intrusion_speed_limit_mps.size() != 81) {
    return;
  }

  // At d=9.0 the lower contour first reaches the d=8 lane line. The hard
  // corridor starts at center d=8.35, corresponding to 0.65 m intrusion.
  // The current d=8.68 contour therefore intrudes by 0.32 m.
  const double expected_mid_limit = 20.0 - (0.32 / 0.65) * (20.0 - 10.0);
  ExpectNear(obstacle.d, 8.68, 1e-12,
             "traffic prediction preserves the current lateral position");
  Expect(obstacle.hard_collision_active.front() == 0U &&
             obstacle.hard_collision_active.back() == 0U,
         "soft current intrusion does not become hard later in the horizon");
  ExpectNear(obstacle.intrusion_speed_limit_mps.front(), expected_mid_limit,
             1e-8,
             "current intrusion depth interpolates toward neighbor speed");
  ExpectNear(obstacle.intrusion_speed_limit_mps.back(), expected_mid_limit,
             1e-8,
             "current intrusion limit stays constant without extrapolation");

  PlannerInput outward_input = input;
  SetFrenetVelocity(&outward_input.traffic.front(), 10.0, 2.0, map);
  const std::vector<PredictedObstacle> outward = PredictRelevantTraffic(
      outward_input, 100.0, 6.0, 6.0, map, prediction_config);
  Expect(outward.size() == 1 &&
             std::fabs(outward.front().intrusion_speed_limit_mps.front() -
                       expected_mid_limit) < 1e-8,
         "opposite lateral velocities give the same current-contour limit");

  const std::vector<PredictedObstacle> expanded_corridor =
      PredictRelevantTraffic(unintruded_input, 100.0, 8.2, 6.0, map,
                             prediction_config);
  Expect(expanded_corridor.size() == 1 &&
             expanded_corridor.front().hard_collision_active[0] != 0U &&
             std::fabs(expanded_corridor.front().intrusion_speed_limit_mps[0] -
                       10.0) < 1e-9,
         "an off-center ego corridor gives hard collision and neighbor-speed "
         "priority");

  PlannerInput lower_input;
  DetectedVehicle lower_cut_in = cut_in;
  lower_cut_in.id = 52.0;
  lower_cut_in.d = 3.32;
  SetFrenetVelocity(&lower_cut_in, 10.0, 2.0, map);
  lower_input.traffic.push_back(lower_cut_in);
  const std::vector<PredictedObstacle> lower_predicted = PredictRelevantTraffic(
      lower_input, 100.0, 6.0, 6.0, map, prediction_config);
  Expect(
      lower_predicted.size() == 1 &&
          lower_predicted.front().hard_collision_active.front() == 0U &&
          lower_predicted.front().hard_collision_active.back() == 0U &&
          std::fabs(lower_predicted.front().intrusion_speed_limit_mps.front() -
                    expected_mid_limit) < 1e-8,
      "lower-side intrusion uses the symmetric depth and hard threshold");

  PlannerInput hard_input;
  DetectedVehicle hard_cut_in = cut_in;
  hard_cut_in.d = 8.35;
  SetFrenetVelocity(&hard_cut_in, 10.0, -2.0, map);
  hard_input.traffic.push_back(hard_cut_in);
  const std::vector<PredictedObstacle> hard_predicted = PredictRelevantTraffic(
      hard_input, 100.0, 6.0, 6.0, map, prediction_config);
  Expect(
      hard_predicted.size() == 1 &&
          hard_predicted.front().hard_collision_active.front() != 0U &&
          hard_predicted.front().hard_collision_active.back() != 0U &&
          std::fabs(hard_predicted.front().intrusion_speed_limit_mps.front() -
                    10.0) < 1e-9,
      "current hard-corridor entry reaches neighbor longitudinal speed");

  SpeedReferenceConfig reference_config;
  reference_config.horizon_steps = prediction_config.horizon_steps;
  reference_config.time_step_seconds = 0.1;
  reference_config.maximum_speed_mps = prediction_config.maximum_speed_mps;
  const SpeedReferenceResult reference =
      SpeedReferenceGenerator(reference_config)
          .Generate(hard_predicted, reference_config.maximum_speed_mps);
  Expect(reference.success, "intrusion-aware speed reference QP should solve");
  Expect(reference.raw_speed_limits_mps.size() == 81,
         "intrusion-aware raw speed limit covers the horizon");
  if (reference.raw_speed_limits_mps.size() == 81) {
    ExpectNear(reference.raw_speed_limits_mps.front(), 10.0, 1e-9,
               "raw reference starts at current neighbor speed");
    ExpectNear(reference.raw_speed_limits_mps.back(), 10.0, 1e-9,
               "raw reference holds current neighbor speed over the horizon");
  }

  PlannerInput slow_input = hard_input;
  SetFrenetVelocity(&slow_input.traffic.front(), 1.0, -2.0, map);
  const std::vector<PredictedObstacle> slow_intrusion = PredictRelevantTraffic(
      slow_input, 100.0, 6.0, 6.0, map, prediction_config);
  const SpeedReferenceResult slow_intrusion_reference =
      SpeedReferenceGenerator(reference_config)
          .Generate(slow_intrusion, reference_config.maximum_speed_mps);
  Expect(slow_intrusion_reference.raw_speed_limits_mps.size() == 81 &&
             std::fabs(slow_intrusion_reference.raw_speed_limits_mps.front() -
                       1.0) < 1e-9,
         "a slow hard intrusion uses its current longitudinal speed directly");

  LongitudinalQpConfig qp_config;
  qp_config.horizon_steps = 30;
  qp_config.maximum_speed_mps = 20.0;
  PredictedObstacle timed_collision;
  timed_collision.relative_s = 4.0;
  timed_collision.speed_mps = 30.0;
  timed_collision.hard_collision_active.assign(31, 0U);
  for (std::size_t node = 10; node < 31; ++node) {
    timed_collision.hard_collision_active[node] = 1U;
  }
  LongitudinalQpInput qp_input;
  qp_input.initial_speed_mps = 20.0;
  qp_input.reference_speed_mps.assign(31, 20.0);
  qp_input.obstacles.push_back(timed_collision);
  qp_input.safety_policy = LongitudinalSafetyPolicy::kDegradedBraking;
  const LongitudinalQpResult timed_result =
      LongitudinalQp(qp_config).Solve(qp_input);
  Expect(timed_result.success,
         "inactive early nodes do not receive a premature collision bound");

  qp_input.obstacles.front().hard_collision_active.clear();
  const LongitudinalQpResult always_active_result =
      LongitudinalQp(qp_config).Solve(qp_input);
  Expect(!always_active_result.success,
         "the same initial body overlap is infeasible when hard-active");
}

void TestMonotoneSpeedReference() {
  SpeedReferenceConfig config;
  config.horizon_steps = 80;
  config.time_step_seconds = 0.1;
  config.maximum_speed_mps = 20.0;
  SpeedReferenceGenerator generator(config);

  const SpeedReferenceResult free_reference = generator.Generate({}, 20.0);
  Expect(free_reference.success, "free-road reference QP should solve");
  if (free_reference.success) {
    ExpectNear(free_reference.trajectory.states.front().v, 20.0, 1e-5,
               "free-road reference starts at the maximum speed");
    ExpectNear(free_reference.trajectory.states.back().v, 20.0, 1e-5,
               "free-road reference holds the maximum speed");
  }

  // With a 40.8 m safety allowance, these create record-low events near
  // t=3 s and t=6 s.  The intervening higher limit must be discarded.
  std::vector<PredictedObstacle> obstacles;
  PredictedObstacle first;
  first.id = 1.0;
  first.relative_s = 55.8;
  first.speed_mps = 15.0;
  first.d = 6.0;
  obstacles.push_back(first);
  PredictedObstacle higher = first;
  higher.id = 2.0;
  higher.relative_s = 48.8;
  higher.speed_mps = 18.0;
  obstacles.push_back(higher);
  PredictedObstacle lower = first;
  lower.id = 3.0;
  lower.relative_s = 100.8;
  lower.speed_mps = 10.0;
  obstacles.push_back(lower);

  const SpeedReferenceResult result = generator.Generate(obstacles, 20.0);
  Expect(result.success, "FOH speed reference QP should solve");
  Expect(result.events.size() == 2,
         "only record-low speed-limit events are retained");
  if (!result.success) {
    return;
  }

  Expect(result.trajectory.states.size() == config.horizon_steps + 1,
         "speed reference covers the complete horizon");
  for (std::size_t k = 0; k + 1 < result.trajectory.states.size(); ++k) {
    const LongitudinalState &current = result.trajectory.states[k];
    const LongitudinalState &next = result.trajectory.states[k + 1];
    Expect(next.v <= current.v + 2e-4,
           "FOH reference speed is monotone nonincreasing");
    Expect(current.a >= config.minimum_acceleration_mps2 - 2e-4 &&
               current.a <= 2e-4,
           "FOH reference acceleration stays within bounds");
    Expect(std::fabs(current.j) <= config.maximum_jerk_mps3 + 2e-4,
           "FOH reference jerk stays within bounds");
  }
  Expect(result.trajectory.states[30].v <= 15.0 + 2e-4,
         "first obstacle speed is reached by its conflict time");
  Expect(result.trajectory.states[60].v <= 10.0 + 2e-4,
         "lower obstacle speed is reached by its conflict time");
  ExpectNear(result.raw_speed_limits_mps.back(), 10.0, 1e-12,
             "last lower speed limit is held to the horizon end");
  ExpectNear(result.trajectory.states.back().v,
             result.trajectory.states[result.trajectory.states.size() - 2].v,
             2e-4, "FOH reference holds its final lower speed");
  ExpectNear(result.trajectory.states.back().a, 0.0, 2e-4,
             "FOH reference ends with zero acceleration");
}

void TestGapClosingSpeedReference() {
  SpeedReferenceConfig config;
  config.horizon_steps = 80;
  config.time_step_seconds = 0.1;
  config.maximum_speed_mps = 20.0;
  config.gap_closing_time_seconds = 6.0;
  SpeedReferenceGenerator generator(config);

  const double ego_speed_mps = 10.0;
  const double lead_speed_mps = 10.0;
  const double fixed_gap =
      config.standstill_gap_meters + config.prediction_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  const double desired_gap =
      fixed_gap + config.time_headway_seconds * ego_speed_mps;

  PredictedObstacle lead;
  lead.id = 11.0;
  lead.relative_s = desired_gap + 12.0;
  lead.speed_mps = lead_speed_mps;
  lead.d = 6.0;
  const SpeedReferenceResult surplus =
      generator.Generate({lead}, ego_speed_mps);
  Expect(surplus.success, "gap-closing speed reference should solve");
  Expect(surplus.events.empty(),
         "equal-speed following does not create a false conflict event");
  if (surplus.raw_speed_limits_mps.size() == config.horizon_steps + 1) {
    ExpectNear(surplus.raw_speed_limits_mps.front(), 12.0, 1e-9,
               "distance surplus creates a gradual closing-speed allowance");
    ExpectNear(surplus.raw_speed_limits_mps.back(), 12.0, 1e-9,
               "gap-closing allowance is held over the reference horizon");
  }

  lead.relative_s = desired_gap;
  const SpeedReferenceResult settled =
      generator.Generate({lead}, ego_speed_mps);
  Expect(settled.raw_speed_limits_mps.size() == config.horizon_steps + 1,
         "settled following reference covers the horizon");
  if (!settled.raw_speed_limits_mps.empty()) {
    ExpectNear(settled.raw_speed_limits_mps.front(), lead_speed_mps, 1e-9,
               "target following distance settles at the lead speed");
  }

  lead.relative_s =
      desired_gap + config.gap_closing_time_seconds *
                        (config.maximum_speed_mps - lead_speed_mps);
  const SpeedReferenceResult far = generator.Generate({lead}, ego_speed_mps);
  if (!far.raw_speed_limits_mps.empty()) {
    ExpectNear(far.raw_speed_limits_mps.front(), config.maximum_speed_mps, 1e-9,
               "large distance surplus releases the cruise limit");
  }

  PredictedObstacle closing = lead;
  const double closing_ego_speed_mps = 15.0;
  const double closing_desired_gap =
      fixed_gap + config.time_headway_seconds * closing_ego_speed_mps;
  closing.relative_s = closing_desired_gap + 25.0;
  const SpeedReferenceResult conflict =
      generator.Generate({closing}, closing_ego_speed_mps);
  Expect(conflict.events.size() == 1,
         "a faster ego vehicle creates one lead-vehicle conflict event");
  if (conflict.events.size() == 1) {
    ExpectNear(conflict.events.front().conflict_time_seconds, 5.0, 1e-9,
               "conflict time uses distance surplus divided by closing speed");
    ExpectNear(conflict.events.front().speed_limit_mps, lead_speed_mps, 1e-9,
               "conflict event reaches the lead speed");
  }
}

} // namespace

int main() {
  TestQpSolverSmoke();
  TestCanonicalTrajectoryRejectsNonFiniteQpOutput();
  TestLongitudinalAcceleration();
  TestLongitudinalDeceleration();
  TestInitialJerkContinuityReference();
  TestCandidateNodeBounds();
  TestLongitudinalHeadwayAndCollisionConstraints();
  TestEmergencyStopObjective();
  TestTrajectorySampler();
  TestTrafficPrediction();
  TestTrafficCorridorAndCurvedRoadDistance();
  TestFixedPathDefersTargetLaneHardCollisionActivation();
  TestAdjacentIntrusionSpeedLimits();
  TestMonotoneSpeedReference();
  TestGapClosingSpeedReference();
  if (failures != 0) {
    std::cerr << failures << " longitudinal test assertion(s) failed"
              << std::endl;
    return 1;
  }
  std::cout << "All longitudinal tests passed" << std::endl;
  return 0;
}
