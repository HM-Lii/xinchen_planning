#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/SparseCore>

#include "longitudinal_qp.h"
#include "qp_solver.h"
#include "speed_reference.h"
#include "traffic_predictor.h"
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
    ExpectNear(result.primal[0], 1.0, 1e-5,
               "OSQP smoke problem has the expected solution");
  }
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

void TestLongitudinalSafetyBoundary() {
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
  const double fixed_gap =
      config.standstill_gap_meters + config.prediction_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  for (std::size_t k = 0; k < result.trajectory.states.size(); ++k) {
    const double time = static_cast<double>(k) * config.time_step_seconds;
    const LongitudinalState &state = result.trajectory.states[k];
    const double safety_residual =
        obstacle.relative_s + obstacle.speed_mps * time - fixed_gap - state.s -
        config.time_headway_seconds * state.v;
    Expect(safety_residual >= -1e-3,
           "lead-vehicle time-headway boundary remains satisfied");
  }
  Expect(result.trajectory.states.back().s < 155.0,
         "hard safety boundary reduces free-road longitudinal progress");

  LongitudinalQpInput impossible = input;
  impossible.obstacles[0].relative_s = 10.0;
  const LongitudinalQpResult impossible_result = optimizer.Solve(impossible);
  Expect(!impossible_result.success,
         "an already violated immutable safety boundary is infeasible");
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

  PlannerInput input;
  input.previous_path_x.assign(10, 0.0);
  input.previous_path_y.assign(10, 0.0);

  DetectedVehicle ahead;
  ahead.id = 1.0;
  ahead.s = 2.0;
  ahead.d = 6.0;
  ahead.vx_mps = 10.0;
  input.traffic.push_back(ahead);

  DetectedVehicle adjacent = ahead;
  adjacent.id = 2.0;
  adjacent.s = 5.0;
  adjacent.d = 10.0;
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
      PredictRelevantTraffic(input, 1.0, 6.0, 100.0, config);
  Expect(predicted.size() == 2,
         "traffic predictor keeps the lane and boundary vehicles only");
  if (predicted.size() == 2) {
    ExpectNear(predicted[0].relative_s, 3.0, 1e-12,
               "traffic is projected to the previous path endpoint");
    Expect(predicted[1].id == 3.0,
           "vehicle on the lane boundary is conservatively retained");
  }
}

void TestMonotoneSpeedReference() {
  SpeedReferenceConfig config;
  config.horizon_steps = 80;
  config.time_step_seconds = 0.1;
  config.maximum_speed_mps = 20.0;
  SpeedReferenceGenerator generator(config);

  const SpeedReferenceResult free_reference = generator.Generate({});
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

  const SpeedReferenceResult result = generator.Generate(obstacles);
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

} // namespace

int main() {
  TestQpSolverSmoke();
  TestLongitudinalAcceleration();
  TestLongitudinalDeceleration();
  TestInitialJerkContinuityReference();
  TestLongitudinalSafetyBoundary();
  TestEmergencyStopObjective();
  TestTrajectorySampler();
  TestTrafficPrediction();
  TestMonotoneSpeedReference();
  if (failures != 0) {
    std::cerr << failures << " longitudinal test assertion(s) failed"
              << std::endl;
    return 1;
  }
  std::cout << "All longitudinal tests passed" << std::endl;
  return 0;
}
