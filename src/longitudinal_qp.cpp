#include "longitudinal_qp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/SparseCore>

namespace {

constexpr double kObstacleConstraintTighteningMeters = 0.02;

class VariableIndex {
public:
  VariableIndex(std::size_t horizon_steps, std::size_t obstacle_count,
                bool include_collision_violation)
      : steps_(horizon_steps), nodes_(horizon_steps + 1),
        obstacle_count_(obstacle_count),
        include_collision_violation_(include_collision_violation) {}

  int s(std::size_t k) const { return static_cast<int>(k); }
  int v(std::size_t k) const { return static_cast<int>(nodes_ + k); }
  int a(std::size_t k) const { return static_cast<int>(2 * nodes_ + k); }
  int j(std::size_t k) const { return static_cast<int>(3 * nodes_ + k); }
  int headway_slack(std::size_t obstacle, std::size_t k) const {
    return static_cast<int>(3 * nodes_ + steps_ + obstacle * nodes_ + k);
  }
  int collision_violation(std::size_t obstacle, std::size_t k) const {
    return static_cast<int>(3 * nodes_ + steps_ + obstacle_count_ * nodes_ +
                            obstacle * nodes_ + k);
  }
  int size() const {
    return static_cast<int>(3 * nodes_ + steps_ + obstacle_count_ * nodes_ +
                            (include_collision_violation_
                                 ? obstacle_count_ * nodes_
                                 : 0));
  }

private:
  std::size_t steps_;
  std::size_t nodes_;
  std::size_t obstacle_count_;
  bool include_collision_violation_;
};

bool IsFiniteVector(const std::vector<double> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool HardCollisionActive(const PredictedObstacle &obstacle,
                         std::size_t node, std::size_t nodes) {
  return obstacle.hard_collision_active.empty() ||
         (obstacle.hard_collision_active.size() == nodes &&
          obstacle.hard_collision_active[node] != 0U);
}

void ValidateConfig(const LongitudinalQpConfig &config) {
  if (config.horizon_steps == 0 || config.time_step_seconds <= 0.0 ||
      config.intermediate_speed_substeps == 0 ||
      !std::isfinite(config.stop_viability_time_seconds) ||
      config.stop_viability_time_seconds <= 0.0 ||
      config.maximum_speed_mps <= 0.0 ||
      config.minimum_acceleration_mps2 >= 0.0 ||
      config.maximum_acceleration_mps2 <= 0.0 ||
      config.maximum_jerk_mps3 <= 0.0 || config.time_headway_seconds < 0.0 ||
      config.standstill_gap_meters < 0.0 || config.ego_length_meters <= 0.0 ||
      config.obstacle_length_meters <= 0.0 ||
      !std::isfinite(config.prediction_margin_meters) ||
      config.prediction_margin_meters < 0.0 ||
      !std::isfinite(config.physical_collision_margin_meters) ||
      config.physical_collision_margin_meters < 0.0 ||
      !std::isfinite(config.headway_slack_weight) ||
      config.headway_slack_weight <= 0.0 || config.speed_weight <= 0.0 ||
      !std::isfinite(config.collision_violation_weight) ||
      config.collision_violation_weight <= 0.0 ||
      config.acceleration_weight < 0.0 || config.jerk_weight < 0.0 ||
      config.initial_jerk_continuity_weight < 0.0 ||
      config.terminal_speed_weight < 0.0) {
    throw std::invalid_argument("invalid longitudinal QP configuration");
  }
}

bool UsesOperationalSlack(LongitudinalSafetyPolicy policy) {
  return policy != LongitudinalSafetyPolicy::kNormalOperational;
}

bool UsesMaximumBrakingObjective(const LongitudinalQpInput &input) {
  return input.emergency_stop ||
         input.safety_policy == LongitudinalSafetyPolicy::kMaximumBraking ||
         input.allow_physical_collision_violation;
}

} // namespace

LongitudinalQp::LongitudinalQp(const LongitudinalQpConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

LongitudinalQpResult LongitudinalQp::Solve(const LongitudinalQpInput &input) {
  const LongitudinalQpResult result = Evaluate(input, warm_start());
  if (result.success) {
    CommitWarmStart(result);
  }
  return result;
}

LongitudinalQpResult
LongitudinalQp::Evaluate(const LongitudinalQpInput &input,
                         const QpWarmStartState &warm_start) const {
  QpSolverOptions solver_options;
  if (input.allow_physical_collision_violation) {
    solver_options.maximum_iterations = 20000;
    solver_options.absolute_tolerance = 1e-5;
    solver_options.relative_tolerance = 1e-5;
  }
  return Evaluate(input, warm_start, solver_options);
}

LongitudinalQpResult
LongitudinalQp::Evaluate(const LongitudinalQpInput &input,
                         const QpWarmStartState &warm_start,
                         const QpSolverOptions &solver_options) const {
  const std::size_t steps = config_.horizon_steps;
  const std::size_t nodes = steps + 1;
  if (!std::isfinite(input.initial_speed_mps) ||
      !std::isfinite(input.initial_acceleration_mps2) ||
      !std::isfinite(input.initial_jerk_mps3) ||
      input.initial_speed_mps < 0.0 ||
      input.reference_speed_mps.size() != nodes ||
      !IsFiniteVector(input.reference_speed_mps) ||
      (!input.minimum_progress_m.empty() &&
       (input.minimum_progress_m.size() != nodes ||
        !IsFiniteVector(input.minimum_progress_m))) ||
      (!input.maximum_progress_m.empty() &&
       (input.maximum_progress_m.size() != nodes ||
        !IsFiniteVector(input.maximum_progress_m))) ||
      (!input.maximum_speed_mps.empty() &&
       (input.maximum_speed_mps.size() != nodes ||
        !IsFiniteVector(input.maximum_speed_mps)))) {
    throw std::invalid_argument("invalid longitudinal QP input");
  }
  for (std::size_t k = 0; k < nodes; ++k) {
    const double lower_progress =
        input.minimum_progress_m.empty() ? 0.0
                                         : input.minimum_progress_m[k];
    const double upper_progress =
        input.maximum_progress_m.empty()
            ? std::numeric_limits<double>::infinity()
            : input.maximum_progress_m[k];
    if (lower_progress < 0.0 || upper_progress < 0.0 ||
        lower_progress > upper_progress ||
        (!input.maximum_speed_mps.empty() &&
         input.maximum_speed_mps[k] < 0.0)) {
      throw std::invalid_argument("invalid longitudinal QP node bounds");
    }
  }
  if ((!input.minimum_progress_m.empty() &&
       input.minimum_progress_m.front() > 1e-9) ||
      (!input.maximum_progress_m.empty() &&
       input.maximum_progress_m.front() < -1e-9) ||
      (!input.maximum_speed_mps.empty() &&
       input.maximum_speed_mps.front() + 1e-9 <
           input.initial_speed_mps)) {
    throw std::invalid_argument(
        "candidate node-zero bounds exclude the inherited state");
  }
  for (const PredictedObstacle &obstacle : input.obstacles) {
    if (!std::isfinite(obstacle.relative_s) ||
        !std::isfinite(obstacle.speed_mps) ||
        obstacle.relative_s < 0.0 || obstacle.speed_mps < 0.0 ||
        (!obstacle.hard_collision_active.empty() &&
         obstacle.hard_collision_active.size() != nodes) ||
        (!obstacle.intrusion_speed_limit_mps.empty() &&
         obstacle.intrusion_speed_limit_mps.size() != nodes) ||
        !IsFiniteVector(obstacle.intrusion_speed_limit_mps)) {
      throw std::invalid_argument("invalid predicted obstacle");
    }
  }

  if (input.allow_physical_collision_violation &&
      input.safety_policy != LongitudinalSafetyPolicy::kMaximumBraking) {
    throw std::invalid_argument(
        "minimum-risk QP must use the maximum-braking policy");
  }

  const VariableIndex index(steps, input.obstacles.size(),
                            input.allow_physical_collision_violation);
  const int variables = index.size();
  const int bound_rows = variables;
  const int dynamics_rows = static_cast<int>(3 * steps);
  const int progress_rows = static_cast<int>(steps);
  const int intermediate_speed_rows = static_cast<int>(
      steps * (config_.intermediate_speed_substeps - 1));
  const int stop_viability_rows = static_cast<int>(
      steps * config_.intermediate_speed_substeps);
  int active_obstacle_node_rows = 0;
  for (const PredictedObstacle &obstacle : input.obstacles) {
    for (std::size_t node = 0; node < nodes; ++node) {
      if (HardCollisionActive(obstacle, node, nodes)) {
        ++active_obstacle_node_rows;
      }
    }
  }
  const int collision_rows = active_obstacle_node_rows;
  const int headway_rows = active_obstacle_node_rows;
  const int constraints_count =
      bound_rows + dynamics_rows + progress_rows + intermediate_speed_rows +
      stop_viability_rows + collision_rows + headway_rows;
  const double infinity = std::numeric_limits<double>::infinity();
  const double dt = config_.time_step_seconds;

  QuadraticProgram problem;
  problem.gradient = Eigen::VectorXd::Zero(variables);
  problem.lower_bound = Eigen::VectorXd::Constant(constraints_count, -infinity);
  problem.upper_bound = Eigen::VectorXd::Constant(constraints_count, infinity);

  std::vector<Eigen::Triplet<double>> hessian_triplets;
  hessian_triplets.reserve(static_cast<std::size_t>(variables));
  const bool maximum_braking = UsesMaximumBrakingObjective(input);
  const double speed_weight_scale = maximum_braking ? 5.0 : 1.0;
  const double comfort_weight_scale = maximum_braking ? 0.25 : 1.0;
  for (int variable = 0; variable < variables; ++variable) {
    hessian_triplets.emplace_back(variable, variable, 2e-8);
  }
  for (std::size_t k = 0; k < nodes; ++k) {
    double speed_weight = config_.speed_weight * speed_weight_scale;
    if (k + 1 == nodes) {
      speed_weight += config_.terminal_speed_weight * speed_weight_scale;
    }
    hessian_triplets.emplace_back(index.v(k), index.v(k), 2.0 * speed_weight);
    problem.gradient[index.v(k)] =
        -2.0 * speed_weight *
        std::max(0.0, std::min(input.reference_speed_mps[k],
                               config_.maximum_speed_mps));
    hessian_triplets.emplace_back(index.a(k), index.a(k),
                                  2.0 * config_.acceleration_weight *
                                      comfort_weight_scale);
  }
  for (std::size_t k = 0; k < steps; ++k) {
    const double continuity_weight =
        k == 0 && input.initial_jerk_valid
            ? config_.initial_jerk_continuity_weight
            : 0.0;
    hessian_triplets.emplace_back(index.j(k), index.j(k),
                                  2.0 *
                                      (config_.jerk_weight *
                                           comfort_weight_scale +
                                       continuity_weight));
    if (continuity_weight > 0.0) {
      const double initial_jerk =
          std::max(-config_.maximum_jerk_mps3,
                   std::min(input.initial_jerk_mps3,
                            config_.maximum_jerk_mps3));
      problem.gradient[index.j(k)] -=
          2.0 * continuity_weight * initial_jerk;
    }
  }
  for (std::size_t obstacle = 0; obstacle < input.obstacles.size();
       ++obstacle) {
    for (std::size_t k = 0; k < nodes; ++k) {
      hessian_triplets.emplace_back(index.headway_slack(obstacle, k),
                                    index.headway_slack(obstacle, k),
                                    2.0 * config_.headway_slack_weight);
    }
  }
  if (input.allow_physical_collision_violation) {
    for (std::size_t obstacle = 0; obstacle < input.obstacles.size();
         ++obstacle) {
      for (std::size_t k = 0; k < nodes; ++k) {
        // Earlier contact and larger overlap dominate comfort and progress.
        const double time_priority =
            1.0 + static_cast<double>(nodes - k) / static_cast<double>(nodes);
        hessian_triplets.emplace_back(
            index.collision_violation(obstacle, k),
            index.collision_violation(obstacle, k),
            2.0 * 1e-3 * config_.collision_violation_weight * time_priority);
        problem.gradient[index.collision_violation(obstacle, k)] +=
            config_.collision_violation_weight * time_priority;
      }
    }
  }
  problem.hessian.resize(variables, variables);
  problem.hessian.setFromTriplets(hessian_triplets.begin(),
                                  hessian_triplets.end());

  std::vector<Eigen::Triplet<double>> constraint_triplets;
  constraint_triplets.reserve(
      static_cast<std::size_t>(
          variables +
          (16 + 6 * (config_.intermediate_speed_substeps - 1)) * steps +
                               4 * active_obstacle_node_rows));

  int row = 0;
  for (int variable = 0; variable < variables; ++variable, ++row) {
    constraint_triplets.emplace_back(row, variable, 1.0);
  }
  problem.lower_bound[index.s(0)] = 0.0;
  problem.upper_bound[index.s(0)] = 0.0;
  for (std::size_t k = 1; k < nodes; ++k) {
    problem.lower_bound[index.s(k)] =
        input.minimum_progress_m.empty()
            ? 0.0
            : input.minimum_progress_m[k];
    if (!input.maximum_progress_m.empty()) {
      problem.upper_bound[index.s(k)] = input.maximum_progress_m[k];
    }
  }

  problem.lower_bound[index.v(0)] = input.initial_speed_mps;
  problem.upper_bound[index.v(0)] = input.initial_speed_mps;
  const double recoverable_speed_upper_bound =
      std::max(config_.maximum_speed_mps, input.initial_speed_mps);
  for (std::size_t k = 1; k < nodes; ++k) {
    problem.lower_bound[index.v(k)] = 0.0;
    problem.upper_bound[index.v(k)] =
        input.maximum_speed_mps.empty()
            ? recoverable_speed_upper_bound
            : std::min(recoverable_speed_upper_bound,
                       input.maximum_speed_mps[k]);
  }

  const double initial_acceleration =
      std::max(config_.minimum_acceleration_mps2,
               std::min(input.initial_acceleration_mps2,
                        config_.maximum_acceleration_mps2));
  problem.lower_bound[index.a(0)] = initial_acceleration;
  problem.upper_bound[index.a(0)] = initial_acceleration;
  for (std::size_t k = 1; k < nodes; ++k) {
    problem.lower_bound[index.a(k)] = config_.minimum_acceleration_mps2;
    problem.upper_bound[index.a(k)] = config_.maximum_acceleration_mps2;
  }
  for (std::size_t k = 0; k < steps; ++k) {
    problem.lower_bound[index.j(k)] = -config_.maximum_jerk_mps3;
    problem.upper_bound[index.j(k)] = config_.maximum_jerk_mps3;
  }

  for (std::size_t k = 1; k < nodes; ++k, ++row) {
    constraint_triplets.emplace_back(row, index.v(k), 1.0);
    constraint_triplets.emplace_back(
        row, index.a(k), config_.stop_viability_time_seconds);
    problem.lower_bound[row] = 0.0;
  }
  for (std::size_t k = 0; k < steps; ++k) {
    for (std::size_t substep = 1;
         substep < config_.intermediate_speed_substeps; ++substep, ++row) {
      const double substep_time =
          dt * static_cast<double>(substep) /
          static_cast<double>(config_.intermediate_speed_substeps);
      constraint_triplets.emplace_back(row, index.v(k), 1.0);
      constraint_triplets.emplace_back(
          row, index.a(k),
          substep_time + config_.stop_viability_time_seconds);
      constraint_triplets.emplace_back(
          row, index.j(k),
          0.5 * substep_time * substep_time +
              config_.stop_viability_time_seconds * substep_time);
      problem.lower_bound[row] = 0.0;
    }
  }
  for (std::size_t obstacle = 0; obstacle < input.obstacles.size();
       ++obstacle) {
    for (std::size_t k = 0; k < nodes; ++k) {
      problem.lower_bound[index.headway_slack(obstacle, k)] = 0.0;
      if (!UsesOperationalSlack(input.safety_policy)) {
        problem.upper_bound[index.headway_slack(obstacle, k)] = 0.0;
      }
      if (input.allow_physical_collision_violation) {
        problem.lower_bound[index.collision_violation(obstacle, k)] = 0.0;
      }
    }
  }

  for (std::size_t k = 0; k < steps; ++k) {
    constraint_triplets.emplace_back(row, index.a(k + 1), 1.0);
    constraint_triplets.emplace_back(row, index.a(k), -1.0);
    constraint_triplets.emplace_back(row, index.j(k), -dt);
    problem.lower_bound[row] = 0.0;
    problem.upper_bound[row] = 0.0;
    ++row;

    constraint_triplets.emplace_back(row, index.v(k + 1), 1.0);
    constraint_triplets.emplace_back(row, index.v(k), -1.0);
    constraint_triplets.emplace_back(row, index.a(k), -dt);
    constraint_triplets.emplace_back(row, index.j(k), -0.5 * dt * dt);
    problem.lower_bound[row] = 0.0;
    problem.upper_bound[row] = 0.0;
    ++row;

    constraint_triplets.emplace_back(row, index.s(k + 1), 1.0);
    constraint_triplets.emplace_back(row, index.s(k), -1.0);
    constraint_triplets.emplace_back(row, index.v(k), -dt);
    constraint_triplets.emplace_back(row, index.a(k), -0.5 * dt * dt);
    constraint_triplets.emplace_back(row, index.j(k), -dt * dt * dt / 6.0);
    problem.lower_bound[row] = 0.0;
    problem.upper_bound[row] = 0.0;
    ++row;
  }

  for (std::size_t k = 0; k < steps; ++k, ++row) {
    constraint_triplets.emplace_back(row, index.s(k + 1), 1.0);
    constraint_triplets.emplace_back(row, index.s(k), -1.0);
    problem.lower_bound[row] = 0.0;
  }

  for (std::size_t k = 0; k < steps; ++k) {
    for (std::size_t substep = 1;
         substep < config_.intermediate_speed_substeps; ++substep, ++row) {
      const double substep_time =
          dt * static_cast<double>(substep) /
          static_cast<double>(config_.intermediate_speed_substeps);
      constraint_triplets.emplace_back(row, index.v(k), 1.0);
      constraint_triplets.emplace_back(row, index.a(k), substep_time);
      constraint_triplets.emplace_back(row, index.j(k),
                                       0.5 * substep_time * substep_time);
      problem.lower_bound[row] = 0.0;
      problem.upper_bound[row] = recoverable_speed_upper_bound;
      if (!input.maximum_speed_mps.empty()) {
        problem.upper_bound[row] =
            std::min(problem.upper_bound[row],
                     std::min(input.maximum_speed_mps[k],
                              input.maximum_speed_mps[k + 1]));
      }
    }
  }

  const double collision_gap =
      0.5 * (config_.ego_length_meters + config_.obstacle_length_meters) +
      config_.physical_collision_margin_meters;
  const double fixed_headway_gap =
      config_.standstill_gap_meters + config_.prediction_margin_meters +
      collision_gap;
  // Ordinary policies never relax body overlap. The independent minimum-risk
  // solve represents unavoidable overlap with a separate evidence variable.
  // The larger desired following gap has its own operational slack variable.
  for (std::size_t obstacle_index = 0;
       obstacle_index < input.obstacles.size(); ++obstacle_index) {
    const PredictedObstacle &obstacle = input.obstacles[obstacle_index];
    for (std::size_t k = 0; k < nodes; ++k) {
      if (!HardCollisionActive(obstacle, k, nodes)) {
        continue;
      }
      const double time = static_cast<double>(k) * dt;
      constraint_triplets.emplace_back(row, index.s(k), 1.0);
      if (input.allow_physical_collision_violation) {
        constraint_triplets.emplace_back(
            row, index.collision_violation(obstacle_index, k), -1.0);
      }
      problem.upper_bound[row] = obstacle.relative_s +
                                 obstacle.speed_mps * time - collision_gap -
                                 kObstacleConstraintTighteningMeters;
      ++row;
    }
  }
  for (std::size_t obstacle_index = 0;
       obstacle_index < input.obstacles.size(); ++obstacle_index) {
    const PredictedObstacle &obstacle = input.obstacles[obstacle_index];
    for (std::size_t k = 0; k < nodes; ++k) {
      if (!HardCollisionActive(obstacle, k, nodes)) {
        continue;
      }
      const double time = static_cast<double>(k) * dt;
      constraint_triplets.emplace_back(row, index.s(k), 1.0);
      constraint_triplets.emplace_back(row, index.v(k),
                                       config_.time_headway_seconds);
      constraint_triplets.emplace_back(
          row, index.headway_slack(obstacle_index, k), -1.0);
      problem.upper_bound[row] = obstacle.relative_s +
                                 obstacle.speed_mps * time -
                                 fixed_headway_gap -
                                 kObstacleConstraintTighteningMeters;
      ++row;
    }
  }
  if (row != constraints_count) {
    throw std::logic_error("longitudinal QP constraint count mismatch");
  }

  problem.constraint_matrix.resize(constraints_count, variables);
  problem.constraint_matrix.setFromTriplets(constraint_triplets.begin(),
                                            constraint_triplets.end());

  Eigen::VectorXd warm_start_vector;
  const Eigen::VectorXd *warm_start_pointer = nullptr;
  if (warm_start.primal.size() == static_cast<std::size_t>(variables)) {
    warm_start_vector.resize(variables);
    for (int variable = 0; variable < variables; ++variable) {
      warm_start_vector[variable] =
          warm_start.primal[static_cast<std::size_t>(variable)];
    }
    warm_start_pointer = &warm_start_vector;
  }
  const QpSolverResult solved =
      solver_.Solve(problem, warm_start_pointer, solver_options);

  LongitudinalQpResult result;
  result.success = solved.success;
  result.status = solved.status;
  result.objective = solved.objective;
  result.safety_policy = input.safety_policy;
  result.trajectory.time_step_seconds = dt;
  result.trajectory.emergency = maximum_braking;
  if (!solved.success) {
    return result;
  }

  result.proposed_warm_start.primal.resize(
      static_cast<std::size_t>(solved.primal.size()));
  for (Eigen::Index variable = 0; variable < solved.primal.size(); ++variable) {
    result.proposed_warm_start.primal[static_cast<std::size_t>(variable)] =
        solved.primal[variable];
  }
  result.trajectory.states.resize(nodes);
  result.trajectory.states.front().s = 0.0;
  result.trajectory.states.front().v = input.initial_speed_mps;
  result.trajectory.states.front().a = initial_acceleration;
  result.trajectory.states.front().j = solved.primal[index.j(0)];
  // OSQP satisfies the dynamics equalities to its numerical tolerance. A
  // controller-rate third difference can amplify that tiny node residual, so
  // reconstruct the public trajectory exactly from the optimized jerk
  // sequence before validation and safety-margin evaluation.
  for (std::size_t k = 0; k < steps; ++k) {
    LongitudinalState &state = result.trajectory.states[k];
    state.j = solved.primal[index.j(k)];
    LongitudinalState &next = result.trajectory.states[k + 1];
    next.s = state.s + state.v * dt + 0.5 * state.a * dt * dt +
             state.j * dt * dt * dt / 6.0;
    next.v = state.v + state.a * dt + 0.5 * state.j * dt * dt;
    next.a = state.a + state.j * dt;
    next.j = k + 1 < steps ? solved.primal[index.j(k + 1)] : 0.0;
  }
  for (std::size_t obstacle = 0; obstacle < input.obstacles.size();
       ++obstacle) {
    for (std::size_t k = 0; k < nodes; ++k) {
      result.maximum_headway_slack_meters =
          std::max(result.maximum_headway_slack_meters,
                   solved.primal[index.headway_slack(obstacle, k)]);
      const double slack = solved.primal[index.headway_slack(obstacle, k)];
      if (slack > 1e-5 &&
          (!result.headway_slack_used ||
           k < result.first_headway_slack_node)) {
        result.headway_slack_used = true;
        result.first_headway_slack_node = k;
      }
      if (input.allow_physical_collision_violation) {
        result.maximum_collision_violation_meters =
            std::max(result.maximum_collision_violation_meters,
                     solved.primal[index.collision_violation(obstacle, k)]);
      }
    }
  }
  result.minimum_physical_margin_meters =
      std::numeric_limits<double>::infinity();
  result.minimum_operational_margin_meters =
      std::numeric_limits<double>::infinity();
  for (const PredictedObstacle &obstacle : input.obstacles) {
    for (std::size_t k = 0; k < nodes; ++k) {
      if (!HardCollisionActive(obstacle, k, nodes)) {
        continue;
      }
      const double time = static_cast<double>(k) * dt;
      const LongitudinalState &state = result.trajectory.states[k];
      const double obstacle_progress =
          obstacle.relative_s + obstacle.speed_mps * time;
      result.minimum_physical_margin_meters =
          std::min(result.minimum_physical_margin_meters,
                   obstacle_progress - collision_gap - state.s);
      result.minimum_operational_margin_meters =
          std::min(result.minimum_operational_margin_meters,
                   obstacle_progress - fixed_headway_gap - state.s -
                       config_.time_headway_seconds * state.v);
    }
  }
  if (!std::isfinite(result.minimum_physical_margin_meters)) {
    result.minimum_physical_margin_meters =
        std::numeric_limits<double>::infinity();
  }
  if (!std::isfinite(result.minimum_operational_margin_meters)) {
    result.minimum_operational_margin_meters =
        std::numeric_limits<double>::infinity();
  }
  result.hard_safe = !input.allow_physical_collision_violation &&
                     result.minimum_physical_margin_meters >= -1e-4;
  return result;
}

void LongitudinalQp::CommitWarmStart(const LongitudinalQpResult &result) {
  if (!result.success) {
    throw std::invalid_argument("cannot commit an unsuccessful QP result");
  }
  warm_start_.resize(
      static_cast<Eigen::Index>(result.proposed_warm_start.primal.size()));
  for (std::size_t variable = 0;
       variable < result.proposed_warm_start.primal.size(); ++variable) {
    warm_start_[static_cast<Eigen::Index>(variable)] =
        result.proposed_warm_start.primal[variable];
  }
}

void LongitudinalQp::ResetWarmStart() { warm_start_.resize(0); }

QpWarmStartState LongitudinalQp::warm_start() const {
  QpWarmStartState result;
  result.primal.resize(static_cast<std::size_t>(warm_start_.size()));
  for (Eigen::Index variable = 0; variable < warm_start_.size(); ++variable) {
    result.primal[static_cast<std::size_t>(variable)] = warm_start_[variable];
  }
  return result;
}
