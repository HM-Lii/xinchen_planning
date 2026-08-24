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
  VariableIndex(std::size_t horizon_steps, std::size_t obstacle_count)
      : steps_(horizon_steps), nodes_(horizon_steps + 1),
        obstacle_count_(obstacle_count) {}

  int s(std::size_t k) const { return static_cast<int>(k); }
  int v(std::size_t k) const { return static_cast<int>(nodes_ + k); }
  int a(std::size_t k) const { return static_cast<int>(2 * nodes_ + k); }
  int j(std::size_t k) const { return static_cast<int>(3 * nodes_ + k); }
  int headway_slack(std::size_t obstacle, std::size_t k) const {
    return static_cast<int>(3 * nodes_ + steps_ + obstacle * nodes_ + k);
  }
  int size() const {
    return static_cast<int>(3 * nodes_ + steps_ + obstacle_count_ * nodes_);
  }

private:
  std::size_t steps_;
  std::size_t nodes_;
  std::size_t obstacle_count_;
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
      config.maximum_speed_mps <= 0.0 ||
      config.minimum_acceleration_mps2 >= 0.0 ||
      config.maximum_acceleration_mps2 <= 0.0 ||
      config.maximum_jerk_mps3 <= 0.0 || config.time_headway_seconds < 0.0 ||
      config.standstill_gap_meters < 0.0 || config.ego_length_meters <= 0.0 ||
      config.obstacle_length_meters <= 0.0 ||
      config.prediction_margin_meters < 0.0 ||
      config.headway_slack_weight <= 0.0 || config.speed_weight <= 0.0 ||
      config.acceleration_weight < 0.0 || config.jerk_weight < 0.0 ||
      config.initial_jerk_continuity_weight < 0.0 ||
      config.terminal_speed_weight < 0.0) {
    throw std::invalid_argument("invalid longitudinal QP configuration");
  }
}

} // namespace

LongitudinalQp::LongitudinalQp(const LongitudinalQpConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

LongitudinalQpResult LongitudinalQp::Solve(const LongitudinalQpInput &input) {
  const std::size_t steps = config_.horizon_steps;
  const std::size_t nodes = steps + 1;
  if (!std::isfinite(input.initial_speed_mps) ||
      !std::isfinite(input.initial_acceleration_mps2) ||
      !std::isfinite(input.initial_jerk_mps3) ||
      input.initial_speed_mps < 0.0 ||
      input.reference_speed_mps.size() != nodes ||
      !IsFiniteVector(input.reference_speed_mps)) {
    throw std::invalid_argument("invalid longitudinal QP input");
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

  const VariableIndex index(steps, input.obstacles.size());
  const int variables = index.size();
  const int bound_rows = variables;
  const int dynamics_rows = static_cast<int>(3 * steps);
  const int progress_rows = static_cast<int>(steps);
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
      bound_rows + dynamics_rows + progress_rows + collision_rows +
      headway_rows;
  const double infinity = std::numeric_limits<double>::infinity();
  const double dt = config_.time_step_seconds;

  QuadraticProgram problem;
  problem.gradient = Eigen::VectorXd::Zero(variables);
  problem.lower_bound = Eigen::VectorXd::Constant(constraints_count, -infinity);
  problem.upper_bound = Eigen::VectorXd::Constant(constraints_count, infinity);

  std::vector<Eigen::Triplet<double>> hessian_triplets;
  hessian_triplets.reserve(static_cast<std::size_t>(variables));
  const double speed_weight_scale = input.emergency_stop ? 5.0 : 1.0;
  const double comfort_weight_scale = input.emergency_stop ? 0.25 : 1.0;
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
  problem.hessian.resize(variables, variables);
  problem.hessian.setFromTriplets(hessian_triplets.begin(),
                                  hessian_triplets.end());

  std::vector<Eigen::Triplet<double>> constraint_triplets;
  constraint_triplets.reserve(
      static_cast<std::size_t>(variables + 14 * steps +
                               4 * active_obstacle_node_rows));

  int row = 0;
  for (int variable = 0; variable < variables; ++variable, ++row) {
    constraint_triplets.emplace_back(row, variable, 1.0);
  }
  problem.lower_bound[index.s(0)] = 0.0;
  problem.upper_bound[index.s(0)] = 0.0;
  for (std::size_t k = 1; k < nodes; ++k) {
    problem.lower_bound[index.s(k)] = 0.0;
  }

  problem.lower_bound[index.v(0)] = input.initial_speed_mps;
  problem.upper_bound[index.v(0)] = input.initial_speed_mps;
  const double recoverable_speed_upper_bound =
      std::max(config_.maximum_speed_mps, input.initial_speed_mps);
  for (std::size_t k = 1; k < nodes; ++k) {
    problem.lower_bound[index.v(k)] = 0.0;
    problem.upper_bound[index.v(k)] = recoverable_speed_upper_bound;
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
  for (std::size_t obstacle = 0; obstacle < input.obstacles.size();
       ++obstacle) {
    for (std::size_t k = 0; k < nodes; ++k) {
      problem.lower_bound[index.headway_slack(obstacle, k)] = 0.0;
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

  const double collision_gap =
      0.5 * (config_.ego_length_meters + config_.obstacle_length_meters);
  const double fixed_headway_gap =
      config_.standstill_gap_meters + config_.prediction_margin_meters +
      collision_gap;
  // Body overlap is never relaxed. The larger desired following gap is kept
  // feasible through a separately penalized nonnegative slack variable.
  for (const PredictedObstacle &obstacle : input.obstacles) {
    for (std::size_t k = 0; k < nodes; ++k) {
      if (!HardCollisionActive(obstacle, k, nodes)) {
        continue;
      }
      const double time = static_cast<double>(k) * dt;
      constraint_triplets.emplace_back(row, index.s(k), 1.0);
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

  const Eigen::VectorXd *warm_start =
      warm_start_.size() == variables ? &warm_start_ : nullptr;
  const QpSolverResult solved = solver_.Solve(problem, warm_start);

  LongitudinalQpResult result;
  result.success = solved.success;
  result.status = solved.status;
  result.objective = solved.objective;
  result.trajectory.time_step_seconds = dt;
  if (!solved.success) {
    warm_start_.resize(0);
    return result;
  }

  warm_start_ = solved.primal;
  result.trajectory.states.resize(nodes);
  for (std::size_t k = 0; k < nodes; ++k) {
    LongitudinalState &state = result.trajectory.states[k];
    state.s = solved.primal[index.s(k)];
    state.v = solved.primal[index.v(k)];
    state.a = solved.primal[index.a(k)];
    state.j = k < steps ? solved.primal[index.j(k)] : 0.0;
  }
  for (std::size_t obstacle = 0; obstacle < input.obstacles.size();
       ++obstacle) {
    for (std::size_t k = 0; k < nodes; ++k) {
      result.maximum_headway_slack_meters =
          std::max(result.maximum_headway_slack_meters,
                   solved.primal[index.headway_slack(obstacle, k)]);
    }
  }
  return result;
}
