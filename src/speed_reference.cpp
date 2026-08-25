#include "speed_reference.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/SparseCore>

namespace {

constexpr double kSpeedLimitToleranceMps = 1e-9;

class ReferenceIndex {
public:
  explicit ReferenceIndex(std::size_t steps)
      : steps_(steps), nodes_(steps + 1) {}

  int v(std::size_t k) const { return static_cast<int>(k); }
  int a(std::size_t k) const { return static_cast<int>(nodes_ + k); }
  int j(std::size_t k) const { return static_cast<int>(2 * nodes_ + k); }
  int size() const { return static_cast<int>(2 * nodes_ + steps_); }

private:
  std::size_t steps_;
  std::size_t nodes_;
};

void ValidateConfig(const SpeedReferenceConfig &config) {
  if (config.horizon_steps == 0 || config.time_step_seconds <= 0.0 ||
      config.maximum_speed_mps <= 0.0 ||
      config.minimum_acceleration_mps2 >= 0.0 ||
      config.maximum_jerk_mps3 <= 0.0 || config.time_headway_seconds < 0.0 ||
      !std::isfinite(config.gap_closing_time_seconds) ||
      config.gap_closing_time_seconds <= 0.0 ||
      config.standstill_gap_meters < 0.0 || config.ego_length_meters <= 0.0 ||
      config.obstacle_length_meters <= 0.0 ||
      config.prediction_margin_meters < 0.0 || config.speed_weight <= 0.0 ||
      config.acceleration_weight < 0.0 || config.jerk_weight < 0.0) {
    throw std::invalid_argument("invalid speed reference configuration");
  }
}

bool HardCollisionActive(const PredictedObstacle &obstacle, std::size_t node,
                         std::size_t nodes) {
  return obstacle.hard_collision_active.empty() ||
         (obstacle.hard_collision_active.size() == nodes &&
          obstacle.hard_collision_active[node] != 0U);
}

std::size_t FirstHardCollisionNode(const PredictedObstacle &obstacle,
                                   std::size_t nodes) {
  if (obstacle.hard_collision_active.empty()) {
    return 0;
  }
  for (std::size_t node = 0; node < nodes; ++node) {
    if (HardCollisionActive(obstacle, node, nodes)) {
      return node;
    }
  }
  return nodes;
}

double IntrusionSpeedLimitFloor(const PredictedObstacle &obstacle,
                                double maximum_speed_mps) {
  double floor = maximum_speed_mps;
  for (double speed_limit : obstacle.intrusion_speed_limit_mps) {
    floor = std::min(floor, speed_limit);
  }
  return floor;
}

void ValidateObstaclePrediction(const PredictedObstacle &obstacle,
                                std::size_t nodes, double maximum_speed_mps) {
  if (!std::isfinite(obstacle.relative_s) ||
      !std::isfinite(obstacle.speed_mps) || obstacle.relative_s < 0.0 ||
      obstacle.speed_mps < 0.0 ||
      (!obstacle.hard_collision_active.empty() &&
       obstacle.hard_collision_active.size() != nodes) ||
      (!obstacle.intrusion_speed_limit_mps.empty() &&
       obstacle.intrusion_speed_limit_mps.size() != nodes)) {
    throw std::invalid_argument("invalid obstacle for speed reference");
  }
  for (double speed_limit : obstacle.intrusion_speed_limit_mps) {
    if (!std::isfinite(speed_limit) || speed_limit < 0.0 ||
        speed_limit > maximum_speed_mps + kSpeedLimitToleranceMps) {
      throw std::invalid_argument("invalid intrusion speed limit");
    }
  }
}

std::vector<SpeedLimitEvent>
BuildEvents(const std::vector<PredictedObstacle> &obstacles,
            const SpeedReferenceConfig &config, double ego_speed_mps) {
  const std::size_t nodes = config.horizon_steps + 1;
  const double fixed_gap =
      config.standstill_gap_meters + config.prediction_margin_meters +
      0.5 * (config.ego_length_meters + config.obstacle_length_meters);
  const double desired_following_distance =
      fixed_gap + config.time_headway_seconds * ego_speed_mps;
  const double horizon =
      static_cast<double>(config.horizon_steps) * config.time_step_seconds;
  std::vector<SpeedLimitEvent> candidates;
  for (const PredictedObstacle &obstacle : obstacles) {
    ValidateObstaclePrediction(obstacle, nodes, config.maximum_speed_mps);
    const std::size_t first_hard_node = FirstHardCollisionNode(obstacle, nodes);
    if (first_hard_node == nodes) {
      continue;
    }
    const double closing_speed = ego_speed_mps - obstacle.speed_mps;
    if (closing_speed <= 1e-6) {
      continue;
    }
    const double available_distance =
        obstacle.relative_s - desired_following_distance;
    const double first_hard_time =
        static_cast<double>(first_hard_node) * config.time_step_seconds;
    const double conflict_time = std::max(
        first_hard_time, std::max(0.0, available_distance / closing_speed));
    if (conflict_time > horizon + 1e-9) {
      continue;
    }
    SpeedLimitEvent event;
    event.conflict_time_seconds = conflict_time;
    const double intrusion_floor =
        IntrusionSpeedLimitFloor(obstacle, config.maximum_speed_mps);
    event.speed_limit_mps =
        intrusion_floor < config.maximum_speed_mps - kSpeedLimitToleranceMps
            ? std::max(obstacle.speed_mps, intrusion_floor)
            : obstacle.speed_mps;
    candidates.push_back(event);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const SpeedLimitEvent &left, const SpeedLimitEvent &right) {
              if (std::fabs(left.conflict_time_seconds -
                            right.conflict_time_seconds) > 1e-9) {
                return left.conflict_time_seconds < right.conflict_time_seconds;
              }
              return left.speed_limit_mps < right.speed_limit_mps;
            });

  std::vector<SpeedLimitEvent> events;
  double running_limit = config.maximum_speed_mps;
  for (const SpeedLimitEvent &candidate : candidates) {
    if (candidate.speed_limit_mps + 1e-6 < running_limit) {
      events.push_back(candidate);
      running_limit = candidate.speed_limit_mps;
    }
  }
  return events;
}

} // namespace

SpeedReferenceGenerator::SpeedReferenceGenerator(
    const SpeedReferenceConfig &config)
    : config_(config) {
  ValidateConfig(config_);
}

SpeedReferenceResult SpeedReferenceGenerator::Generate(
    const std::vector<PredictedObstacle> &obstacles,
    double ego_speed_mps) const {
  if (!std::isfinite(ego_speed_mps) || ego_speed_mps < 0.0) {
    throw std::invalid_argument("invalid ego speed for speed reference");
  }
  const std::size_t steps = config_.horizon_steps;
  const std::size_t nodes = steps + 1;
  const double dt = config_.time_step_seconds;
  const ReferenceIndex index(steps);
  const int variables = index.size();
  const int constraints_count = variables + static_cast<int>(2 * steps);
  const double infinity = std::numeric_limits<double>::infinity();

  SpeedReferenceResult result;
  result.events = BuildEvents(obstacles, config_, ego_speed_mps);
  result.raw_speed_limits_mps.assign(nodes, config_.maximum_speed_mps);
  const double fixed_gap =
      config_.standstill_gap_meters + config_.prediction_margin_meters +
      0.5 * (config_.ego_length_meters + config_.obstacle_length_meters);
  const double desired_following_distance =
      fixed_gap + config_.time_headway_seconds * ego_speed_mps;
  for (const PredictedObstacle &obstacle : obstacles) {
    const std::size_t first_hard_node = FirstHardCollisionNode(obstacle, nodes);
    if (first_hard_node != nodes) {
      const double gap_surplus =
          std::max(0.0, obstacle.relative_s - desired_following_distance);
      const double gap_closing_speed_limit = std::min(
          config_.maximum_speed_mps,
          obstacle.speed_mps + gap_surplus / config_.gap_closing_time_seconds);
      for (std::size_t k = first_hard_node; k < nodes; ++k) {
        result.raw_speed_limits_mps[k] =
            std::min(result.raw_speed_limits_mps[k], gap_closing_speed_limit);
      }
    }
    if (!obstacle.intrusion_speed_limit_mps.empty()) {
      for (std::size_t k = 0; k < nodes; ++k) {
        result.raw_speed_limits_mps[k] =
            std::min(result.raw_speed_limits_mps[k],
                     obstacle.intrusion_speed_limit_mps[k]);
      }
    }
  }
  for (const SpeedLimitEvent &event : result.events) {
    const std::size_t event_index =
        std::min(steps, static_cast<std::size_t>(std::floor(
                            event.conflict_time_seconds / dt + 1e-9)));
    for (std::size_t k = event_index; k < nodes; ++k) {
      result.raw_speed_limits_mps[k] =
          std::min(result.raw_speed_limits_mps[k], event.speed_limit_mps);
    }
  }
  for (std::size_t k = 1; k < nodes; ++k) {
    result.raw_speed_limits_mps[k] = std::min(
        result.raw_speed_limits_mps[k], result.raw_speed_limits_mps[k - 1]);
  }

  QuadraticProgram problem;
  problem.gradient = Eigen::VectorXd::Zero(variables);
  problem.lower_bound = Eigen::VectorXd::Constant(constraints_count, -infinity);
  problem.upper_bound = Eigen::VectorXd::Constant(constraints_count, infinity);

  std::vector<Eigen::Triplet<double>> hessian_triplets;
  hessian_triplets.reserve(static_cast<std::size_t>(variables));
  for (int variable = 0; variable < variables; ++variable) {
    hessian_triplets.emplace_back(variable, variable, 2e-8);
  }
  for (std::size_t k = 0; k < nodes; ++k) {
    hessian_triplets.emplace_back(index.v(k), index.v(k),
                                  2.0 * config_.speed_weight);
    problem.gradient[index.v(k)] =
        -2.0 * config_.speed_weight * result.raw_speed_limits_mps[k];
    hessian_triplets.emplace_back(index.a(k), index.a(k),
                                  2.0 * config_.acceleration_weight);
  }
  for (std::size_t k = 0; k < steps; ++k) {
    hessian_triplets.emplace_back(index.j(k), index.j(k),
                                  2.0 * config_.jerk_weight);
  }
  problem.hessian.resize(variables, variables);
  problem.hessian.setFromTriplets(hessian_triplets.begin(),
                                  hessian_triplets.end());

  std::vector<Eigen::Triplet<double>> constraint_triplets;
  constraint_triplets.reserve(static_cast<std::size_t>(variables + 8 * steps));
  int row = 0;
  for (int variable = 0; variable < variables; ++variable, ++row) {
    constraint_triplets.emplace_back(row, variable, 1.0);
  }
  for (std::size_t k = 0; k < nodes; ++k) {
    problem.lower_bound[index.v(k)] = 0.0;
    problem.upper_bound[index.v(k)] = result.raw_speed_limits_mps[k];
    problem.lower_bound[index.a(k)] = config_.minimum_acceleration_mps2;
    problem.upper_bound[index.a(k)] = 0.0;
  }
  problem.lower_bound[index.a(0)] = 0.0;
  problem.upper_bound[index.a(0)] = 0.0;
  problem.lower_bound[index.a(steps)] = 0.0;
  problem.upper_bound[index.a(steps)] = 0.0;
  for (std::size_t k = 0; k < steps; ++k) {
    problem.lower_bound[index.j(k)] = -config_.maximum_jerk_mps3;
    problem.upper_bound[index.j(k)] = config_.maximum_jerk_mps3;
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
  }
  if (row != constraints_count) {
    throw std::logic_error("speed reference constraint count mismatch");
  }
  problem.constraint_matrix.resize(constraints_count, variables);
  problem.constraint_matrix.setFromTriplets(constraint_triplets.begin(),
                                            constraint_triplets.end());

  const QpSolverResult solved = solver_.Solve(problem);
  result.success = solved.success;
  result.status = solved.status;
  result.trajectory.time_step_seconds = dt;
  if (!solved.success) {
    return result;
  }

  result.trajectory.states.resize(nodes);
  double position = 0.0;
  for (std::size_t k = 0; k < nodes; ++k) {
    LongitudinalState &state = result.trajectory.states[k];
    state.s = position;
    state.v = solved.primal[index.v(k)];
    state.a = solved.primal[index.a(k)];
    state.j = k < steps ? solved.primal[index.j(k)] : 0.0;
    if (k < steps) {
      position +=
          state.v * dt + 0.5 * state.a * dt * dt + state.j * dt * dt * dt / 6.0;
    }
  }
  return result;
}
