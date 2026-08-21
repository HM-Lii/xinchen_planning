#include "planner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "map.h"

namespace {

constexpr double kMilesPerHourToMetersPerSecond = 0.44704;

bool IsFiniteVector(const std::vector<double> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

double MoveToward(double value, double target, double maximum_delta) {
  if (value < target) {
    return std::min(value + maximum_delta, target);
  }
  return std::max(value - maximum_delta, target);
}

} // namespace

PathPlanner::PathPlanner(const PlannerConfig &config) : config_(config) {
  if (config_.output_points == 0 || config_.time_step_seconds <= 0.0 ||
      config_.target_speed_mph < 0.0 || config_.max_acceleration_mps2 <= 0.0 ||
      config_.lane_width_meters <= 0.0 || config_.lane_count <= 0) {
    throw std::invalid_argument("invalid planner configuration");
  }
}

PlannerOutput PathPlanner::Plan(const PlannerInput &input, const MapData &map) {
  std::string map_error;
  if (!ValidateMap(map, &map_error)) {
    throw std::invalid_argument("invalid map: " + map_error);
  }
  if (input.previous_path_x.size() != input.previous_path_y.size()) {
    throw std::invalid_argument(
        "previous path x/y arrays have different lengths");
  }
  if (input.previous_path_x.size() > config_.output_points) {
    throw std::invalid_argument("previous path is longer than planner output");
  }
  if (!IsFiniteVector(input.previous_path_x) ||
      !IsFiniteVector(input.previous_path_y) || !std::isfinite(input.ego.s) ||
      !std::isfinite(input.ego.d) || !std::isfinite(input.ego.speed_mph) ||
      !std::isfinite(input.end_path_s) || !std::isfinite(input.end_path_d)) {
    throw std::invalid_argument("planner input contains a non-finite value");
  }

  if (!speed_initialized_ || input.previous_path_x.empty()) {
    reference_speed_mps_ =
        std::max(0.0, input.ego.speed_mph * kMilesPerHourToMetersPerSecond);
    speed_initialized_ = true;
  }

  PlannerOutput output;
  output.next_x = input.previous_path_x;
  output.next_y = input.previous_path_y;
  output.next_x.reserve(config_.output_points);
  output.next_y.reserve(config_.output_points);

  const bool has_previous_path = !input.previous_path_x.empty();
  double next_s = has_previous_path ? input.end_path_s : input.ego.s;
  const double current_d = has_previous_path ? input.end_path_d : input.ego.d;
  int lane =
      static_cast<int>(std::floor(current_d / config_.lane_width_meters));
  lane = std::max(0, std::min(lane, config_.lane_count - 1));
  const double lane_center_d =
      (static_cast<double>(lane) + 0.5) * config_.lane_width_meters;

  const double target_speed_mps =
      config_.target_speed_mph * kMilesPerHourToMetersPerSecond;
  const double speed_delta =
      config_.max_acceleration_mps2 * config_.time_step_seconds;

  while (output.next_x.size() < config_.output_points) {
    reference_speed_mps_ =
        MoveToward(reference_speed_mps_, target_speed_mps, speed_delta);
    next_s += reference_speed_mps_ * config_.time_step_seconds;
    const auto xy = FrenetToCartesian(next_s, lane_center_d, map);
    output.next_x.push_back(xy.first);
    output.next_y.push_back(xy.second);
  }

  return output;
}
