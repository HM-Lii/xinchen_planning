#include "traffic_predictor.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "map.h"

namespace {

double ForwardLaneArcDistance(double from_s, double to_s, double lane_d,
                              double stop_after_meters, const MapData &map) {
  const double parameter_distance =
      ForwardTrackDistance(from_s, to_s, map.track_length);
  const double parameter_chunk = std::max(10.0, 2.0 * stop_after_meters);
  double processed_parameter = 0.0;
  double accumulated_arc_length = 0.0;
  while (processed_parameter + 1e-12 < parameter_distance) {
    const double chunk =
        std::min(parameter_chunk, parameter_distance - processed_parameter);
    accumulated_arc_length +=
        RoadArcLength(from_s + processed_parameter, chunk, lane_d, map);
    processed_parameter += chunk;
    if (accumulated_arc_length > stop_after_meters) {
      break;
    }
  }
  return accumulated_arc_length;
}

} // namespace

double ForwardTrackDistance(double from_s, double to_s, double track_length) {
  if (!std::isfinite(from_s) || !std::isfinite(to_s) ||
      !std::isfinite(track_length) || track_length <= 0.0) {
    throw std::invalid_argument("invalid track distance input");
  }
  const double from = NormalizeS(from_s, track_length);
  const double to = NormalizeS(to_s, track_length);
  double distance = to - from;
  if (distance < 0.0) {
    distance += track_length;
  }
  return distance;
}

std::vector<PredictedObstacle>
PredictRelevantTraffic(const PlannerInput &input, double plan_start_s,
                       double plan_start_d, double lane_center_d,
                       const MapData &map,
                       const TrafficPredictionConfig &config) {
  if (!std::isfinite(plan_start_s) || !std::isfinite(plan_start_d) ||
      !std::isfinite(lane_center_d) ||
      config.simulator_time_step_seconds <= 0.0 ||
      config.lane_width_meters <= 0.0 ||
      config.lane_boundary_margin_meters < 0.0 ||
      config.ego_width_meters <= 0.0 ||
      config.obstacle_width_meters <= 0.0 ||
      config.lookahead_distance_meters <= 0.0) {
    throw std::invalid_argument("invalid traffic prediction input");
  }

  const double prediction_delay =
      static_cast<double>(input.previous_path_x.size()) *
      config.simulator_time_step_seconds;
  const double lateral_corridor_min = std::min(plan_start_d, lane_center_d);
  const double lateral_corridor_max = std::max(plan_start_d, lane_center_d);
  const double lateral_clearance =
      0.5 * (config.ego_width_meters + config.obstacle_width_meters) +
      config.lane_boundary_margin_meters;

  std::vector<PredictedObstacle> predicted;
  predicted.reserve(input.traffic.size());
  for (const DetectedVehicle &vehicle : input.traffic) {
    if (!std::isfinite(vehicle.id) || !std::isfinite(vehicle.vx_mps) ||
        !std::isfinite(vehicle.vy_mps) || !std::isfinite(vehicle.s) ||
        !std::isfinite(vehicle.d)) {
      throw std::invalid_argument("traffic contains a non-finite value");
    }
    if (vehicle.d < lateral_corridor_min - lateral_clearance ||
        vehicle.d > lateral_corridor_max + lateral_clearance) {
      continue;
    }
    const double speed = std::hypot(vehicle.vx_mps, vehicle.vy_mps);
    if (!std::isfinite(speed)) {
      throw std::invalid_argument("traffic speed is non-finite");
    }
    // Model every relevant vehicle in the target-lane arc-length coordinate.
    // This keeps the Frenet projection, physical obstacle speed and the ego
    // QP's Cartesian path distance in one meter-based longitudinal frame.
    const double projected_s = AdvanceRoadParameter(
        vehicle.s, speed * prediction_delay, lane_center_d, map);
    const double relative_s = ForwardLaneArcDistance(
        plan_start_s, projected_s, lane_center_d,
        config.lookahead_distance_meters, map);
    if (relative_s > config.lookahead_distance_meters) {
      continue;
    }

    PredictedObstacle obstacle;
    obstacle.id = vehicle.id;
    obstacle.relative_s = relative_s;
    obstacle.speed_mps = speed;
    obstacle.d = vehicle.d;
    predicted.push_back(obstacle);
  }

  std::sort(predicted.begin(), predicted.end(),
            [](const PredictedObstacle &left, const PredictedObstacle &right) {
              return left.relative_s < right.relative_s;
            });
  return predicted;
}
