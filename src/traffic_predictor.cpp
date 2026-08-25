#include "traffic_predictor.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "map.h"

namespace {

constexpr double kMinimumFrenetBasisDeterminant = 1e-8;
constexpr double kSpeedLimitToleranceMps = 1e-9;

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

double ProjectLongitudinalSpeed(const DetectedVehicle &vehicle,
                                const MapData &map) {
  const RoadGeometrySample road =
      EvaluateRoadGeometry(vehicle.s, vehicle.d, map);
  const RoadGeometrySample center =
      EvaluateRoadGeometry(vehicle.s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometry(vehicle.s, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  const double determinant =
      road.first_derivative_x * normal_y -
      road.first_derivative_y * normal_x;
  if (!std::isfinite(determinant) ||
      std::fabs(determinant) <= kMinimumFrenetBasisDeterminant) {
    throw std::runtime_error("traffic Frenet basis is degenerate");
  }
  const double parameter_rate =
      (vehicle.vx_mps * normal_y - vehicle.vy_mps * normal_x) /
      determinant;
  return std::max(0.0, parameter_rate *
                           std::hypot(road.first_derivative_x,
                                      road.first_derivative_y));
}

double IntrusionSpeedLimit(double intrusion_depth_meters,
                           double hard_intrusion_depth_meters,
                           bool hard_collision_active,
                           double obstacle_speed_mps,
                           const TrafficPredictionConfig &config) {
  const double target_speed_mps =
      std::max(0.0, std::min(obstacle_speed_mps, config.maximum_speed_mps));
  if (hard_collision_active) {
    return target_speed_mps;
  }
  if (intrusion_depth_meters <= 0.0) {
    return config.maximum_speed_mps;
  }
  // Linearly map first contour contact to the maximum speed and first hard
  // corridor contact to the adjacent vehicle's current longitudinal speed.
  double ratio = 0.0;
  if (hard_intrusion_depth_meters > 1e-9) {
    ratio = std::max(
        0.0, std::min(1.0, intrusion_depth_meters /
                               hard_intrusion_depth_meters));
  }
  return config.maximum_speed_mps -
         ratio * (config.maximum_speed_mps -
                  target_speed_mps);
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
      config.horizon_steps == 0 ||
      !std::isfinite(config.maximum_speed_mps) ||
      config.maximum_speed_mps <= 0.0 ||
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
  const double hard_corridor_min = lateral_corridor_min - lateral_clearance;
  const double hard_corridor_max = lateral_corridor_max + lateral_clearance;
  const double lane_boundary_min =
      lane_center_d - 0.5 * config.lane_width_meters;
  const double lane_boundary_max =
      lane_center_d + 0.5 * config.lane_width_meters;
  const double obstacle_half_width = 0.5 * config.obstacle_width_meters;
  const std::size_t nodes = config.horizon_steps + 1;

  std::vector<PredictedObstacle> predicted;
  predicted.reserve(input.traffic.size());
  for (const DetectedVehicle &vehicle : input.traffic) {
    if (!std::isfinite(vehicle.id) || !std::isfinite(vehicle.vx_mps) ||
        !std::isfinite(vehicle.vy_mps) || !std::isfinite(vehicle.s) ||
        !std::isfinite(vehicle.d)) {
      throw std::invalid_argument("traffic contains a non-finite value");
    }
    const double speed = ProjectLongitudinalSpeed(vehicle, map);
    if (!std::isfinite(speed)) {
      throw std::invalid_argument("traffic speed prediction is non-finite");
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

    // Lateral relevance is a current-frame geometric decision. Do not project
    // an adjacent vehicle's contour with measured lateral velocity: a vehicle
    // that has not crossed the lane line in the current sensor frame must not
    // create a future intrusion cap or hard-collision activation.
    const bool hard_collision_active =
        vehicle.d >= hard_corridor_min && vehicle.d <= hard_corridor_max;
    obstacle.hard_collision_active.assign(
        nodes, hard_collision_active ? 1U : 0U);

    // Vehicles already centered inside the target lane use the ordinary
    // longitudinal following model without an intrusion-specific speed cap.
    int intrusion_side = 0;
    if (vehicle.d <= lane_boundary_min) {
      intrusion_side = -1;
    } else if (vehicle.d >= lane_boundary_max) {
      intrusion_side = 1;
    }
    const double hard_intrusion_depth =
        intrusion_side < 0
            ? hard_corridor_min + obstacle_half_width - lane_boundary_min
            : lane_boundary_max -
                  (hard_corridor_max - obstacle_half_width);
    bool any_intrusion_speed_limit = false;
    if (intrusion_side != 0) {
      const double intrusion_depth =
          intrusion_side < 0
              ? vehicle.d + obstacle_half_width - lane_boundary_min
              : lane_boundary_max -
                    (vehicle.d - obstacle_half_width);
      const double speed_limit =
          IntrusionSpeedLimit(intrusion_depth, hard_intrusion_depth,
                              hard_collision_active, speed, config);
      any_intrusion_speed_limit =
          speed_limit < config.maximum_speed_mps - kSpeedLimitToleranceMps;
      if (any_intrusion_speed_limit) {
        obstacle.intrusion_speed_limit_mps.assign(nodes, speed_limit);
      }
    }
    if (!hard_collision_active && !any_intrusion_speed_limit) {
      continue;
    }
    predicted.push_back(obstacle);
  }

  std::sort(predicted.begin(), predicted.end(),
            [](const PredictedObstacle &left, const PredictedObstacle &right) {
              return left.relative_s < right.relative_s;
            });
  return predicted;
}
