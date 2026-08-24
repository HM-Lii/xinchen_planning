#include "traffic_predictor.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "map.h"

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
                       double lane_center_d, double track_length,
                       const TrafficPredictionConfig &config) {
  if (!std::isfinite(plan_start_s) || !std::isfinite(lane_center_d) ||
      config.simulator_time_step_seconds <= 0.0 ||
      config.lane_width_meters <= 0.0 ||
      config.lane_boundary_margin_meters < 0.0 ||
      config.lookahead_distance_meters <= 0.0) {
    throw std::invalid_argument("invalid traffic prediction input");
  }

  const double prediction_delay =
      static_cast<double>(input.previous_path_x.size()) *
      config.simulator_time_step_seconds;
  const double lateral_threshold =
      0.5 * config.lane_width_meters + config.lane_boundary_margin_meters;

  std::vector<PredictedObstacle> predicted;
  predicted.reserve(input.traffic.size());
  for (const DetectedVehicle &vehicle : input.traffic) {
    if (std::fabs(vehicle.d - lane_center_d) > lateral_threshold) {
      continue;
    }
    const double speed = std::hypot(vehicle.vx_mps, vehicle.vy_mps);
    const double projected_s =
        NormalizeS(vehicle.s + speed * prediction_delay, track_length);
    const double relative_s =
        ForwardTrackDistance(plan_start_s, projected_s, track_length);
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
