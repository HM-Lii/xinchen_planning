#include "map.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace {

constexpr double kMinimumRoadTangentLength = 1e-8;
constexpr double kArcLengthIntegrationParameterStep = 0.2;
constexpr std::size_t kMaximumRoadArcLengthIndexEntries = 1000000;

double Distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

bool IsFiniteVector(const std::vector<double> &values) {
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool HasPreparedSplines(const MapData &map) {
  const std::size_t size = map.x.size();
  return map.spline_x_second.size() == size &&
         map.spline_y_second.size() == size &&
         map.spline_dx_second.size() == size &&
         map.spline_dy_second.size() == size;
}

std::vector<double> PeriodicSecondDerivatives(const std::vector<double> &values,
                                              const std::vector<double> &knots,
                                              double period) {
  const std::size_t size = values.size();
  if (size < 2 || knots.size() != size || period <= knots.back()) {
    throw std::invalid_argument("invalid periodic spline data");
  }

  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(
      static_cast<Eigen::Index>(size), static_cast<Eigen::Index>(size));
  Eigen::VectorXd right_hand_side =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(size));
  for (std::size_t index = 0; index < size; ++index) {
    const std::size_t previous = (index + size - 1) % size;
    const std::size_t next = (index + 1) % size;
    const double previous_interval =
        index == 0 ? period - knots.back() : knots[index] - knots[previous];
    const double next_interval =
        index + 1 == size ? period - knots.back() : knots[next] - knots[index];
    const double previous_slope =
        (values[index] - values[previous]) / previous_interval;
    const double next_slope = (values[next] - values[index]) / next_interval;

    matrix(static_cast<Eigen::Index>(index),
           static_cast<Eigen::Index>(previous)) += previous_interval;
    matrix(static_cast<Eigen::Index>(index),
           static_cast<Eigen::Index>(index)) +=
        2.0 * (previous_interval + next_interval);
    matrix(static_cast<Eigen::Index>(index), static_cast<Eigen::Index>(next)) +=
        next_interval;
    right_hand_side[static_cast<Eigen::Index>(index)] =
        6.0 * (next_slope - previous_slope);
  }

  const Eigen::LDLT<Eigen::MatrixXd> factorization(matrix);
  if (factorization.info() != Eigen::Success) {
    throw std::runtime_error("periodic map spline factorization failed");
  }
  const Eigen::VectorXd solution = factorization.solve(right_hand_side);
  if (factorization.info() != Eigen::Success || !solution.allFinite()) {
    throw std::runtime_error("periodic map spline solve failed");
  }
  return std::vector<double>(solution.data(),
                             solution.data() + solution.size());
}

struct SplineValue {
  double value = 0.0;
  double first_derivative = 0.0;
  double second_derivative = 0.0;
};

SplineValue EvaluatePeriodicSpline(const std::vector<double> &values,
                                   const std::vector<double> &second,
                                   const std::vector<double> &knots,
                                   double period, double wrapped_s) {
  const auto upper = std::upper_bound(knots.begin(), knots.end(), wrapped_s);
  std::size_t previous = 0;
  std::size_t next = 1;
  double segment_start_s = knots.front();
  double segment_end_s = knots[1];
  if (upper == knots.end()) {
    previous = knots.size() - 1;
    next = 0;
    segment_start_s = knots.back();
    segment_end_s = period;
  } else {
    next = static_cast<std::size_t>(upper - knots.begin());
    previous = next - 1;
    segment_start_s = knots[previous];
    segment_end_s = knots[next];
  }

  const double interval = segment_end_s - segment_start_s;
  const double right_weight = (wrapped_s - segment_start_s) / interval;
  const double left_weight = 1.0 - right_weight;
  const double left_second = second[previous];
  const double right_second = second[next];

  SplineValue result;
  result.value =
      left_weight * values[previous] + right_weight * values[next] +
      ((left_weight * left_weight * left_weight - left_weight) * left_second +
       (right_weight * right_weight * right_weight - right_weight) *
           right_second) *
          interval * interval / 6.0;
  result.first_derivative =
      (values[next] - values[previous]) / interval +
      interval / 6.0 *
          (-(3.0 * left_weight * left_weight - 1.0) * left_second +
           (3.0 * right_weight * right_weight - 1.0) * right_second);
  result.second_derivative =
      left_weight * left_second + right_weight * right_second;
  return result;
}

} // namespace

MapData LoadMap(const std::string &path) {
  std::ifstream stream(path.c_str(), std::ifstream::in);
  if (!stream) {
    throw std::runtime_error("failed to open map file: " + path);
  }

  MapData map;
  std::string line;
  while (std::getline(stream, line)) {
    std::istringstream waypoint(line);
    double x = 0.0;
    double y = 0.0;
    double s = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    if (!(waypoint >> x >> y >> s >> dx >> dy)) {
      throw std::runtime_error("invalid waypoint: " + line);
    }
    map.x.push_back(x);
    map.y.push_back(y);
    map.s.push_back(s);
    map.dx.push_back(dx);
    map.dy.push_back(dy);
  }

  if (map.x.size() >= 2) {
    map.track_length =
        map.s.back() +
        Distance(map.x.back(), map.y.back(), map.x.front(), map.y.front()) -
        map.s.front();
  }

  std::string error;
  if (!ValidateMap(map, &error)) {
    throw std::runtime_error("invalid map: " + error);
  }
  PrepareMapSplines(&map);
  return map;
}

void PrepareMapSplines(MapData *map) {
  if (map == nullptr) {
    throw std::invalid_argument("map pointer must not be null");
  }
  std::string error;
  if (!ValidateMap(*map, &error)) {
    throw std::invalid_argument("invalid map: " + error);
  }
  map->spline_x_second =
      PeriodicSecondDerivatives(map->x, map->s, map->track_length);
  map->spline_y_second =
      PeriodicSecondDerivatives(map->y, map->s, map->track_length);
  map->spline_dx_second =
      PeriodicSecondDerivatives(map->dx, map->s, map->track_length);
  map->spline_dy_second =
      PeriodicSecondDerivatives(map->dy, map->s, map->track_length);
}

bool ValidateMap(const MapData &map, std::string *error) {
  const std::size_t size = map.x.size();
  if (size < 2) {
    if (error != nullptr) {
      *error = "at least two waypoints are required";
    }
    return false;
  }
  if (map.y.size() != size || map.s.size() != size || map.dx.size() != size ||
      map.dy.size() != size) {
    if (error != nullptr) {
      *error = "waypoint arrays have different lengths";
    }
    return false;
  }
  if (!IsFiniteVector(map.x) || !IsFiniteVector(map.y) ||
      !IsFiniteVector(map.s) || !IsFiniteVector(map.dx) ||
      !IsFiniteVector(map.dy) || !std::isfinite(map.track_length)) {
    if (error != nullptr) {
      *error = "map contains a non-finite value";
    }
    return false;
  }
  if (std::fabs(map.s.front()) > 1e-6) {
    if (error != nullptr) {
      *error = "the first Frenet s value must be zero";
    }
    return false;
  }
  for (std::size_t i = 1; i < size; ++i) {
    if (map.s[i] <= map.s[i - 1]) {
      if (error != nullptr) {
        *error = "Frenet s values must be strictly increasing";
      }
      return false;
    }
  }
  if (map.track_length <= map.s.back()) {
    if (error != nullptr) {
      *error = "track length must include the closing map segment";
    }
    return false;
  }
  const bool any_spline_data =
      !map.spline_x_second.empty() || !map.spline_y_second.empty() ||
      !map.spline_dx_second.empty() || !map.spline_dy_second.empty();
  if (any_spline_data &&
      (!HasPreparedSplines(map) || !IsFiniteVector(map.spline_x_second) ||
       !IsFiniteVector(map.spline_y_second) ||
       !IsFiniteVector(map.spline_dx_second) ||
       !IsFiniteVector(map.spline_dy_second))) {
    if (error != nullptr) {
      *error = "map spline arrays are incomplete or non-finite";
    }
    return false;
  }
  return true;
}

double NormalizeS(double s, double track_length) {
  if (!std::isfinite(s) || !std::isfinite(track_length) ||
      track_length <= 0.0) {
    throw std::invalid_argument("invalid Frenet s or track length");
  }
  double wrapped = std::fmod(s, track_length);
  if (wrapped < 0.0) {
    wrapped += track_length;
  }
  return wrapped;
}

RoadGeometrySample EvaluateRoadGeometryOnValidatedMap(double s, double d,
                                                       const MapData &map) {
  if (!std::isfinite(d)) {
    throw std::invalid_argument("Frenet d must be finite");
  }

  const bool has_prepared_splines = HasPreparedSplines(map);
  std::vector<double> x_second;
  std::vector<double> y_second;
  std::vector<double> dx_second;
  std::vector<double> dy_second;
  if (!has_prepared_splines) {
    x_second = PeriodicSecondDerivatives(map.x, map.s, map.track_length);
    y_second = PeriodicSecondDerivatives(map.y, map.s, map.track_length);
    dx_second = PeriodicSecondDerivatives(map.dx, map.s, map.track_length);
    dy_second = PeriodicSecondDerivatives(map.dy, map.s, map.track_length);
  }
  const std::vector<double> &prepared_x =
      has_prepared_splines ? map.spline_x_second : x_second;
  const std::vector<double> &prepared_y =
      has_prepared_splines ? map.spline_y_second : y_second;
  const std::vector<double> &prepared_dx =
      has_prepared_splines ? map.spline_dx_second : dx_second;
  const std::vector<double> &prepared_dy =
      has_prepared_splines ? map.spline_dy_second : dy_second;

  const double wrapped_s = NormalizeS(s, map.track_length);
  const SplineValue center_x = EvaluatePeriodicSpline(
      map.x, prepared_x, map.s, map.track_length, wrapped_s);
  const SplineValue center_y = EvaluatePeriodicSpline(
      map.y, prepared_y, map.s, map.track_length, wrapped_s);
  const SplineValue normal_x = EvaluatePeriodicSpline(
      map.dx, prepared_dx, map.s, map.track_length, wrapped_s);
  const SplineValue normal_y = EvaluatePeriodicSpline(
      map.dy, prepared_dy, map.s, map.track_length, wrapped_s);

  RoadGeometrySample result;
  result.x = center_x.value + d * normal_x.value;
  result.y = center_y.value + d * normal_y.value;
  result.first_derivative_x =
      center_x.first_derivative + d * normal_x.first_derivative;
  result.first_derivative_y =
      center_y.first_derivative + d * normal_y.first_derivative;
  result.second_derivative_x =
      center_x.second_derivative + d * normal_x.second_derivative;
  result.second_derivative_y =
      center_y.second_derivative + d * normal_y.second_derivative;
  return result;
}

RoadGeometrySample EvaluateRoadGeometry(double s, double d,
                                        const MapData &map) {
  std::string error;
  if (!ValidateMap(map, &error)) {
    throw std::invalid_argument("invalid map: " + error);
  }
  return EvaluateRoadGeometryOnValidatedMap(s, d, map);
}

namespace {

void RequireValidMap(const MapData &map) {
  std::string error;
  if (!ValidateMap(map, &error)) {
    throw std::invalid_argument("invalid map: " + error);
  }
}

double RoadParameterMetricOnValidatedMap(double s, double d,
                                         const MapData &map) {
  const RoadGeometrySample geometry =
      EvaluateRoadGeometryOnValidatedMap(s, d, map);
  const double magnitude =
      std::hypot(geometry.first_derivative_x, geometry.first_derivative_y);
  if (!std::isfinite(magnitude) || magnitude <= kMinimumRoadTangentLength) {
    throw std::runtime_error("road spline has a degenerate tangent");
  }
  return magnitude;
}

double RoadArcLengthOnValidatedMap(double start_s, double parameter_distance,
                                   double d, const MapData &map) {
  std::size_t intervals = static_cast<std::size_t>(
      std::ceil(parameter_distance / kArcLengthIntegrationParameterStep));
  intervals = std::max<std::size_t>(2, intervals);
  if (intervals % 2 != 0) {
    ++intervals;
  }
  const double step = parameter_distance / static_cast<double>(intervals);
  double weighted_sum = RoadParameterMetricOnValidatedMap(start_s, d, map) +
                        RoadParameterMetricOnValidatedMap(
                            start_s + parameter_distance, d, map);
  for (std::size_t index = 1; index < intervals; ++index) {
    const double weight = index % 2 == 0 ? 2.0 : 4.0;
    weighted_sum +=
        weight * RoadParameterMetricOnValidatedMap(
                     start_s + static_cast<double>(index) * step, d, map);
  }
  return weighted_sum * step / 3.0;
}

} // namespace

double RoadParameterMetric(double s, double d, const MapData &map) {
  RequireValidMap(map);
  return RoadParameterMetricOnValidatedMap(s, d, map);
}

double RoadArcLength(double start_s, double parameter_distance, double d,
                     const MapData &map) {
  if (!std::isfinite(start_s) || !std::isfinite(parameter_distance) ||
      !std::isfinite(d) || parameter_distance < 0.0) {
    throw std::invalid_argument("invalid road arc-length input");
  }
  if (parameter_distance <= 0.0) {
    return 0.0;
  }
  RequireValidMap(map);
  return RoadArcLengthOnValidatedMap(start_s, parameter_distance, d, map);
}

double AdvanceRoadParameter(double start_s, double distance_meters, double d,
                            const MapData &map) {
  if (!std::isfinite(start_s) || !std::isfinite(distance_meters) ||
      !std::isfinite(d) || distance_meters < 0.0) {
    throw std::invalid_argument("invalid road-parameter advance input");
  }
  if (distance_meters <= 1e-12) {
    return start_s;
  }

  RequireValidMap(map);
  double parameter_distance =
      distance_meters / RoadParameterMetricOnValidatedMap(start_s, d, map);
  parameter_distance = std::max(parameter_distance, 1e-9);
  for (int iteration = 0; iteration < 6; ++iteration) {
    const double measured_distance = RoadArcLengthOnValidatedMap(
        start_s, parameter_distance, d, map);
    const double error = measured_distance - distance_meters;
    if (std::fabs(error) <= 1e-9) {
      break;
    }
    const double end_metric = RoadParameterMetricOnValidatedMap(
        start_s + parameter_distance, d, map);
    parameter_distance -= error / end_metric;
    parameter_distance = std::max(parameter_distance, 1e-9);
  }
  return start_s + parameter_distance;
}

RoadArcLengthIndex BuildRoadArcLengthIndexOnValidatedMap(
    double start_s, double maximum_distance_meters, double d,
    const MapData &map) {
  if (!std::isfinite(start_s) || !std::isfinite(maximum_distance_meters) ||
      !std::isfinite(d) || maximum_distance_meters < 0.0) {
    throw std::invalid_argument("invalid road arc-length index input");
  }

  RoadArcLengthIndex index;
  index.start_road_parameter_s = start_s;
  index.lateral_offset_m = d;
  index.road_parameter_s.push_back(start_s);
  index.cumulative_arc_length_m.push_back(0.0);
  index.parameter_metric.push_back(
      RoadParameterMetricOnValidatedMap(start_s, d, map));
  index.interval_midpoint_metric.push_back(0.0);

  while (index.cumulative_arc_length_m.back() + 1e-12 <
         maximum_distance_meters) {
    if (index.road_parameter_s.size() >=
        kMaximumRoadArcLengthIndexEntries) {
      throw std::runtime_error("road arc-length index exceeded its bound");
    }
    const double lower_s = index.road_parameter_s.back();
    const double midpoint_s =
        lower_s + 0.5 * kArcLengthIntegrationParameterStep;
    const double upper_s = lower_s + kArcLengthIntegrationParameterStep;
    const double midpoint_metric =
        RoadParameterMetricOnValidatedMap(midpoint_s, d, map);
    const double upper_metric =
        RoadParameterMetricOnValidatedMap(upper_s, d, map);
    const double interval_length =
        kArcLengthIntegrationParameterStep *
        (index.parameter_metric.back() + 4.0 * midpoint_metric +
         upper_metric) /
        6.0;
    if (!std::isfinite(interval_length) || interval_length <= 0.0) {
      throw std::runtime_error("road arc-length index is not monotonic");
    }
    index.road_parameter_s.push_back(upper_s);
    index.cumulative_arc_length_m.push_back(
        index.cumulative_arc_length_m.back() + interval_length);
    index.parameter_metric.push_back(upper_metric);
    index.interval_midpoint_metric.push_back(midpoint_metric);
  }
  return index;
}

RoadArcLengthIndex BuildRoadArcLengthIndex(double start_s,
                                           double maximum_distance_meters,
                                           double d, const MapData &map) {
  RequireValidMap(map);
  return BuildRoadArcLengthIndexOnValidatedMap(
      start_s, maximum_distance_meters, d, map);
}

double RoadParameterAtArcLength(const RoadArcLengthIndex &index,
                                double distance_meters) {
  const std::size_t size = index.road_parameter_s.size();
  if (!std::isfinite(distance_meters) || distance_meters < 0.0 || size == 0 ||
      index.cumulative_arc_length_m.size() != size ||
      index.parameter_metric.size() != size ||
      index.interval_midpoint_metric.size() != size) {
    throw std::invalid_argument("invalid road arc-length lookup");
  }
  const double maximum_distance = index.cumulative_arc_length_m.back();
  if (distance_meters > maximum_distance + 1e-9) {
    throw std::out_of_range("road arc-length lookup exceeds index coverage");
  }
  const double target = std::min(distance_meters, maximum_distance);
  const auto upper = std::lower_bound(index.cumulative_arc_length_m.begin(),
                                      index.cumulative_arc_length_m.end(),
                                      target);
  if (upper == index.cumulative_arc_length_m.begin()) {
    return index.road_parameter_s.front();
  }
  if (upper == index.cumulative_arc_length_m.end()) {
    return index.road_parameter_s.back();
  }

  const std::size_t upper_index = static_cast<std::size_t>(
      upper - index.cumulative_arc_length_m.begin());
  const std::size_t lower_index = upper_index - 1;
  const double arc_offset =
      target - index.cumulative_arc_length_m[lower_index];
  const double interval_arc_length =
      index.cumulative_arc_length_m[upper_index] -
      index.cumulative_arc_length_m[lower_index];
  const double parameter_step = index.road_parameter_s[upper_index] -
                                index.road_parameter_s[lower_index];
  const double lower_metric = index.parameter_metric[lower_index];
  const double midpoint_metric =
      index.interval_midpoint_metric[upper_index];
  const double upper_metric = index.parameter_metric[upper_index];
  const double quadratic =
      2.0 * (lower_metric + upper_metric - 2.0 * midpoint_metric);
  const double linear =
      4.0 * midpoint_metric - 3.0 * lower_metric - upper_metric;
  double u = arc_offset / interval_arc_length;
  u = std::max(0.0, std::min(1.0, u));
  for (int iteration = 0; iteration < 6; ++iteration) {
    const double integral =
        parameter_step *
        (quadratic * u * u * u / 3.0 + linear * u * u / 2.0 +
         lower_metric * u);
    const double derivative =
        parameter_step *
        (quadratic * u * u + linear * u + lower_metric);
    if (!std::isfinite(derivative) ||
        std::fabs(derivative) <= kMinimumRoadTangentLength) {
      break;
    }
    u = std::max(0.0,
                 std::min(1.0, u - (integral - arc_offset) / derivative));
  }
  return index.road_parameter_s[lower_index] + u * parameter_step;
}

RoadProjection ProjectCartesianToRoad(double x, double y,
                                      double initial_road_s_unwrapped_m,
                                      const MapData &map) {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(initial_road_s_unwrapped_m)) {
    throw std::invalid_argument("invalid Cartesian road projection input");
  }
  std::string map_error;
  if (!ValidateMap(map, &map_error)) {
    throw std::invalid_argument("invalid map: " + map_error);
  }

  double road_s = initial_road_s_unwrapped_m;
  double d = 0.0;
  for (int iteration = 0; iteration < 16; ++iteration) {
    const RoadGeometrySample center =
        EvaluateRoadGeometryOnValidatedMap(road_s, 0.0, map);
    const RoadGeometrySample unit_offset =
        EvaluateRoadGeometryOnValidatedMap(road_s, 1.0, map);
    const double normal_x = unit_offset.x - center.x;
    const double normal_y = unit_offset.y - center.y;
    const double normal_squared = normal_x * normal_x + normal_y * normal_y;
    if (!std::isfinite(normal_squared) || normal_squared <= 1e-12) {
      throw std::runtime_error("road projection normal is degenerate");
    }
    d = ((x - center.x) * normal_x + (y - center.y) * normal_y) /
        normal_squared;

    const RoadGeometrySample road =
        EvaluateRoadGeometryOnValidatedMap(road_s, d, map);
    const double residual_x = road.x - x;
    const double residual_y = road.y - y;
    const double first = residual_x * road.first_derivative_x +
                         residual_y * road.first_derivative_y;
    const double second =
        road.first_derivative_x * road.first_derivative_x +
        road.first_derivative_y * road.first_derivative_y +
        residual_x * road.second_derivative_x +
        residual_y * road.second_derivative_y;
    if (!std::isfinite(second) || std::fabs(second) <= 1e-12) {
      break;
    }
    double step = first / second;
    const double maximum_step = 0.05 * map.track_length;
    step = std::max(-maximum_step, std::min(step, maximum_step));
    road_s -= step;
    if (std::fabs(step) <= 1e-10) {
      break;
    }
  }

  const RoadGeometrySample center =
      EvaluateRoadGeometryOnValidatedMap(road_s, 0.0, map);
  const RoadGeometrySample unit_offset =
      EvaluateRoadGeometryOnValidatedMap(road_s, 1.0, map);
  const double normal_x = unit_offset.x - center.x;
  const double normal_y = unit_offset.y - center.y;
  const double normal_squared = normal_x * normal_x + normal_y * normal_y;
  if (!std::isfinite(normal_squared) || normal_squared <= 1e-12) {
    throw std::runtime_error("road projection normal is degenerate");
  }
  d = ((x - center.x) * normal_x + (y - center.y) * normal_y) /
      normal_squared;
  const RoadGeometrySample projected =
      EvaluateRoadGeometryOnValidatedMap(road_s, d, map);

  RoadProjection result;
  result.road_s_unwrapped_m = road_s;
  result.d_m = d;
  result.residual_m = std::hypot(projected.x - x, projected.y - y);
  if (!std::isfinite(result.road_s_unwrapped_m) ||
      !std::isfinite(result.d_m) || !std::isfinite(result.residual_m)) {
    throw std::runtime_error("road projection produced a non-finite result");
  }
  return result;
}

std::pair<double, double> FrenetToCartesian(double s, double d,
                                            const MapData &map) {
  const RoadGeometrySample geometry = EvaluateRoadGeometry(s, d, map);
  return {geometry.x, geometry.y};
}
