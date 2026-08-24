#include "map.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace {

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

RoadGeometrySample EvaluateRoadGeometry(double s, double d,
                                        const MapData &map) {
  std::string error;
  if (!ValidateMap(map, &error)) {
    throw std::invalid_argument("invalid map: " + error);
  }
  if (!std::isfinite(d)) {
    throw std::invalid_argument("Frenet d must be finite");
  }

  std::vector<double> x_second;
  std::vector<double> y_second;
  std::vector<double> dx_second;
  std::vector<double> dy_second;
  if (!HasPreparedSplines(map)) {
    x_second = PeriodicSecondDerivatives(map.x, map.s, map.track_length);
    y_second = PeriodicSecondDerivatives(map.y, map.s, map.track_length);
    dx_second = PeriodicSecondDerivatives(map.dx, map.s, map.track_length);
    dy_second = PeriodicSecondDerivatives(map.dy, map.s, map.track_length);
  }
  const std::vector<double> &prepared_x =
      HasPreparedSplines(map) ? map.spline_x_second : x_second;
  const std::vector<double> &prepared_y =
      HasPreparedSplines(map) ? map.spline_y_second : y_second;
  const std::vector<double> &prepared_dx =
      HasPreparedSplines(map) ? map.spline_dx_second : dx_second;
  const std::vector<double> &prepared_dy =
      HasPreparedSplines(map) ? map.spline_dy_second : dy_second;

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

std::pair<double, double> FrenetToCartesian(double s, double d,
                                            const MapData &map) {
  const RoadGeometrySample geometry = EvaluateRoadGeometry(s, d, map);
  return {geometry.x, geometry.y};
}
