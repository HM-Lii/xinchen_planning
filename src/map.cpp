#include "map.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

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
  return map;
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

std::pair<double, double> FrenetToCartesian(double s, double d,
                                            const MapData &map) {
  std::string error;
  if (!ValidateMap(map, &error)) {
    throw std::invalid_argument("invalid map: " + error);
  }
  if (!std::isfinite(d)) {
    throw std::invalid_argument("Frenet d must be finite");
  }

  const double wrapped_s = NormalizeS(s, map.track_length);
  const auto upper = std::upper_bound(map.s.begin(), map.s.end(), wrapped_s);

  std::size_t previous = 0;
  std::size_t next = 1;
  double segment_start_s = map.s.front();
  double segment_end_s = map.s[1];
  if (upper == map.s.end()) {
    previous = map.s.size() - 1;
    next = 0;
    segment_start_s = map.s.back();
    segment_end_s = map.track_length;
  } else {
    next = static_cast<std::size_t>(upper - map.s.begin());
    previous = next - 1;
    segment_start_s = map.s[previous];
    segment_end_s = map.s[next];
  }

  const double segment_length = segment_end_s - segment_start_s;
  const double ratio = (wrapped_s - segment_start_s) / segment_length;
  const double center_x =
      map.x[previous] + ratio * (map.x[next] - map.x[previous]);
  const double center_y =
      map.y[previous] + ratio * (map.y[next] - map.y[previous]);

  double normal_x =
      map.dx[previous] + ratio * (map.dx[next] - map.dx[previous]);
  double normal_y =
      map.dy[previous] + ratio * (map.dy[next] - map.dy[previous]);
  const double normal_length = std::hypot(normal_x, normal_y);
  if (normal_length <= 1e-9) {
    const double tangent_x = map.x[next] - map.x[previous];
    const double tangent_y = map.y[next] - map.y[previous];
    const double tangent_length = std::hypot(tangent_x, tangent_y);
    if (tangent_length <= 1e-9) {
      throw std::runtime_error("map contains a zero-length segment");
    }
    normal_x = tangent_y / tangent_length;
    normal_y = -tangent_x / tangent_length;
  } else {
    normal_x /= normal_length;
    normal_y /= normal_length;
  }

  return {center_x + d * normal_x, center_y + d * normal_y};
}
