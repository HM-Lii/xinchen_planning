#ifndef HELPERS_H
#define HELPERS_H

// Compatibility helpers retained for existing student code. New code should
// use map.h and simulator_protocol.h, whose interfaces validate their inputs.

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using std::string;
using std::vector;

inline string hasData(const string &value) {
  const std::size_t array_start = value.find('[');
  if (array_start == string::npos) {
    return "";
  }
  return value.substr(array_start);
}

constexpr double pi() { return 3.14159265358979323846; }
inline double deg2rad(double value) { return value * pi() / 180.0; }
inline double rad2deg(double value) { return value * 180.0 / pi(); }

inline double distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

inline void ValidateCoordinates(const vector<double> &maps_x,
                                const vector<double> &maps_y) {
  if (maps_x.size() < 2 || maps_x.size() != maps_y.size()) {
    throw std::invalid_argument(
        "waypoint x/y arrays must have equal lengths of at least two");
  }
}

inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                           const vector<double> &maps_y) {
  ValidateCoordinates(maps_x, maps_y);
  double closest_length = std::numeric_limits<double>::max();
  int closest_waypoint = 0;
  for (std::size_t i = 0; i < maps_x.size(); ++i) {
    const double candidate = distance(x, y, maps_x[i], maps_y[i]);
    if (candidate < closest_length) {
      closest_length = candidate;
      closest_waypoint = static_cast<int>(i);
    }
  }
  return closest_waypoint;
}

inline int NextWaypoint(double x, double y, double theta,
                        const vector<double> &maps_x,
                        const vector<double> &maps_y) {
  int closest_waypoint = ClosestWaypoint(x, y, maps_x, maps_y);
  const double heading =
      std::atan2(maps_y[closest_waypoint] - y, maps_x[closest_waypoint] - x);
  double angle = std::fabs(theta - heading);
  angle = std::fmod(angle, 2.0 * pi());
  angle = std::min(2.0 * pi() - angle, angle);
  if (angle > pi() / 2.0) {
    closest_waypoint = (closest_waypoint + 1) % static_cast<int>(maps_x.size());
  }
  return closest_waypoint;
}

inline vector<double> getFrenet(double x, double y, double theta,
                                const vector<double> &maps_x,
                                const vector<double> &maps_y) {
  ValidateCoordinates(maps_x, maps_y);
  const int next_waypoint = NextWaypoint(x, y, theta, maps_x, maps_y);
  const int previous_waypoint =
      (next_waypoint + static_cast<int>(maps_x.size()) - 1) %
      static_cast<int>(maps_x.size());

  const double segment_x = maps_x[next_waypoint] - maps_x[previous_waypoint];
  const double segment_y = maps_y[next_waypoint] - maps_y[previous_waypoint];
  const double position_x = x - maps_x[previous_waypoint];
  const double position_y = y - maps_y[previous_waypoint];
  const double segment_length_squared =
      segment_x * segment_x + segment_y * segment_y;
  if (segment_length_squared <= 1e-12) {
    throw std::invalid_argument("map contains a zero-length segment");
  }

  const double projection = (position_x * segment_x + position_y * segment_y) /
                            segment_length_squared;
  const double projection_x = projection * segment_x;
  const double projection_y = projection * segment_y;
  double frenet_d =
      distance(position_x, position_y, projection_x, projection_y);
  const double cross = segment_x * position_y - segment_y * position_x;
  if (cross > 0.0) {
    frenet_d *= -1.0;
  }

  double frenet_s = 0.0;
  for (int i = 0; i < previous_waypoint; ++i) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }
  frenet_s += std::hypot(projection_x, projection_y);
  return {frenet_s, frenet_d};
}

inline vector<double> getXY(double s, double d, const vector<double> &maps_s,
                            const vector<double> &maps_x,
                            const vector<double> &maps_y) {
  ValidateCoordinates(maps_x, maps_y);
  if (maps_s.size() != maps_x.size()) {
    throw std::invalid_argument("waypoint s/x/y arrays have different lengths");
  }
  for (std::size_t i = 1; i < maps_s.size(); ++i) {
    if (maps_s[i] <= maps_s[i - 1]) {
      throw std::invalid_argument("waypoint s values must be increasing");
    }
  }

  const double track_length =
      maps_s.back() +
      distance(maps_x.back(), maps_y.back(), maps_x.front(), maps_y.front()) -
      maps_s.front();
  if (!std::isfinite(s) || !std::isfinite(d) || track_length <= maps_s.back()) {
    throw std::invalid_argument("invalid Frenet coordinate or track length");
  }
  double wrapped_s = std::fmod(s, track_length);
  if (wrapped_s < 0.0) {
    wrapped_s += track_length;
  }

  const auto upper = std::upper_bound(maps_s.begin(), maps_s.end(), wrapped_s);
  std::size_t previous = 0;
  std::size_t next = 1;
  double segment_start_s = maps_s.front();
  if (upper == maps_s.end()) {
    previous = maps_s.size() - 1;
    next = 0;
    segment_start_s = maps_s.back();
  } else {
    next = static_cast<std::size_t>(upper - maps_s.begin());
    previous = next - 1;
    segment_start_s = maps_s[previous];
  }

  const double heading = std::atan2(maps_y[next] - maps_y[previous],
                                    maps_x[next] - maps_x[previous]);
  const double segment_s = wrapped_s - segment_start_s;
  const double segment_x = maps_x[previous] + segment_s * std::cos(heading);
  const double segment_y = maps_y[previous] + segment_s * std::sin(heading);
  const double perpendicular_heading = heading - pi() / 2.0;
  return {segment_x + d * std::cos(perpendicular_heading),
          segment_y + d * std::sin(perpendicular_heading)};
}

#endif // HELPERS_H
