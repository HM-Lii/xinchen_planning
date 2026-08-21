#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "helpers.h"
#include "map.h"
#include "planner.h"
#include "simulator_protocol.h"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

void ExpectNear(double actual, double expected, double tolerance,
                const std::string &message) {
  Expect(std::fabs(actual - expected) <= tolerance, message);
}

MapData SquareMap() {
  MapData map;
  map.x = {0.0, 10.0, 10.0, 0.0};
  map.y = {0.0, 0.0, 10.0, 10.0};
  map.s = {0.0, 10.0, 20.0, 30.0};
  map.dx = {0.0, 1.0, 0.0, -1.0};
  map.dy = {-1.0, 0.0, 1.0, 0.0};
  map.track_length = 40.0;
  return map;
}

void TestMapBoundaries() {
  const MapData map = SquareMap();
  std::string error;
  Expect(ValidateMap(map, &error), "square map should be valid");

  MapData shifted_s_map = map;
  shifted_s_map.s.front() = 1.0;
  Expect(!ValidateMap(shifted_s_map, &error),
         "map must use zero as the Frenet origin");

  const auto at_zero = FrenetToCartesian(0.0, 2.0, map);
  const auto at_wrap = FrenetToCartesian(40.0, 2.0, map);
  const auto at_negative_wrap = FrenetToCartesian(-40.0, 2.0, map);
  ExpectNear(at_zero.first, 0.0, 1e-9, "s=0 x coordinate");
  ExpectNear(at_zero.second, -2.0, 1e-9, "s=0 d offset");
  ExpectNear(at_wrap.first, at_zero.first, 1e-9, "track end wraps x");
  ExpectNear(at_wrap.second, at_zero.second, 1e-9, "track end wraps y");
  ExpectNear(at_negative_wrap.first, at_zero.first, 1e-9,
             "negative full lap wraps x");

  const std::vector<double> legacy = getXY(0.0, 0.0, map.s, map.x, map.y);
  ExpectNear(legacy[0], 0.0, 1e-9, "legacy getXY handles s=0");
  ExpectNear(legacy[1], 0.0, 1e-9, "legacy getXY handles s=0 y");
}

void TestHighwayMapLoading() {
  const MapData map = LoadMap("data/highway_map.csv");
  Expect(map.x.size() > 100, "highway map loads all waypoints");
  ExpectNear(map.track_length, 6945.554, 0.1,
             "highway map closing segment defines the full lap");
  const auto before_wrap = FrenetToCartesian(map.track_length - 0.01, 6.0, map);
  const auto after_wrap = FrenetToCartesian(map.track_length + 0.01, 6.0, map);
  Expect(
      std::isfinite(before_wrap.first) && std::isfinite(before_wrap.second) &&
          std::isfinite(after_wrap.first) && std::isfinite(after_wrap.second),
      "highway map conversion remains finite across the lap boundary");
}

std::string ValidTelemetryMessage() {
  return "42[\"telemetry\",{\"x\":0,\"y\":-6,\"s\":0,\"d\":6,"
         "\"yaw\":0,\"speed\":0,\"previous_path_x\":[],"
         "\"previous_path_y\":[],\"end_path_s\":0,\"end_path_d\":6,"
         "\"sensor_fusion\":[[7,1,2,3,4,5,6]]}]";
}

void TestProtocolContract() {
  const SimulatorMessage valid = ParseSimulatorMessage(ValidTelemetryMessage());
  Expect(valid.kind == SimulatorMessageKind::kTelemetry,
         "valid telemetry should parse");
  Expect(valid.input.traffic.size() == 1,
         "sensor fusion row should be converted");
  ExpectNear(valid.input.traffic[0].vx_mps, 3.0, 1e-9,
             "sensor fusion velocity mapping");

  const SimulatorMessage ignored = ParseSimulatorMessage("42[\"ping\",{}]");
  Expect(ignored.kind == SimulatorMessageKind::kIgnore,
         "unknown event should be ignored");

  const SimulatorMessage malformed =
      ParseSimulatorMessage("42[\"telemetry\",{\"x\":0}]");
  Expect(malformed.kind == SimulatorMessageKind::kManual,
         "missing telemetry fields should select manual mode");

  bool rejected_bad_output = false;
  try {
    MakeControlMessage(PlannerOutput{});
  } catch (const std::invalid_argument &) {
    rejected_bad_output = true;
  }
  Expect(
      rejected_bad_output,
      "control serializer rejects output that violates the 50-point contract");
}

void TestBaselinePlanner() {
  const MapData map = SquareMap();
  const SimulatorMessage parsed =
      ParseSimulatorMessage(ValidTelemetryMessage());
  PathPlanner planner;
  const PlannerOutput first = planner.Plan(parsed.input, map);
  Expect(first.next_x.size() == 50, "planner returns 50 x points");
  Expect(first.next_y.size() == 50, "planner returns 50 y points");
  ExpectNear(planner.reference_speed_mps(), 5.0, 1e-9,
             "one-second horizon respects 5 m/s^2 acceleration");
  for (std::size_t i = 0; i < first.next_x.size(); ++i) {
    Expect(std::isfinite(first.next_x[i]) && std::isfinite(first.next_y[i]),
           "planned coordinates are finite");
  }

  PlannerInput with_history = parsed.input;
  with_history.previous_path_x = {11.0, 12.0};
  with_history.previous_path_y = {21.0, 22.0};
  with_history.end_path_s = 5.0;
  with_history.end_path_d = 6.0;
  const PlannerOutput continued = planner.Plan(with_history, map);
  Expect(continued.next_x.size() == 50,
         "continued path is replenished to 50 points");
  ExpectNear(continued.next_x[0], 11.0, 1e-9,
             "first historical x point is preserved");
  ExpectNear(continued.next_y[1], 22.0, 1e-9,
             "historical y points are preserved");

  const std::string control = MakeControlMessage(continued);
  Expect(control.find("42[\"control\"") == 0,
         "planner output uses the control event contract");
}

} // namespace

int main() {
  TestMapBoundaries();
  TestHighwayMapLoading();
  TestProtocolContract();
  TestBaselinePlanner();
  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed" << std::endl;
    return 1;
  }
  std::cout << "All planning tests passed" << std::endl;
  return 0;
}
