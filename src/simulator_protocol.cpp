#include "simulator_protocol.h"

#include <cmath>
#include <exception>
#include <stdexcept>
#include <vector>

#include "json.hpp"

namespace {

using nlohmann::json;

bool ReadNumber(const json &object, const char *key, double *value,
                std::string *error) {
  if (object.count(key) != 1 || !object.at(key).is_number()) {
    *error = std::string("missing or non-numeric field: ") + key;
    return false;
  }
  *value = object.at(key).get<double>();
  if (!std::isfinite(*value)) {
    *error = std::string("non-finite field: ") + key;
    return false;
  }
  return true;
}

bool ReadNumberArray(const json &object, const char *key,
                     std::vector<double> *values, std::string *error) {
  if (object.count(key) != 1 || !object.at(key).is_array()) {
    *error = std::string("missing or non-array field: ") + key;
    return false;
  }
  values->clear();
  for (const auto &item : object.at(key)) {
    if (!item.is_number()) {
      *error = std::string("non-numeric array item in: ") + key;
      return false;
    }
    const double value = item.get<double>();
    if (!std::isfinite(value)) {
      *error = std::string("non-finite array item in: ") + key;
      return false;
    }
    values->push_back(value);
  }
  return true;
}

bool ReadTraffic(const json &object, std::vector<DetectedVehicle> *traffic,
                 std::string *error) {
  const char *key = "sensor_fusion";
  if (object.count(key) != 1 || !object.at(key).is_array()) {
    *error = "missing or non-array field: sensor_fusion";
    return false;
  }

  traffic->clear();
  for (const auto &row : object.at(key)) {
    if (!row.is_array() || row.size() != 7) {
      *error = "each sensor_fusion row must contain exactly 7 numbers";
      return false;
    }
    double values[7] = {};
    for (std::size_t i = 0; i < 7; ++i) {
      if (!row[i].is_number()) {
        *error = "sensor_fusion contains a non-numeric value";
        return false;
      }
      values[i] = row[i].get<double>();
      if (!std::isfinite(values[i])) {
        *error = "sensor_fusion contains a non-finite value";
        return false;
      }
    }
    DetectedVehicle vehicle;
    vehicle.id = values[0];
    vehicle.x = values[1];
    vehicle.y = values[2];
    vehicle.vx_mps = values[3];
    vehicle.vy_mps = values[4];
    vehicle.s = values[5];
    vehicle.d = values[6];
    traffic->push_back(vehicle);
  }
  return true;
}

SimulatorMessage InvalidMessage(const std::string &error) {
  SimulatorMessage result;
  result.kind = SimulatorMessageKind::kManual;
  result.error = error;
  return result;
}

} // namespace

SimulatorMessage ParseSimulatorMessage(const std::string &data) {
  if (data.size() < 2 || data[0] != '4' || data[1] != '2') {
    return SimulatorMessage{};
  }
  if (data.size() == 2 || data.compare(2, std::string::npos, "null") == 0) {
    return InvalidMessage("Socket.IO event has no payload");
  }

  try {
    const json message = json::parse(data.substr(2));
    if (!message.is_array() || message.size() != 2 || !message[0].is_string()) {
      return InvalidMessage("Socket.IO payload must be [event, object]");
    }
    if (message[0].get<std::string>() != "telemetry") {
      return SimulatorMessage{};
    }
    if (!message[1].is_object()) {
      return InvalidMessage("telemetry payload must be an object");
    }

    SimulatorMessage result;
    result.kind = SimulatorMessageKind::kTelemetry;
    const json &telemetry = message[1];
    if (!ReadNumber(telemetry, "x", &result.input.ego.x, &result.error) ||
        !ReadNumber(telemetry, "y", &result.input.ego.y, &result.error) ||
        !ReadNumber(telemetry, "s", &result.input.ego.s, &result.error) ||
        !ReadNumber(telemetry, "d", &result.input.ego.d, &result.error) ||
        !ReadNumber(telemetry, "yaw", &result.input.ego.yaw_deg,
                    &result.error) ||
        !ReadNumber(telemetry, "speed", &result.input.ego.speed_mph,
                    &result.error) ||
        !ReadNumberArray(telemetry, "previous_path_x",
                         &result.input.previous_path_x, &result.error) ||
        !ReadNumberArray(telemetry, "previous_path_y",
                         &result.input.previous_path_y, &result.error) ||
        !ReadNumber(telemetry, "end_path_s", &result.input.end_path_s,
                    &result.error) ||
        !ReadNumber(telemetry, "end_path_d", &result.input.end_path_d,
                    &result.error) ||
        !ReadTraffic(telemetry, &result.input.traffic, &result.error)) {
      result.kind = SimulatorMessageKind::kManual;
      return result;
    }

    if (result.input.previous_path_x.size() !=
        result.input.previous_path_y.size()) {
      return InvalidMessage("previous path x/y arrays have different lengths");
    }
    if (result.input.previous_path_x.size() > 50) {
      return InvalidMessage("previous path contains more than 50 points");
    }
    return result;
  } catch (const std::exception &error) {
    return InvalidMessage(std::string("invalid JSON payload: ") + error.what());
  }
}

std::string MakeControlMessage(const PlannerOutput &output) {
  if (output.next_x.size() != 50 || output.next_y.size() != 50 ||
      output.next_x.size() != output.next_y.size()) {
    throw std::invalid_argument(
        "control output must contain exactly 50 x/y point pairs");
  }
  for (std::size_t i = 0; i < output.next_x.size(); ++i) {
    if (!std::isfinite(output.next_x[i]) || !std::isfinite(output.next_y[i])) {
      throw std::invalid_argument("control output contains a non-finite value");
    }
  }
  json response;
  response["next_x"] = output.next_x;
  response["next_y"] = output.next_y;
  return "42[\"control\"," + response.dump() + "]";
}

std::string MakeManualMessage() { return "42[\"manual\",{}]"; }
