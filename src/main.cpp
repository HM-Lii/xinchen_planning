#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = boost::asio::ip::tcp;
using nlohmann::json;
using std::string;
using std::vector;

string HandleMessage(const string& data,
                     const vector<double>& map_waypoints_x,
                     const vector<double>& map_waypoints_y,
                     const vector<double>& map_waypoints_s,
                     const vector<double>& map_waypoints_dx,
                     const vector<double>& map_waypoints_dy) {
  if (data.size() <= 2 || data[0] != '4' || data[1] != '2') {
    return "";
  }

  const string payload = hasData(data);
  if (payload.empty()) {
    return "42[\"manual\",{}]";
  }

  try {
    const auto message = json::parse(payload);
    const string event = message[0].get<string>();
    if (event != "telemetry") {
      return "";
    }

    const auto& telemetry = message[1];

    // Main car localization data.
    const double car_x = telemetry["x"];
    const double car_y = telemetry["y"];
    const double car_s = telemetry["s"];
    const double car_d = telemetry["d"];
    const double car_yaw = telemetry["yaw"];
    const double car_speed = telemetry["speed"];

    // Remaining points from the path sent during the previous update.
    const auto previous_path_x = telemetry["previous_path_x"];
    const auto previous_path_y = telemetry["previous_path_y"];
    const double end_path_s = telemetry["end_path_s"];
    const double end_path_d = telemetry["end_path_d"];

    // Other vehicles: [id, x, y, vx, vy, s, d].
    const auto sensor_fusion = telemetry["sensor_fusion"];

    // Keep the complete simulator interface close to the trajectory TODO. These
    // values will be consumed as the behavior and trajectory planners are added.
    (void)car_x;
    (void)car_y;
    (void)car_s;
    (void)car_d;
    (void)car_yaw;
    (void)car_speed;
    (void)previous_path_x;
    (void)previous_path_y;
    (void)end_path_s;
    (void)end_path_d;
    (void)sensor_fusion;
    (void)map_waypoints_x;
    (void)map_waypoints_y;
    (void)map_waypoints_s;
    (void)map_waypoints_dx;
    (void)map_waypoints_dy;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    /**
     * TODO: define a path made up of (x,y) points that the car will visit
     * sequentially every .02 seconds.
     */

    json response;
    response["next_x"] = next_x_vals;
    response["next_y"] = next_y_vals;
    return "42[\"control\"," + response.dump() + "]";
  } catch (const std::exception& error) {
    std::cerr << "Invalid simulator message: " << error.what() << std::endl;
    return "42[\"manual\",{}]";
  }
}

int main() {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  const string map_file = "data/highway_map.csv";
  std::ifstream map_stream(map_file.c_str(), std::ifstream::in);
  if (!map_stream) {
    std::cerr << "Failed to open map file: " << map_file << std::endl;
    return -1;
  }

  string line;
  while (getline(map_stream, line)) {
    std::istringstream waypoint(line);
    double x;
    double y;
    double s;
    double dx;
    double dy;
    if (!(waypoint >> x >> y >> s >> dx >> dy)) {
      std::cerr << "Invalid waypoint: " << line << std::endl;
      return -1;
    }
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(dx);
    map_waypoints_dy.push_back(dy);
  }

  if (map_waypoints_x.empty()) {
    std::cerr << "Map file contains no waypoints: " << map_file << std::endl;
    return -1;
  }

  const unsigned short port = 4567;
  try {
    boost::asio::io_context io_context(1);
    tcp::acceptor acceptor(io_context);
    acceptor.open(tcp::v4());
    acceptor.set_option(boost::asio::socket_base::reuse_address(true));
    acceptor.bind(tcp::endpoint(tcp::v4(), port));
    acceptor.listen(boost::asio::socket_base::max_listen_connections);
    std::cout << "Listening to port " << port << std::endl;

    for (;;) {
      tcp::socket socket(io_context);
      acceptor.accept(socket);

      websocket::stream<tcp::socket> ws(std::move(socket));
      ws.set_option(websocket::stream_base::timeout::suggested(
          beast::role_type::server));

      beast::error_code error;
      ws.accept(error);
      if (error) {
        std::cerr << "WebSocket handshake failed: " << error.message()
                  << std::endl;
        continue;
      }
      std::cout << "Connected" << std::endl;

      for (;;) {
        beast::flat_buffer buffer;
        ws.read(buffer, error);
        if (error == websocket::error::closed) {
          break;
        }
        if (error) {
          std::cerr << "WebSocket read failed: " << error.message()
                    << std::endl;
          break;
        }

        const string request = beast::buffers_to_string(buffer.data());
        const string response = HandleMessage(
            request, map_waypoints_x, map_waypoints_y, map_waypoints_s,
            map_waypoints_dx, map_waypoints_dy);
        if (response.empty()) {
          continue;
        }

        ws.text(true);
        ws.write(boost::asio::buffer(response), error);
        if (error) {
          std::cerr << "WebSocket write failed: " << error.message()
                    << std::endl;
          break;
        }
      }

      std::cout << "Disconnected" << std::endl;
    }
  } catch (const std::exception& error) {
    std::cerr << "Server failed: " << error.what() << std::endl;
    return -1;
  }
}
