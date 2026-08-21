#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include <exception>
#include <iostream>
#include <string>
#include <utility>

#include "map.h"
#include "planner.h"
#include "simulator_protocol.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = boost::asio::ip::tcp;

int main() {
  const std::string map_file = "data/highway_map.csv";
  MapData map;
  try {
    map = LoadMap(map_file);
  } catch (const std::exception &error) {
    std::cerr << "Map loading failed: " << error.what() << std::endl;
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
      ws.set_option(
          websocket::stream_base::timeout::suggested(beast::role_type::server));

      beast::error_code error;
      ws.accept(error);
      if (error) {
        std::cerr << "WebSocket handshake failed: " << error.message()
                  << std::endl;
        continue;
      }
      std::cout << "Connected" << std::endl;
      PathPlanner planner;

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

        const std::string request = beast::buffers_to_string(buffer.data());
        const SimulatorMessage message = ParseSimulatorMessage(request);
        if (message.kind == SimulatorMessageKind::kIgnore) {
          continue;
        }

        std::string response;
        if (message.kind == SimulatorMessageKind::kManual) {
          std::cerr << "Invalid simulator message: " << message.error
                    << std::endl;
          response = MakeManualMessage();
        } else {
          try {
            response = MakeControlMessage(planner.Plan(message.input, map));
          } catch (const std::exception &planning_error) {
            std::cerr << "Planning failed: " << planning_error.what()
                      << std::endl;
            response = MakeManualMessage();
          }
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
  } catch (const std::exception &error) {
    std::cerr << "Server failed: " << error.what() << std::endl;
    return -1;
  }
}
