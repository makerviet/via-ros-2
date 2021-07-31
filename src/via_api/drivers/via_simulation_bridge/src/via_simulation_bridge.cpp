#include "via_simulation_bridge/via_simulation_bridge.hpp"

namespace via {
namespace drivers {
namespace simulation {

using namespace std;
using json = nlohmann::json;

VIASimulationBridge::VIASimulationBridge(
    const std::string websocket_server_path,
    std::function<void(cv::Mat)> callback) {
  callback_ = callback;
  capturing_ = true;

  ws_client = std::make_shared<WsClient>(websocket_server_path);
  ws_client->on_message = [this](shared_ptr<WsClient::Connection> connection,
                                 shared_ptr<WsClient::InMessage> in_message) {
    json json_data;
    try {
      json_data = json::parse(in_message->string());
    } catch (json::parse_error &ex) {
      std::cerr << "parse error at byte " << ex.byte << std::endl;
      cout << in_message->string() << endl;
      return;
    }
    std::string dec_jpg = base64_decode(json_data["image"]);
    std::vector<uchar> data(dec_jpg.begin(), dec_jpg.end());
    cv::Mat img = cv::imdecode(cv::Mat(data), 1);
    if (capturing_) {
      callback_(img);
    }

    last_in_message_time_mutex_.lock();
    last_in_message_time_ = std::chrono::high_resolution_clock::now();
    last_in_message_time_mutex_.unlock();
  };

  ws_client->on_open = [](shared_ptr<WsClient::Connection> connection) {
    cout << "Opened connection" << endl;
    cout << connection << endl;
  };

  ws_client->on_close = [this](shared_ptr<WsClient::Connection> /*connection*/,
                               int status, const string & /*reason*/) {
    cout << "Closed connection with status code " << status << endl;
  };

  // See
  // http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html,
  // Error Codes for error code meanings
  ws_client->on_error = [this](shared_ptr<WsClient::Connection> /*connection*/,
                               const SimpleWeb::error_code &ec) {
    cout << "Client: Error: " << ec << ", error message: " << ec.message()
         << endl;
  };

  // Websocket monitoring
  std::thread t([this]() {
    while (true) {
      cout << "Hey" << endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      last_in_message_time_mutex_.lock();
      auto stop = std::chrono::high_resolution_clock::now();
      long long int conn_lost_duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              stop - last_in_message_time_)
              .count();
      cout << conn_lost_duration << endl;
      if (conn_lost_duration > 5000) {  // Connection lost
        cout << "Connection lost. Trying to reconnect..." << endl;
        Reconnect();
        std::this_thread::sleep_for(std::chrono::seconds(5));
      }
      last_in_message_time_mutex_.unlock();
    }
  });
  t.detach();

  Reconnect();
}

VIASimulationBridge::~VIASimulationBridge() { Stop(); }

void VIASimulationBridge::Reconnect() {
  restart_timer_.setTimeout(
      [&]() {
        ws_client->stop();
        ws_client->start();
      },
      1000);
}

void VIASimulationBridge::OpenConnection() { ws_client->start(); }

void VIASimulationBridge::CloseConnection() { ws_client->stop(); }

void VIASimulationBridge::Start() {
  if (capturing_) {
    std::cerr << "Simulation bridge is still running. Please stop it before "
                 "starting again."
              << std::endl;
    throw std::runtime_error(
        "Simulation bridge is still running. Please stop it before starting "
        "again.");
  }
  try {
    OpenConnection();
  } catch (const char *excp) {
    throw excp;
  }

  capturing_ = true;
}

void VIASimulationBridge::Stop() {
  capturing_ = false;
  CloseConnection();
}

}  // namespace simulation
}  // namespace drivers
}  // namespace via
