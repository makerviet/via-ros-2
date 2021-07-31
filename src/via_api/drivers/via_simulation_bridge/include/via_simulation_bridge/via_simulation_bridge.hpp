#ifndef VIA_SIMULATION_BRIDGE_HPP_
#define VIA_SIMULATION_BRIDGE_HPP_

#include <stdio.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <simple_websocket_server/client_ws.hpp>
#include <string>
#include <thread>
#include <via_common/json.hpp>
#include <via_common/timer.hpp>
#include <thread>         // std::this_thread::sleep_for
#include "base64.hpp"


namespace via {
namespace drivers {
namespace simulation {

using WsClient = SimpleWeb::SocketClient<SimpleWeb::WS>;

class VIASimulationBridge {
 public:
  VIASimulationBridge(const std::string websocket_server_path, std::function<void(cv::Mat)> callback);
  ~VIASimulationBridge();
  void Start();
  void Stop();
  void OpenConnection();
  void CloseConnection();
  void Reconnect();

  void setThrottle(float throttle);
  void setSteering(float steering);
  void sendCommand();

 private:
  std::shared_ptr<WsClient> ws_client;
  std::mutex connection_mutex_;
  std::shared_ptr<SimpleWeb::SocketClient<SimpleWeb::WS>::Connection> ws_connection_ GUARDED_BY(connection_mutex_);

  cv::Mat frame_;
  bool is_active_ = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_in_message_time_;
  std::mutex last_in_message_time_mutex_;
  float current_throttle_ = 0;
  float current_steering_ = 0;

  std::function<void(cv::Mat)> callback_;
  
  Timer restart_timer_;
};
}  // namespace simulation
}  // namespace drivers
}  // namespace via
#endif
