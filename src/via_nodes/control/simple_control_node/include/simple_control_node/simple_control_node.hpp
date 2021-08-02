#ifndef SIMPLE_CONTROL_NODE_HPP_
#define SIMPLE_CONTROL_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <via_converters/lane_converter.hpp>
#include <via_converters/traffic_sign_converter.hpp>
#include <via_definitions/msg/lane.hpp>
#include <via_definitions/msg/traffic_signs.hpp>
#include <via_definitions/perception/lane_line.hpp>
#include <via_definitions/perception/traffic_sign.hpp>

namespace via {
namespace control {

class SimpleControlNode : public rclcpp::Node {
 public:
  SimpleControlNode(const rclcpp::NodeOptions &node_options);

 private:
  rclcpp::Subscription<via_definitions::msg::Lane>::SharedPtr lane_sub_;
  rclcpp::Subscription<via_definitions::msg::TrafficSigns>::SharedPtr traffic_signs_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;

  std::vector<via::definitions::perception::LaneLine> lane_lines_;
  std::mutex lane_lines_mutex_;
  std::vector<via::definitions::perception::TrafficSign> traffic_signs_;
  std::mutex traffic_signs_mutex_;

  void LaneCallback(const via_definitions::msg::Lane::SharedPtr msg);
  void TrafficSignsCallback(const via_definitions::msg::TrafficSigns::SharedPtr msg);
  void UpdateControl();
  
};

}  // namespace control
}  // namespace via
#endif
