#ifndef LANE_LINE_PERCEPTION_HPP_
#define LANE_LINE_PERCEPTION_HPP_

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <lane_line_detector_simple/lane_line_detector_simple.hpp>
#include <memory>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <via_converters/lane_converter.hpp>
#include <via_definitions/msg/lane.hpp>
#include <via_definitions/perception/lane_line.hpp>


namespace via {
namespace perception {
namespace lane_line {

class LaneLinePerceptionNode : public rclcpp::Node {
 public:
  LaneLinePerceptionNode(const rclcpp::NodeOptions &node_options);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<via_definitions::msg::Lane>::SharedPtr lane_pub_;
  std::shared_ptr<LaneLineDetectorSimple> detector_;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};
}  // namespace lane_line
}  // namespace perception
}  // namespace via
#endif
