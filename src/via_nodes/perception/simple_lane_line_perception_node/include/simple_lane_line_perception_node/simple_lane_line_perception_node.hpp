#ifndef SIMPLE_LANE_LINE_PERCEPTION_HPP_
#define SIMPLE_LANE_LINE_PERCEPTION_HPP_

#include <stdio.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

namespace via {
namespace perception {
namespace laneline {

class SimpleLaneLinePerceptionNode : public rclcpp::Node {
 public:
  SimpleLaneLinePerceptionNode(const rclcpp::NodeOptions &node_options);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::Mat frame_;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};
}  // namespace laneline
}  // namespace perception
}  // namespace via
#endif
