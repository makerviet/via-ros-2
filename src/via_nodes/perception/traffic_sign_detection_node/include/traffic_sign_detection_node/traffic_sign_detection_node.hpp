#ifndef TRAFFIC_SIGN_DETECTOR_NODE_HPP_
#define TRAFFIC_SIGN_DETECTOR_NODE_HPP_

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
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <traffic_sign_detector_yolox/traffic_sign_detector_yolox.hpp>

namespace via {
namespace perception {
namespace traffic_sign {

class TrafficSignDetectionNode : public rclcpp::Node {
 public:
  TrafficSignDetectionNode(const rclcpp::NodeOptions &node_options);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::Mat frame_;
  std::shared_ptr<TrafficSignDetectorYOLOX> model_;

  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void DetectSigns(const cv::Mat &org);
};
}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
#endif
