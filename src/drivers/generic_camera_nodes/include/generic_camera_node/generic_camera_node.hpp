#ifndef VIA_GENERIC_CAMERA_NODE_HPP_
#define VIA_GENERIC_CAMERA_NODE_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <stdio.h>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "generic_camera_driver/generic_camera_driver.hpp"

namespace via {
namespace drivers {
namespace camera {

class GenericCameraNode : public rclcpp::Node {
 public:
  explicit GenericCameraNode(const rclcpp::NodeOptions&);
  ~GenericCameraNode(){};

 private:
  std::string frame_id_;
  int image_height_;
  int image_width_;
  std::string filename_;
  std::shared_ptr<GenericCameraDriver> driver_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  image_transport::CameraPublisher camera_info_pub_;
  std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
  std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(
      const cv::Mat& frame);

  void ImageCallback(const cv::Mat& frame);
};
}  // namespace camera
}  // namespace drivers
}  // namespace via
#endif
