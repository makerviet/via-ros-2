#ifndef VIA_GENERIC_CAMERA_NODE_HPP_
#define VIA_GENERIC_CAMERA_NODE_HPP_

#include <stdio.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <via_definitions/msg_defs.hpp>
#include <via_converters/image_converter.hpp>
#include <generic_camera_driver/generic_camera_driver.hpp>

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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service;

  std::shared_ptr<via_definitions::msg::Image> image_msg_;

  void StartCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                     std_srvs::srv::Trigger::Response::SharedPtr res);
  void StopCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res);
  void ImageCallback(const cv::Mat& frame);
};
}  // namespace camera
}  // namespace drivers
}  // namespace via
#endif
