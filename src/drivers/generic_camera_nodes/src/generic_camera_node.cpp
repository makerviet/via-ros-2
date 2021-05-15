#include "generic_camera_node/generic_camera_node.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;
namespace via {
namespace drivers {
namespace camera {

GenericCameraNode::GenericCameraNode(const rclcpp::NodeOptions &node_options)
    : Node("generic_camera_node", node_options) {
  frame_id_ = this->declare_parameter("frame_id", "camera");
  filename_ = this->declare_parameter("file_name", "0");
  image_width_ = this->declare_parameter("image_width", 1280);
  image_height_ = this->declare_parameter("image_height", 720);

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  camera_info_pub_ = image_transport::create_camera_publisher(
      this, "image", custom_qos_profile);

  camera_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(this);

  auto camera_calibration_file_param_ = this->declare_parameter(
      "camera_calibration_file", "file://config/camera.yaml");
  camera_info_manager_->loadCameraInfo(camera_calibration_file_param_);

  driver_ = std::make_shared<GenericCameraDriver>(
      filename_, image_width_, image_height_,
      std::bind(&GenericCameraNode::ImageCallback, this,
                std::placeholders::_1));

  driver_->Start();
}

std::shared_ptr<sensor_msgs::msg::Image>
GenericCameraNode::ConvertFrameToMessage(const cv::Mat &frame) {
  std_msgs::msg::Header header_;
  sensor_msgs::msg::Image ros_image;
  ros_image.header = header_;
  ros_image.height = frame.rows;
  ros_image.width = frame.cols;
  ros_image.encoding = "bgr8";
  ros_image.is_bigendian = false;
  ros_image.step = frame.cols * frame.elemSize();
  size_t size = ros_image.step * frame.rows;
  ros_image.data.resize(size);

  if (frame.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
  } else {
    uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar *cv_data_ptr = frame.data;
    for (int i = 0; i < frame.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += frame.step;
    }
  }

  auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
  return msg_ptr_;
}

void GenericCameraNode::ImageCallback(const cv::Mat &frame) {
  image_msg_ = ConvertFrameToMessage(frame);

  // Put the message into a queue to be processed by the middleware -
  // Non-blocking.
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
      new sensor_msgs::msg::CameraInfo(camera_info_manager_->getCameraInfo()));

  rclcpp::Time timestamp = this->get_clock()->now();

  image_msg_->header.stamp = timestamp;
  image_msg_->header.frame_id = frame_id_;

  camera_info_msg_->header.stamp = timestamp;
  camera_info_msg_->header.frame_id = frame_id_;

  camera_info_pub_.publish(image_msg_, camera_info_msg_);
}
}  // namespace camera
}  // namespace drivers
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node = std::make_shared<via::drivers::camera::GenericCameraNode>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
