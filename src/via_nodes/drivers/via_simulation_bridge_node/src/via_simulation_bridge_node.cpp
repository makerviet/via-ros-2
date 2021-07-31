#include "via_simulation_bridge_node/via_simulation_bridge_node.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;
namespace via {
namespace drivers {
namespace simulation {

VIASimulationBridgeNode::VIASimulationBridgeNode(
    const rclcpp::NodeOptions &node_options)
    : Node("via_simulation", node_options) {
  std::string camera_calibration_file_param_ =
      this->declare_parameter<std::string>("camera_calibration_file",
                                           "file://config/camera.yaml");

  // Initialize camera publisher
  camera_info_pub_ = image_transport::create_camera_publisher(this, "image");
  camera_info_manager_ =
      std::make_shared<camera_info_manager::CameraInfoManager>(this);
  camera_info_manager_->loadCameraInfo(camera_calibration_file_param_);

  // Initialize services to control camera
  start_service = this->create_service<std_srvs::srv::Trigger>(
      "start", std::bind(&VIASimulationBridgeNode::StartCallback, this,
                         std::placeholders::_1, std::placeholders::_2));
  stop_service = this->create_service<std_srvs::srv::Trigger>(
      "stop", std::bind(&VIASimulationBridgeNode::StopCallback, this,
                        std::placeholders::_1, std::placeholders::_2));

  // Initialize car control signals
  throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "set_throttle", 10, std::bind(&VIASimulationBridgeNode::ThrottleCallback, this, std::placeholders::_1));
  steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "set_steering", 10, std::bind(&VIASimulationBridgeNode::SteeringCallback, this, std::placeholders::_1));

  // Initialize camera driver and start reading images
  bridge_ = std::make_shared<VIASimulationBridge>(
      "127.0.0.1:4567/simulation",
      std::bind(&VIASimulationBridgeNode::ImageCallback, this,
                std::placeholders::_1));
}

VIASimulationBridgeNode::~VIASimulationBridgeNode() {
  bridge_->Stop();
}

void VIASimulationBridgeNode::StartCallback(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  try {
    bridge_->Start();
  } catch (const std::runtime_error &e) {
    res->success = false;
    res->message = e.what();
    return;
  }
  res->success = true;
  res->message = "Successfully started camera.";
}

void VIASimulationBridgeNode::StopCallback(
    std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
  try {
    bridge_->Stop();
  } catch (const std::runtime_error &e) {
    res->success = false;
    res->message = e.what();
    return;
  }
  res->success = true;
  res->message = "Successfully stopped camera.";
}

void VIASimulationBridgeNode::ImageCallback(const cv::Mat &frame) {
  image_msg_ = std::make_shared<via_definitions::msg::Image>(
      via::converters::ImageConverter::OpenCVMatToImageMsg(frame));

  std::shared_ptr<via_definitions::msg::CameraInfo> camera_info_msg_(
      new via_definitions::msg::CameraInfo(
          camera_info_manager_->getCameraInfo()));

  rclcpp::Time timestamp = this->get_clock()->now();

  image_msg_->header.stamp = timestamp;
  image_msg_->header.frame_id = frame_id_;

  camera_info_msg_->header.stamp = timestamp;
  camera_info_msg_->header.frame_id = frame_id_;

  camera_info_pub_.publish(image_msg_, camera_info_msg_);
}

void VIASimulationBridgeNode::ThrottleCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  bridge_->setThrottle(msg->data);
}

void VIASimulationBridgeNode::SteeringCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  bridge_->setSteering(msg->data);
}

void VIASimulationBridgeNode::Stop() {
  bridge_->Stop();
}

}  // namespace simulation
}  // namespace drivers
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto bridge_node =
      std::make_shared<via::drivers::simulation::VIASimulationBridgeNode>(
          options);

  exec.add_node(bridge_node);

  while (rclcpp::ok()) {
    exec.spin_once();
  }

  bridge_node->Stop();

  rclcpp::shutdown();
  return 0;
}
