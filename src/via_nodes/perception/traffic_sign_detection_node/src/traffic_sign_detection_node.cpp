#include "traffic_sign_detection_node/traffic_sign_detection_node.hpp"

namespace via {
namespace perception {
namespace traffic_sign {

using namespace std;
using namespace cv;

TrafficSignDetectionNode::TrafficSignDetectionNode(
    const rclcpp::NodeOptions &node_options)
    : Node("traffic_sign_detection", node_options) {
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/simulation/image", 1,
      std::bind(&TrafficSignDetectionNode::ImageCallback, this,
                std::placeholders::_1));
  
  model_ = std::make_shared<TrafficSignDetectorYOLOX>();
}

void TrafficSignDetectionNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  cv::Mat img = cv_ptr->image;
  DetectSigns(img);
  std::cout << "Ping" << std::endl;
}

void TrafficSignDetectionNode::DetectSigns(const cv::Mat &org) {
  model_->Detect(org);
}

}  // namespace traffic_sign
}  // namespace perception
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node =
      std::make_shared<via::perception::traffic_sign::TrafficSignDetectionNode>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
