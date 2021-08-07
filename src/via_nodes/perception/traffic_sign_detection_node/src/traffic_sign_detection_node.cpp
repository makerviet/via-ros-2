#include "traffic_sign_detection_node/traffic_sign_detection_node.hpp"

namespace via {
namespace perception {
namespace traffic_sign {

using namespace std;
using namespace cv;

TrafficSignDetectionNode::TrafficSignDetectionNode(
    const rclcpp::NodeOptions &node_options)
    : Node("traffic_sign_detection", node_options) {
  model_ = std::make_shared<TrafficSignDetectorSimple>();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/simulation/image", 1,
      std::bind(&TrafficSignDetectionNode::ImageCallback, this,
                std::placeholders::_1));
  traffic_signs_pub_ = this->create_publisher<via_definitions::msg::TrafficSigns>("/perception/traffic_signs", 10);
}

void TrafficSignDetectionNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::cout << "Ping" << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image;

  std::vector<via::definitions::perception::TrafficSign> signs = model_->Detect(img);

  // Publish message
  via_definitions::msg::TrafficSigns traffic_signs_msg = via::converters::TrafficSignConverter::TrafficSignsToTrafficSignsMsg(signs);
  traffic_signs_pub_->publish(traffic_signs_msg);
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
