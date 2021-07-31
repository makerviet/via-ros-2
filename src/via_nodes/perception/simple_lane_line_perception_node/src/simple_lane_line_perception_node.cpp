#include "simple_lane_line_perception_node/simple_lane_line_perception_node.hpp"

namespace via {
namespace perception {
namespace laneline {

using namespace std;

SimpleLaneLinePerceptionNode::SimpleLaneLinePerceptionNode(const rclcpp::NodeOptions &node_options): Node("simple_lane_line_perception", node_options) {
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/simulation/image", 10,
      std::bind(&SimpleLaneLinePerceptionNode::ImageCallback, this,
                std::placeholders::_1));
}

void SimpleLaneLinePerceptionNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  
  cv::Mat img = cv_ptr->image;
  std::cout << "Ping" << std::endl;
}

}  // namespace laneline
}  // namespace perception
}  // namespace via


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node =
      std::make_shared<via::perception::laneline::SimpleLaneLinePerceptionNode>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
