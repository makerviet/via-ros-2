#include "lane_line_perception_node/lane_line_perception_node.hpp"

namespace via {
namespace perception {
namespace lane_line {

using namespace std;
using namespace cv;

LaneLinePerceptionNode::LaneLinePerceptionNode(
    const rclcpp::NodeOptions &node_options)
    : Node("lane_line_perception", node_options) {
  detector_ = std::make_shared<LaneLineDetectorSimple>();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/simulation/image", 1,
      std::bind(&LaneLinePerceptionNode::ImageCallback, this,
                std::placeholders::_1));
  lane_pub_ = this->create_publisher<via_definitions::msg::Lane>("/perception/lane", 1);
}

void LaneLinePerceptionNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::cout << "Ping" << std::endl;
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image;
  std::vector<via::definitions::perception::LaneLine> lane_lines = detector_->Detect(img);

  // Publish message
  via_definitions::msg::Lane lane_msg = via::converters::LaneConverter::LaneLinesToLaneMsg(lane_lines);
  lane_pub_->publish(lane_msg);
}

}  // namespace laneline
}  // namespace perception
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node =
      std::make_shared<via::perception::lane_line::LaneLinePerceptionNode>(
          options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
