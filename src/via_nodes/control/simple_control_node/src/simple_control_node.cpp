#include "simple_control_node/simple_control_node.hpp"

namespace via {
namespace control {

using namespace std;
using namespace cv;

SimpleControlNode::SimpleControlNode(const rclcpp::NodeOptions &node_options)
    : Node("lane_line_perception", node_options) {
  lane_sub_ = this->create_subscription<via_definitions::msg::Lane>(
      "/perception/lane", 1,
      std::bind(&SimpleControlNode::LaneCallback, this, std::placeholders::_1));
  traffic_signs_sub_ =
      this->create_subscription<via_definitions::msg::TrafficSigns>(
          "/perception/traffic_signs", 1,
          std::bind(&SimpleControlNode::TrafficSignsCallback, this,
                    std::placeholders::_1));
  throttle_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("/simulation/set_throttle", 20);
  steering_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("/simulation/set_steering", 20);
}

void SimpleControlNode::LaneCallback(
    const via_definitions::msg::Lane::SharedPtr msg) {
  std::vector<via::definitions::perception::LaneLine> lane_lines =
      via::converters::LaneConverter::LaneMsgToLaneLines(*msg);
  lane_lines_mutex_.lock();
  lane_lines_ = lane_lines;
  lane_lines_mutex_.unlock();
  UpdateControl();
}

void SimpleControlNode::TrafficSignsCallback(
    const via_definitions::msg::TrafficSigns::SharedPtr msg) {
  std::vector<via::definitions::perception::TrafficSign> traffic_signs =
      via::converters::TrafficSignConverter::TrafficSignsMsgToTrafficSigns(
          *msg);
  traffic_signs_mutex_.lock();
  traffic_signs_ = traffic_signs;
  traffic_signs_mutex_.unlock();
  UpdateControl();
}

void SimpleControlNode::UpdateControl() {

  // Data for planning & control
  std::vector<via::definitions::perception::LaneLine> lane_lines;
  std::vector<via::definitions::perception::TrafficSign> traffic_signs;

  // Get lane lines
  lane_lines_mutex_.lock();
  lane_lines = lane_lines_;
  lane_lines_mutex_.unlock();

  // Get traffic signs
  traffic_signs_mutex_.lock();
  traffic_signs = traffic_signs_;
  traffic_signs_mutex_.unlock();

  if (lane_lines.size() != 3) {
    std::cout << "Wrong lane line number. Expected 3. Received " << lane_lines.size() << std::endl;
    return;
  }

  std::vector<cv::Point2f> target_pts = lane_lines[2].points;

  if (target_pts.size() <= 80) {
    std::cout << "Too few target points for planning & control" << std::endl;
    return;
  }

  float car_x = 320;
  float target_x = target_pts[-80].x;
  float diff = car_x - target_x;
  float steering = - diff / 320 * 5;

  std_msgs::msg::Float32 throttle_msg;
  throttle_msg.data = 0.5;
  throttle_pub_->publish(throttle_msg);

  std_msgs::msg::Float32 steering_msg;
  steering_msg.data = steering;
  steering_pub_->publish(steering_msg);

  std::cout << "Steering: " << steering << std::endl;

}

}  // namespace control
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto node = std::make_shared<via::control::SimpleControlNode>(options);

  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
