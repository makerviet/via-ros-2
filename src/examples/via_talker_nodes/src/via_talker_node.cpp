#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#include "via_talker_nodes/via_talker_node.hpp"

using namespace std::chrono_literals;

namespace via {
namespace examples {
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.

Talker::Talker(const rclcpp::NodeOptions &options) : Node("talker", options) {
  // Create a function for when messages are to be sent.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto publish_message = [this]() -> void {
    msg_ = std::make_unique<std_msgs::msg::String>();
    msg_->data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(msg_));
  };
  // Create a publisher with a custom Quality of Service profile.
  rclcpp::QoS qos(rclcpp::KeepLast(7));
  pub_ = this->create_publisher<std_msgs::msg::String>("chatter", qos);

  // Use a timer to schedule periodic message publishing.
  timer_ = this->create_wall_timer(1s, publish_message);
}
}  // namespace examples
}  // namespace via

RCLCPP_COMPONENTS_REGISTER_NODE(via::examples::Talker)

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node = std::make_shared<via::examples::Talker>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

