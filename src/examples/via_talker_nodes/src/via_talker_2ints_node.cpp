#include "via_talker_nodes/via_talker_2ints_node.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace via {
namespace examples {
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.

Talker2Ints::Talker2Ints(const rclcpp::NodeOptions &options)
    : Node("talker", options) {
  // Create a function for when messages are to be sent.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto publish_message = [this]() -> void {
    if (num1_ > num2_) {
      ++num2_;
    } else {
      ++num1_;
    }
    msg_ = std::make_unique<via_common::msg::TwoInts>();
    msg_->num1 = num1_;
    msg_->num2 = num2_;
    RCLCPP_INFO(this->get_logger(), "Publishing: [%d] [%d]", msg_->num1,
                msg_->num2);
    // Put the message into a queue to be processed by the middleware.
    // This call is non-blocking.
    pub_->publish(std::move(msg_));
  };
  // Create a publisher with a custom Quality of Service profile.
  rclcpp::QoS qos(rclcpp::KeepLast(7));
  pub_ = this->create_publisher<via_common::msg::TwoInts>("chatter", qos);

  // Use a timer to schedule periodic message publishing.
  timer_ = this->create_wall_timer(1s, publish_message);
}
}  // namespace examples
}  // namespace via

RCLCPP_COMPONENTS_REGISTER_NODE(via::examples::Talker2Ints)

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node = std::make_shared<via::examples::Talker2Ints>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
