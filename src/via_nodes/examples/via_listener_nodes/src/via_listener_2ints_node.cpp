#include "via_listener_nodes/via_listener_2ints_node.hpp"

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
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.

Listener2Ints::Listener2Ints(const rclcpp::NodeOptions &options)
    : Node("listener", options) {
  hello_text_ = this->declare_parameter<std::string>("hello_text", "Hello");
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example UniquePtr for
  // zero-copy transport.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto callback = [this](const via_msgs::msg::TwoInts::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "%s, I heard: [%d] [%d]", hello_text_.c_str(), msg->num1, msg->num2);
  };
  // Create a subscription to the topic which can be matched with one or more
  // compatible ROS publishers. Note that not all publishers on the same topic
  // with the same type will be compatible: they must have compatible Quality of
  // Service policies.
  sub_ = create_subscription<via_msgs::msg::TwoInts>("chatter_2ints", 10, callback);
}
}  // namespace examples
}  // namespace via

RCLCPP_COMPONENTS_REGISTER_NODE(via::examples::Listener2Ints)

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node = std::make_shared<via::examples::Listener2Ints>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
