#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "via_listener_nodes/via_listener_node.hpp"

using namespace std::chrono_literals;

namespace via {
namespace examples {
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.

Listener::Listener(const rclcpp::NodeOptions &options)
    : Node("listener", options) {
  hello_text_ = this->declare_parameter<std::string>("hello_text", "Hello");
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example UniquePtr for
  // zero-copy transport.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
    RCLCPP_INFO(this->get_logger(), "%s, I heard: [%s]", hello_text_.c_str(), msg->data.c_str());
  };
  // Create a subscription to the topic which can be matched with one or more
  // compatible ROS publishers. Note that not all publishers on the same topic
  // with the same type will be compatible: they must have compatible Quality of
  // Service policies.
  sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
}
}  // namespace examples
}  // namespace via

RCLCPP_COMPONENTS_REGISTER_NODE(via::examples::Listener)

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node = std::make_shared<via::examples::Listener>(options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
