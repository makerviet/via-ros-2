#ifndef VIA_LISTENER_NODE_HPP_
#define VIA_LISTENER_NODE_HPP_

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
class Listener : public rclcpp::Node {
 public:
  explicit Listener(const rclcpp::NodeOptions &options);

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
}  // namespace examples
}  // namespace via

#endif
