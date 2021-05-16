#ifndef VIA_LISTENER_2INTS_NODE_HPP_
#define VIA_LISTENER_2INTS_NODE_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "via_common/msg/two_ints.hpp"

using namespace std::chrono_literals;

namespace via {
namespace examples {
// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener2Ints : public rclcpp::Node {
 public:
  explicit Listener2Ints(const rclcpp::NodeOptions &options);

 private:
  rclcpp::Subscription<via_common::msg::TwoInts>::SharedPtr sub_;
  std::string hello_text_;
};
}  // namespace examples
}  // namespace via

#endif
