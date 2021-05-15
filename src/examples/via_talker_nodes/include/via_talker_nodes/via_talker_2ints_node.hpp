#ifndef VIA_TALKER_2INTS_NODE_HPP_
#define VIA_TALKER_2INTS_NODE_HPP_

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
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker2Ints : public rclcpp::Node {
 public:
  explicit Talker2Ints(const rclcpp::NodeOptions &options);

 private:
  size_t num1_ = 0;
  size_t num2_ = 0;
  std::unique_ptr<via_common::msg::TwoInts> msg_;
  rclcpp::Publisher<via_common::msg::TwoInts>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace examples
}  // namespace via

#endif
