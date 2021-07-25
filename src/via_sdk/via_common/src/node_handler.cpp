#include "via_common/node_handler.hpp"

using namespace via::common;

NodeHandler::NodeHandler(const std::string& node_name) {
  const rclcpp::NodeOptions options;
  this->node_ = std::make_shared<rclcpp::Node>(node_name, options);
};

std::string NodeHandler::DeclareStringParameter(const std::string& k,
                                                const std::string& v) {
  return this->node_->declare_parameter<std::string>(k, v);
}

int NodeHandler::DeclareIntParameter(const std::string& k, int v) {
  return this->node_->declare_parameter<int>(k, v);
}

rclcpp::Time NodeHandler::GetCurrentTime() {
  return this->node_->get_clock()->now();
}