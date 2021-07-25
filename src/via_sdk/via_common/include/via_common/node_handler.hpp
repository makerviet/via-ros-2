#ifndef VIA_NODE_HANDLER_HPP_
#define VIA_NODE_HANDLER_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace via {
namespace common {

class NodeHandler {
 public:
  NodeHandler(const std::string& node_name);
  virtual ~NodeHandler(){};
  NodePtr node_;
  std::string DeclareStringParameter(const std::string& k,
                                     const std::string& v);
  int DeclareIntParameter(const std::string& k, int v);
  rclcpp::Time GetCurrentTime();
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace common
}  // namespace via
#endif