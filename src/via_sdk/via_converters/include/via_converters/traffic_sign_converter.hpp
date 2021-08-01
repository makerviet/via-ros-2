#ifndef VIA_TRAFFIC_SIGN_CONVERTER_HPP_
#define VIA_TRAFFIC_SIGN_CONVERTER_HPP_

#include <via_definitions/msg/traffic_sign.hpp>
#include <via_definitions/msg/traffic_signs.hpp>
#include <via_definitions/perception/traffic_sign.hpp>

namespace via {
namespace converters {

class TrafficSignConverter {
 public:
  static via_definitions::msg::TrafficSigns TrafficSignsToTrafficSignsMsg(
      const std::vector<via::definitions::perception::TrafficSign>&
          traffic_signs);
  static via_definitions::msg::TrafficSign TrafficSignToTrafficSignMsg(
      const via::definitions::perception::TrafficSign& traffic_sign);
  static std::vector<via::definitions::perception::TrafficSign>
  TrafficSignsMsgToTrafficSigns(
      const via_definitions::msg::TrafficSigns& signs_msg);
  static via::definitions::perception::TrafficSign TrafficSignMsgToTrafficSign(
      const via_definitions::msg::TrafficSign& sign_msg);
};

}  // namespace converters
}  // namespace via
#endif