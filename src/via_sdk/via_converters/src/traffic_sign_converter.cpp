#include <via_converters/traffic_sign_converter.hpp>

namespace via {
namespace converters {

via_definitions::msg::TrafficSigns
TrafficSignConverter::TrafficSignsToTrafficSignsMsg(
    const std::vector<via::definitions::perception::TrafficSign>&
        signs) {
    via_definitions::msg::TrafficSigns signs_msg;
    for (size_t i = 0; i < signs.size(); ++i) {
        via_definitions::msg::TrafficSign sign_msg = TrafficSignToTrafficSignMsg(signs[i]);
        signs_msg.sign_list.push_back(sign_msg);
    }
    return signs_msg;
}

via_definitions::msg::TrafficSign TrafficSignConverter::TrafficSignToTrafficSignMsg(
      const via::definitions::perception::TrafficSign& sign){
    via_definitions::msg::TrafficSign sign_msg;
    sign_msg.class_id = sign.class_id;
    sign_msg.confidence = sign.confidence;
    sign_msg.box.x = sign.box.x;
    sign_msg.box.y = sign.box.y;
    sign_msg.box.width = sign.box.width;
    sign_msg.box.height = sign.box.height;
    return sign_msg;
}

std::vector<via::definitions::perception::TrafficSign>
TrafficSignConverter::TrafficSignsMsgToTrafficSigns(
    const via_definitions::msg::TrafficSigns&
        signs_msg) {
    std::vector<via::definitions::perception::TrafficSign> signs;
    for (size_t i = 0; i < signs_msg.sign_list.size(); ++i) {
        via::definitions::perception::TrafficSign sign = TrafficSignMsgToTrafficSign(signs_msg.sign_list[i]);
        signs.push_back(sign);
    }
    return signs;
}

via::definitions::perception::TrafficSign TrafficSignConverter::TrafficSignMsgToTrafficSign(
      const via_definitions::msg::TrafficSign& sign_msg){
    via::definitions::perception::TrafficSign sign;
    sign.class_id = sign_msg.class_id;
    sign.confidence = sign_msg.confidence;
    sign.box.x = sign_msg.box.x;
    sign.box.y = sign_msg.box.y;
    sign.box.width = sign_msg.box.width;
    sign.box.height = sign_msg.box.height;
    return sign;
}

}  // namespace converters
}  // namespace via