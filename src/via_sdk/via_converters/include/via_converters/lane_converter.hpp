#ifndef VIA_LANE_CONVERTER_HPP_
#define VIA_LANE_CONVERTER_HPP_

#include <via_definitions/msg/lane.hpp>
#include <via_definitions/perception/lane_line.hpp>

namespace via {
namespace converters {

class LaneConverter {
 public:
  static via_definitions::msg::Lane LaneLinesToLaneMsg(
      const std::vector<via::definitions::perception::LaneLine>& lane_lines);
  static via_definitions::msg::LaneLine LaneLineToLaneLineMsg(
      const via::definitions::perception::LaneLine& lane_line);
};

}  // namespace converters
}  // namespace via
#endif