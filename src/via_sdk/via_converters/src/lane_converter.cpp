#include <via_converters/lane_converter.hpp>

namespace via {
namespace converters {

via::definitions::perception::LaneLine LaneConverter::LaneLineMsgToLaneLine(
    const via_definitions::msg::LaneLine& line_msg) {
  via::definitions::perception::LaneLine lane_line;
  lane_line.line_type = line_msg.line_type;
  lane_line.confidence = line_msg.confidence;
  for (size_t i = 0; i < line_msg.points.size(); ++i) {
    lane_line.points.push_back(
        cv::Point2f(line_msg.points[i].x, line_msg.points[i].y));
  }
  return lane_line;
}

via_definitions::msg::LaneLine LaneConverter::LaneLineToLaneLineMsg(
    const via::definitions::perception::LaneLine& lane_line) {
  via_definitions::msg::LaneLine line_msg;
  line_msg.line_type = lane_line.line_type;
  line_msg.confidence = lane_line.confidence;
  for (size_t i = 0; i < lane_line.points.size(); ++i) {
    via_definitions::msg::Point2f p;
    p.x = lane_line.points[i].x;
    p.y = lane_line.points[i].y;
    line_msg.points.push_back(p);
  }
  return line_msg;
}

via_definitions::msg::Lane LaneConverter::LaneLinesToLaneMsg(
    const std::vector<via::definitions::perception::LaneLine>& lane_lines) {
  via_definitions::msg::Lane lane;

  if (lane_lines.size() != 3) {
    std::cerr << "Error converting lane lines to lane message." << std::endl;
    return lane;
  }

  lane.left_line = LaneLineToLaneLineMsg(lane_lines[0]);
  lane.right_line = LaneLineToLaneLineMsg(lane_lines[1]);
  lane.target_line = LaneLineToLaneLineMsg(lane_lines[2]);

  return lane;
}

std::vector<via::definitions::perception::LaneLine>
LaneConverter::LaneMsgToLaneLines(const via_definitions::msg::Lane& lane) {
  std::vector<via::definitions::perception::LaneLine> lane_lines;
  lane_lines.push_back(LaneLineMsgToLaneLine(lane.left_line));
  lane_lines.push_back(LaneLineMsgToLaneLine(lane.right_line));
  lane_lines.push_back(LaneLineMsgToLaneLine(lane.target_line));
  return lane_lines;
}

}  // namespace converters
}  // namespace via