#include "visualization_node/visualization_node.hpp"

namespace via {
namespace visualization {

using namespace std;
using namespace cv;

VisualizationNode::VisualizationNode(const rclcpp::NodeOptions &node_options)
    : Node("visualization", node_options) {
  lane_sub_ = this->create_subscription<via_definitions::msg::Lane>(
      "/perception/lane", 1,
      std::bind(&VisualizationNode::LaneCallback, this, std::placeholders::_1));
  traffic_signs_sub_ = this->create_subscription<via_definitions::msg::TrafficSigns>(
      "/perception/traffic_signs", 1,
      std::bind(&VisualizationNode::TrafficSignsCallback, this, std::placeholders::_1));
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/simulation/image", 1,
      std::bind(&VisualizationNode::ImageCallback, this,
                std::placeholders::_1));
}

void VisualizationNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (cv_ptr->image.empty()) {
    return;
  }
  img_mutex_.lock();
  img_ = cv_ptr->image;
  img_mutex_.unlock();
  Render();
}

void VisualizationNode::LaneCallback(
    const via_definitions::msg::Lane::SharedPtr msg) {
  std::vector<via::definitions::perception::LaneLine> lane_lines =
      via::converters::LaneConverter::LaneMsgToLaneLines(*msg);
  lane_lines_mutex_.lock();
  lane_lines_ = lane_lines;
  lane_lines_mutex_.unlock();
}

void VisualizationNode::TrafficSignsCallback(
    const via_definitions::msg::TrafficSigns::SharedPtr msg) {
  std::vector<via::definitions::perception::TrafficSign> traffic_signs =
      via::converters::TrafficSignConverter::TrafficSignsMsgToTrafficSigns(*msg);
  traffic_signs_mutex_.lock();
  traffic_signs_ = traffic_signs;
  traffic_signs_mutex_.unlock();
}

void VisualizationNode::Render() {
  // Visualization data
  cv::Mat img;
  std::vector<via::definitions::perception::LaneLine> lane_lines;
  std::vector<via::definitions::perception::TrafficSign> traffic_signs;

  // Get image
  img_mutex_.lock();
  img = img_;
  img_mutex_.unlock();

  if (img.empty()) {
    return;
  }

  // Get lane lines
  lane_lines_mutex_.lock();
  lane_lines = lane_lines_;
  lane_lines_mutex_.unlock();

  // Get traffic signs
  traffic_signs_mutex_.lock();
  traffic_signs = traffic_signs_;
  traffic_signs_mutex_.unlock();

  // Draw lane lines
  RenderLaneLines(img, lane_lines);

  // Draw traffic signs
  RenderTrafficSigns(img, traffic_signs);

  imshow("Visualization", img);
  waitKey(1);
}

void VisualizationNode::RenderLaneLines(
    cv::Mat &img,
    std::vector<via::definitions::perception::LaneLine> lane_lines) {
  if (lane_lines.empty()) {
    return;
  }

  std::vector<cv::Scalar> colors{cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255),
                                 cv::Scalar(0, 255, 255)};

  vector<Point> left_right_lane_line_pts;

  // Draw points
  for (size_t i = 0; i < lane_lines.size(); ++i) {
    if (lane_lines[i].points.empty()) {
      continue;
    }
    vector<Point2f> out_pts;
    // Transform points back  into original image space
    perspectiveTransform(lane_lines[i].points, out_pts,
                         via::perception::lane_line::LaneLineDetectorSimple::
                             getInvertPerspectiveMatrix());
    // Draw the points onto the out image
    for (size_t j = 0; j < out_pts.size() - 1; ++j) {
      line(img, out_pts[j], out_pts[j + 1], colors[i % 3], 3);
    }

    if (i < 2) { // Only left and right lane line
      for (size_t j = 0; j < out_pts.size(); ++j) {
        // We need to revert right line order to draw mask correctly
        int pid = i == 0 ? j : out_pts.size() - j - 1;
        left_right_lane_line_pts.push_back(Point(out_pts[pid].x, out_pts[pid].y));
      }
    }
  }

  // Create a green-ish overlay
  if (!left_right_lane_line_pts.empty()) {
    vector<vector<Point>> arr;
    arr.push_back(left_right_lane_line_pts);
    Mat overlay = Mat::zeros(img.size(), img.type());
    fillPoly(overlay, arr, Scalar(0, 255, 100));
    addWeighted(img, 1, overlay, 0.5, 0, img);  // Overlay it
  }
}

void VisualizationNode::RenderTrafficSigns(
    cv::Mat &img,
    std::vector<via::definitions::perception::TrafficSign> signs) {
  if (signs.empty()) {
    return;
  }

  static const char* class_names[] = {"stop",     "left",    "right",
                                      "straight", "no_left", "no_right"};

  for (size_t i = 0; i < signs.size(); i++) {
    const via::definitions::perception::TrafficSign& obj = signs[i];

    fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.class_id, obj.confidence,
            obj.box.x, obj.box.y, obj.box.width, obj.box.height);

    cv::rectangle(img, obj.box, cv::Scalar(255, 0, 0));

    char text[256];
    sprintf(text, "%s %.1f%%", class_names[obj.class_id], obj.confidence * 100);

    int baseLine = 0;
    cv::Size label_size =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    int x = obj.box.x;
    int y = obj.box.y - label_size.height - baseLine;
    if (y < 0) y = 0;
    if (x + label_size.width > img.cols) x = img.cols - label_size.width;

    cv::rectangle(
        img,
        cv::Rect(cv::Point(x, y),
                 cv::Size(label_size.width, label_size.height + baseLine)),
        cv::Scalar(255, 255, 255), -1);

    cv::putText(img, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
  }
  
}

}  // namespace visualization
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto visualization_node =
      std::make_shared<via::visualization::VisualizationNode>(options);

  exec.add_node(visualization_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
