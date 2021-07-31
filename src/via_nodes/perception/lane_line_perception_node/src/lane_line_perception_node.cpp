#include "lane_line_perception_node/lane_line_perception_node.hpp"

namespace via {
namespace perception {
namespace laneline {

using namespace std;
using namespace cv;

LaneLinePerceptionNode::LaneLinePerceptionNode(
    const rclcpp::NodeOptions &node_options)
    : Node("lane_line_perception", node_options) {
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/simulation/image", 1,
      std::bind(&LaneLinePerceptionNode::ImageCallback, this,
                std::placeholders::_1));
}

void LaneLinePerceptionNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  cv::Mat img = cv_ptr->image;
  DetectLaneLines(img);
  std::cout << "Ping" << std::endl;
}

vector<Point2f> slidingWindow(Mat image, Rect window) {
  vector<Point2f> points;
  const Size imgSize = image.size();
  bool shouldBreak = false;

  // Find points in the first window
  Mat roi = image(window);
  if (cv::countNonZero(roi) == 0) {
    return points;
  }

  int empty_count = 0;
  while (true) {
    float currentX = window.x + window.width * 0.5f;

    Mat roi = image(window);  // Extract region of interest
    vector<Point2f> locations;

    findNonZero(
        roi, locations);  // Get all non-black pixels. All are white in our case

    // Early stopping
    if (locations.empty()) {
      ++empty_count;
      if (empty_count > 2) {
        return points;
      }
    }

    float avgX = 0.0f;

    for (int i = 0; i < locations.size(); ++i)  // Calculate average X position
    {
      float x = locations[i].x;
      avgX += window.x + x;
    }

    avgX = locations.empty() ? currentX : avgX / locations.size();

    Point point(avgX, window.y + window.height * 0.5f);
    points.push_back(point);

    // Move the window up
    window.y -= window.height;

    // For the uppermost position
    if (window.y < 0) {
      window.y = 0;
      shouldBreak = true;
    }

    // Move x position
    window.x += (point.x - currentX);

    // Make sure the window doesn't overflow, we get an error if we try to get
    // data outside the matrix
    if (window.x < 0) window.x = 0;
    if (window.x + window.width >= imgSize.width)
      window.x = imgSize.width - window.width - 1;

    if (shouldBreak) break;
  }

  return points;
}

std::vector<Point2f> FindTargetPoints(int im_width, int im_height,
                                      const vector<Point2f> &left_pts,
                                      const vector<Point2f> &right_pts) {
  vector<Point2f> target_pts;  // Init target points

  // Check left / right points
  if (left_pts.empty() || right_pts.empty()) {
    return target_pts;
  }

  // Find drivable area
  cv::Mat drivable_area(im_height, im_width, CV_8UC1);
  vector<Point> all_pts;
  for (int i = 0; i < left_pts.size(); ++i) {
    all_pts.push_back(Point(left_pts[i].x, left_pts[i].y));
  }
  for (int i = right_pts.size() - 1; i >= 0; --i) {
    all_pts.push_back(Point(right_pts[i].x, right_pts[i].y));
  }
  vector<vector<Point>> arr; arr.push_back(all_pts);
  fillPoly(drivable_area, arr, Scalar(255));

  imshow("Drivable area", drivable_area);
  // waitKey(1);

  // Find interested area by y-axis
  float min_y = max(0.0f, min(min(left_pts[0].y, left_pts[-1].y),
                         min(right_pts[0].y, right_pts[-1].y)));
  float max_y = min(float(im_height - 1), max(max(left_pts[0].y, left_pts[-1].y),
                                     max(right_pts[0].y, right_pts[-1].y)));

  // Regress target points (middle line)
  for (int y = min_y; y <= max_y; ++y) {
    // Find left point
    int left_x = -1;
    for (int x = 0; x <= drivable_area.cols; ++x) {
      if (drivable_area.at<unsigned char>(y, x) > 0) {
        left_x = x;
        break;
      }
    }
    // Find right point
    int right_x = -1;
    for (int x = drivable_area.cols - 1; x >= 0; --x) {
      if (drivable_area.at<unsigned char>(y, x) > 0) {
        right_x = x;
        break;
      }
    }
    if (left_x != -1 && right_x != -1 && left_x != right_x) {
      target_pts.push_back(Point((left_x + right_x) / 2, y));
    }
  }

  return target_pts;
}

void LaneLinePerceptionNode::DetectLaneLines(const cv::Mat &org) {
  // Prepare things that don't need to be computed on every frame.
  Point2f srcVertices[4];

  // Define points that are used for generating bird's eye view. This was done
  // by trial and error. Best to prepare sliders and configure for each use
  // case.
  int x_delta = 250;
  int y_delta = 200;
  srcVertices[0] = Point(0, y_delta);
  srcVertices[1] = Point(640, y_delta);
  srcVertices[2] = Point(640, 480);
  srcVertices[3] = Point(0, 480);

  // Destination vertices. Output is 640 by 480px
  Point2f dstVertices[4];
  dstVertices[0] = Point(0, 0);
  dstVertices[1] = Point(640, 0);
  dstVertices[2] = Point(640 - x_delta, 480);
  dstVertices[3] = Point(x_delta, 480);

  // Prepare matrix for transform and get the warped image
  Mat perspectiveMatrix = getPerspectiveTransform(srcVertices, dstVertices);
  Mat dst(480, 640, CV_8UC3);  // Destination for warped image

  // For transforming back into original image space
  Mat invertedPerspectiveMatrix;
  invert(perspectiveMatrix, invertedPerspectiveMatrix);

  Mat img;  // Working image

  resize(org, img, Size(640, 480));

  // Generate bird's eye view
  warpPerspective(org, dst, perspectiveMatrix, dst.size(), INTER_LINEAR,
                  BORDER_CONSTANT);

  // Convert to gray
  cvtColor(dst, img, COLOR_RGB2GRAY);

  // Extract yellow and white info
  Mat maskYellow, maskWhite;

  inRange(img, Scalar(20, 100, 100), Scalar(30, 255, 255), maskYellow);
  inRange(img, Scalar(150, 150, 150), Scalar(255, 255, 255), maskWhite);

  Mat mask, processed;
  bitwise_or(maskYellow, maskWhite, mask);  // Combine the two masks
  bitwise_and(img, mask, processed);        // Extrect what matches

  // Blur the image a bit so that gaps are smoother
  const Size kernelSize = Size(9, 9);
  GaussianBlur(processed, processed, kernelSize, 0);

  // Try to fill the gaps
  Mat kernel = Mat::ones(15, 15, CV_8U);
  dilate(processed, processed, kernel);
  erode(processed, processed, kernel);
  morphologyEx(processed, processed, MORPH_CLOSE, kernel);

  // Keep only what's above 150 value, other is then black
  const int thresholdVal = 150;
  threshold(processed, processed, thresholdVal, 255, THRESH_BINARY);

  Mat out;
  cvtColor(processed, out,
           COLOR_GRAY2BGR);  // Conver the processing image to color so that we
                             // can visualise the lines

  vector<Point> all_pts;  // Used for the end polygon at the end.
  vector<Point2f> out_pts;

  // Sliding window for 2 sides
  vector<Point2f> left_pts = slidingWindow(processed, Rect(237, 460, 60, 20));
  vector<Point2f> right_pts = slidingWindow(processed, Rect(340, 460, 60, 20));

  int lane_width = 160;
  if (left_pts.empty() && !right_pts.empty()) {
    for (int i = 0; i < right_pts.size(); ++i) {
      left_pts.push_back(Point2f(right_pts[i].x - lane_width, right_pts[i].y));
    }
  }
  if (right_pts.empty() && !left_pts.empty()) {
    for (int i = 0; i < left_pts.size(); ++i) {
      right_pts.push_back(Point2f(left_pts[i].x + lane_width, left_pts[i].y));
    }
  }

  vector<Point2f> target_pts = FindTargetPoints(640, 480, left_pts, right_pts);

  if (!left_pts.empty()) {
    // Transform points back  into original image space
    perspectiveTransform(left_pts, out_pts, invertedPerspectiveMatrix);
    // Draw the points onto the out image
    for (int i = 0; i < out_pts.size() - 1; ++i) {
      line(org, out_pts[i], out_pts[i + 1], Scalar(255, 0, 0), 3);
      all_pts.push_back(Point(out_pts[i].x, out_pts[i].y));
    }
    all_pts.push_back(
        Point(out_pts[out_pts.size() - 1].x, out_pts[out_pts.size() - 1].y));
    for (int i = 0; i < left_pts.size() - 1; ++i)  // Draw a line on the
                                                   // processed image
      line(out, left_pts[i], left_pts[i + 1], Scalar(255, 0, 0), 3);
  }

  if (!right_pts.empty()) {
    // Transform points back  into original image space
    perspectiveTransform(right_pts, out_pts, invertedPerspectiveMatrix);
    // Draw the other lane and append points
    for (int i = 0; i < out_pts.size() - 1; ++i) {
      line(org, out_pts[i], out_pts[i + 1], Scalar(0, 0, 255), 3);
      all_pts.push_back(Point(out_pts[out_pts.size() - i - 1].x,
                              out_pts[out_pts.size() - i - 1].y));
    }
    all_pts.push_back(Point(out_pts[0].x - (out_pts.size() - 1), out_pts[0].y));
    for (int i = 0; i < right_pts.size() - 1; ++i)
      line(out, right_pts[i], right_pts[i + 1], Scalar(0, 0, 255), 3);
  }

  if (!target_pts.empty()) {
    // Transform points back  into original image space
    perspectiveTransform(target_pts, out_pts, invertedPerspectiveMatrix);
    // Draw the other lane and append points
    for (int i = 0; i < out_pts.size() - 1; ++i) {
      line(org, out_pts[i], out_pts[i + 1], Scalar(0, 255, 255), 3);
    }
    for (int i = 0; i < target_pts.size() - 1; ++i)
      line(out, target_pts[i], target_pts[i + 1], Scalar(0, 255, 255), 3);
  }

  // Create a green-ish overlay
  if (!all_pts.empty()) {
    vector<vector<Point>> arr;
    arr.push_back(all_pts);
    Mat overlay = Mat::zeros(org.size(), org.type());
    fillPoly(overlay, arr, Scalar(0, 255, 100));
    addWeighted(org, 1, overlay, 0.5, 0, org);  // Overlay it
  }

  // Show results
  imshow("Result", out);
  imshow("Overlay", org);
  waitKey(1);
}

}  // namespace laneline
}  // namespace perception
}  // namespace via

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  const rclcpp::NodeOptions options;
  auto camera_node =
      std::make_shared<via::perception::laneline::LaneLinePerceptionNode>(
          options);

  exec.add_node(camera_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
