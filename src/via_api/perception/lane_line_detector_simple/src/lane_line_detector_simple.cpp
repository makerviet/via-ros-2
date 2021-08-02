#include "lane_line_detector_simple/lane_line_detector_simple.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace via {
namespace perception {
namespace lane_line {

using namespace std;
using namespace cv;

cv::Mat LaneLineDetectorSimple::perspective_matrix_;
cv::Mat LaneLineDetectorSimple::inverted_perspective_matrix_;

int LaneLineDetectorSimple::x_delta = 0;
int LaneLineDetectorSimple::y_delta = 0;

LaneLineDetectorSimple::LaneLineDetectorSimple() {
  InitPerspectiveMatrices();
}

void LaneLineDetectorSimple::InitPerspectiveMatrices() {
  // Prepare things that don't need to be computed on every frame.
  Point2f src_vertices[4];

  // Define points that are used for generating bird's eye view. This was done
  // by trial and error. Best to prepare sliders and configure for each use
  // case.
  x_delta = 1800;
  y_delta = 200;
  src_vertices[0] = Point(0, y_delta);
  src_vertices[1] = Point(640, y_delta);
  src_vertices[2] = Point(640, 480);
  src_vertices[3] = Point(0, 480);

  // Destination vertices. Output is 640 by 480px
  Point2f dst_vertices[4];
  dst_vertices[0] = Point(-x_delta, 0);
  dst_vertices[1] = Point(640 + x_delta, 0);
  dst_vertices[2] = Point(640, 480);
  dst_vertices[3] = Point(0, 480);

  dst_vertices[0].x += x_delta;
  dst_vertices[1].x += x_delta;
  dst_vertices[2].x += x_delta;
  dst_vertices[3].x += x_delta;

  // Prepare matrix for transform and get the warped image
  perspective_matrix_ = getPerspectiveTransform(src_vertices, dst_vertices);

  // For transforming back into original image space
  invert(perspective_matrix_, inverted_perspective_matrix_);
}

cv::Mat &LaneLineDetectorSimple::getPerspectiveMatrix() {
  if (perspective_matrix_.empty()) {
    InitPerspectiveMatrices();
  }
  return perspective_matrix_;
}

cv::Mat &LaneLineDetectorSimple::getInvertPerspectiveMatrix() {
  if (inverted_perspective_matrix_.empty()) {
    InitPerspectiveMatrices();
  }
  return inverted_perspective_matrix_;
}

std::vector<via::definitions::perception::LaneLine>
LaneLineDetectorSimple::Detect(const cv::Mat &org) {
  Mat img;                     // Working image
  Mat dst(480, 640 + 2 * x_delta, CV_8UC3);  // Destination for warped image
  Mat viz_img = org.clone();

  via::definitions::perception::LaneLine left_line;
  left_line.confidence = 1.0;
  via::definitions::perception::LaneLine right_line;
  right_line.confidence = 1.0;
  via::definitions::perception::LaneLine target_line;

  resize(org, img, Size(640, 480));

  // Generate bird's eye view
  warpPerspective(img, dst, perspective_matrix_, dst.size(), INTER_LINEAR,
                  BORDER_CONSTANT);

  // Convert to gray
  cvtColor(dst, img, COLOR_RGB2GRAY);

  // Extract yellow and white info
  Mat mask_yellow, mask_white;

  inRange(img, Scalar(20, 100, 100), Scalar(30, 255, 255), mask_yellow);
  inRange(img, Scalar(150, 150, 150), Scalar(255, 255, 255), mask_white);

  Mat mask, processed;
  bitwise_or(mask_yellow, mask_white, mask);  // Combine the two masks
  bitwise_and(img, mask, processed);          // Extrect what matches

  // Blur the image a bit so that gaps are smoother
  const Size kernel_size = Size(9, 9);
  GaussianBlur(processed, processed, kernel_size, 0);

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
  vector<Point2f> left_pts = SlidingWindow(processed, Rect(1776, 470, 220, 10));
  vector<Point2f> right_pts = SlidingWindow(processed, Rect(2241, 470, 220, 10));

  int lane_width = 400;
  if (left_pts.empty() && !right_pts.empty()) {
    for (size_t i = 0; i < right_pts.size(); ++i) {
      left_pts.push_back(Point2f(right_pts[i].x - lane_width, right_pts[i].y));
    }
    left_line.confidence = 0.5;
  }
  if (right_pts.empty() && !left_pts.empty()) {
    for (size_t i = 0; i < left_pts.size(); ++i) {
      right_pts.push_back(Point2f(left_pts[i].x + lane_width, left_pts[i].y));
    }
    right_line.confidence = 0.5;
  }

  vector<Point2f> target_pts = FindTargetPoints(640 + 2 * x_delta, 480, left_pts, right_pts);

  if (!left_pts.empty()) {
    // Transform points back  into original image space
    perspectiveTransform(left_pts, out_pts, inverted_perspective_matrix_);
    // Draw the points onto the out image
    for (size_t i = 0; i < out_pts.size() - 1; ++i) {
      line(viz_img, out_pts[i], out_pts[i + 1], Scalar(255, 0, 0), 3);
      all_pts.push_back(Point(out_pts[i].x, out_pts[i].y));
    }
    all_pts.push_back(
        Point(out_pts[out_pts.size() - 1].x, out_pts[out_pts.size() - 1].y));
    for (size_t i = 0; i < left_pts.size() - 1; ++i)  // Draw a line on the
                                                      // processed image
      line(out, left_pts[i], left_pts[i + 1], Scalar(255, 0, 0), 3);
  }

  if (!right_pts.empty()) {
    // Transform points back  into original image space
    perspectiveTransform(right_pts, out_pts, inverted_perspective_matrix_);
    // Draw the other lane and append points
    for (size_t i = 0; i < out_pts.size() - 1; ++i) {
      line(viz_img, out_pts[i], out_pts[i + 1], Scalar(0, 0, 255), 3);
      all_pts.push_back(Point(out_pts[out_pts.size() - i - 1].x,
                              out_pts[out_pts.size() - i - 1].y));
    }
    all_pts.push_back(Point(out_pts[0].x - (out_pts.size() - 1), out_pts[0].y));
    for (size_t i = 0; i < right_pts.size() - 1; ++i)
      line(out, right_pts[i], right_pts[i + 1], Scalar(0, 0, 255), 3);
  }

  if (!target_pts.empty()) {
    // Transform points back  into original image space
    perspectiveTransform(target_pts, out_pts, inverted_perspective_matrix_);
    // Draw the other lane and append points
    for (size_t i = 0; i < out_pts.size() - 1; ++i) {
      line(viz_img, out_pts[i], out_pts[i + 1], Scalar(0, 255, 255), 3);
    }
    for (size_t i = 0; i < target_pts.size() - 1; ++i)
      line(out, target_pts[i], target_pts[i + 1], Scalar(0, 255, 255), 3);
  }

  // Create a green-ish overlay
  if (!all_pts.empty()) {
    vector<vector<Point>> arr;
    arr.push_back(all_pts);
    Mat overlay = Mat::zeros(org.size(), org.type());
    fillPoly(overlay, arr, Scalar(0, 255, 100));
    addWeighted(viz_img, 1, overlay, 0.5, 0, org);  // Overlay it
  }

  // Show results
  // imshow("Result", out);
  // imshow("Overlay", viz_img);
  // waitKey(1);

  left_line.points = left_pts;
  right_line.points = right_pts;
  target_line.points = target_pts;
  target_line.confidence = min(left_line.confidence, right_line.confidence);
  std::vector<via::definitions::perception::LaneLine> lane_lines{
      left_line, right_line, target_line};
  return lane_lines;
}

vector<Point2f> LaneLineDetectorSimple::SlidingWindow(Mat image, Rect window) {
  vector<Point2f> points;
  const Size img_size = image.size();
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

    for (size_t i = 0; i < locations.size();
         ++i)  // Calculate average X position
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
    if (window.x + window.width >= img_size.width)
      window.x = img_size.width - window.width - 1;

    if (shouldBreak) break;
  }

  return points;
}

std::vector<Point2f> LaneLineDetectorSimple::FindTargetPoints(
    int im_width, int im_height, const vector<Point2f> &left_pts,
    const vector<Point2f> &right_pts) {
  vector<Point2f> target_pts;  // Init target points

  // Check left / right points
  if (left_pts.empty() || right_pts.empty()) {
    return target_pts;
  }

  // Find drivable area
  cv::Mat drivable_area = cv::Mat::zeros(im_height, im_width, CV_8UC1);
  vector<Point> all_pts;
  for (size_t i = 0; i < left_pts.size(); ++i) {
    all_pts.push_back(Point(left_pts[i].x, left_pts[i].y));
  }
  for (int i = right_pts.size() - 1; i >= 0; --i) {
    all_pts.push_back(Point(right_pts[i].x, right_pts[i].y));
  }
  vector<vector<Point>> arr;
  arr.push_back(all_pts);
  fillPoly(drivable_area, arr, Scalar(255));

  // imshow("Drivable", drivable_area);
  // waitKey(1);

  // Find interested area by y-axis
  float min_y = max(0.0f, min(min(left_pts[0].y, left_pts[-1].y),
                              min(right_pts[0].y, right_pts[-1].y)));
  float max_y =
      min(float(im_height - 1), max(max(left_pts[0].y, left_pts[-1].y),
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

}  // namespace lane_line
}  // namespace perception
}  // namespace via
