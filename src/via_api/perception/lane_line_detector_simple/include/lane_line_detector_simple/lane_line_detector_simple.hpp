#ifndef LANE_LINE_DETECTOR_SIMPLE_HPP_
#define LANE_LINE_DETECTOR_SIMPLE_HPP_

#include <stdio.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>
#include <via_definitions/perception/lane_line.hpp>

namespace via {
namespace perception {
namespace lane_line {

class LaneLineDetectorSimple {
 public:
  LaneLineDetectorSimple();
  std::vector<via::definitions::perception::LaneLine> Detect(
      const cv::Mat &bgr);
  static void InitPerspectiveMatrices();
  static cv::Mat &getPerspectiveMatrix();
  static cv::Mat &getInvertPerspectiveMatrix();

 private:
  static cv::Mat perspective_matrix_;
  static cv::Mat inverted_perspective_matrix_;
  std::vector<cv::Point2f> SlidingWindow(cv::Mat image, cv::Rect window);
  std::vector<cv::Point2f> FindTargetPoints(
      int im_width, int im_height, const std::vector<cv::Point2f> &left_pts,
      const std::vector<cv::Point2f> &right_pts);

  static int x_delta;
  static int y_delta;
};
}  // namespace lane_line
}  // namespace perception
}  // namespace via
#endif
