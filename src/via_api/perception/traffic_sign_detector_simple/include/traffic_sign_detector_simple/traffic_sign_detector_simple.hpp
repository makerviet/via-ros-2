#ifndef TRAFFIC_SIGN_DETECTOR_SIMPLE_HPP_
#define TRAFFIC_SIGN_DETECTOR_SIMPLE_HPP_

#include <stdio.h>

#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>
#include <via_definitions/perception/traffic_sign.hpp>

namespace via {
namespace perception {
namespace traffic_sign {

class TrafficSignDetectorSimple {
 public:
  TrafficSignDetectorSimple();
  std::vector<via::definitions::perception::TrafficSign> Detect(
      const cv::Mat &bgr);

 private:
  cv::dnn::Net model;
  cv::Mat FilterSignsByColor(const cv::Mat &bgr);
      std::vector<cv::Rect> MaskToBoxes(const cv::Mat &mask);
  std::vector<via::definitions::perception::TrafficSign> ClassifySigns(
      const cv::Mat &img, const std::vector<cv::Rect> &boxes);
  void ClassifySign(const cv::Mat &img, int &class_id, double &confidence);
  cv::Mat PreprocessImage(const cv::Mat &img);
};
}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
#endif
