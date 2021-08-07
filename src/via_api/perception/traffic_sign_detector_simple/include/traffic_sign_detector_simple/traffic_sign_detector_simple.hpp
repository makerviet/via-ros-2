#ifndef TRAFFIC_SIGN_DETECTOR_SIMPLE_HPP_
#define TRAFFIC_SIGN_DETECTOR_SIMPLE_HPP_

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
#include <via_definitions/perception/traffic_sign.hpp>

namespace via {
namespace perception {
namespace traffic_sign {

class TrafficSignDetectorSimple {
 public:
  TrafficSignDetectorSimple();
  std::vector<via::definitions::perception::TrafficSign> Detect(const cv::Mat& bgr);

};
}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
#endif
