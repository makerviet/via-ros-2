#ifndef TRAFFIC_SIGN_DETECTOR_YOLOX_HPP_
#define TRAFFIC_SIGN_DETECTOR_YOLOX_HPP_

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

#include "utils.hpp"

namespace via {
namespace perception {
namespace traffic_sign {

class TrafficSignDetectorYOLOX {
 public:
  TrafficSignDetectorYOLOX();
  std::vector<via::definitions::perception::TrafficSign> Detect(const cv::Mat& bgr);

 private:
  ncnn::Net *yolox;
  std::mutex yolox_mutex;
};
}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
#endif
