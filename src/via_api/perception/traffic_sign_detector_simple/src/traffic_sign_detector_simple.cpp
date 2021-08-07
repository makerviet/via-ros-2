#include "traffic_sign_detector_simple/traffic_sign_detector_simple.hpp"

#include <float.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace via {
namespace perception {
namespace traffic_sign {

using namespace std;
using namespace cv;

TrafficSignDetectorSimple::TrafficSignDetectorSimple() {}


std::vector<via::definitions::perception::TrafficSign>
TrafficSignDetectorSimple::Detect(const cv::Mat& bgr) {
  cv::Mat img;
  cv::medianBlur(bgr, img, 3);

  Mat img_hsv;
  cvtColor(img, img_hsv, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  // Filter red color for stop and prohibition signs
  Mat mask1, mask2;
  inRange(img_hsv, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
  inRange(img_hsv, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
  Mat red_mask = mask1 | mask2;

  // Filter blue color for turn left, turn right and go straight sign
  Mat blue_mask;
  inRange(img_hsv, cv::Scalar(85, 50, 50), cv::Scalar(135, 250, 180), blue_mask);

  // Show blobs
  imshow("red_mask", red_mask);
  imshow("blue_mask", blue_mask);
  waitKey(1);

  // Convert to signs
  std::vector<via::definitions::perception::TrafficSign> signs;
  // for (size_t i = 0; i < objects.size(); ++i) {
  //   via::definitions::perception::TrafficSign sign;
  //   sign.box = objects[i].rect;
  //   sign.confidence = objects[i].prob;
  //   sign.sign_id = objects[i].label;
  //   signs.push_back(sign);
  // }

  return signs;
}

}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
