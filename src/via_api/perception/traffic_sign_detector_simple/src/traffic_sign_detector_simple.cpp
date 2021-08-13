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

TrafficSignDetectorSimple::TrafficSignDetectorSimple() {
  std::string package_dir = ament_index_cpp::get_package_share_directory("traffic_sign_detector_simple");
  std::string model_path = package_dir + "/models/resnet18.onnx";
  std::cout << "Loading sign classification model: " << model_path << std::endl;
  model = cv::dnn::readNet(model_path);
}

std::vector<via::definitions::perception::TrafficSign>
TrafficSignDetectorSimple::Detect(const cv::Mat &bgr) {
  // Filter traffic sign by color
  // to obtain mask
  Mat traffic_sign_mask = FilterSignsByColor(bgr);

  // Convert mask to bounding boxes
  std::vector<cv::Rect> bboxes = MaskToBoxes(traffic_sign_mask);

  // Classify traffic signs
  std::vector<via::definitions::perception::TrafficSign> signs =
      ClassifySigns(bgr, bboxes);

  return signs;
}

cv::Mat TrafficSignDetectorSimple::FilterSignsByColor(const cv::Mat &bgr) {
  Mat img;
  cv::medianBlur(bgr, img, 3);

  Mat img_hsv;
  cvtColor(img, img_hsv,
           cv::COLOR_BGR2HSV);  // Convert the captured frame from BGR to HSV

  // Filter red color for stop and prohibition signs
  Mat mask1, mask2;
  inRange(img_hsv, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
  inRange(img_hsv, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
  Mat red_mask = mask1 | mask2;

  // Filter blue color for turn left, turn right and go straight sign
  Mat blue_mask;
  inRange(img_hsv, cv::Scalar(85, 50, 50), cv::Scalar(135, 250, 180),
          blue_mask);

  Mat traffic_sign_mask = red_mask | blue_mask;
  return traffic_sign_mask;
}

std::vector<cv::Rect> TrafficSignDetectorSimple::MaskToBoxes(
    const cv::Mat &mask) {
  // Find connected components
  cv::Mat labels, centroids, img_color, stats;
  int nccomps =
             cv::connectedComponentsWithStats(mask, labels, stats, centroids);
  // Find bboxes
  std::vector<cv::Rect> bboxes;
  for (int i = 1; i < nccomps; i++) {
    int x = stats.at<int>(i, CC_STAT_LEFT);
    int y = stats.at<int>(i, CC_STAT_TOP);
    int w = stats.at<int>(i, CC_STAT_WIDTH);
    int h = stats.at<int>(i, CC_STAT_HEIGHT);
    if (w < 20 || h < 20) continue;  // Filter small boxes
    bboxes.push_back(cv::Rect(x, y, w, h));
  }
  return bboxes;
}

std::vector<via::definitions::perception::TrafficSign>
TrafficSignDetectorSimple::ClassifySigns(const cv::Mat &img,
                                         const std::vector<cv::Rect> &boxes) {
  std::vector<via::definitions::perception::TrafficSign> signs;
  for (size_t i = 0; i < boxes.size(); ++i) {
    // Run classifier
    double confidence;
    int class_id;
    ClassifySign(img(boxes[i]), class_id, confidence);

    // Skip unknown signs
    // or signs with low confidence
    if (class_id == 6 || confidence < 0.6) {
      continue;
    }

    // Add sign
    via::definitions::perception::TrafficSign sign;
    sign.box = boxes[i];
    sign.confidence = confidence;
    sign.class_id = class_id;
    signs.push_back(sign);
  }
  return signs;
}

std::vector<float> softmax(const std::vector<float> &input) {
  std::vector<float> result(input.size());
  float sum = 0;
  for (size_t i = 0; i < input.size(); i++) sum += exp(input[i]);
  for (size_t i = 0; i < input.size(); i++) result[i] = exp(input[i]) / sum;
  return result;
}

void TrafficSignDetectorSimple::ClassifySign(const cv::Mat &img, int &class_id,
                                             double &confidence) {
  cv::Mat processed_img = PreprocessImage(img);
  model.setInput(processed_img);
  cv::Mat prob = model.forward();

  // Apply softmax to get prob
  cv::Mat prob_reshaped = prob.reshape(1, prob.total() * prob.channels());
  std::vector<float> prob_vec =
      prob_reshaped.isContinuous() ? prob_reshaped : prob_reshaped.clone();
  std::vector<float> prob_normalized = softmax(prob_vec);

  // Minmax loc
  cv::Point class_id_point;
  minMaxLoc(prob.reshape(1, 1), 0, &confidence, 0, &class_id_point);
  class_id = class_id_point.x;
  confidence = prob_normalized[class_id];
}

cv::Mat TrafficSignDetectorSimple::PreprocessImage(const cv::Mat &img) {
  cv::Mat processed_img;
  cv::resize(img, processed_img, cv::Size(128, 128));
  cv::cvtColor(processed_img, processed_img,
               cv::ColorConversionCodes::COLOR_BGR2RGB);
  processed_img.convertTo(processed_img, CV_32F, 1.0 / 255);

  cv::Mat channels[3];
  cv::split(processed_img, channels);
  cv::Scalar mean{0.4151, 0.3771, 0.4568};
  cv::Scalar std{0.2011, 0.2108, 0.1896};
  channels[0] = (channels[0] - 0.4151) / 0.2011;
  channels[1] = (channels[1] - 0.3771) / 0.2108;
  channels[2] = (channels[2] - 0.4568) / 0.1896;

  cv::merge(channels, 3, processed_img);
  cv::dnn::blobFromImage(processed_img, processed_img);

  return processed_img;
}

}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
