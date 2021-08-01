#include "layer.h"
#include "net.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <float.h>
#include <stdio.h>
#include <vector>

#include "traffic_sign_detector_yolox/traffic_sign_detector_yolox.hpp"

namespace via {
namespace perception {
namespace traffic_sign {

using namespace std;

DEFINE_LAYER_CREATOR(YoloV5Focus)

TrafficSignDetectorYOLOX::TrafficSignDetectorYOLOX() {
  yolox = new ncnn::Net();
  yolox->opt.use_vulkan_compute = true;
  yolox->opt.use_bf16_storage = true;

  yolox->register_custom_layer("YoloV5Focus", YoloV5Focus_layer_creator);

  // original pretrained model from https://github.com/yolox
  // TODO ncnn model https://github.com/nihui/ncnn-assets/tree/master/models
  yolox->load_param("data/models/trafficsign_nano.param");
  yolox->load_model("data/models/trafficsign_nano.bin");
}

std::vector<via::definitions::perception::TrafficSign> TrafficSignDetectorYOLOX::Detect(const cv::Mat& bgr) {
  std::vector<Object> objects;
  yolox_mutex.lock();
  detect_yolox(yolox, bgr, objects);
  yolox_mutex.unlock();

  // Convert to signs
  std::vector<via::definitions::perception::TrafficSign> signs;
  for (size_t i = 0; i < objects.size(); ++i) {
    via::definitions::perception::TrafficSign sign;
    sign.box = objects[i].rect;
    sign.confidence = objects[i].prob;
    sign.sign_id = objects[i].label;
    signs.push_back(sign);
  }

  return signs;
}

}  // namespace traffic_sign
}  // namespace perception
}  // namespace via
