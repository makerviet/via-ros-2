#ifndef VIA_IMAGE_CONVERTER_HPP_
#define VIA_IMAGE_CONVERTER_HPP_

#include <opencv2/opencv.hpp>
#include <via_definitions/msg_defs.hpp>

namespace via {
namespace converters {

class ImageConverter {
 public:
  static via_definitions::msg::Image OpenCVMatToImageMsg(const cv::Mat& image);
};

}  // namespace converters
}  // namespace via
#endif