#ifndef VIA_IMAGE_CONVERTER_HPP_
#define VIA_IMAGE_CONVERTER_HPP_

#include <opencv2/opencv.hpp>
#include </workspaces/via-sdk/src/via_msgs/include/via_msgs/msg_defs.hpp>

namespace via {
namespace converters {

class ImageConverter {
 public:
  static via_msgs::msg::Image OpenCVMatToImageMsg(const cv::Mat& image);
};

}  // namespace converters
}  // namespace via
#endif