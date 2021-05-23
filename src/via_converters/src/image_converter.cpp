#include <via_converters/image_converter.hpp>

namespace via {
namespace converters {

via_msgs::msg::Image ImageConverter::OpenCVMatToImageMsg(const cv::Mat &frame) {
  sensor_msgs::msg::Image ros_image;
  ros_image.height = frame.rows;
  ros_image.width = frame.cols;
  ros_image.encoding = "bgr8";
  ros_image.is_bigendian = false;
  ros_image.step = frame.cols * frame.elemSize();
  size_t size = ros_image.step * frame.rows;
  ros_image.data.resize(size);

  if (frame.isContinuous()) {
    memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
  } else {
    uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
    uchar *cv_data_ptr = frame.data;
    for (int i = 0; i < frame.rows; ++i) {
      memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
      ros_data_ptr += ros_image.step;
      cv_data_ptr += frame.step;
    }
  }
  return ros_image;
}

}  // namespace converters
}  // namespace via