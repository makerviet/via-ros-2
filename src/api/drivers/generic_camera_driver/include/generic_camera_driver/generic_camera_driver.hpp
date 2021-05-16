#ifndef VIA_GENERIC_CAMERA_DRIVER_HPP_
#define VIA_GENERIC_CAMERA_DRIVER_HPP_

#include <stdio.h>

#include <functional>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

namespace via {
namespace drivers {
namespace camera {

class GenericCameraDriver {
 public:
  GenericCameraDriver(const std::string filename, int image_width,
                      int image_height, std::function<void(cv::Mat)> callback);
  ~GenericCameraDriver();
  void Start();
  void Stop();
  void OpenCamera();
  void CloseCamera();

 private:
  std::string filename_;
  cv::Mat frame_;
  cv::VideoCapture cap_;
  std::thread cap_thread_;
  bool capturing_;
  std::mutex cap_mutex_;

  int image_height_;
  int image_width_;
  std::function<void(cv::Mat)> callback_;

  void CaptureLoop();
  bool IsInteger(const std::string& s);
};
}  // namespace camera
}  // namespace drivers
}  // namespace via
#endif
