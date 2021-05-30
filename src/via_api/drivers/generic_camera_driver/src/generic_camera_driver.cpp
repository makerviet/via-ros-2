#include "generic_camera_driver/generic_camera_driver.hpp"

namespace via {
namespace drivers {
namespace camera {

GenericCameraDriver::GenericCameraDriver(
    const std::string filename, int image_width, int image_height,
    std::function<void(cv::Mat)> callback) {
  image_width_ = image_width;
  image_height_ = image_height;
  callback_ = callback;
  capturing_ = false;
  filename_ = filename;
}

GenericCameraDriver::~GenericCameraDriver() { Stop(); }

void GenericCameraDriver::CaptureLoop() {
  while (true) {
    if (!capturing_) {
      break;
    }
    cap_ >> frame_;
    if (!frame_.empty()) {
      callback_(frame_);
    }
  }
  std::cout << "Exitting camera loop" << std::endl;
}

void GenericCameraDriver::OpenCamera() {
  const std::lock_guard<std::mutex> lock(cap_mutex_);
  if (IsInteger(filename_)) {
    cap_.open(std::stoi(filename_), cv::CAP_V4L2);
    std::cout << "Capturing from camera: " << std::stoi(filename_) << std::endl;
  } else {
    cap_.open(filename_, cv::CAP_V4L2);
    std::cout << "Capturing from stream: " << filename_ << std::endl;
  }

  if (!cap_.isOpened()) {
    std::cerr << std::string("Error: Could not read camera stream from: ") + filename_ << std::endl;
    throw std::runtime_error(std::string("Error: Could not read camera stream from: ") + filename_);
  }

  // Set frame properties
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

  // Check to confirm frame properties
  cap_ >> frame_;
  if (frame_.rows != image_height_) {
    std::cerr << "Warning: Could not set image height." << std::endl;
  }
  if (frame_.cols != image_width_) {
    std::cerr << "Warning: Could not set image width." << std::endl;
  }
}

void GenericCameraDriver::CloseCamera() {
  const std::lock_guard<std::mutex> lock(cap_mutex_);
  if (cap_.isOpened()) {
    cap_.release();
  }
}

void GenericCameraDriver::Start() {
  if (capturing_) {
    std::cerr << "Camera is still running. Please stop it before starting again." << std::endl;
    throw std::runtime_error("Camera is still running. Please stop it before starting again.");
  } try {
    OpenCamera();
  } catch (const char *excp) {
    throw excp;
  }
  
  capturing_ = true;

  const std::lock_guard<std::mutex> lock(cap_thread_mutex_);
  cap_thread_ = std::thread(&GenericCameraDriver::CaptureLoop, this);
}

void GenericCameraDriver::Stop() {
  capturing_ = false;
  const std::lock_guard<std::mutex> lock(cap_thread_mutex_);
  if (cap_thread_.joinable()) {
    cap_thread_.join();
  }
  CloseCamera();
}

bool GenericCameraDriver::IsInteger(const std::string& s) {
  return !s.empty() && std::find_if(s.begin(), s.end(), [](unsigned char c) {
                         return !std::isdigit(c);
                       }) == s.end();
}

}  // namespace camera
}  // namespace drivers
}  // namespace via
