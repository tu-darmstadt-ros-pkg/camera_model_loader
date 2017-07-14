#ifndef CAMERA_MODEL_LOADER_CAMERA_H
#define CAMERA_MODEL_LOADER_CAMERA_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>

#include <aslam/cameras.hpp>

#include <image_transport/image_transport.h>

namespace camera_model {

constexpr double INVALID = std::numeric_limits<double>::max();

template<typename T>
std::string vecToString(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

struct Color {
  Color() : r(0), g(0), b(0) {}
  Color(uint8_t _r, uint8_t _g, uint8_t _b)
    : r(_r), g(_g), b(_b) {}

  double r, g, b;
};

struct IntrinsicCalibration {
  std::string camera_model;
  std::vector<double> intrinsics;
  std::string distortion_model;
  std::vector<double> distortion_coeffs;
  std::vector<int> resolution;

  std::string toString() {
    std::stringstream ss;
    ss << "Intrinsic calibration:" << std::endl;
    ss << " -- Camera model: " << camera_model << std::endl;
    ss << " -- Camera coeffs: " << vecToString(intrinsics) << std::endl;
    ss << " -- Distortion mode: " << distortion_model << std::endl;
    ss << " -- Distortion coeffs: " << vecToString(distortion_coeffs) << std::endl;
    ss << " -- Resolution: " << vecToString(resolution) << std::endl;
    return ss.str();
  }
};

class Camera {
public:
  std::string name;
  IntrinsicCalibration calibration;
  sensor_msgs::ImageConstPtr last_image;
  boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_model;
  image_transport::Subscriber sub;
  std::string frame_id; //overrides header.frame_id if set

  Color worldToColor(const Eigen::Vector3d& point3d, double& confidence) const;
private:
  cv::Vec3b getColorSubpix(const cv::Mat& img, cv::Point2f pt) const;
};

}

#endif
