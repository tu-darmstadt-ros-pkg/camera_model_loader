#ifndef CAMERA_MODEL_LOADER_CAMERA_H
#define CAMERA_MODEL_LOADER_CAMERA_H

#include <ros/ros.h>

#include <Eigen/Eigen>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#pragma GCC diagnostic pop

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
  Camera();

  Color worldToColor(const Eigen::Vector3d& point3d, double& confidence);
  double distanceFromCenter(int width, int height, Eigen::Vector2d& pixel) const;
  bool worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const;

  void shutdownSubscriber();

  std::string getName() const;
  void setName(const std::string &value);

  IntrinsicCalibration getCalibration() const;
  void setCalibration(const IntrinsicCalibration &value);

  sensor_msgs::ImageConstPtr getLastImage() const;
  void setLastImage(const sensor_msgs::ImageConstPtr &value);

  boost::shared_ptr<aslam::cameras::CameraGeometryBase> getCameraModel() const;
  void setCameraModel(const boost::shared_ptr<aslam::cameras::CameraGeometryBase> &value);

  image_transport::Subscriber getSubscriber() const;
  void setSubscriber(const image_transport::Subscriber &value);

  std::string getFrameId() const;
  void setFrameId(const std::string &value);

  cv::Mat getLastImageCV();
  std::string getRostopic() const;
  void setRostopic(const std::string &rostopic);

private:
  cv::Vec3b interpolate(const cv::Mat& img, const Eigen::Vector2d &pixel) const;

  std::string name_;
  IntrinsicCalibration calibration_;
  sensor_msgs::ImageConstPtr last_image_;
  cv_bridge::CvImageConstPtr cv_last_image_;
  bool cv_updated_;
  boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_model_;
  image_transport::Subscriber sub_;
  std::string rostopic_;
  std::string frame_id_; //overrides header.frame_id if set
};

}

#endif
