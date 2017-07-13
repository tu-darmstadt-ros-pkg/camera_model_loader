#ifndef CAMERA_MODEL_LOADER_H
#define CAMERA_MODEL_LOADER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <aslam/cameras.hpp>


namespace camera_model {

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

template<typename T> bool getParam(ros::NodeHandle& nh, const std::string& key, T& var) {
  if (!nh.getParam(key, var)) {
    ROS_ERROR_STREAM("Could not get parameter '" + nh.getNamespace() + "/" << key << "'");
    return false;
  } else {
    return true;
  }
}

using namespace aslam::cameras;


struct IntrinsicCalibration {
  std::string camera_model;
  std::vector<double> intrinsics;
  std::string distortion_model;
  std::vector<double> distortion_coeffs;
  std::vector<int> resolution;
};

struct Camera {
  std::string name;
  IntrinsicCalibration calibration;
  sensor_msgs::ImageConstPtr last_image;
  boost::shared_ptr<aslam::cameras::CameraGeometryBase> camera_model;
  image_transport::Subscriber sub;
  std::string frame_id; //overrides header.frame_id if set
};

class CameraModelLoader {
public:
  CameraModelLoader();
  bool loadCamerasFromNamespace(ros::NodeHandle &nh);
//    void addCamera(std::string name, std::string topic, std::string frame_id, const IntrinsicCalibration& calibration);
  bool loadCamera(std::string name , ros::NodeHandle &nh);
  bool loadCalibration(ros::NodeHandle &nh, IntrinsicCalibration &calibration);

  const std::map<std::string, Camera>& getCameraMap();

  std::string intrinsicsToString(const IntrinsicCalibration& calibration);
private:
  void imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr);
  boost::shared_ptr<CameraGeometryBase> createCameraGeometry(const Camera &cam); // camera geometry factory

  std::map<std::string, Camera> cameras_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_;
};

}

#endif
