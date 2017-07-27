#ifndef CAMERA_MODEL_LOADER_H
#define CAMERA_MODEL_LOADER_H

#include <camera_model_loader/camera.h>


namespace camera_model {

template<typename T> bool getParam(ros::NodeHandle& nh, const std::string& key, T& var) {
  if (!nh.getParam(key, var)) {
    ROS_ERROR_STREAM("Could not get parameter '" + nh.getNamespace() + "/" << key << "'");
    return false;
  } else {
    return true;
  }
}

using namespace aslam::cameras;

class CameraModelLoader {
public:
  CameraModelLoader();
  bool loadCamerasFromNamespace(ros::NodeHandle &nh);
//    void addCamera(std::string name, std::string topic, std::string frame_id, const IntrinsicCalibration& calibration);
  bool loadCamera(std::string name , ros::NodeHandle &nh);
  bool loadCalibration(ros::NodeHandle &nh, IntrinsicCalibration &calibration);

  void startSubscribers();
  void shutdownSubscribers();

  std::map<std::string, Camera> &getCameraMap();

private:
  void imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr);
  boost::shared_ptr<CameraGeometryBase> createCameraGeometry(const Camera &cam); // camera geometry factory

  std::map<std::string, Camera> cameras_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_;
};

}

#endif
