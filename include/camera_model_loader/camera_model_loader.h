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

  void startSubscribers();
  void shutdownSubscribers();

  const std::map<std::string, Camera> &getCameraMap() const;
  const Camera& getCamera(std::string name) const;

  bool waitForImages(const ros::Duration timeout = ros::Duration(0));
  bool receivedImages();

private:
  bool loadCamera(std::string name , ros::NodeHandle &nh);
  bool loadCalibration(ros::NodeHandle &nh, IntrinsicCalibration &calibration);
  void imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr);
  boost::shared_ptr<CameraGeometryBase> createCameraGeometry(const Camera &cam); // camera geometry factory

  std::map<std::string, Camera> cameras_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_;
  std::string mask_path_;
};

}

#endif
