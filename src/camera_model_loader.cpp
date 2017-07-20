#include <camera_model_loader/camera_model_loader.h>

namespace camera_model {

CameraModelLoader::CameraModelLoader()
  : nh_(ros::NodeHandle()){
  it_.reset(new image_transport::ImageTransport(nh_));
}

bool CameraModelLoader::loadCamerasFromNamespace(ros::NodeHandle& nh) {
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace() + "/cameras");
  XmlRpc::XmlRpcValue cams;
  nh.getParam("cameras", cams);
  ROS_ASSERT(cams.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = cams.begin(); it != cams.end(); ++it) {
    std::string cam_name = (std::string)(it->first);
    ros::NodeHandle cam_nh(nh, "cameras/" + cam_name);
    if (!loadCamera(cam_name, cam_nh)) {
      return false;
    }
  }
  return true;
}

bool CameraModelLoader::loadCamera(std::string name, ros::NodeHandle &nh) {
  Camera cam;
  cam.name_ = name;
  std::string rostopic;
  if (!nh.getParam("rostopic", rostopic)) {
    ROS_ERROR_STREAM("Could not get parameter " + nh.getNamespace() + "/rostopic");
    return false;
  }

  nh.param<std::string>("frame_id", cam.frame_id_, ""); //overrides image frame_id (optional)
  bool success = loadCalibration(nh, cam.calibration_);
  if (!success) {
    ROS_ERROR_STREAM("Could not get intrinsic calibration in ns '" << nh.getNamespace() << "'");
    return false;
  }
  RadialTangentialDistortion distortion(cam.calibration_.distortion_coeffs[0], cam.calibration_.distortion_coeffs[1],
                                        cam.calibration_.distortion_coeffs[2], cam.calibration_.distortion_coeffs[3]);
  OmniProjection<RadialTangentialDistortion> projection(cam.calibration_.intrinsics[0], cam.calibration_.intrinsics[1], cam.calibration_.intrinsics[2],
                                                        cam.calibration_.intrinsics[3], cam.calibration_.intrinsics[4], cam.calibration_.resolution[0],
                                                        cam.calibration_.resolution[1], distortion);
  cam.camera_model_.reset(new CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, NoMask>(projection));
  std::pair<std::string, Camera> entry(name, cam);
  ROS_INFO_STREAM("Found cam: " << cam.name_ << std::endl
                  << " -- topic: " << rostopic << std::endl
                  << " -- frame_id: " << cam.frame_id_ << std::endl
                  << cam.calibration_.toString());
  std::pair<std::map<std::string, Camera>::iterator, bool> result = cameras_.emplace(entry);
  if (!result.second) {
    ROS_WARN_STREAM("Couldn't create camera of name '" << cam.name_ << "' because it already existed.");
    return false;
  }

  result.first->second.sub = it_->subscribe(rostopic, 1, boost::bind(&CameraModelLoader::imageCallback, this, name, _1));
  return true;
}

bool CameraModelLoader::loadCalibration(ros::NodeHandle& nh, IntrinsicCalibration& calibration) {
  ROS_INFO_STREAM("Loading intrinsics from nh: " << nh.getNamespace());
  bool valid = true;
  valid = valid && getParam<std::string>(nh, "camera_model", calibration.camera_model);
  valid = valid && getParam<std::vector<double>>(nh, "intrinsics", calibration.intrinsics);
  valid = valid && getParam<std::string>(nh, "distortion_model", calibration.distortion_model);
  valid = valid && getParam<std::vector<double>>(nh, "distortion_coeffs", calibration.distortion_coeffs);
  valid = valid && getParam<std::vector<int>>(nh, "resolution", calibration.resolution);
  return valid;
}

//boost::shared_ptr<CameraGeometryBase> ColorCloudFromImage::createCameraGeometry(const Camera& cam) {

//  if (cam.calibration.distortion_model == "radtan") {
//    RadialTangentialDistortion distortion(cam.calibration.distortion_coeffs[0], cam.calibration.distortion_coeffs[1],
//        cam.calibration.distortion_coeffs[2], cam.calibration.distortion_coeffs[3]);


//  }
//}

void CameraModelLoader::imageCallback(std::string cam_name, const sensor_msgs::ImageConstPtr& image_ptr) {
  try {
    cameras_.at(cam_name).last_image_ = image_ptr;
  } catch (std::out_of_range) {
    ROS_ERROR_STREAM("Could not find cam " << cam_name << ". This should not have happened. Please contact the maintainer.");
  }
}

const std::map<std::string, Camera>& CameraModelLoader::getCameraMap() {
  return cameras_;
}

}
