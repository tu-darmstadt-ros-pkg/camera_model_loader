#include <camera_model_loader/camera_model_loader.h>

#include <opencv2/highgui.hpp>

namespace camera_model {

CameraModelLoader::CameraModelLoader()
  : nh_(ros::NodeHandle()){
  it_.reset(new image_transport::ImageTransport(nh_));
}

bool CameraModelLoader::loadCamerasFromNamespace(ros::NodeHandle& nh) {
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace() + "/cameras");
  XmlRpc::XmlRpcValue cams;
  nh.getParam("cameras", cams);
  nh.getParam("mask_path", mask_path_);
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
  cam.setName(name);
  std::string rostopic;
  if (!nh.getParam("rostopic", rostopic)) {
    ROS_ERROR_STREAM("Could not get parameter " + nh.getNamespace() + "/rostopic");
    return false;
  }
  cam.setRostopic(rostopic);

  cam.setFrameId(nh.param<std::string>("frame_id", "")); //overrides image frame_id (optional)
  IntrinsicCalibration calibration;
  bool success = loadCalibration(nh, calibration);
  cam.setCalibration(calibration);
  if (!success) {
    ROS_ERROR_STREAM("Could not get intrinsic calibration in ns '" << nh.getNamespace() << "'");
    return false;
  }
  RadialTangentialDistortion distortion(cam.getCalibration().distortion_coeffs[0], cam.getCalibration().distortion_coeffs[1],
                                        cam.getCalibration().distortion_coeffs[2], cam.getCalibration().distortion_coeffs[3]);
  OmniProjection<RadialTangentialDistortion> projection(cam.getCalibration().intrinsics[0], cam.getCalibration().intrinsics[1], cam.getCalibration().intrinsics[2],
                                                        cam.getCalibration().intrinsics[3], cam.getCalibration().intrinsics[4], cam.getCalibration().resolution[0],
                                                        cam.getCalibration().resolution[1], distortion);
  std::string mask_filename;
  cv::Mat mask;
  if (nh.getParam("mask", mask_filename)) {
    std::string full_mask_path = mask_path_ + "/" + mask_filename;
    mask = cv::imread(full_mask_path, cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
      ROS_WARN_STREAM("Failed to load mask from '" << full_mask_path << "'.");
    }
  }
  ImageMask image_mask(mask);
  GlobalShutter global_shutter;
  cam.setCameraModel(boost::make_shared<CameraGeometry<OmniProjection<RadialTangentialDistortion>, GlobalShutter, ImageMask>>(projection, global_shutter, image_mask));
  std::pair<std::string, Camera> entry(name, cam);
  ROS_INFO_STREAM("Found cam: " << cam.getName() << std::endl
                  << " -- topic: " << rostopic << std::endl
                  << " -- frame_id: " << cam.getFrameId() << std::endl
                  << cam.getCalibration().toString());
  std::pair<std::map<std::string, Camera>::iterator, bool> result = cameras_.emplace(entry);
  if (!result.second) {
    ROS_WARN_STREAM("Couldn't create camera of name '" << cam.getName() << "' because it already existed.");
    return false;
  }

  //result.first->second.setSubscriber(it_->subscribe(rostopic, 1, boost::bind(&CameraModelLoader::imageCallback, this, name, _1)));
  return true;
}

void CameraModelLoader::startSubscribers() {
  for (std::map<std::string, camera_model::Camera>::iterator c = getCameraMap().begin(); c != getCameraMap().end(); ++c) {
    if (!c->second.getSubscriber()) {
      c->second.setSubscriber(it_->subscribe(c->second.getRostopic(), 1, boost::bind(&CameraModelLoader::imageCallback, this, c->second.getName(), _1)));
    }
  }
}

void CameraModelLoader::shutdownSubscribers() {
  for (std::map<std::string, camera_model::Camera>::iterator c = getCameraMap().begin(); c != getCameraMap().end(); ++c) {
    c->second.shutdownSubscriber();
  }
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
    cameras_.at(cam_name).setLastImage(image_ptr);
  } catch (std::out_of_range) {
    ROS_ERROR_STREAM("Could not find cam " << cam_name << ". This should not have happened. Please contact the maintainer.");
  }
}

std::map<std::string, Camera>& CameraModelLoader::getCameraMap() {
  return cameras_;
}

const Camera& CameraModelLoader::getCamera(std::string name) const{
  return cameras_.at(name);
}

}
