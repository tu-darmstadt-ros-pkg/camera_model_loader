#include <camera_model_loader/camera.h>

namespace camera_model {

Camera::Camera()
  : cv_updated_(false) {

}

bool Camera::worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const {
  Eigen::VectorXd pixel(2);
  if (camera_model_->vsEuclideanToKeypoint(point3d, pixel)) {
    pixel_out = Eigen::Vector2d(pixel(0), pixel(1));
    return true;
  } else {
    pixel_out = Eigen::Vector2d::Zero();
    return false;
  }
}

double Camera::distanceFromCenter(int width, int height, Eigen::Vector2d& pixel) const {
  return std::pow(pixel(0) - width / 2.0, 2) + std::pow(pixel(1) - height / 2.0, 2);
}

Color Camera::worldToColor(const Eigen::Vector3d& point3d, double& confidence) {
  Eigen::Vector2d pixel(2);
  if (worldToPixel(point3d, pixel)) {


    const cv::Mat& img = getLastImageCV();
    cv::Vec3b color_vec = interpolate(img, pixel);
    confidence = distanceFromCenter(img.cols, img.rows, pixel);
    return Color(color_vec[0], color_vec[1], color_vec[2]);
  } else {
    confidence = INVALID; // kinda hacky?
    return Color();
  }
}

cv::Vec3b Camera::interpolate(const cv::Mat& img, const Eigen::Vector2d& pixel) const {
  cv::Point2f pt(pixel(0), pixel(1));
  cv::Mat patch;
  cv::getRectSubPix(img, cv::Size(1,1), pt, patch);
  return patch.at<cv::Vec3b>(0,0);
}

void Camera::shutdownSubscriber() {
  sub_.shutdown();
}

std::string Camera::getRostopic() const
{
  return rostopic_;
}

void Camera::setRostopic(const std::string &rostopic)
{
  rostopic_ = rostopic;
}

cv::Mat Camera::getLastImageCV() {
  if (!cv_updated_) {
    try
    {
      cv_last_image_ = cv_bridge::toCvShare(getLastImage());
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("CV Bridge conversion failed: " << e.what());
      return cv::Mat();
    }
    cv_updated_ = true;
  }

  return cv_last_image_->image;
}

std::string Camera::getFrameId() const {
  return frame_id_;
}

void Camera::setFrameId(const std::string &value) {
  frame_id_ = value;
}

image_transport::Subscriber Camera::getSubscriber() const {
  return sub_;
}

void Camera::setSubscriber(const image_transport::Subscriber &value) {
  sub_ = value;
}

boost::shared_ptr<aslam::cameras::CameraGeometryBase> Camera::getCameraModel() const {
  return camera_model_;
}

void Camera::setCameraModel(const boost::shared_ptr<aslam::cameras::CameraGeometryBase> &value) {
  camera_model_ = value;
}

sensor_msgs::ImageConstPtr Camera::getLastImage() const {
  return last_image_;
}

void Camera::setLastImage(const sensor_msgs::ImageConstPtr &value) {
  cv_updated_ = false;
  last_image_ = value;
}

IntrinsicCalibration Camera::getCalibration() const {
  return calibration_;
}

void Camera::setCalibration(const IntrinsicCalibration &value) {
  calibration_ = value;
}

std::string Camera::getName() const {
  return name_;
}

void Camera::setName(const std::string &value) {
  name_ = value;
}

}
