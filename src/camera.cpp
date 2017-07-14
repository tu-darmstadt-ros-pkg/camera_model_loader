#include <camera_model_loader/camera.h>

namespace camera_model {

Color Camera::worldToColor(const Eigen::Vector3d& point3d, double& confidence) const {
  Eigen::VectorXd pixel(2);
  if (camera_model->vsEuclideanToKeypoint(point3d, pixel)) {
    cv_bridge::CvImageConstPtr cv_image;
    try
    {
      cv_image = cv_bridge::toCvShare(last_image);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("Conversion failed: " << e.what());
      confidence = INVALID;
      return Color();
    }
    const cv::Mat& img = cv_image->image;
    Eigen::Vector2i pixel_i(std::round(pixel(0)), std::round(pixel(1)));
    cv::Vec3b color_vec = img.at<cv::Vec3b>(pixel_i(1), pixel_i(0));
    Color c;
    c.r = color_vec[0];
    c.g = color_vec[1];
    c.b = color_vec[2];

    confidence = std::pow(pixel(0) - img.rows / 2.0, 2) + std::pow(pixel(1) - img.cols / 2.0, 2);

    return c;
  } else {
    confidence = INVALID; // kinda hacky?
    return Color();
  }
}

std::string intrinsicsToString(const IntrinsicCalibration& calibration) {
  std::stringstream ss;
  ss << "Intrinsic calibration:" << std::endl;
  ss << " -- Camera model: " << calibration.camera_model << std::endl;
  ss << " -- Camera coeffs: " << vecToString(calibration.intrinsics) << std::endl;
  ss << " -- Distortion mode: " << calibration.distortion_model << std::endl;
  ss << " -- Distortion coeffs: " << vecToString(calibration.distortion_coeffs) << std::endl;
  ss << " -- Resolution: " << vecToString(calibration.resolution) << std::endl;
  return ss.str();
}

}
