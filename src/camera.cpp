#include <camera_model_loader/camera.h>

namespace camera_model {

bool Camera::worldToPixel(const Eigen::Vector3d& point3d, Eigen::Vector2d& pixel_out) const {
  Eigen::VectorXd pixel(2);
  if (camera_model->vsEuclideanToKeypoint(point3d, pixel)) {
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

Color Camera::worldToColor(const Eigen::Vector3d& point3d, double& confidence) const {
  Eigen::Vector2d pixel(2);
  if (worldToPixel(point3d, pixel)) {

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

}
