#ifndef STEREO_VISION_HPP
#define STEREO_VISION_HPP

#include <memory>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "opencv2/core.hpp"

#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace bumperbot_vision
{
class StereoVision : public rclcpp::Node
{
public:
  StereoVision(const std::string & name);

private:
  image_transport::Subscriber left_image_sub_;
  image_transport::Subscriber right_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_sub_;
  image_transport::Publisher disparity_image_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  cv::Mat last_right_image_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr left_camera_info_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr right_camera_info_;

  void leftImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & left_msg);
  void rightImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & right_msg);
  void leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);
  void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);
};
}  // namespace bumperbot_vision

#endif  // STEREO_VISION_HPP