#include "bumperbot_vision/stereo_vision.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace bumperbot_vision
{
StereoVision::StereoVision(const std::string & name) : Node(name)
{
  left_image_sub_ = image_transport::create_subscription(
    this, "left_camera/image_raw",
    std::bind(&StereoVision::leftImageCallback, this, std::placeholders::_1), "raw",
    rmw_qos_profile_sensor_data);
  right_image_sub_ = image_transport::create_subscription(
    this, "right_camera/image_raw",
    std::bind(&StereoVision::rightImageCallback, this, std::placeholders::_1), "raw",
    rmw_qos_profile_sensor_data);
  left_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "left_camera/camera_info", 10,
    std::bind(&StereoVision::leftCameraInfoCallback, this, std::placeholders::_1));
  right_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "right_camera/camera_info", 10,
    std::bind(&StereoVision::rightCameraInfoCallback, this, std::placeholders::_1));
  disparity_image_pub_ =
    image_transport::create_publisher(this, "disparity", rmw_qos_profile_sensor_data);
  point_cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS());
}

void StereoVision::leftImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & left_msg)
{
  // Ensure we have the right data to process
  if (last_right_image_.empty() || !left_camera_info_ || !right_camera_info_) {
    RCLCPP_WARN(get_logger(), "Waiting for all data to be available before processing.");
    return;
  }

  cv::Mat left = cv_bridge::toCvCopy(left_msg, left_msg->encoding)->image;
  cv::Ptr<cv::StereoSGBM> sgbm =
    cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, last_right_image_, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0);
  cv::Mat normalized_disparity;
  double min_disp = 0.0;
  double max_disp = 96.0 * 16.0;
  cv::normalize(disparity_sgbm, normalized_disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

  auto disparity_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", normalized_disparity).toImageMsg();
  disparity_msg->header = left_msg->header;
  disparity_image_pub_.publish(disparity_msg);

  // Generate PointCloud
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pointcloud_msg.header = left_msg->header;
  pointcloud_msg.height = 1;
  pointcloud_msg.width = left.rows * left.cols;
  pointcloud_msg.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pointcloud_msg, "intensity");

  for (int v = 0; v < left.rows; v++) {
    for (int u = 0; u < left.cols; u++) {
      if (disparity.at<float>(v, u) <= 10.0 || disparity.at<float>(v, u) >= 96.0) {
        continue;
      }
      double x = (u - left_camera_info_->k[2]) / left_camera_info_->k[0];
      double y = (v - left_camera_info_->k[5]) / left_camera_info_->k[4];
      double depth = left_camera_info_->k[0] * 0.06 / (disparity.at<float>(v, u));
      *iter_x = x * depth;
      *iter_y = y * depth;
      *iter_z = depth;
      *iter_intensity = left.at<uchar>(v, u) / 255.0;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
  }
  point_cloud_pub_->publish(pointcloud_msg);
}

void StereoVision::rightImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
{
  last_right_image_ = cv_bridge::toCvCopy(right_msg, right_msg->encoding)->image;
}

void StereoVision::leftCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  left_camera_info_ = info_msg;
}

void StereoVision::rightCameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  right_camera_info_ = info_msg;
}
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::StereoVision>("stereo_vision"));
  rclcpp::shutdown();
  return 0;
}