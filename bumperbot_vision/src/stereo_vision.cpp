#include "bumperbot_vision/stereo_vision.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace bumperbot_vision
{
StereoVision::StereoVision(const std::string & name) : Node(name)
{
  approximate_sync_ = std::make_shared<ApproximateSync>(
    ApproximatePolicy(10), left_image_sub_, left_info_sub_, right_image_sub_, right_info_sub_);
  approximate_sync_->registerCallback(
    std::bind(&StereoVision::imageCallback, this, _1, _2, _3, _4));
  image_transport::TransportHints hints{this};
  const auto sensor_data_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

  left_image_sub_.subscribe(
    this, "left_camera/image_raw", hints.getTransport(), sensor_data_qos);
  left_info_sub_.subscribe(this, "left_camera/camera_info", sensor_data_qos);
  right_image_sub_.subscribe(
    this, "right_camera/image_raw", hints.getTransport(), sensor_data_qos);
  right_info_sub_.subscribe(this, "right_camera/camera_info", sensor_data_qos);

  disparity_image_pub_ =
    image_transport::create_publisher(this, "disparity", rmw_qos_profile_sensor_data);
  point_cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS());
}

void StereoVision::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & left_info_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & right_info_msg)
{
  cv::Mat left = cv_bridge::toCvCopy(left_image_msg, left_image_msg->encoding)->image;
  cv::Mat right = cv_bridge::toCvCopy(right_image_msg, right_image_msg->encoding)->image;
  cv::Ptr<cv::StereoSGBM> sgbm =
    cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
  cv::Mat disparity_sgbm, disparity;
  sgbm->compute(left, right, disparity_sgbm);
  disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0);
  cv::Mat normalized_disparity;
  double min_disp = 0.0;
  double max_disp = 96.0 * 16.0;
  cv::normalize(disparity_sgbm, normalized_disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

  auto disparity_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", normalized_disparity).toImageMsg();
  disparity_msg->header = left_image_msg->header;
  disparity_image_pub_.publish(disparity_msg);

  // Generate PointCloud
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pointcloud_msg.header = left_image_msg->header;
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
      double x = (u - left_info_msg->k[2]) / left_info_msg->k[0];
      double y = (v - left_info_msg->k[5]) / left_info_msg->k[4];
      double depth = left_info_msg->k[0] * 0.06 / (disparity.at<float>(v, u));
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
}  // namespace bumperbot_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_vision::StereoVision>("stereo_vision"));
  rclcpp::shutdown();
  return 0;
}