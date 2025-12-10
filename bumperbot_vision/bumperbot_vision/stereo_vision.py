#!/usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class StereoVision(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.bridge = CvBridge()

        self.last_right_image = None
        self.left_camera_info = None
        self.right_camera_info = None

        self.left_image_sub = self.create_subscription(
            Image,
            "left_camera/image_raw",
            self.left_image_callback,
            qos_profile_sensor_data
        )

        self.right_image_sub = self.create_subscription(
            Image,
            "right_camera/image_raw",
            self.right_image_callback,
            qos_profile_sensor_data
        )

        self.left_camera_info_sub = self.create_subscription(
            CameraInfo,
            "left_camera/camera_info",
            self.left_camera_info_callback,
            10
        )

        self.right_camera_info_sub = self.create_subscription(
            CameraInfo,
            "right_camera/camera_info",
            self.right_camera_info_callback,
            10
        )

        self.disparity_image_pub = self.create_publisher(
            Image,
            "disparity",
            qos_profile_sensor_data
        )
        
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            "point_cloud",
            qos_profile_sensor_data
        )

    def right_image_callback(self, msg):
        try:
            self.last_right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert right image: {e}")

    def left_camera_info_callback(self, msg):
        self.left_camera_info = msg

    def right_camera_info_callback(self, msg):
        self.right_camera_info = msg

    def left_image_callback(self, msg):
        if (self.last_right_image is None or 
            self.left_camera_info is None or 
            self.right_camera_info is None):
            self.get_logger().warn("Waiting for all data to be available before processing.", throttle_duration_sec=2.0)
            return

        try:
            left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert left image: {e}")
            return

        sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=96,
            blockSize=9,
            P1=8 * 9 * 9,
            P2=32 * 9 * 9,
            disp12MaxDiff=1,
            preFilterCap=63,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        disparity_sgbm = sgbm.compute(left_image, self.last_right_image)
        disparity = disparity_sgbm.astype(np.float32) / 16.0
        normalized_disparity = cv2.normalize(
            disparity_sgbm, None, alpha=0, beta=255, 
            norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U
        )
        disparity_msg = self.bridge.cv2_to_imgmsg(normalized_disparity, encoding="mono8")
        disparity_msg.header = msg.header
        self.disparity_image_pub.publish(disparity_msg)
        
        fx = self.left_camera_info.k[0]
        fy = self.left_camera_info.k[4]
        cx = self.left_camera_info.k[2]
        cy = self.left_camera_info.k[5]
        baseline = 0.06
        mask = (disparity > 10.0) & (disparity < 96.0)
        rows, cols = left_image.shape
        u, v = np.meshgrid(np.arange(cols), np.arange(rows))
        u_masked = u[mask]
        v_masked = v[mask]
        d_masked = disparity[mask]
        intensity_masked = left_image[mask]

        if d_masked.size == 0:
            return

        z_points = (fx * baseline) / d_masked
        x_points = (u_masked - cx) * z_points / fx
        fy = self.left_camera_info.k[4]
        y_points = (v_masked - cy) * z_points / fy
        intensity_normalized = intensity_masked.astype(np.float32) / 255.0

        points = np.stack([x_points, y_points, z_points, intensity_normalized], axis=-1)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc_msg = pc2.create_cloud(msg.header, fields, points)
        self.point_cloud_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoVision("stereo_vision")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()