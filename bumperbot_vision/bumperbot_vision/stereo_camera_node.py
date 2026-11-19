#!/usr/bin/env python3
"""
stereo_camera_node.py - side-by-side USB stereo camera driver for ROS2 Jazzy.

Grabs the wide side-by-side frame from a UVC stereo camera (e.g. 32e4:0035
"3D USB Camera"), splits it down the middle, and publishes:

    /stereo/left/image_raw      sensor_msgs/Image
    /stereo/left/camera_info    sensor_msgs/CameraInfo
    /stereo/right/image_raw     sensor_msgs/Image
    /stereo/right/camera_info   sensor_msgs/CameraInfo

Both halves come from the same frame, so they get the same timestamp
(that's what stereo_image_proc needs to pair them).

Parameters:
    video_device      (str)   default /dev/video0
    width, height     (int)   full side-by-side size, default 1280 x 480
    fps               (int)   default 30
    use_mjpg          (bool)  default True (needed for high res / fps on USB 2.0)
    left_frame_id     (str)   default stereo_left_optical
    right_frame_id    (str)   default stereo_right_optical
    left_info_file    (str)   calibration YAML for the left lens  ('' = uncalibrated)
    right_info_file   (str)   calibration YAML for the right lens ('' = uncalibrated)
    swap_lr           (bool)  default False - set True if left/right come out reversed
    best_effort       (bool)  default False - publish with sensor-data (best-effort) QoS
"""

import cv2
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def load_camera_info(path, width, height, frame_id):
    """Load a camera_calibration / camera_info_manager YAML. Blank info if no file."""
    info = CameraInfo()
    info.width = width
    info.height = height
    info.header.frame_id = frame_id
    if not path:
        return info
    with open(path, "r") as f:
        calib = yaml.safe_load(f)
    info.width = calib["image_width"]
    info.height = calib["image_height"]
    info.distortion_model = calib.get("distortion_model", "plumb_bob")
    info.d = [float(x) for x in calib["distortion_coefficients"]["data"]]
    info.k = [float(x) for x in calib["camera_matrix"]["data"]]
    info.r = [float(x) for x in calib["rectification_matrix"]["data"]]
    info.p = [float(x) for x in calib["projection_matrix"]["data"]]
    return info


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_camera_node")

        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("use_mjpg", True)
        self.declare_parameter("left_frame_id", "stereo_left_optical")
        self.declare_parameter("right_frame_id", "stereo_right_optical")
        self.declare_parameter("left_info_file", "")
        self.declare_parameter("right_info_file", "")
        self.declare_parameter("swap_lr", False)

        device = self.get_parameter("video_device").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fps = self.get_parameter("fps").value
        use_mjpg = self.get_parameter("use_mjpg").value
        self.left_frame = self.get_parameter("left_frame_id").value
        self.right_frame = self.get_parameter("right_frame_id").value
        self.swap_lr = self.get_parameter("swap_lr").value

        # --- open the camera ---
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open {device}")
        if use_mjpg:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f"Opened {device}: {actual_w}x{actual_h} @ {actual_fps:.0f} fps "
            f"({'MJPG' if use_mjpg else 'YUYV'}) -> two {actual_w // 2}x{actual_h} images"
        )
        self.half_w = actual_w // 2

        # --- camera info ---
        self.left_info = load_camera_info(
            self.get_parameter("left_info_file").value, self.half_w, actual_h, self.left_frame)
        self.right_info = load_camera_info(
            self.get_parameter("right_info_file").value, self.half_w, actual_h, self.right_frame)

        # --- publishers ---
        # Reliable QoS: a reliable publisher works with BOTH reliable and
        # best-effort subscribers (RViz2, rqt_image_view, camera_calibration,
        # stereo_image_proc). Set best_effort:=true only if you need it.
        if self.declare_parameter("best_effort", False).value:
            qos = qos_profile_sensor_data
        else:
            qos = 10
        self.left_img_pub = self.create_publisher(Image, "stereo/left/image_raw", qos)
        self.right_img_pub = self.create_publisher(Image, "stereo/right/image_raw", qos)
        self.left_info_pub = self.create_publisher(CameraInfo, "stereo/left/camera_info", qos)
        self.right_info_pub = self.create_publisher(CameraInfo, "stereo/right/camera_info", qos)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.grab_and_publish)

    def grab_and_publish(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Frame grab failed", throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()
        left = frame[:, : self.half_w]
        right = frame[:, self.half_w :]
        if self.swap_lr:
            left, right = right, left

        left_msg = self.bridge.cv2_to_imgmsg(left, encoding="bgr8")
        right_msg = self.bridge.cv2_to_imgmsg(right, encoding="bgr8")
        left_msg.header.stamp = right_msg.header.stamp = stamp
        left_msg.header.frame_id = self.left_frame
        right_msg.header.frame_id = self.right_frame

        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp

        self.left_img_pub.publish(left_msg)
        self.right_img_pub.publish(right_msg)
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()