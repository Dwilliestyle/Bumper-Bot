#!/usr/bin/env python3
"""
Image processing labs, one node.

Instead of commenting/uncommenting sections, each lab is its own method.
Pick one with the 'lab' parameter, and change it live:

    ros2 run bumperbot_vision image_processing --ros-args -p lab:=contours
    ros2 param set /image_processing lab blur_gaussian

Available labs: original, gray, hsv, hsv_channel, hue_colorized,
                blur_gaussian, blur_median, drawing, mask, contours
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from image_transport_py import ImageTransport
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import numpy as np


class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_processing')

        # ---------------- Parameters ----------------
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('lab', 'mask')
        self.declare_parameter('hsv_channel', 'v')      # for hsv_channel lab: h, s, or v
        self.declare_parameter('kernel_size', 7)        # for blur labs (must be odd)
        self.declare_parameter('lower_hsv', [79, 50, 50])
        self.declare_parameter('upper_hsv', [99, 255, 255])

        self.camera_name_ = self.get_parameter('camera_name').value

        # Map lab name -> method. Adding a new lab = write a method, add a line here.
        self.labs = {
            'original':      self.lab_original,
            'gray':          self.lab_gray,
            'hsv':           self.lab_hsv,
            'hsv_channel':   self.lab_hsv_channel,
            'hue_colorized': self.lab_hue_colorized,
            'blur_gaussian': self.lab_blur_gaussian,
            'blur_median':   self.lab_blur_median,
            'drawing':       self.lab_drawing,
            'mask':          self.lab_mask,
            'contours':      self.lab_contours,
        }

        # ---------------- Helpers ----------------
        self.bridge = CvBridge()
        self.camera_info = None

        # ---------------- Subscribers ----------------
        self.image_sub = self.create_subscription(
            Image, f'{self.camera_name_}/image_rect',
            self.image_callback, qos_profile_sensor_data)

        self.info_sub = self.create_subscription(
            CameraInfo, f'{self.camera_name_}/camera_info',
            self.image_info_callback, qos_profile_sensor_data)

        # ---------------- Publisher ----------------
        self.image_transport = ImageTransport(
            'imagetransport_pub', image_transport='compressed')
        self.img_pub = self.image_transport.advertise(
            f'{self.camera_name_}/processed_image', qos_profile_sensor_data)

        self.get_logger().info(
            f'Image Processing node started for {self.camera_name_} '
            f'(lab: {self.get_parameter("lab").value})')

    # =================================================================
    # Plumbing - this part never changes between labs
    # =================================================================
    def image_info_callback(self, info_msg):
        self.camera_info = info_msg

    def image_callback(self, image_msg):
        if self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Read 'lab' every frame so `ros2 param set` takes effect immediately
        lab_name = self.get_parameter('lab').value
        lab = self.labs.get(lab_name)
        if lab is None:
            self.get_logger().warn(
                f'Unknown lab "{lab_name}". Options: {list(self.labs)}',
                throttle_duration_sec=5.0)
            return

        processed_image = lab(cv_image)

        # Single-channel results (gray, masks) are mono8; color results are bgr8
        encoding = 'mono8' if processed_image.ndim == 2 else 'bgr8'
        out_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding=encoding)
        out_msg.header = image_msg.header
        self.img_pub.publish(out_msg)

    # =================================================================
    # LAB - Color Spaces
    # =================================================================
    def lab_original(self, img):
        return img

    def lab_gray(self, img):
        # Single channel, brightness only
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def lab_hsv(self, img):
        # Raw HSV shown as if it were BGR - looks "wrong" on purpose,
        # it shows that HSV is just three numbers per pixel with different meaning
        return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    def lab_hsv_channel(self, img):
        h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        choice = self.get_parameter('hsv_channel').value
        return {'h': h, 's': s, 'v': v}.get(choice, v)

    def lab_hue_colorized(self, img):
        # Keep only the hue; force saturation and value to max
        h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
        full_s = np.full_like(s, 255)
        full_v = np.full_like(v, 255)
        return cv2.cvtColor(cv2.merge([h, full_s, full_v]), cv2.COLOR_HSV2BGR)

    # =================================================================
    # LAB - Blurring
    # =================================================================
    def _kernel_size(self):
        k = int(self.get_parameter('kernel_size').value)
        return k if k % 2 == 1 else k + 1   # blur kernels must be odd

    def lab_blur_gaussian(self, img):
        # Weighted average of neighbours - smooth, general-purpose
        k = self._kernel_size()
        return cv2.GaussianBlur(img, (k, k), 0)

    def lab_blur_median(self, img):
        # Median of neighbours - kills salt-and-pepper noise, keeps edges
        return cv2.medianBlur(img, self._kernel_size())

    # =================================================================
    # LAB - Drawing on Images
    # =================================================================
    def lab_drawing(self, img):
        out = img.copy()   # draw on a copy, not the original frame
        height, width = out.shape[:2]

        cv2.line(out, (0, 0), (width // 2, height // 2), (255, 0, 0), 2)

        top_left = (width // 4, height // 4)
        bottom_right = (3 * width // 4, 3 * height // 4)
        cv2.rectangle(out, top_left, bottom_right, (0, 255, 0), 2)

        cv2.circle(out, (width // 2, height // 2), 5, (0, 0, 255), -1)

        cv2.putText(out, 'Detection', (top_left[0], top_left[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return out

    # =================================================================
    # LAB - Color Detection with Masks
    # =================================================================
    def make_mask(self, img):
        """Shared by the mask and contours labs - this is the dependency
        that was hidden when the code was commented in and out."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('lower_hsv').value)
        upper = np.array(self.get_parameter('upper_hsv').value)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        # Opening: removes small specks of background noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        # Closing: fills holes and gaps inside the object
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=5)
        return mask

    def lab_mask(self, img):
        return self.make_mask(img)

    # =================================================================
    # LAB - Contour Detection
    # =================================================================
    def lab_contours(self, img):
        mask = self.make_mask(img)   # explicit: contours need a mask first
        out = img.copy()

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest)

            if radius > 10.0:   # ignore tiny blobs
                xr, yr, w, h = cv2.boundingRect(largest)
                cv2.rectangle(out, (xr, yr), (xr + w, yr + h), (0, 255, 0), 2)
                cv2.circle(out, (int(x), int(y)), 3, (0, 0, 255), -1)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
