import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory("bumperbot_vision"), "config")

    video_device_arg = DeclareLaunchArgument("video_device", default_value="/dev/video0")
    fps_arg = DeclareLaunchArgument("fps", default_value="30")
    left_info_arg = DeclareLaunchArgument(
        "left_info_file", default_value=os.path.join(config_dir, "left.yaml"),
        description="Left lens calibration YAML ('' = uncalibrated)")
    right_info_arg = DeclareLaunchArgument(
        "right_info_file", default_value=os.path.join(config_dir, "right.yaml"),
        description="Right lens calibration YAML ('' = uncalibrated)")

    stereo_camera = Node(
        package="bumperbot_vision",
        executable="stereo_camera_node.py",
        name="stereo_camera_node",
        output="screen",
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "width": 1280,
            "height": 480,
            "fps": LaunchConfiguration("fps"),
            "use_mjpg": True,
            "left_frame_id": "left_camera_optical",
            "right_frame_id": "right_camera_optical",
            "left_info_file": LaunchConfiguration("left_info_file"),
            "right_info_file": LaunchConfiguration("right_info_file"),
        }],
        # Match the topic names used by the course nodes
        # (stereo_vision.py and stereo_vision.launch.py)
        remappings=[
            ("stereo/left/image_raw", "/left_camera/image_raw"),
            ("stereo/left/camera_info", "/left_camera/camera_info"),
            ("stereo/right/image_raw", "/right_camera/image_raw"),
            ("stereo/right/camera_info", "/right_camera/camera_info"),
        ],
    )

    return LaunchDescription([
        video_device_arg,
        fps_arg,
        left_info_arg,
        right_info_arg,
        stereo_camera,
    ])