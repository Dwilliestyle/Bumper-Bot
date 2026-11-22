import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_visual_slam = LaunchConfiguration("use_visual_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
        description="Map with the lidar (slam_toolbox)"
    )

    use_visual_slam_arg = DeclareLaunchArgument(
        "use_visual_slam",
        default_value="false",
        description="Map with the stereo camera (RTAB-Map)"
    )

    # True only when NEITHER kind of SLAM is running -> use the saved map + AMCL.
    # Only one node may publish map -> odom, so localization and both SLAMs
    # must never run together.
    no_slam = PythonExpression([
        "not ('", use_slam, "'.lower() == 'true' or '",
        use_visual_slam, "'.lower() == 'true')"
    ])

    # Lidar SLAM wins if both are set by mistake, so they never run together
    lidar_slam_only = PythonExpression([
        "'", use_slam, "'.lower() == 'true'"
    ])
    visual_slam_only = PythonExpression([
        "'", use_visual_slam, "'.lower() == 'true' and '",
        use_slam, "'.lower() != 'true'"
    ])

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    stereo_camera = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_vision"),
            "launch",
            "stereo_camera.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": "False"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(no_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(lidar_slam_only)
    )

    # Stereo RTAB-Map. Same launch file as the simulation, but with real time.
    visual_slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "visual_slam.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=IfCondition(visual_slam_only)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
    )

    return LaunchDescription([
        use_slam_arg,
        use_visual_slam_arg,
        hardware_interface,
        stereo_camera,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        localization,
        slam,
        visual_slam,
        navigation
    ])
