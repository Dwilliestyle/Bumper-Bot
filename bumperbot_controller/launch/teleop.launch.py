from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='keyboard',
        description='Teleop mode: keyboard or joystick'
    )

    keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'stamped': True, 'frame_id': 'base_link'}],
        remappings=[('/cmd_vel', '/bumperbot_controller/cmd_vel')],
        condition=LaunchConfigurationEquals('mode', 'keyboard')
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bumperbot_controller'),
                'launch',
                'joystick_teleop.launch.py'
            )
        ),
        condition=LaunchConfigurationEquals('mode', 'joystick')
    )

    return LaunchDescription([
        mode_arg,
        keyboard_node,
        joystick_launch,
    ])
