import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("rover_localization"),
        "config",
        "ekf.yaml",
    )

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_file],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_static_transform",
            arguments=[
                "--x", "0.003",
                "--y", "0.0",
                "--z", "0.1053",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "imu_link",
            ],
        ),
    ])
