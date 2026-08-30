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

    # Rover centroid/base_link is at the geometric center of frame layer 1,
    # at floor level. Encoder odometry is assumed to describe this point.
    #
    # IMU measurement origin relative to base_link:
    # x = +3.01 mm, y = 0 mm, z = +105.3 mm
    imu_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_link",
        arguments=[
            "--x", "0.00301",
            "--y", "0.0",
            "--z", "0.1053",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "imu_link",
        ],
        output="screen",
    )

    # LiDAR rotor centroid relative to base_link:
    # x = +7.00 mm, y = 0 mm, z = +181.25 mm
    lidar_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        arguments=[
            "--x", "0.007",
            "--y", "0.0",
            "--z", "0.18125",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        imu_static_transform,
        lidar_static_transform,
        ekf_node,
    ])
