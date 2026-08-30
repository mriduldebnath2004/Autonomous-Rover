import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_directory = os.path.join(
        get_package_share_directory("rover_slam"),
        "config",
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory",
            config_directory,
            "-configuration_basename",
            "rover_2d.lua",
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odometry/filtered"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        arguments=[
            "-resolution", "0.05",
            "-publish_period_sec", "1.0",
        ],
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
    ])
