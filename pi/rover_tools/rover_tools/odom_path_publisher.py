#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')

        self.wheel_path = Path()
        self.ekf_path = Path()

        self.wheel_pub = self.create_publisher(
            Path, '/wheel/path', 10)

        self.ekf_pub = self.create_publisher(
            Path, '/ekf/path', 10)

        self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self.wheel_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.ekf_callback,
            10
        )

        self.get_logger().info(
            'Publishing /wheel/path and /ekf/path'
        )

    def wheel_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.wheel_path.header = msg.header
        self.wheel_path.poses.append(pose)

        self.wheel_pub.publish(self.wheel_path)

    def ekf_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.ekf_path.header = msg.header
        self.ekf_path.poses.append(pose)

        self.ekf_pub.publish(self.ekf_path)


def main(args=None):
    rclpy.init(args=args)

    node = OdomPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
