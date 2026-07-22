#!/usr/bin/env python3

"""
Rover motion test node.

Runs one of four tests:
    straight
    rotation
    circle
    square

The node:
- publishes velocity commands to /cmd_vel
- subscribes to /wheel/odometry
- prints the starting and final odometry pose
- always sends a stop command when the test finishes or Ctrl+C is pressed

The tests are time-based. Ground-truth measurements should be taken manually.

Examples:
    python3 rover_motion_test.py straight --distance 2.0 --speed 0.25
    python3 rover_motion_test.py rotation --angle-deg 360 --yaw-rate 0.6
    python3 rover_motion_test.py circle --radius 0.5 --speed 0.30
    python3 rover_motion_test.py square --side-length 1.0 --speed 0.25 --yaw-rate 0.6
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class RoverMotionTest(Node):
    COMMAND_RATE_HZ = 10.0
    STOP_PUBLISH_COUNT = 10

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("rover_motion_test")

        self.args = args
        self.latest_pose: Optional[Pose2D] = None
        self.start_pose: Optional[Pose2D] = None
        self.test_finished = False

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            args.cmd_topic,
            10,
        )

        self.odom_subscriber = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(
            f"Publishing commands to {args.cmd_topic}"
        )
        self.get_logger().info(
            f"Reading odometry from {args.odom_topic}"
        )

    def odom_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation

        # Yaw from quaternion.
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        yaw = math.atan2(sin_yaw, cos_yaw)

        self.latest_pose = Pose2D(
            x=message.pose.pose.position.x,
            y=message.pose.pose.position.y,
            yaw=yaw,
        )

    def publish_command(
        self,
        linear_velocity: float,
        yaw_rate: float,
    ) -> None:
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = yaw_rate
        self.cmd_vel_publisher.publish(command)

    def stop_rover(self) -> None:
        for _ in range(self.STOP_PUBLISH_COUNT):
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_odometry(self, timeout_seconds: float = 5.0) -> bool:
        self.get_logger().info("Waiting for odometry...")

        end_time = time.monotonic() + timeout_seconds

        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.latest_pose is not None:
                self.start_pose = Pose2D(
                    self.latest_pose.x,
                    self.latest_pose.y,
                    self.latest_pose.yaw,
                )

                self.print_pose("Starting odometry", self.start_pose)
                return True

        self.get_logger().error(
            f"No odometry received from {self.args.odom_topic}"
        )
        return False

    def run_command_for_duration(
        self,
        linear_velocity: float,
        yaw_rate: float,
        duration_seconds: float,
        description: str,
    ) -> None:
        if duration_seconds <= 0.0:
            raise ValueError("Command duration must be greater than zero.")

        self.get_logger().info(description)
        self.get_logger().info(
            f"Command: v={linear_velocity:.3f} m/s, "
            f"w={yaw_rate:.3f} rad/s, "
            f"duration={duration_seconds:.3f} s"
        )

        period = 1.0 / self.COMMAND_RATE_HZ
        end_time = time.monotonic() + duration_seconds

        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_command(linear_velocity, yaw_rate)
            rclpy.spin_once(self, timeout_sec=period)

        self.stop_rover()

    def pause_between_segments(self) -> None:
        pause_end = time.monotonic() + self.args.segment_pause

        while rclpy.ok() and time.monotonic() < pause_end:
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_straight_test(self) -> None:
        speed = self.args.speed
        distance = self.args.distance

        if speed == 0.0:
            raise ValueError("Straight-line speed cannot be zero.")

        duration = abs(distance / speed)
        command_speed = math.copysign(abs(speed), distance)

        self.run_command_for_duration(
            linear_velocity=command_speed,
            yaw_rate=0.0,
            duration_seconds=duration,
            description=(
                f"Straight-line test: commanded distance "
                f"{distance:.3f} m"
            ),
        )

    def run_rotation_test(self) -> None:
        angle_radians = math.radians(self.args.angle_deg)
        yaw_rate = self.args.yaw_rate

        if yaw_rate == 0.0:
            raise ValueError("Yaw rate cannot be zero.")

        duration = abs(angle_radians / yaw_rate)
        command_yaw_rate = math.copysign(
            abs(yaw_rate),
            angle_radians,
        )

        self.run_command_for_duration(
            linear_velocity=0.0,
            yaw_rate=command_yaw_rate,
            duration_seconds=duration,
            description=(
                f"In-place rotation test: commanded angle "
                f"{self.args.angle_deg:.1f} degrees"
            ),
        )

    def run_circle_test(self) -> None:
        radius = self.args.radius
        speed = self.args.speed
        direction = self.args.direction

        if radius <= 0.0:
            raise ValueError("Circle radius must be greater than zero.")

        if speed <= 0.0:
            raise ValueError("Circle speed must be greater than zero.")

        direction_sign = 1.0 if direction == "ccw" else -1.0
        yaw_rate = direction_sign * speed / radius
        duration = 2.0 * math.pi / abs(yaw_rate)

        self.run_command_for_duration(
            linear_velocity=speed,
            yaw_rate=yaw_rate,
            duration_seconds=duration,
            description=(
                f"Circle test: radius={radius:.3f} m, "
                f"direction={direction.upper()}"
            ),
        )

    def run_square_test(self) -> None:
        side_length = self.args.side_length
        speed = self.args.speed
        yaw_rate = self.args.yaw_rate
        direction = self.args.direction

        if side_length <= 0.0:
            raise ValueError(
                "Square side length must be greater than zero."
            )

        if speed <= 0.0:
            raise ValueError("Square speed must be greater than zero.")

        if yaw_rate <= 0.0:
            raise ValueError(
                "Square yaw rate magnitude must be greater than zero."
            )

        straight_duration = side_length / speed
        turn_duration = (math.pi / 2.0) / yaw_rate
        direction_sign = 1.0 if direction == "ccw" else -1.0

        self.get_logger().info(
            f"Square test: side={side_length:.3f} m, "
            f"direction={direction.upper()}"
        )

        for side_number in range(1, 5):
            self.run_command_for_duration(
                linear_velocity=speed,
                yaw_rate=0.0,
                duration_seconds=straight_duration,
                description=f"Square side {side_number}/4",
            )

            self.pause_between_segments()

            self.run_command_for_duration(
                linear_velocity=0.0,
                yaw_rate=direction_sign * yaw_rate,
                duration_seconds=turn_duration,
                description=f"Square turn {side_number}/4",
            )

            self.pause_between_segments()

    def run_selected_test(self) -> None:
        if not self.wait_for_odometry():
            return

        try:
            if self.args.test == "straight":
                self.run_straight_test()
            elif self.args.test == "rotation":
                self.run_rotation_test()
            elif self.args.test == "circle":
                self.run_circle_test()
            elif self.args.test == "square":
                self.run_square_test()
            else:
                raise ValueError(f"Unknown test: {self.args.test}")

        finally:
            self.stop_rover()

        # Process remaining odometry messages after stopping.
        final_wait_end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < final_wait_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.latest_pose is not None:
            self.print_pose("Final odometry", self.latest_pose)
            self.print_relative_odom()

        self.test_finished = True

    def print_pose(self, label: str, pose: Pose2D) -> None:
        self.get_logger().info(
            f"{label}: "
            f"x={pose.x:.4f} m, "
            f"y={pose.y:.4f} m, "
            f"yaw={math.degrees(pose.yaw):.2f} deg"
        )

    def print_relative_odom(self) -> None:
        if self.start_pose is None or self.latest_pose is None:
            return

        dx_world = self.latest_pose.x - self.start_pose.x
        dy_world = self.latest_pose.y - self.start_pose.y

        cos_start = math.cos(self.start_pose.yaw)
        sin_start = math.sin(self.start_pose.yaw)

        # Express displacement in the rover's starting frame.
        relative_x = (
            cos_start * dx_world
            + sin_start * dy_world
        )
        relative_y = (
            -sin_start * dx_world
            + cos_start * dy_world
        )

        relative_yaw = normalize_angle(
            self.latest_pose.yaw - self.start_pose.yaw
        )

        self.get_logger().info(
            "Relative odometry: "
            f"x={relative_x:.4f} m, "
            f"y={relative_y:.4f} m, "
            f"yaw={math.degrees(relative_yaw):.2f} deg"
        )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run time-based rover motion tests."
    )

    parser.add_argument(
        "test",
        choices=["straight", "rotation", "circle", "square"],
        help="Test to run.",
    )

    parser.add_argument(
        "--cmd-topic",
        default="/cmd_vel",
        help="Velocity command topic. Default: /cmd_vel",
    )

    parser.add_argument(
        "--odom-topic",
        default="/wheel/odometry",
        help="Odometry topic. Default: /wheel/odometry",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=0.25,
        help="Linear speed in m/s. Default: 0.25",
    )

    parser.add_argument(
        "--distance",
        type=float,
        default=2.0,
        help=(
            "Straight-test distance in metres. "
            "Use a negative value to reverse. Default: 2.0"
        ),
    )

    parser.add_argument(
        "--angle-deg",
        type=float,
        default=360.0,
        help=(
            "Rotation-test angle in degrees. "
            "Positive is CCW, negative is CW. Default: 360"
        ),
    )

    parser.add_argument(
        "--yaw-rate",
        type=float,
        default=0.6,
        help=(
            "Yaw-rate magnitude in rad/s for rotation and square tests. "
            "Default: 0.6"
        ),
    )

    parser.add_argument(
        "--radius",
        type=float,
        default=0.5,
        help="Circle radius in metres. Default: 0.5",
    )

    parser.add_argument(
        "--side-length",
        type=float,
        default=1.0,
        help="Square side length in metres. Default: 1.0",
    )

    parser.add_argument(
        "--direction",
        choices=["ccw", "cw"],
        default="ccw",
        help="Circle or square direction. Default: ccw",
    )

    parser.add_argument(
        "--segment-pause",
        type=float,
        default=0.5,
        help=(
            "Pause between square segments in seconds. "
            "Default: 0.5"
        ),
    )

    return parser


def main() -> None:
    parser = create_argument_parser()
    args = parser.parse_args()

    rclpy.init()

    node = RoverMotionTest(args)

    try:
        node.run_selected_test()
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Test interrupted. Stopping rover."
        )
        node.stop_rover()
    except ValueError as error:
        node.get_logger().error(str(error))
        node.stop_rover()
        sys.exit(1)
    finally:
        node.stop_rover()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
