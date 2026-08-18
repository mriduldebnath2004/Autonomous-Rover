#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
SSH Rover Teleop
----------------
W       drive forward
S       drive backward
A       turn left
D       turn right
X       straighten while continuing forward/reverse
Space   emergency stop
Q       stop and exit

Commands remain active until changed.
"""


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("linear_speed", 0.30)
        self.declare_parameter("angular_speed", 0.60)
        self.declare_parameter("publish_rate", 20.0)

        self.linear_speed = float(
            self.get_parameter("linear_speed").value
        )

        self.angular_speed = float(
            self.get_parameter("angular_speed").value
        )

        publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        if publish_rate <= 0.0:
            raise ValueError(
                "publish_rate must be greater than zero"
            )

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.linear_command = 0.0
        self.angular_command = 0.0

        self.running = True
        self.last_display = None

        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.update,
        )

        self.get_logger().info(
            f"Teleop ready: "
            f"linear={self.linear_speed:.2f} m/s, "
            f"angular={self.angular_speed:.2f} rad/s"
        )

    def handle_key(self, key):
        key = key.lower()

        if key == "w":
            self.linear_command = self.linear_speed

        elif key == "s":
            self.linear_command = -self.linear_speed

        elif key == "a":
            self.angular_command = self.angular_speed

        elif key == "d":
            self.angular_command = -self.angular_speed

        elif key == "x":
            # Continue forward/reverse but stop turning.
            self.angular_command = 0.0

        elif key == " ":
            self.stop_motion()

        elif key == "q" or key == "\x03":
            self.stop_motion()
            self.publish_command()
            self.running = False

    def update(self):
        # Process every keyboard character currently waiting.
        while select.select(
            [sys.stdin],
            [],
            [],
            0.0,
        )[0]:
            key = os.read(
                sys.stdin.fileno(),
                1,
            ).decode(errors="ignore")

            self.handle_key(key)

        # Continuously refresh cmd_vel so the Pico remains connected.
        self.publish_command()
        self.display_command()

    def publish_command(self):
        message = Twist()

        message.linear.x = self.linear_command
        message.angular.z = self.angular_command

        self.publisher.publish(message)

    def stop_motion(self):
        self.linear_command = 0.0
        self.angular_command = 0.0

    def display_command(self):
        current_display = (
            self.linear_command,
            self.angular_command,
        )

        if current_display != self.last_display:
            sys.stdout.write(
                f"\rCommand: "
                f"Vx={self.linear_command:+.2f} m/s  "
                f"Wz={self.angular_command:+.2f} rad/s    "
            )

            sys.stdout.flush()
            self.last_display = current_display

    def destroy_node(self):
        self.stop_motion()

        # Send several zero commands before shutting down.
        for _ in range(5):
            self.publish_command()

        super().destroy_node()


def main(args=None):
    if not sys.stdin.isatty():
        print(
            "keyboard_teleop must run in an interactive terminal.",
            file=sys.stderr,
        )
        return

    original_terminal_settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = None

    try:
        tty.setcbreak(sys.stdin.fileno())

        print(HELP)

        node = KeyboardTeleop()

        while rclpy.ok() and node.running:
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            original_terminal_settings,
        )

        print("\nRover stopped.")


if __name__ == "__main__":
    main()
