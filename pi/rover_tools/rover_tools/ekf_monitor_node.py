#!/usr/bin/env python3

import csv
import math
import os
import statistics
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_to_yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )

    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )

    return math.atan2(sin_yaw, cos_yaw)


def safe_sigma(variance):
    if variance < 0.0:
        return math.nan

    return math.sqrt(variance)


def covariance_fieldnames(prefix):
    return [
        f"{prefix}_{row}{column}"
        for row in range(6)
        for column in range(6)
    ]


def covariance_values(values, prefix):
    return [
        values[f"{prefix}_{row}{column}"]
        for row in range(6)
        for column in range(6)
    ]


def format_covariance(covariance):
    rows = []

    for row in range(6):
        start = row * 6
        rows.append(
            "[" + "  ".join(
                f"{value:+.3e}"
                for value in covariance[start:start + 6]
            ) + "]"
        )

    return "\n".join(rows)


class TopicSample:

    def __init__(self, history_length=100):
        self.message = None
        self.received_at = None
        self.arrival_times = deque(maxlen=history_length)

    def update(self, message):
        now = time.monotonic()

        self.message = message
        self.received_at = now
        self.arrival_times.append(now)

    def age(self):
        if self.received_at is None:
            return math.inf

        return time.monotonic() - self.received_at

    def frequency(self):
        if len(self.arrival_times) < 2:
            return 0.0

        elapsed = (
            self.arrival_times[-1]
            - self.arrival_times[0]
        )

        if elapsed <= 0.0:
            return 0.0

        return (
            len(self.arrival_times) - 1
        ) / elapsed


class EkfMonitor(Node):

    def __init__(self):
        super().__init__("ekf_monitor")

        self.declare_parameter("display_rate", 5.0)
        self.declare_parameter("stale_timeout", 0.5)
        self.declare_parameter("stationary_speed_threshold", 0.02)
        self.declare_parameter("stationary_window", 500)
        self.declare_parameter("log_csv", False)
        self.declare_parameter(
            "csv_path",
            "/tmp/ekf_monitor.csv",
        )

        display_rate = float(
            self.get_parameter("display_rate").value
        )

        self.stale_timeout = float(
            self.get_parameter("stale_timeout").value
        )

        self.stationary_speed_threshold = float(
            self.get_parameter(
                "stationary_speed_threshold"
            ).value
        )

        stationary_window = int(
            self.get_parameter("stationary_window").value
        )

        self.log_csv = bool(
            self.get_parameter("log_csv").value
        )

        self.csv_path = str(
            self.get_parameter("csv_path").value
        )

        if display_rate <= 0.0:
            raise ValueError(
                "display_rate must be greater than zero"
            )

        self.command = TopicSample()
        self.wheel = TopicSample()
        self.imu = TopicSample()
        self.filtered = TopicSample()
        self.diagnostics = TopicSample()

        self.stationary_imu_wz = deque(
            maxlen=stationary_window
        )

        # Store subscription references so they are not destroyed.
        self.subscriptions_list = []

        self.subscriptions_list.append(
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self.command.update,
                10,
            )
        )

        self.subscriptions_list.append(
            self.create_subscription(
                Odometry,
                "/wheel/odometry",
                self.wheel_callback,
                10,
            )
        )

        self.subscriptions_list.append(
            self.create_subscription(
                Imu,
                "/imu/data_raw",
                self.imu_callback,
                10,
            )
        )

        self.subscriptions_list.append(
            self.create_subscription(
                Odometry,
                "/odometry/filtered",
                self.filtered.update,
                10,
            )
        )

        self.subscriptions_list.append(
            self.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                self.diagnostics.update,
                10,
            )
        )

        self.csv_file = None
        self.csv_writer = None

        if self.log_csv:
            self.open_csv()

        self.timer = self.create_timer(
            1.0 / display_rate,
            self.display,
        )

        self.get_logger().info(
            "EKF monitor started"
        )

    def wheel_callback(self, message):
        self.wheel.update(message)

    def imu_callback(self, message):
        self.imu.update(message)

        if self.is_stationary():
            self.stationary_imu_wz.append(
                message.angular_velocity.z
            )

    def is_stationary(self):
        if (
            self.command.message is None
            or self.wheel.message is None
        ):
            return False

        command_vx = self.command.message.linear.x
        command_wz = self.command.message.angular.z

        wheel_vx = (
            self.wheel.message.twist.twist.linear.x
        )

        return (
            abs(command_vx) < 0.001
            and abs(command_wz) < 0.001
            and abs(wheel_vx)
            < self.stationary_speed_threshold
        )

    def status(self, sample):
        if sample.message is None:
            return "WAIT"

        if sample.age() > self.stale_timeout:
            return "STALE"

        return "OK"

    def diagnostic_summary(self):
        if self.diagnostics.message is None:
            return "No diagnostics received"

        relevant_statuses = []

        for status in self.diagnostics.message.status:
            name = status.name.lower()

            if (
                "ekf" in name
                or "localization" in name
            ):
                relevant_statuses.append(status)

        if not relevant_statuses:
            return "No EKF diagnostic entry"

        worst = max(
            relevant_statuses,
            key=lambda status: status.level,
        )

        labels = {
            DiagnosticStatus.OK: "OK",
            DiagnosticStatus.WARN: "WARN",
            DiagnosticStatus.ERROR: "ERROR",
            DiagnosticStatus.STALE: "STALE",
        }

        label = labels.get(
            worst.level,
            str(worst.level),
        )

        return f"{label}: {worst.message}"

    def read_values(self):
        values = {
            "command_vx": math.nan,
            "command_wz": math.nan,
            "wheel_vx": math.nan,
            "wheel_wz": math.nan,
            "imu_wz": math.nan,
            "ekf_x": math.nan,
            "ekf_y": math.nan,
            "ekf_yaw": math.nan,
            "ekf_vx": math.nan,
            "ekf_wz": math.nan,
            "wheel_vx_variance": math.nan,
            "imu_wz_variance": math.nan,
            "ekf_x_variance": math.nan,
            "ekf_y_variance": math.nan,
            "ekf_yaw_variance": math.nan,
            "ekf_vx_variance": math.nan,
            "ekf_wz_variance": math.nan,
        }

        # Preserve every element, including off-diagonal correlations.
        for fieldname in covariance_fieldnames("ekf_pose_cov"):
            values[fieldname] = math.nan

        for fieldname in covariance_fieldnames("ekf_twist_cov"):
            values[fieldname] = math.nan

        if self.command.message is not None:
            values["command_vx"] = (
                self.command.message.linear.x
            )
            values["command_wz"] = (
                self.command.message.angular.z
            )

        if self.wheel.message is not None:
            twist = self.wheel.message.twist

            values["wheel_vx"] = (
                twist.twist.linear.x
            )
            values["wheel_wz"] = (
                twist.twist.angular.z
            )
            values["wheel_vx_variance"] = (
                twist.covariance[0]
            )

        if self.imu.message is not None:
            values["imu_wz"] = (
                self.imu.message.angular_velocity.z
            )
            values["imu_wz_variance"] = (
                self.imu.message
                .angular_velocity_covariance[8]
            )

        if self.filtered.message is not None:
            pose = self.filtered.message.pose
            twist = self.filtered.message.twist

            values["ekf_x"] = (
                pose.pose.position.x
            )
            values["ekf_y"] = (
                pose.pose.position.y
            )
            values["ekf_yaw"] = quaternion_to_yaw(
                pose.pose.orientation
            )

            values["ekf_vx"] = (
                twist.twist.linear.x
            )
            values["ekf_wz"] = (
                twist.twist.angular.z
            )

            values["ekf_x_variance"] = (
                pose.covariance[0]
            )
            values["ekf_y_variance"] = (
                pose.covariance[7]
            )
            values["ekf_yaw_variance"] = (
                pose.covariance[35]
            )
            values["ekf_vx_variance"] = (
                twist.covariance[0]
            )
            values["ekf_wz_variance"] = (
                twist.covariance[35]
            )

            for index, covariance in enumerate(
                pose.covariance
            ):
                row, column = divmod(index, 6)
                values[
                    f"ekf_pose_cov_{row}{column}"
                ] = covariance

            for index, covariance in enumerate(
                twist.covariance
            ):
                row, column = divmod(index, 6)
                values[
                    f"ekf_twist_cov_{row}{column}"
                ] = covariance

        return values

    def open_csv(self):
        directory = os.path.dirname(self.csv_path)

        if directory:
            os.makedirs(
                directory,
                exist_ok=True,
            )

        self.csv_file = open(
            self.csv_path,
            "w",
            newline="",
        )

        fieldnames = [
            "time",
            "command_vx",
            "command_wz",
            "wheel_vx",
            "wheel_wz",
            "imu_wz",
            "ekf_x",
            "ekf_y",
            "ekf_yaw",
            "ekf_vx",
            "ekf_wz",
            "wheel_vx_variance",
            "imu_wz_variance",
            "ekf_x_variance",
            "ekf_y_variance",
            "ekf_yaw_variance",
            "ekf_vx_variance",
            "ekf_wz_variance",
        ]

        fieldnames.extend(
            covariance_fieldnames("ekf_pose_cov")
        )

        fieldnames.extend(
            covariance_fieldnames("ekf_twist_cov")
        )

        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=fieldnames,
        )

        self.csv_writer.writeheader()

        self.get_logger().info(
            f"Logging data to {self.csv_path}"
        )

    def write_csv(self, values):
        if self.csv_writer is None:
            return

        row = {
            "time": self.get_clock().now().nanoseconds
            * 1e-9,
            **values,
        }

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def display(self):
        values = self.read_values()

        stationary_mean = math.nan
        stationary_std = math.nan

        if len(self.stationary_imu_wz) >= 2:
            stationary_mean = statistics.mean(
                self.stationary_imu_wz
            )

            stationary_std = statistics.stdev(
                self.stationary_imu_wz
            )

        wheel_command_difference = (
            values["wheel_vx"]
            - values["command_vx"]
        )

        imu_command_difference = (
            values["imu_wz"]
            - values["command_wz"]
        )

        wheel_imu_difference = (
            values["wheel_wz"]
            - values["imu_wz"]
        )

        yaw_degrees = math.degrees(
            values["ekf_yaw"]
        )

        yaw_sigma_degrees = math.degrees(
            safe_sigma(
                values["ekf_yaw_variance"]
            )
        )

        pose_covariance = covariance_values(
            values,
            "ekf_pose_cov",
        )

        twist_covariance = covariance_values(
            values,
            "ekf_twist_cov",
        )

        pose_covariance_text = format_covariance(
            pose_covariance
        )

        twist_covariance_text = format_covariance(
            twist_covariance
        )

        screen = f"""
ROVER EKF MONITOR
=================

Motion                       Vx (m/s)    Wz (rad/s)
Command                      {values["command_vx"]:+9.4f}    {values["command_wz"]:+10.4f}
Wheel odometry               {values["wheel_vx"]:+9.4f}    {values["wheel_wz"]:+10.4f}
IMU                                ---    {values["imu_wz"]:+10.4f}
EKF                          {values["ekf_vx"]:+9.4f}    {values["ekf_wz"]:+10.4f}

EKF pose
x:   {values["ekf_x"]:+9.4f} m
y:   {values["ekf_y"]:+9.4f} m
yaw: {yaw_degrees:+9.3f} deg

Command tracking
Wheel Vx - commanded Vx:     {wheel_command_difference:+10.4f} m/s
IMU Wz - commanded Wz:       {imu_command_difference:+10.4f} rad/s
Wheel Wz - IMU Wz:           {wheel_imu_difference:+10.4f} rad/s

Measurement uncertainty
Wheel Vx variance R:         {values["wheel_vx_variance"]:.8f}
Wheel Vx sigma:              {safe_sigma(values["wheel_vx_variance"]):.6f} m/s
IMU Wz variance R:           {values["imu_wz_variance"]:.8f}
IMU Wz sigma:                {safe_sigma(values["imu_wz_variance"]):.6f} rad/s

EKF uncertainty
x sigma:                     {safe_sigma(values["ekf_x_variance"]):.6f} m
y sigma:                     {safe_sigma(values["ekf_y_variance"]):.6f} m
yaw sigma:                   {yaw_sigma_degrees:.4f} deg
Vx sigma:                    {safe_sigma(values["ekf_vx_variance"]):.6f} m/s
Wz sigma:                    {safe_sigma(values["ekf_wz_variance"]):.6f} rad/s

EKF pose covariance P_pose [x, y, z, roll, pitch, yaw]
{pose_covariance_text}

EKF twist covariance P_twist [Vx, Vy, Vz, Wx, Wy, Wz]
{twist_covariance_text}

Stationary IMU statistics
samples:                     {len(self.stationary_imu_wz)}
Wz mean/bias:                {stationary_mean:+.7f} rad/s
Wz standard deviation:       {stationary_std:.7f} rad/s
Wz estimated variance:       {stationary_std ** 2:.9f}

Topic health
/cmd_vel:                    {self.command.frequency():6.1f} Hz  {self.status(self.command)}
/wheel/odometry:             {self.wheel.frequency():6.1f} Hz  {self.status(self.wheel)}
/imu/data_raw:               {self.imu.frequency():6.1f} Hz  {self.status(self.imu)}
/odometry/filtered:          {self.filtered.frequency():6.1f} Hz  {self.status(self.filtered)}

Diagnostics: {self.diagnostic_summary()}

Note: command differences are tracking metrics, not ground-truth error.
True innovation, NIS and Kalman gain are not published by robot_localization.
The Odometry message publishes separate pose and twist 6x6 covariance
matrices, not the complete internal 15x15 EKF covariance.
"""

        print(
            "\033[2J\033[H" + screen,
            end="",
            flush=True,
        )

        self.write_csv(values)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = EkfMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
