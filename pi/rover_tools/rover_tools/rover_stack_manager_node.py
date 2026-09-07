import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_srvs.srv import Trigger

from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid


class RoverStackManager(Node):

    def __init__(self):
        super().__init__("rover_stack_manager")

        self.callback_group = ReentrantCallbackGroup()

        # Prevent reset/start/shutdown operations from
        # happening simultaneously.
        self.operation_lock = threading.Lock()

        # =====================================================
        # PROCESS HANDLES
        # =====================================================

        self.serial_process = None
        self.odom_process = None
        self.localization_process = None
        self.lidar_process = None
        self.slam_process = None
        self.nav_process = None

        # =====================================================
        # LAST MESSAGE TIMES
        # =====================================================

        self.last_imu_time = 0.0
        self.last_wheel_odom_time = 0.0
        self.last_filtered_odom_time = 0.0
        self.last_scan_time = 0.0
        self.last_map_time = 0.0
        self.last_global_costmap_time = 0.0

        # =====================================================
        # QOS
        # =====================================================

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        normal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =====================================================
        # READINESS SUBSCRIPTIONS
        # =====================================================

        # Serial ready signal
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.imu_callback,
            sensor_qos,
            callback_group=self.callback_group,
        )

        # Rover odom ready signal
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            "/wheel/odometry",
            self.wheel_odom_callback,
            normal_qos,
            callback_group=self.callback_group,
        )

        # EKF ready signal
        self.filtered_odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.filtered_odom_callback,
            normal_qos,
            callback_group=self.callback_group,
        )

        # LiDAR ready signal
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            sensor_qos,
            callback_group=self.callback_group,
        )

        # Cartographer ready signal
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
            callback_group=self.callback_group,
        )

        # Nav2 ready signal
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.global_costmap_callback,
            map_qos,
            callback_group=self.callback_group,
        )

        # =====================================================
        # SINGLE RESET SERVICE
        # =====================================================

        self.reset_service = self.create_service(
            Trigger,
            "/reset_slam",
            self.reset_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Rover stack manager initialized"
        )

        # Startup must happen separately from the executor
        # thread so topic callbacks can continue to execute
        # while startup waits for readiness.
        self.startup_thread = threading.Thread(
            target=self.initial_startup,
            daemon=True,
        )

        self.startup_thread.start()

    # =========================================================
    # TOPIC CALLBACKS
    # =========================================================

    def imu_callback(self, msg):
        self.last_imu_time = time.monotonic()

    def wheel_odom_callback(self, msg):
        self.last_wheel_odom_time = time.monotonic()

    def filtered_odom_callback(self, msg):
        self.last_filtered_odom_time = time.monotonic()

    def scan_callback(self, msg):
        self.last_scan_time = time.monotonic()

    def map_callback(self, msg):
        self.last_map_time = time.monotonic()

    def global_costmap_callback(self, msg):
        self.last_global_costmap_time = time.monotonic()

    # =========================================================
    # WAIT FOR FRESH DATA
    # =========================================================

    def wait_for_fresh_data(
        self,
        attribute_name,
        start_time,
        description,
        timeout,
    ):

        self.get_logger().info(
            f"Waiting for {description}..."
        )

        deadline = time.monotonic() + timeout

        while rclpy.ok():

            last_time = getattr(
                self,
                attribute_name,
            )

            # Must have received a message AFTER this
            # component was launched.
            if last_time > start_time:

                elapsed = (
                    time.monotonic()
                    - start_time
                )

                self.get_logger().info(
                    f"{description} READY "
                    f"({elapsed:.2f} s)"
                )

                return True

            if time.monotonic() >= deadline:

                self.get_logger().error(
                    f"TIMEOUT waiting for "
                    f"{description}"
                )

                return False

            time.sleep(0.05)

        return False

    # =========================================================
    # GENERIC PROCESS START
    # =========================================================

    def start_command(
        self,
        command,
        name,
    ):

        self.get_logger().info(
            f"Starting {name}..."
        )

        process = subprocess.Popen(
            command,
            start_new_session=True,
        )

        self.get_logger().info(
            f"{name} launched "
            f"(PID {process.pid})"
        )

        return process

    # =========================================================
    # GENERIC PROCESS STOP
    # =========================================================

    def stop_process(
        self,
        process,
        name,
    ):

        if process is None:
            return None

        if process.poll() is not None:

            self.get_logger().info(
                f"{name} already stopped"
            )

            return None

        pid = process.pid

        self.get_logger().info(
            f"Stopping {name} "
            f"(PID {pid})..."
        )

        try:

            # Normal Ctrl+C style shutdown first.
            os.killpg(
                os.getpgid(pid),
                signal.SIGINT,
            )

            try:

                process.wait(
                    timeout=6.0
                )

                self.get_logger().info(
                    f"{name} stopped"
                )

            except subprocess.TimeoutExpired:

                self.get_logger().warn(
                    f"{name} did not stop after SIGINT; "
                    f"sending SIGTERM"
                )

                os.killpg(
                    os.getpgid(pid),
                    signal.SIGTERM,
                )

                try:

                    process.wait(
                        timeout=3.0
                    )

                except subprocess.TimeoutExpired:

                    self.get_logger().error(
                        f"{name} still running; "
                        f"sending SIGKILL"
                    )

                    os.killpg(
                        os.getpgid(pid),
                        signal.SIGKILL,
                    )

                    process.wait(
                        timeout=2.0
                    )

        except ProcessLookupError:
            pass

        except Exception as exc:

            self.get_logger().error(
                f"Error stopping {name}: {exc}"
            )

        return None

    # =========================================================
    # 1. SERIAL
    # =========================================================

    def start_serial(self):

        start_time = time.monotonic()

        self.serial_process = self.start_command(
            [
                "ros2",
                "run",
                "serial",
                "serial_node",
            ],
            "Serial node",
        )

        if not self.wait_for_fresh_data(
            "last_imu_time",
            start_time,
            "/imu/data_raw",
            timeout=10.0,
        ):

            raise RuntimeError(
                "Serial node started but "
                "/imu/data_raw did not appear"
            )

    def stop_serial(self):

        self.serial_process = self.stop_process(
            self.serial_process,
            "Serial node",
        )

        self.last_imu_time = 0.0

    # =========================================================
    # 2. ROVER ODOM
    # =========================================================

    def start_odom(self):

        start_time = time.monotonic()

        self.odom_process = self.start_command(
            [
                "ros2",
                "run",
                "rover_odom",
                "odom_node",
            ],
            "Rover odom",
        )

        if not self.wait_for_fresh_data(
            "last_wheel_odom_time",
            start_time,
            "/wheel/odometry",
            timeout=8.0,
        ):

            raise RuntimeError(
                "Rover odom started but "
                "/wheel/odometry did not appear"
            )

    def stop_odom(self):

        self.odom_process = self.stop_process(
            self.odom_process,
            "Rover odom",
        )

        self.last_wheel_odom_time = 0.0

    # =========================================================
    # 3. EKF / LOCALIZATION
    # =========================================================

    def start_localization(self):

        start_time = time.monotonic()

        self.localization_process = self.start_command(
            [
                "ros2",
                "launch",
                "rover_localization",
                "localization.launch.py",
            ],
            "EKF / localization",
        )

        if not self.wait_for_fresh_data(
            "last_filtered_odom_time",
            start_time,
            "/odometry/filtered",
            timeout=8.0,
        ):

            raise RuntimeError(
                "EKF started but "
                "/odometry/filtered did not appear"
            )

    def stop_localization(self):

        self.localization_process = self.stop_process(
            self.localization_process,
            "EKF / localization",
        )

        self.last_filtered_odom_time = 0.0

    # =========================================================
    # 4. LIDAR
    # =========================================================

    def start_lidar(self):

        start_time = time.monotonic()

        self.lidar_process = self.start_command(
            [
                "ros2",
                "launch",
                "sllidar_ros2",
                "sllidar_a1_launch.py",
            ],
            "LiDAR driver",
        )

        if not self.wait_for_fresh_data(
            "last_scan_time",
            start_time,
            "/scan",
            timeout=12.0,
        ):

            raise RuntimeError(
                "LiDAR driver started but "
                "/scan did not appear"
            )

    def stop_lidar(self):

        self.lidar_process = self.stop_process(
            self.lidar_process,
            "LiDAR driver",
        )

        self.last_scan_time = 0.0

    # =========================================================
    # 5. CARTOGRAPHER
    # =========================================================

    def start_slam(self):

        start_time = time.monotonic()

        self.slam_process = self.start_command(
            [
                "ros2",
                "launch",
                "rover_slam",
                "cartographer.launch.py",
            ],
            "Cartographer",
        )

        if not self.wait_for_fresh_data(
            "last_map_time",
            start_time,
            "/map",
            timeout=15.0,
        ):

            raise RuntimeError(
                "Cartographer started but "
                "/map did not appear"
            )

    def stop_slam(self):

        self.slam_process = self.stop_process(
            self.slam_process,
            "Cartographer",
        )

        self.last_map_time = 0.0

    # =========================================================
    # 6. NAV2
    # =========================================================

    def start_nav(self):

        start_time = time.monotonic()

        self.nav_process = self.start_command(
            [
                "ros2",
                "launch",
                "rover_navigation",
                "navigation.launch.py",
            ],
            "Nav2",
        )

        if not self.wait_for_fresh_data(
            "last_global_costmap_time",
            start_time,
            "/global_costmap/costmap",
            timeout=20.0,
        ):

            raise RuntimeError(
                "Nav2 started but "
                "/global_costmap/costmap did not appear"
            )

    def stop_nav(self):

        self.nav_process = self.stop_process(
            self.nav_process,
            "Nav2",
        )

        self.last_global_costmap_time = 0.0

    # =========================================================
    # START ENTIRE STACK
    #
    # Serial
    #   ↓
    # Rover odom
    #   ↓
    # EKF
    #   ↓
    # LiDAR
    #   ↓
    # Cartographer
    #   ↓
    # Nav2
    # =========================================================

    def start_full_stack(self):

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "STARTING FULL ROVER STACK"
        )

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "[1/6] Serial"
        )
        self.start_serial()

        self.get_logger().info(
            "[2/6] Rover odom"
        )
        self.start_odom()

        self.get_logger().info(
            "[3/6] EKF"
        )
        self.start_localization()

        self.get_logger().info(
            "[4/6] LiDAR"
        )
        self.start_lidar()

        self.get_logger().info(
            "[5/6] Cartographer"
        )
        self.start_slam()

        self.get_logger().info(
            "[6/6] Nav2"
        )
        self.start_nav()

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "FULL ROVER STACK READY"
        )

        self.get_logger().info(
            "======================================"
        )

    # =========================================================
    # INITIAL STARTUP
    # =========================================================

    def initial_startup(self):

        # Let executor begin spinning first.
        time.sleep(0.5)

        if not self.operation_lock.acquire(
            blocking=False
        ):
            return

        try:

            self.start_full_stack()

        except Exception as exc:

            self.get_logger().error(
                f"STACK STARTUP FAILED: {exc}"
            )

        finally:

            self.operation_lock.release()

    # =========================================================
    # SINGLE FULL RESET
    #
    # Stops:
    #
    #   Nav2
    #   Cartographer
    #   LiDAR
    #   EKF
    #   Rover odom
    #   Serial
    #
    # Then restarts:
    #
    #   Serial
    #   Rover odom
    #   EKF
    #   LiDAR
    #   Cartographer
    #   Nav2
    #
    # =========================================================

    def reset_callback(
        self,
        request,
        response,
    ):

        if not self.operation_lock.acquire(
            blocking=False
        ):

            response.success = False
            response.message = (
                "Another stack operation "
                "is already in progress"
            )

            return response

        try:

            self.get_logger().warn(
                "======================================"
            )

            self.get_logger().warn(
                "FULL ROVER RESET REQUESTED"
            )

            self.get_logger().warn(
                "======================================"
            )

            # =================================================
            # SHUTDOWN - REVERSE DEPENDENCY ORDER
            # =================================================

            self.get_logger().info(
                "[1/12] Stopping Nav2..."
            )
            self.stop_nav()

            self.get_logger().info(
                "[2/12] Stopping Cartographer..."
            )
            self.stop_slam()

            self.get_logger().info(
                "[3/12] Stopping LiDAR..."
            )
            self.stop_lidar()

            self.get_logger().info(
                "[4/12] Stopping EKF..."
            )
            self.stop_localization()

            self.get_logger().info(
                "[5/12] Stopping rover odom..."
            )
            self.stop_odom()

            self.get_logger().info(
                "[6/12] Stopping Serial..."
            )
            self.stop_serial()

            # Short period for old processes, UART and USB
            # resources to disappear cleanly.
            time.sleep(0.75)

            # =================================================
            # STARTUP
            # =================================================

            self.get_logger().info(
                "[7/12] Starting Serial..."
            )
            self.start_serial()

            self.get_logger().info(
                "[8/12] Starting rover odom..."
            )
            self.start_odom()

            self.get_logger().info(
                "[9/12] Starting EKF..."
            )
            self.start_localization()

            self.get_logger().info(
                "[10/12] Starting LiDAR..."
            )
            self.start_lidar()

            self.get_logger().info(
                "[11/12] Starting Cartographer..."
            )
            self.start_slam()

            self.get_logger().info(
                "[12/12] Starting Nav2..."
            )
            self.start_nav()

            response.success = True

            response.message = (
                "Serial, rover odom, EKF, LiDAR, "
                "Cartographer and Nav2 reset successfully"
            )

            self.get_logger().info(
                "======================================"
            )

            self.get_logger().info(
                "FULL ROVER RESET COMPLETE"
            )

            self.get_logger().info(
                "======================================"
            )

        except Exception as exc:

            response.success = False

            response.message = (
                f"Full rover reset failed: {exc}"
            )

            self.get_logger().error(
                response.message
            )

        finally:

            self.operation_lock.release()

        return response

    # =========================================================
    # MANAGER SHUTDOWN
    # =========================================================

    def shutdown(self):

        self.get_logger().info(
            "Shutting down rover stack..."
        )

        with self.operation_lock:

            # Reverse dependency order
            self.stop_nav()
            self.stop_slam()
            self.stop_lidar()
            self.stop_localization()
            self.stop_odom()
            self.stop_serial()

        self.get_logger().info(
            "Rover stack shutdown complete"
        )


def main(args=None):

    rclpy.init(args=args)

    node = RoverStackManager()

    # Multiple executor threads are required because the
    # startup/reset routines wait for topic callbacks.
    executor = MultiThreadedExecutor(
        num_threads=4
    )

    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown()

        executor.shutdown()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
