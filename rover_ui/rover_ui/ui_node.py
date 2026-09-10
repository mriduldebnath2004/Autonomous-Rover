"""
Rover UI — PyQt6 + ROS2 map/nav console.

Features:
- 4x high-resolution map render canvas
- occupancy-grid map
- stale-map guard
- live Nav2 feedback
- true-scale 0.16 m x 0.16 m rover
- clear dark rover heading arrow
- clean waypoint + heading arrow
- global path visualization
- goal/path cleared after successful navigation
- toggleable global/local Nav2 costmap overlays (RViz-style color gradient)
- strong latched STOP visualization
- bottom-left red RESET ROVER button via /reset_slam Trigger service\n- mouse-wheel map zoom\n- right-click drag map panning\n- rover-follow 5 m x 5 m heading-up camera\n- live forward/angular velocity\n"""

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import sys
import math
import time
import threading
import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QSizePolicy,
)

from PyQt6.QtCore import Qt, QTimer, QPointF

from PyQt6.QtGui import (
    QImage,
    QPixmap,
    QPainter,
    QPolygonF,
    QPen,
    QFont,
    QColor,
)


# ============================================================
# RENDER SETTINGS
# ============================================================

# Raw occupancy grid is rendered internally at 4x resolution.
# Example:
# 85 x 59 grid -> 340 x 236 render canvas
RENDER_SCALE = 4


# ============================================================
# COLORS / STYLE
# ============================================================

BG = "#1B1E23"
PANEL = "#22262D"
LINE = "#343941"

TEXT = "#E8E6DE"
MUTED = "#8B8F96"

AMBER = "#E0A458"
OK = "#7FA773"
WARN = "#D9B24C"

DANGER = "#C4544B"
DANGER_HOVER = "#DD6459"

MONO = "Consolas, 'DejaVu Sans Mono', monospace"

TITLE_FONT = "'Segoe UI Semibold', 'Segoe UI', 'Trebuchet MS', sans-serif"


# ------------------------------------------------------------
# COSTMAP COLOR GRADIENTS (RViz2-style "costmap" color scheme)
# ------------------------------------------------------------
#
# Each gradient is a list of (normalized_cost, QColor) stops running from
# low cost -> high/lethal cost.
#
# Global: yellow -> orange -> red -> dark red
# Local:  pale cyan -> cyan -> blue -> purple
#
# The two families stay visually distinct while also making cost severity
# obvious at a glance.

GLOBAL_COSTMAP_GRADIENT = [
    (0.0, QColor(255, 238, 140)),   # yellow — low cost
    (0.45, QColor(255, 166, 45)),   # orange — medium cost
    (0.75, QColor(235, 82, 45)),    # red-orange — high cost
    (1.0, QColor(150, 20, 25)),     # dark red — lethal
]

LOCAL_COSTMAP_GRADIENT = [
    (0.0, QColor(170, 245, 255)),   # pale cyan — low cost
    (0.45, QColor(40, 205, 235)),   # cyan — medium cost
    (0.75, QColor(55, 105, 235)),   # blue — high cost
    (1.0, QColor(105, 45, 190)),    # purple — lethal
]


def gradient_color(value, stops):
    """Interpolate a QColor from a sorted list of (t, QColor) stops."""

    if value <= stops[0][0]:
        base = stops[0][1]
        return QColor(base)

    if value >= stops[-1][0]:
        base = stops[-1][1]
        return QColor(base)

    for i in range(len(stops) - 1):

        t0, c0 = stops[i]
        t1, c1 = stops[i + 1]

        if t0 <= value <= t1:

            span = (t1 - t0) if (t1 - t0) != 0 else 1.0

            t = (value - t0) / span

            r = c0.red() + (c1.red() - c0.red()) * t
            g = c0.green() + (c1.green() - c0.green()) * t
            b = c0.blue() + (c1.blue() - c0.blue()) * t

            return QColor(
                int(round(r)),
                int(round(g)),
                int(round(b))
            )

    return QColor(stops[-1][1])


STYLESHEET = f"""
QMainWindow {{
    background-color: {BG};
}}

QLabel {{
    color: {TEXT};
}}

QFrame#SidePanel {{
    background-color: {PANEL};
    border-left: 1px solid {LINE};
}}

QFrame#MapFrame {{
    background-color: #14161A;
    border: 1px solid {LINE};
}}

QLabel#SectionTitle {{
    color: {TEXT};
    font-size: 12.5px;
    font-weight: 800;
    letter-spacing: 1.3px;
}}

QLabel#PanelTitle {{
    color: {AMBER};
    font-size: 15px;
    font-weight: 700;
}}

QLabel#PanelSub {{
    color: {MUTED};
    font-size: 11px;
}}

QFrame#Divider {{
    background-color: {LINE};
    max-height: 1px;
    min-height: 1px;
}}

QLabel#TopAppTitle {{
    color: {TEXT};
    font-family: {TITLE_FONT};
    font-size: 23px;
    font-weight: 800;
    letter-spacing: 0.6px;
}}

QLabel#TopAppSub {{
    color: {MUTED};
    font-size: 11.5px;
    letter-spacing: 0.3px;
}}

QFrame#SystemStatusGroup, QFrame#LinkStatusGroup {{
    background-color: transparent;
}}

QLabel#SystemStatusTitle, QLabel#LinkStatusTitle {{
    color: {TEXT};
    font-size: 11.5px;
    font-weight: 800;
    letter-spacing: 1.2px;
}}

QLabel#TopStatusName {{
    color: {MUTED};
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.5px;
}}

QLabel#TopStatusValue {{
    font-size: 10px;
    font-weight: 700;
}}

QPushButton {{
    background-color: #2C313A;
    color: {TEXT};
    border: 1px solid {LINE};
    padding: 10px;
    border-radius: 2px;
    font-size: 12.5px;
}}

QPushButton:hover {{
    border-color: {AMBER};
}}

QPushButton:pressed {{
    background-color: #343941;
}}

QPushButton:checked {{
    background-color: #3A3326;
    border-color: {AMBER};
    color: {AMBER};
    font-weight: 700;
}}

QPushButton#StopButton {{
    background-color: {DANGER};
    color: #1A1A1A;
    font-weight: 700;
    font-size: 14px;
    padding: 16px;
    border: none;
}}

QPushButton#StopButton:hover {{
    background-color: {DANGER_HOVER};
}}

QPushButton#StopButton:checked {{
    background-color: #5A1715;
    color: #FFF3F2;
    border: 2px solid {DANGER_HOVER};
}}

QPushButton#ResetMapButton {{
    background-color: {DANGER};
    color: #FFF3F2;
    border: 1px solid {DANGER_HOVER};
    font-weight: 700;
    font-size: 12px;
    padding: 9px 12px;
    border-radius: 3px;
}}

QPushButton#ResetMapButton:hover {{
    background-color: {DANGER_HOVER};
    border-color: #FF817A;
}}

QPushButton#ResetMapButton:pressed {{
    background-color: #8E2B27;
}}

QPushButton#ResetMapButton:disabled {{
    background-color: #5A2A27;
    color: #C6AAA8;
    border-color: #74413D;
}}
"""


# ============================================================
# HELPERS
# ============================================================

def quaternion_to_yaw(q):

    siny_cosp = 2.0 * (
        q.w * q.z +
        q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y +
        q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


# ============================================================
# ROS NODE
# ============================================================

class RoverUINode(Node):

    def __init__(self):

        super().__init__("rover_ui")

        self.latest_map = None
        self.latest_path = None
        self.latest_global_costmap = None
        self.latest_local_costmap = None

        self.last_map_time = None
        self.last_path_time = None
        self.last_global_costmap_time = None
        self.last_local_costmap_time = None

        self.current_goal_handle = None

        # Nav2 UI state
        self.navigation_state = "IDLE"
        self.distance_remaining = None

        # Final rover command velocity, taken from /cmd_vel after the Nav2
        # smoother/collision-monitor chain.
        self.forward_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_cmd_vel_time = None

        # Signals Qt thread to clear finished goal visuals
        self.goal_completed = False

        # Full rover-stack reset service/UI state. The rover-side /reset_slam
        # service restarts Serial, Rover Odom, EKF, LiDAR, Cartographer, and Nav2.
        self.reset_in_progress = False
        self.reset_finished = False
        self.reset_success = False
        self.reset_message = ""

        # ----------------------------
        # ROVER SYSTEM STATUS
        # ----------------------------
        #
        # Published by rover_stack_manager on /rover/system_status.
        # The manager sends a JSON std_msgs/String containing:
        # overall, operation, serial, odom, ekf, lidar, slam, nav2.
        self.system_status = {
            "overall": "WAITING",
            "operation": "WAITING",
            "serial": "WAITING",
            "odom": "WAITING",
            "ekf": "WAITING",
            "lidar": "WAITING",
            "slam": "WAITING",
            "nav2": "WAITING",
        }
        self.last_system_status_time = None

        # ----------------------------
        # SYSTEM STATUS
        # ----------------------------

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.system_status_sub = self.create_subscription(
            String,
            "/rover/system_status",
            self.system_status_callback,
            status_qos
        )

        # ----------------------------
        # MAP
        # ----------------------------

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )

        # Nav2 costmaps use transient-local durability so a new UI
        # subscriber immediately receives the most recently published grid.
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.global_costmap_callback,
            costmap_qos
        )

        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.local_costmap_callback,
            costmap_qos
        )

        # ----------------------------
        # GLOBAL PLAN
        # ----------------------------

        self.path_sub = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )

        # ----------------------------
        # CMD VEL
        # ----------------------------

        # /cmd_vel is the final command sent toward the rover after Nav2's
        # velocity smoother and collision monitor. Subscribe so the UI shows
        # the command the rover is actually being asked to execute.
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # ----------------------------
        # TF
        # ----------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ----------------------------
        # NAV2
        # ----------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose"
        )

        # Rover-side stack manager. It exposes std_srvs/Trigger on /reset_slam
        # and restarts Serial, Rover Odom, EKF, LiDAR, Cartographer, and Nav2.
        self.reset_slam_client = self.create_client(
            Trigger,
            "/reset_slam"
        )

        self.get_logger().info(
            "Rover UI node started"
        )

    # ========================================================
    # CALLBACKS
    # ========================================================

    def system_status_callback(self, msg):

        try:
            data = json.loads(msg.data)

            for key in (
                "overall",
                "operation",
                "serial",
                "odom",
                "ekf",
                "lidar",
                "slam",
                "nav2",
            ):
                if key in data:
                    self.system_status[key] = str(data[key]).upper()

            self.last_system_status_time = time.monotonic()

        except Exception as exc:
            self.get_logger().warn(
                f"Invalid /rover/system_status message: {exc}"
            )

    def map_callback(self, msg):

        self.latest_map = msg
        self.last_map_time = time.monotonic()

    def path_callback(self, msg):

        self.latest_path = msg
        self.last_path_time = time.monotonic()

    def global_costmap_callback(self, msg):

        self.latest_global_costmap = msg
        self.last_global_costmap_time = time.monotonic()

    def local_costmap_callback(self, msg):

        self.latest_local_costmap = msg
        self.last_local_costmap_time = time.monotonic()

    def cmd_vel_callback(self, msg):

        self.forward_velocity = float(
            msg.linear.x
        )

        self.angular_velocity = float(
            msg.angular.z
        )

        self.last_cmd_vel_time = time.monotonic()

    # ========================================================
    # TF
    # ========================================================

    def get_rover_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

        except Exception:

            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        yaw = quaternion_to_yaw(
            transform.transform.rotation
        )

        return x, y, yaw

    def get_frame_transform_2d(
        self,
        target_frame,
        source_frame,
        stamp=None
    ):
        """
        Return the 2D transform target_frame <- source_frame as
        (x, y, yaw).

        The local Nav2 costmap is published in odom, while the base
        Cartographer map is in map.  RViz applies this TF automatically;
        the custom UI must do the same or the local overlay appears offset.

        Prefer the costmap message timestamp so the rolling costmap is drawn
        where it actually was when published. Fall back to the latest TF if
        that exact timestamp is no longer available.
        """

        target_frame = (target_frame or "").lstrip("/")
        source_frame = (source_frame or "").lstrip("/")

        if not target_frame or not source_frame:
            return None

        if target_frame == source_frame:
            return 0.0, 0.0, 0.0

        try:
            if stamp is not None:
                tf_time = rclpy.time.Time.from_msg(stamp)
            else:
                tf_time = rclpy.time.Time()

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                tf_time
            )

        except Exception:
            # A tiny amount of TF history can occasionally be unavailable.
            # Latest TF is much better than drawing an odom-frame grid as if
            # it were already in map.
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time()
                )
            except Exception:
                return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            quaternion_to_yaw(transform.transform.rotation)
        )

    # ========================================================
    # NAVIGATION GOAL
    # ========================================================

    def send_nav_goal(
        self,
        x,
        y,
        yaw
    ):

        if not self.nav_client.wait_for_server(
            timeout_sec=1.0
        ):

            self.navigation_state = "NAV2 OFFLINE"

            self.get_logger().warn(
                "NavigateToPose server not available"
            )

            return

        goal = NavigateToPose.Goal()

        goal.pose = PoseStamped()

        goal.pose.header.frame_id = "map"

        goal.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0

        goal.pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )

        goal.pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        self.goal_completed = False
        self.navigation_state = "SENDING"
        self.distance_remaining = None

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ========================================================
    # NAV2 RESPONSE
    # ========================================================

    def goal_response_callback(
        self,
        future
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.navigation_state = "REJECTED"
            self.current_goal_handle = None

            return

        self.navigation_state = "NAVIGATING"

        self.current_goal_handle = (
            goal_handle
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.nav_result_callback
        )

    # ========================================================
    # LIVE NAV2 FEEDBACK
    # ========================================================

    def nav_feedback_callback(
        self,
        feedback_msg
    ):

        feedback = (
            feedback_msg.feedback
        )

        self.distance_remaining = (
            feedback.distance_remaining
        )

        self.navigation_state = (
            "NAVIGATING"
        )

    # ========================================================
    # NAV2 RESULT
    # ========================================================

    def nav_result_callback(
        self,
        future
    ):

        wrapped_result = (
            future.result()
        )

        status = (
            wrapped_result.status
        )

        if status == 4:

            self.navigation_state = "SUCCEEDED"
            self.goal_completed = True

        elif status == 5:

            self.navigation_state = "CANCELED"

        elif status == 6:

            self.navigation_state = "ABORTED"

        else:

            self.navigation_state = (
                f"FINISHED ({status})"
            )

        self.distance_remaining = None
        self.current_goal_handle = None

    # ========================================================
    # CANCEL
    # ========================================================

    def cancel_navigation(self):

        if self.current_goal_handle is None:

            self.navigation_state = "IDLE"

            return

        self.navigation_state = "CANCELING"

        self.current_goal_handle.cancel_goal_async()

        self.current_goal_handle = None

    # ========================================================
    # FULL ROVER STACK RESET
    # ========================================================

    def request_slam_reset(self):

        if self.reset_in_progress:
            return True

        if not self.reset_slam_client.service_is_ready():
            self.reset_finished = True
            self.reset_success = False
            self.reset_message = "RESET SERVICE OFFLINE"
            self.get_logger().warn(
                "/reset_slam service is not available"
            )
            return False

        self.reset_in_progress = True
        self.reset_finished = False
        self.reset_success = False
        self.reset_message = "RESETTING ROVER..."

        future = self.reset_slam_client.call_async(
            Trigger.Request()
        )

        future.add_done_callback(
            self.reset_slam_done_callback
        )

        return True

    def reset_slam_done_callback(self, future):

        try:
            response = future.result()
            self.reset_success = bool(response.success)
            self.reset_message = (
                response.message
                if response.message
                else ("RESET COMPLETE" if response.success else "RESET FAILED")
            )

        except Exception as exc:
            self.reset_success = False
            self.reset_message = f"RESET FAILED: {exc}"

        self.reset_in_progress = False
        self.reset_finished = True

    # ========================================================
    # ZERO VELOCITY
    # ========================================================

    def publish_zero_velocity(self):

        self.cmd_vel_pub.publish(
            Twist()
        )


# ============================================================
# MAP VIEW
# ============================================================

class MapView(QLabel):

    MIN_ZOOM = 0.35
    MAX_ZOOM = 8.0
    ZOOM_STEP = 1.15

    def __init__(
        self,
        on_goal_selected
    ):

        super().__init__(
            "Waiting for map…"
        )

        self.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

        self.setStyleSheet(
            f"""
            color: {MUTED};
            font-size: 13px;
            border: none;
            """
        )

        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding
        )

        self.setMinimumSize(
            400,
            400
        )

        # Mouse tracking makes the interaction feel smoother while panning.
        self.setMouseTracking(
            True
        )

        self._on_goal_selected = (
            on_goal_selected
        )

        # Left mouse = navigation-goal drag.
        self.dragging = False
        self.drag_start = None
        self.drag_end = None

        # Right mouse = map pan.
        self.panning = False
        self.pan_last = None

        # View transform in widget/screen pixels.
        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

        # When enabled, MainWindow ignores the normal pan/zoom camera and
        # renders a rover-centered 5 m x 5 m heading-up view instead.
        self.follow_mode = False

    # ========================================================
    # VIEW CONTROLS
    # ========================================================

    def reset_view(self):

        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0

        self.update()

    def set_follow_mode(
        self,
        enabled
    ):

        self.follow_mode = bool(
            enabled
        )

        if self.follow_mode:
            self.panning = False
            self.pan_last = None
            self.unsetCursor()

        self.update()

    def wheelEvent(
        self,
        event
    ):

        # The rover-follow camera has a fixed 5 m x 5 m scale.
        if self.follow_mode:
            event.accept()
            return

        # Prefer the normal wheel delta, but also support high-resolution
        # touchpads where angleDelta() can occasionally be zero.
        delta = event.angleDelta().y()

        if delta == 0:
            delta = event.pixelDelta().y()

        if delta == 0:
            event.accept()
            return

        old_zoom = self.zoom_factor

        # Use a smooth exponential step so both mouse wheels and touchpads
        # behave consistently.
        steps = max(
            -4.0,
            min(
                4.0,
                delta / 120.0
            )
        )

        if abs(steps) < 0.10:
            steps = 0.10 if delta > 0 else -0.10

        new_zoom = (
            old_zoom
            * (self.ZOOM_STEP ** steps)
        )

        new_zoom = max(
            self.MIN_ZOOM,
            min(
                self.MAX_ZOOM,
                new_zoom
            )
        )

        if abs(new_zoom - old_zoom) < 1.0e-9:
            event.accept()
            return

        # Zoom around the mouse cursor instead of around the center.
        # This is much more stable and fixes the "sometimes it jumps /
        # sometimes it feels like nothing happened" behavior.
        cursor = event.position()

        widget_cx = (
            self.width() / 2.0
        )

        widget_cy = (
            self.height() / 2.0
        )

        ratio = (
            new_zoom / old_zoom
        )

        self.pan_x = (
            cursor.x()
            - widget_cx
            - (
                cursor.x()
                - widget_cx
                - self.pan_x
            ) * ratio
        )

        self.pan_y = (
            cursor.y()
            - widget_cy
            - (
                cursor.y()
                - widget_cy
                - self.pan_y
            ) * ratio
        )

        self.zoom_factor = (
            new_zoom
        )

        self.update()

        event.accept()

    # ========================================================
    # MOUSE INPUT
    # ========================================================

    def mousePressEvent(
        self,
        event
    ):

        if (
            event.button()
            == Qt.MouseButton.RightButton
        ):

            # Camera position is locked to the rover in follow mode.
            if self.follow_mode:
                event.accept()
                return

            self.panning = True

            self.pan_last = (
                event.position()
            )

            self.setCursor(
                Qt.CursorShape.ClosedHandCursor
            )

            event.accept()
            return

        if (
            event.button()
            == Qt.MouseButton.LeftButton
        ):

            self.dragging = True

            self.drag_start = (
                event.position()
            )

            self.drag_end = (
                event.position()
            )

            event.accept()
            return

        super().mousePressEvent(
            event
        )

    def mouseMoveEvent(
        self,
        event
    ):

        if (
            self.panning
            and self.pan_last is not None
        ):

            current = (
                event.position()
            )

            delta = (
                current - self.pan_last
            )

            self.pan_x += (
                delta.x()
            )

            self.pan_y += (
                delta.y()
            )

            self.pan_last = (
                current
            )

            self.update()

            event.accept()
            return

        if self.dragging:

            self.drag_end = (
                event.position()
            )

            self.update()

            event.accept()
            return

        super().mouseMoveEvent(
            event
        )

    def mouseReleaseEvent(
        self,
        event
    ):

        if (
            self.panning
            and event.button()
            == Qt.MouseButton.RightButton
        ):

            self.panning = False
            self.pan_last = None

            self.unsetCursor()

            self.update()

            event.accept()
            return

        if (
            self.dragging
            and event.button()
            == Qt.MouseButton.LeftButton
        ):

            self.dragging = False

            self.drag_end = (
                event.position()
            )

            self._on_goal_selected(
                self.drag_start,
                self.drag_end
            )

            self.drag_start = None
            self.drag_end = None

            self.update()

            event.accept()
            return

        super().mouseReleaseEvent(
            event
        )



# ============================================================
# TOP STATUS BAR
# ============================================================

class TopStatusBar(QFrame):

    def __init__(self):

        super().__init__()

        self.setObjectName("TopStatusBar")
        self.setFixedHeight(94)

        outer = QHBoxLayout(self)
        outer.setContentsMargins(20, 10, 18, 10)
        outer.setSpacing(14)

        # ----------------------------
        # LEFT: APP TITLE
        # ----------------------------

        title_box = QVBoxLayout()
        title_box.setSpacing(2)

        app_title = QLabel("AUTONOMOUS ROVER")
        app_title.setObjectName("TopAppTitle")

        app_sub = QLabel("ROS2 mapping / navigation console")
        app_sub.setObjectName("TopAppSub")

        title_box.addWidget(app_title)
        title_box.addWidget(app_sub)

        outer.addLayout(title_box, stretch=0)

        # Intentionally leave the center open.
        outer.addStretch(1)

        # ----------------------------
        # RIGHT: SYSTEM STATUS
        # ----------------------------

        system_frame = QFrame()
        system_frame.setObjectName("SystemStatusGroup")

        system_box = QVBoxLayout(system_frame)
        system_box.setContentsMargins(13, 8, 13, 8)
        system_box.setSpacing(5)

        system_title = QLabel("SYSTEM STATUS")
        system_title.setObjectName("SystemStatusTitle")
        system_box.addWidget(system_title)

        system_row = QHBoxLayout()
        system_row.setSpacing(14)

        self.system_overall = self._status_item(
            system_row, "ROVER", "WAITING"
        )
        self.serial_status = self._status_item(
            system_row, "SERIAL", "WAITING"
        )
        self.odom_status = self._status_item(
            system_row, "ODOM", "WAITING"
        )
        self.ekf_status = self._status_item(
            system_row, "EKF", "WAITING"
        )
        self.lidar_status = self._status_item(
            system_row, "LIDAR", "WAITING"
        )
        self.slam_status = self._status_item(
            system_row, "SLAM", "WAITING"
        )
        self.nav2_status = self._status_item(
            system_row, "NAV2", "WAITING"
        )

        system_box.addLayout(system_row)
        outer.addWidget(system_frame, stretch=0)

        # Strong visual separation between the two status groups.
        separator = QFrame()
        separator.setFixedWidth(2)
        separator.setStyleSheet(
            f"background-color:{LINE}; border:none;"
        )
        outer.addWidget(separator)

        # ----------------------------
        # FAR RIGHT: LINK STATUS
        # ----------------------------

        link_frame = QFrame()
        link_frame.setObjectName("LinkStatusGroup")

        link_box = QVBoxLayout(link_frame)
        link_box.setContentsMargins(13, 8, 13, 8)
        link_box.setSpacing(5)

        link_title = QLabel("LINK STATUS")
        link_title.setObjectName("LinkStatusTitle")
        link_box.addWidget(link_title)

        link_row = QHBoxLayout()
        link_row.setSpacing(14)

        self.map_status = self._status_item(
            link_row, "MAP", "WAITING"
        )
        self.path_status = self._status_item(
            link_row, "PLAN", "WAITING"
        )
        self.pose_status = self._status_item(
            link_row, "POSE", "WAITING"
        )

        link_box.addLayout(link_row)
        outer.addWidget(link_frame, stretch=0)

    def _status_item(
        self,
        layout,
        name,
        initial
    ):

        container = QVBoxLayout()
        container.setSpacing(0)

        name_label = QLabel(name)
        name_label.setObjectName("TopStatusName")

        value_label = QLabel(initial)
        value_label.setObjectName("TopStatusValue")
        value_label.setAlignment(Qt.AlignmentFlag.AlignLeft)

        container.addWidget(name_label)
        container.addWidget(value_label)

        layout.addLayout(container)

        return value_label

    def set_status(
        self,
        label,
        state
    ):

        colors = {
            "OK": OK,
            "READY": OK,
            "ONLINE": OK,

            "STARTING": AMBER,
            "RESETTING": AMBER,
            "STOPPING": AMBER,
            "SHUTTING_DOWN": AMBER,
            "WAITING": MUTED,
            "STALE": WARN,

            "OFFLINE": DANGER_HOVER,
            "ERROR": DANGER_HOVER,
            "STARTUP_FAILED": DANGER_HOVER,
            "RESET_FAILED": DANGER_HOVER,
        }

        label.setText(state)

        label.setStyleSheet(
            f"""
            color:{colors.get(state, MUTED)};
            font-size:10px;
            font-weight:700;
            """
        )


# ============================================================
# SIDE PANEL
# ============================================================

class SidePanel(QFrame):

    def __init__(
        self,
        on_stop,
        on_cancel_goal,
        on_global_costmap_toggle,
        on_local_costmap_toggle,
        on_follow_rover_toggle
    ):

        super().__init__()

        self.setObjectName(
            "SidePanel"
        )

        self.setFixedWidth(
            300
        )

        layout = QVBoxLayout(
            self
        )

        layout.setContentsMargins(
            20,
            22,
            20,
            20
        )

        layout.setSpacing(
            14
        )

        # ----------------------------
        # TITLE
        # ----------------------------

        title = QLabel(
            "ROVER CONTROL"
        )

        title.setObjectName(
            "PanelTitle"
        )

        sub = QLabel(
            "map / navigation console"
        )

        sub.setObjectName(
            "PanelSub"
        )

        layout.addWidget(
            title
        )

        layout.addWidget(
            sub
        )

        layout.addWidget(
            self._divider()
        )

        # ----------------------------
        # NAVIGATION
        # ----------------------------

        layout.addWidget(
            self._section_title(
                "NAVIGATION"
            )
        )

        self.nav_state = (
            self._data_row(
                layout,
                "State"
            )
        )

        self.nav_distance = (
            self._data_row(
                layout,
                "Remaining"
            )
        )

        layout.addWidget(
            self._divider()
        )

        # ----------------------------
        # MAP OVERLAYS
        # ----------------------------

        layout.addWidget(
            self._section_title(
                "MAP OVERLAYS"
            )
        )

        overlay_row = QHBoxLayout()
        overlay_row.setSpacing(8)

        self.global_costmap_btn = QPushButton(
            "Global"
        )
        self.global_costmap_btn.setCheckable(
            True
        )
        self.global_costmap_btn.setChecked(
            False
        )
        self.global_costmap_btn.setToolTip(
            "Toggle the Nav2 global costmap overlay"
        )
        self.global_costmap_btn.clicked.connect(
            on_global_costmap_toggle
        )

        self.local_costmap_btn = QPushButton(
            "Local"
        )
        self.local_costmap_btn.setCheckable(
            True
        )
        self.local_costmap_btn.setChecked(
            False
        )
        self.local_costmap_btn.setToolTip(
            "Toggle the Nav2 local costmap overlay"
        )
        self.local_costmap_btn.clicked.connect(
            on_local_costmap_toggle
        )

        overlay_row.addWidget(
            self.global_costmap_btn
        )
        overlay_row.addWidget(
            self.local_costmap_btn
        )

        layout.addLayout(
            overlay_row
        )

        self.follow_rover_btn = QPushButton(
            "◎  FOLLOW ROVER — 5 m"
        )
        self.follow_rover_btn.setCheckable(
            True
        )
        self.follow_rover_btn.setChecked(
            False
        )
        self.follow_rover_btn.setToolTip(
            "Lock the camera to the rover with a 5 m x 5 m heading-up view"
        )
        self.follow_rover_btn.clicked.connect(
            on_follow_rover_toggle
        )

        layout.addWidget(
            self.follow_rover_btn
        )

        self.costmap_hint = QLabel(
            'Global Costmap: <span style="color:#8B1E1E; font-weight:700;">Red</span><br>'
            'Local Costmap: <span style="color:#8E5BD9; font-weight:700;">Purple</span>'
        )

        self.costmap_hint.setTextFormat(
            Qt.TextFormat.RichText
        )

        self.costmap_hint.setObjectName(
            "PanelSub"
        )
        self.costmap_hint.setWordWrap(
            True
        )
        layout.addWidget(
            self.costmap_hint
        )

        layout.addWidget(
            self._divider()
        )

        # ----------------------------
        # ROVER POSE
        # ----------------------------

        layout.addWidget(
            self._section_title(
                "ROVER POSE"
            )
        )

        self.pose_x = (
            self._data_row(
                layout,
                "X"
            )
        )

        self.pose_y = (
            self._data_row(
                layout,
                "Y"
            )
        )

        self.pose_yaw = (
            self._data_row(
                layout,
                "Heading"
            )
        )

        self.forward_vel = (
            self._data_row(
                layout,
                "Forward vel"
            )
        )

        self.angular_vel = (
            self._data_row(
                layout,
                "Angular vel"
            )
        )

        layout.addWidget(
            self._divider()
        )

        # ----------------------------
        # GOAL
        # ----------------------------

        layout.addWidget(
            self._section_title(
                "NAVIGATION GOAL"
            )
        )

        self.goal_label = QLabel(
            "none"
        )

        self.goal_label.setStyleSheet(
            f"""
            color:{AMBER};
            font-family:{MONO};
            font-size:12.5px;
            """
        )

        self.goal_label.setWordWrap(
            True
        )

        layout.addWidget(
            self.goal_label
        )

        cancel_btn = QPushButton(
            "Cancel goal"
        )

        cancel_btn.clicked.connect(
            on_cancel_goal
        )

        layout.addWidget(
            cancel_btn
        )

        layout.addStretch()

        # ----------------------------
        # STOP
        # ----------------------------

        layout.addWidget(
            self._divider()
        )

        stop_hint = QLabel(
            "Halts navigation and holds zero velocity."
        )

        stop_hint.setObjectName(
            "PanelSub"
        )

        stop_hint.setWordWrap(
            True
        )

        layout.addWidget(
            stop_hint
        )

        self.stop_btn = QPushButton(
            "■  STOP"
        )

        self.stop_btn.setObjectName(
            "StopButton"
        )

        self.stop_btn.setCheckable(
            True
        )

        self.stop_btn.clicked.connect(
            on_stop
        )

        layout.addWidget(
            self.stop_btn
        )

    # ========================================================
    # PANEL HELPERS
    # ========================================================

    def _section_title(
        self,
        text
    ):

        label = QLabel(
            text
        )

        label.setObjectName(
            "SectionTitle"
        )

        return label

    def _divider(
        self
    ):

        line = QFrame()

        line.setObjectName(
            "Divider"
        )

        return line

    def _status_row(
        self,
        layout,
        name
    ):

        row = QHBoxLayout()

        name_label = QLabel(
            name
        )

        name_label.setStyleSheet(
            f"color:{TEXT}; font-size:12.5px;"
        )

        value_label = QLabel(
            "WAITING"
        )

        value_label.setStyleSheet(
            f"""
            color:{MUTED};
            font-size:11px;
            font-weight:600;
            """
        )

        value_label.setAlignment(
            Qt.AlignmentFlag.AlignRight
        )

        row.addWidget(
            name_label
        )

        row.addWidget(
            value_label
        )

        layout.addLayout(
            row
        )

        return value_label

    def _data_row(
        self,
        layout,
        name
    ):

        row = QHBoxLayout()

        name_label = QLabel(
            name
        )

        name_label.setStyleSheet(
            f"color:{MUTED}; font-size:12.5px;"
        )

        value_label = QLabel(
            "--"
        )

        value_label.setStyleSheet(
            f"""
            color:{TEXT};
            font-family:{MONO};
            font-size:13px;
            """
        )

        value_label.setAlignment(
            Qt.AlignmentFlag.AlignRight
        )

        row.addWidget(
            name_label
        )

        row.addWidget(
            value_label
        )

        layout.addLayout(
            row
        )

        return value_label

    def set_status(
        self,
        label,
        state
    ):

        colors = {
            "OK": OK,
            "READY": OK,
            "ONLINE": OK,

            "STARTING": AMBER,
            "RESETTING": AMBER,
            "STOPPING": AMBER,
            "SHUTTING_DOWN": AMBER,
            "WAITING": MUTED,
            "STALE": WARN,

            "OFFLINE": DANGER_HOVER,
            "ERROR": DANGER_HOVER,
            "STARTUP_FAILED": DANGER_HOVER,
            "RESET_FAILED": DANGER_HOVER,
        }

        label.setText(
            state
        )

        label.setStyleSheet(
            f"""
            color:{colors.get(state, MUTED)};
            font-size:11px;
            font-weight:600;
            """
        )


# ============================================================
# MAIN WINDOW
# ============================================================

class MainWindow(QMainWindow):

    STALE_SECONDS = 1.5
    SYSTEM_STATUS_STALE_SECONDS = 2.5

    def __init__(
        self,
        ros_node
    ):

        super().__init__()

        self.ros_node = (
            ros_node
        )

        self.goal_pose = None

        self.show_global_costmap = False
        self.show_local_costmap = False

        # Rover-follow camera:
        # - centered on base_link
        # - rover heading points toward the top of the display
        # - square viewport represents exactly 5 m x 5 m
        self.follow_rover_camera = False
        self.follow_view_size_m = 5.0

        self.setWindowTitle(
            "Autonomous Rover"
        )

        self.resize(
            1280,
            820
        )

        self.setStyleSheet(
            STYLESHEET
        )

        central = QWidget()

        root = QVBoxLayout(
            central
        )

        root.setContentsMargins(
            0,
            0,
            0,
            0
        )

        root.setSpacing(
            0
        )

        # ----------------------------
        # TOP STATUS BAR
        # ----------------------------

        self.top_status = TopStatusBar()
        root.addWidget(
            self.top_status,
            stretch=0
        )

        # Main content below the status bar:
        # map on the left, controls on the right.
        body = QHBoxLayout()
        body.setContentsMargins(
            0,
            0,
            0,
            0
        )
        body.setSpacing(
            0
        )

        # ----------------------------
        # MAP
        # ----------------------------

        self.map_frame = QFrame()
        map_frame = self.map_frame

        map_frame.setObjectName(
            "MapFrame"
        )

        map_layout = QVBoxLayout(
            map_frame
        )

        map_layout.setContentsMargins(
            1,
            1,
            1,
            1
        )

        self.map_view = MapView(
            self.set_goal_from_mouse
        )

        map_layout.addWidget(
            self.map_view
        )

        # Bottom-left red full-stack reset button.
        self.reset_map_btn = QPushButton(
            "↻  RESET ROVER",
            map_frame
        )
        self.reset_map_btn.setObjectName(
            "ResetMapButton"
        )
        self.reset_map_btn.setFixedSize(
            168,
            38
        )
        self.reset_map_btn.setToolTip(
            "Cancel navigation, latch STOP, then restart Serial, Rover Odom, EKF, LiDAR, Cartographer, and Nav2"
        )
        self.reset_map_btn.clicked.connect(
            self.on_reset_map_clicked
        )
        self.reset_map_btn.raise_()

        # ----------------------------
        # SIDE PANEL
        # ----------------------------

        self.side_panel = SidePanel(
            self.on_stop_clicked,
            self.on_cancel_clicked,
            self.on_global_costmap_toggled,
            self.on_local_costmap_toggled,
            self.on_follow_rover_toggled
        )

        body.addWidget(
            map_frame,
            stretch=1
        )

        body.addWidget(
            self.side_panel,
            stretch=0
        )

        root.addLayout(
            body,
            stretch=1
        )

        self.setCentralWidget(
            central
        )

        # ----------------------------
        # TIMER
        # ----------------------------

        self.timer = QTimer()

        self.timer.timeout.connect(
            self.update_ros
        )

        self.timer.start(
            50
        )

    # ========================================================
    # MAP STALE CHECK
    # ========================================================

    def map_is_stale(
        self
    ):

        last_time = (
            self.ros_node.last_map_time
        )

        if last_time is None:

            return True

        return (
            time.monotonic()
            - last_time
        ) >= self.STALE_SECONDS

    # ========================================================
    # UPDATE LOOP
    # ========================================================

    def update_ros(
        self
    ):

        # Successful goal reached
        if self.ros_node.goal_completed:

            self.goal_pose = None

            self.ros_node.latest_path = None

            self.side_panel.goal_label.setText(
                "reached"
            )

            self.ros_node.goal_completed = False

        # While STOP is latched, continuously command zero velocity.
        # This makes the visual STOP state match the actual control state.
        if self.side_panel.stop_btn.isChecked():

            self.ros_node.publish_zero_velocity()

        self._position_reset_button()
        self._handle_reset_status()

        self.refresh_side_panel()

        if (
            self.ros_node.latest_map
            is not None
        ):

            self.draw_map(
                self.ros_node.latest_map
            )

    # ========================================================
    # SIDE PANEL REFRESH
    # ========================================================

    def refresh_side_panel(
        self
    ):

        now = time.monotonic()

        def freshness(
            last_time
        ):

            if last_time is None:

                return "WAITING"

            if (
                now - last_time
                < self.STALE_SECONDS
            ):

                return "OK"

            return "STALE"

        # ----------------------------
        # ROVER SYSTEM STATUS
        # ----------------------------

        status_age = None

        if self.ros_node.last_system_status_time is not None:
            status_age = (
                now
                - self.ros_node.last_system_status_time
            )

        manager_online = (
            status_age is not None
            and status_age < self.SYSTEM_STATUS_STALE_SECONDS
        )

        if manager_online:
            system = self.ros_node.system_status

            self.top_status.set_status(
                self.top_status.system_overall,
                system.get("overall", "WAITING")
            )

            self.top_status.set_status(
                self.top_status.serial_status,
                system.get("serial", "WAITING")
            )

            self.top_status.set_status(
                self.top_status.odom_status,
                system.get("odom", "WAITING")
            )

            self.top_status.set_status(
                self.top_status.ekf_status,
                system.get("ekf", "WAITING")
            )

            self.top_status.set_status(
                self.top_status.lidar_status,
                system.get("lidar", "WAITING")
            )

            self.top_status.set_status(
                self.top_status.slam_status,
                system.get("slam", "WAITING")
            )

            self.top_status.set_status(
                self.top_status.nav2_status,
                system.get("nav2", "WAITING")
            )

        else:
            # If the manager heartbeat disappears, the UI should not
            # leave old green statuses on screen.
            stale_state = (
                "WAITING"
                if self.ros_node.last_system_status_time is None
                else "OFFLINE"
            )

            self.top_status.set_status(
                self.top_status.system_overall,
                stale_state
            )
            self.top_status.set_status(
                self.top_status.serial_status,
                stale_state
            )
            self.top_status.set_status(
                self.top_status.odom_status,
                stale_state
            )
            self.top_status.set_status(
                self.top_status.ekf_status,
                stale_state
            )
            self.top_status.set_status(
                self.top_status.lidar_status,
                stale_state
            )
            self.top_status.set_status(
                self.top_status.slam_status,
                stale_state
            )
            self.top_status.set_status(
                self.top_status.nav2_status,
                stale_state
            )

        # ----------------------------
        # LINK STATUS
        # ----------------------------

        self.top_status.set_status(
            self.top_status.map_status,
            freshness(
                self.ros_node.last_map_time
            )
        )

        self.top_status.set_status(
            self.top_status.path_status,
            freshness(
                self.ros_node.last_path_time
            )
        )

        pose = (
            self.ros_node.get_rover_pose()
        )

        self.top_status.set_status(
            self.top_status.pose_status,
            "OK" if pose else "WAITING"
        )

        # ----------------------------
        # NAV2 feedback
        # ----------------------------

        self.side_panel.nav_state.setText(
            self.ros_node.navigation_state
        )

        if self.side_panel.stop_btn.isChecked():
            self.side_panel.nav_state.setStyleSheet(
                f"color:{DANGER_HOVER}; font-family:{MONO}; font-size:13px; font-weight:700;"
            )
        else:
            self.side_panel.nav_state.setStyleSheet(
                f"color:{TEXT}; font-family:{MONO}; font-size:13px;"
            )

        if (
            self.ros_node.distance_remaining
            is None
        ):

            self.side_panel.nav_distance.setText(
                "--"
            )

        else:

            self.side_panel.nav_distance.setText(
                f"{self.ros_node.distance_remaining:.2f} m"
            )

        # ----------------------------
        # POSE
        # ----------------------------

        if pose:

            x, y, yaw = pose

            self.side_panel.pose_x.setText(
                f"{x:+.3f} m"
            )

            self.side_panel.pose_y.setText(
                f"{y:+.3f} m"
            )

            self.side_panel.pose_yaw.setText(
                f"{math.degrees(yaw):+.1f}°"
            )

        else:

            self.side_panel.pose_x.setText(
                "--"
            )

            self.side_panel.pose_y.setText(
                "--"
            )

            self.side_panel.pose_yaw.setText(
                "--"
            )

        # ----------------------------
        # FINAL COMMAND VELOCITY
        # ----------------------------

        velocity_fresh = (
            self.ros_node.last_cmd_vel_time is not None
            and
            now - self.ros_node.last_cmd_vel_time
            < self.STALE_SECONDS
        )

        if velocity_fresh:

            self.side_panel.forward_vel.setText(
                f"{self.ros_node.forward_velocity:+.3f} m/s"
            )

            self.side_panel.angular_vel.setText(
                f"{self.ros_node.angular_velocity:+.3f} rad/s"
            )

        else:

            self.side_panel.forward_vel.setText(
                "--"
            )

            self.side_panel.angular_vel.setText(
                "--"
            )

    # ========================================================
    # FULL ROVER RESET
    # ========================================================

    def _position_reset_button(self):

        margin = 14

        x = margin
        y = max(
            margin,
            self.map_frame.height()
            - self.reset_map_btn.height()
            - margin
        )

        self.reset_map_btn.move(
            x,
            y
        )
        self.reset_map_btn.raise_()

    def on_reset_map_clicked(self):

        if self.ros_node.reset_in_progress:
            return

        # Safe state first: cancel Nav2 and latch STOP.
        self.ros_node.cancel_navigation()
        self.side_panel.stop_btn.setChecked(True)
        self.ros_node.navigation_state = "RESETTING ROVER"
        self.side_panel.stop_btn.setText(
            "■  STOP ACTIVE — CLICK TO RELEASE"
        )

        # Immediately clear old navigation/map-derived UI state so the user
        # never mistakes stale data for the new SLAM session.
        self.goal_pose = None
        self.ros_node.latest_map = None
        self.ros_node.latest_path = None
        self.ros_node.latest_global_costmap = None
        self.ros_node.latest_local_costmap = None
        self.ros_node.last_map_time = None
        self.ros_node.last_path_time = None
        self.ros_node.last_global_costmap_time = None
        self.ros_node.last_local_costmap_time = None

        # A fresh SLAM session should start with the normal full-map camera.
        self.follow_rover_camera = False
        self.map_view.set_follow_mode(
            False
        )
        self.map_view.reset_view()
        self.side_panel.follow_rover_btn.setChecked(
            False
        )
        self.side_panel.follow_rover_btn.setText(
            "◎  FOLLOW ROVER — 5 m"
        )

        self.side_panel.goal_label.setText(
            "none (resetting rover)"
        )

        self.reset_map_btn.setEnabled(False)
        self.reset_map_btn.setText(
            "↻  RESETTING ROVER..."
        )

        started = self.ros_node.request_slam_reset()

        if not started:
            # request_slam_reset() sets reset_finished/reset_message.
            # _handle_reset_status() will show the temporary failure state
            # and automatically restore the normal button after 3 seconds.
            self.reset_map_btn.setEnabled(True)

    def _handle_reset_status(self):

        if not self.ros_node.reset_finished:
            return

        self.ros_node.reset_finished = False

        self.reset_map_btn.setEnabled(True)

        if self.ros_node.reset_success:
            # The manager has restarted the full rover runtime stack. A fresh /map
            # and fresh Nav2 costmaps will repopulate the UI. STOP remains latched.
            self.ros_node.latest_map = None
            self.ros_node.last_map_time = None
            self.ros_node.navigation_state = "ROVER RESET — STOPPED"
            self.reset_map_btn.setText(
                "✓  ROVER RESET — STOP HELD"
            )
            self.side_panel.goal_label.setText(
                "none (fresh stack)"
            )

            # Return the button label after a short, visible confirmation.
            QTimer.singleShot(
                1800,
                lambda: self.reset_map_btn.setText(
                    "↻  RESET ROVER"
                )
            )

        else:
            self.ros_node.navigation_state = "RESET FAILED — STOPPED"

            if self.ros_node.reset_message == "RESET SERVICE OFFLINE":
                self.reset_map_btn.setText(
                    "!  RESET SERVICE OFFLINE"
                )
            else:
                self.reset_map_btn.setText(
                    "!  RESET FAILED"
                )

            self.side_panel.goal_label.setText(
                self.ros_node.reset_message
            )

            # Failure/offline status is temporary so an old error does not
            # look like a current rover fault.
            QTimer.singleShot(
                3000,
                lambda: self.reset_map_btn.setText(
                    "↻  RESET ROVER"
                )
            )

    # ========================================================
    # STOP
    # ========================================================

    def on_stop_clicked(
        self
    ):

        active = (
            self.side_panel.stop_btn.isChecked()
        )

        if active:

            self.ros_node.cancel_navigation()
            self.ros_node.navigation_state = "STOPPED"

            self.goal_pose = None
            self.ros_node.latest_path = None

            self.side_panel.goal_label.setText(
                "none (STOP active)"
            )

            self.side_panel.stop_btn.setText(
                "■  STOP ACTIVE — CLICK TO RELEASE"
            )

        else:

            self.ros_node.navigation_state = "IDLE"

            self.side_panel.stop_btn.setText(
                "■  STOP"
            )

    # ========================================================
    # COSTMAP TOGGLES
    # ========================================================

    def on_global_costmap_toggled(
        self,
        checked
    ):

        self.show_global_costmap = bool(
            checked
        )

    def on_local_costmap_toggled(
        self,
        checked
    ):

        self.show_local_costmap = bool(
            checked
        )

    # ========================================================
    # ROVER FOLLOW CAMERA
    # ========================================================

    def on_follow_rover_toggled(
        self,
        checked
    ):

        self.follow_rover_camera = bool(
            checked
        )

        self.map_view.set_follow_mode(
            self.follow_rover_camera
        )

        if self.follow_rover_camera:
            # The fixed camera replaces free pan/zoom while enabled.
            self.map_view.reset_view()

            self.side_panel.follow_rover_btn.setText(
                "◎  FOLLOWING ROVER — 5 m"
            )

        else:
            # Return to the normal full-map fit view.
            self.map_view.reset_view()

            self.side_panel.follow_rover_btn.setText(
                "◎  FOLLOW ROVER — 5 m"
            )

    # ========================================================
    # CANCEL
    # ========================================================

    def on_cancel_clicked(
        self
    ):

        self.ros_node.cancel_navigation()

        self.goal_pose = None
        self.ros_node.latest_path = None

        self.side_panel.goal_label.setText(
            "none"
        )

    # ========================================================
    # MAP DISPLAY FIT
    # ========================================================

    def map_fit(
        self,
        msg
    ):

        width = msg.info.width
        height = msg.info.height

        label_w = (
            self.map_view.width()
        )

        label_h = (
            self.map_view.height()
        )

        # Base scale fits the complete map in the widget.
        base_scale = min(
            label_w / width,
            label_h / height
        )

        # Mouse-wheel zoom is applied on top of the normal fit-to-view scale.
        scale = (
            base_scale
            * self.map_view.zoom_factor
        )

        displayed_w = (
            width * scale
        )

        displayed_h = (
            height * scale
        )

        # Keep the map centered by default, then apply right-drag pan in
        # screen pixels.
        offset_x = (
            (label_w - displayed_w)
            / 2.0
            + self.map_view.pan_x
        )

        offset_y = (
            (label_h - displayed_h)
            / 2.0
            + self.map_view.pan_y
        )

        return (
            scale,
            offset_x,
            offset_y
        )

    # ========================================================
    # CAMERA COORDINATE HELPERS
    # ========================================================

    def _follow_view_geometry(
        self
    ):
        """Return (left, top, side, center_x, center_y, pixels_per_meter)."""

        side = float(
            min(
                self.map_view.width(),
                self.map_view.height()
            )
        )

        left = (
            self.map_view.width() - side
        ) / 2.0

        top = (
            self.map_view.height() - side
        ) / 2.0

        center_x = (
            left + side / 2.0
        )

        center_y = (
            top + side / 2.0
        )

        pixels_per_meter = (
            side / self.follow_view_size_m
        )

        return (
            left,
            top,
            side,
            center_x,
            center_y,
            pixels_per_meter
        )

    def _screen_to_world(
        self,
        pos,
        msg
    ):
        """Convert a mouse position in the map widget to map-frame x/y."""

        if self.follow_rover_camera:

            pose = self.ros_node.get_rover_pose()

            if pose is None:
                return None

            rover_x, rover_y, rover_yaw = pose

            (
                left,
                top,
                side,
                center_x,
                center_y,
                pixels_per_meter
            ) = self._follow_view_geometry()

            # Ignore clicks outside the actual 5 m x 5 m square.
            if not (
                left <= pos.x() <= left + side
                and
                top <= pos.y() <= top + side
            ):
                return None

            screen_dx = (
                pos.x() - center_x
            )

            screen_dy = (
                pos.y() - center_y
            )

            # In the heading-up view:
            #   screen up    = rover forward
            #   screen left  = rover left
            forward = (
                -screen_dy
                / pixels_per_meter
            )

            left_m = (
                -screen_dx
                / pixels_per_meter
            )

            c = math.cos(
                rover_yaw
            )

            s = math.sin(
                rover_yaw
            )

            world_dx = (
                forward * c
                - left_m * s
            )

            world_dy = (
                forward * s
                + left_m * c
            )

            return (
                rover_x + world_dx,
                rover_y + world_dy
            )

        # Normal north-up free camera.
        scale, offset_x, offset_y = (
            self.map_fit(msg)
        )

        if scale <= 0.0:
            return None

        raw_x = (
            pos.x() - offset_x
        ) / scale

        raw_y = (
            pos.y() - offset_y
        ) / scale

        if not (
            0.0 <= raw_x < msg.info.width
            and
            0.0 <= raw_y < msg.info.height
        ):
            return None

        return (
            msg.info.origin.position.x
            + raw_x * msg.info.resolution,
            msg.info.origin.position.y
            + (
                msg.info.height - 1 - raw_y
            ) * msg.info.resolution
        )

    def _world_to_render(
        self,
        x,
        y,
        msg
    ):

        return QPointF(
            (
                (x - msg.info.origin.position.x)
                / msg.info.resolution
                * RENDER_SCALE
            ),
            (
                (
                    msg.info.height - 1
                    - (
                        y - msg.info.origin.position.y
                    ) / msg.info.resolution
                )
                * RENDER_SCALE
            )
        )

    # ========================================================
    # GOAL SELECTION
    # ========================================================

    def set_goal_from_mouse(
        self,
        start_pos,
        end_pos
    ):

        if (
            self.side_panel.stop_btn.isChecked()
        ):

            return

        if self.map_is_stale():

            self.side_panel.goal_label.setText(
                "blocked — map data stale"
            )

            return

        msg = (
            self.ros_node.latest_map
        )

        if msg is None:

            return

        start_world = self._screen_to_world(
            start_pos,
            msg
        )

        end_world = self._screen_to_world(
            end_pos,
            msg
        )

        if (
            start_world is None
            or end_world is None
        ):
            return

        map_x, map_y = (
            start_world
        )

        end_x, end_y = (
            end_world
        )

        dx = (
            end_x - map_x
        )

        dy = (
            end_y - map_y
        )

        # A tiny click with no meaningful drag keeps a neutral heading.
        if (
            math.hypot(
                dx,
                dy
            )
            < 0.02
        ):

            yaw = 0.0

        else:

            yaw = math.atan2(
                dy,
                dx
            )

        self.goal_pose = (
            map_x,
            map_y,
            yaw
        )

        self.side_panel.goal_label.setText(
            f"x={map_x:.2f} m\n"
            f"y={map_y:.2f} m\n"
            f"heading={math.degrees(yaw):.0f}°"
        )

        self.ros_node.send_nav_goal(
            map_x,
            map_y,
            yaw
        )

    # ========================================================
    # DRAW MAP
    # ========================================================

    def draw_map(
        self,
        msg
    ):

        raw_width = (
            msg.info.width
        )

        raw_height = (
            msg.info.height
        )

        resolution = (
            msg.info.resolution
        )

        origin_x = (
            msg.info.origin.position.x
        )

        origin_y = (
            msg.info.origin.position.y
        )

        # High-resolution dimensions
        render_width = (
            raw_width * RENDER_SCALE
        )

        render_height = (
            raw_height * RENDER_SCALE
        )

        image = self._grid_to_image(
            msg,
            raw_width,
            raw_height
        )

        painter = QPainter(
            image
        )

        painter.setRenderHint(
            QPainter.RenderHint.Antialiasing,
            True
        )

        stale = (
            self.map_is_stale()
        )

        if not stale:

            if self.show_global_costmap:
                self._draw_costmap_overlay(
                    painter,
                    self.ros_node.latest_global_costmap,
                    msg,
                    GLOBAL_COSTMAP_GRADIENT,
                    alpha_min=8,
                    alpha_max=85,
                )

            if self.show_local_costmap:
                self._draw_costmap_overlay(
                    painter,
                    self.ros_node.latest_local_costmap,
                    msg,
                    LOCAL_COSTMAP_GRADIENT,
                    alpha_min=18,
                    alpha_max=145,
                )

            # Keep real Cartographer walls visually dominant over the
            # translucent Nav2 costmap overlays. This redraws only strongly
            # occupied SLAM cells, so inflation remains visible around walls.
            if (
                self.show_global_costmap
                or self.show_local_costmap
            ):
                self._draw_map_walls_overlay(
                    painter,
                    msg
                )

            self._draw_path(
                painter,
                origin_x,
                origin_y,
                resolution,
                raw_height
            )

            self._draw_rover(
                painter,
                origin_x,
                origin_y,
                resolution,
                raw_height
            )

            self._draw_goal(
                painter,
                origin_x,
                origin_y,
                resolution,
                raw_height
            )

            self._draw_drag_preview(
                painter,
                msg
            )

            if self.side_panel.stop_btn.isChecked():
                self._draw_stop_overlay(
                    painter,
                    render_width,
                    render_height
                )

        else:

            self._draw_stale_overlay(
                painter,
                render_width,
                render_height
            )

        painter.end()

        # ----------------------------------------------------
        # FINAL VIEWPORT COMPOSITION
        # ----------------------------------------------------

        viewport = QPixmap(
            self.map_view.size()
        )

        viewport.fill(
            QColor("#14161A")
        )

        viewport_painter = QPainter(
            viewport
        )

        viewport_painter.setRenderHint(
            QPainter.RenderHint.SmoothPixmapTransform,
            True
        )

        source_pixmap = QPixmap.fromImage(
            image
        )

        if self.follow_rover_camera:

            pose = self.ros_node.get_rover_pose()

            if pose is not None:

                rover_x, rover_y, rover_yaw = pose

                (
                    view_left,
                    view_top,
                    view_side,
                    center_x,
                    center_y,
                    pixels_per_meter
                ) = self._follow_view_geometry()

                # Exact 5 m x 5 m camera square. Anything outside is hidden.
                viewport_painter.setClipRect(
                    int(round(view_left)),
                    int(round(view_top)),
                    int(round(view_side)),
                    int(round(view_side))
                )

                rover_render_x = (
                    (
                        rover_x - origin_x
                    )
                    / resolution
                    * RENDER_SCALE
                )

                rover_render_y = (
                    (
                        raw_height - 1
                        - (
                            rover_y - origin_y
                        ) / resolution
                    )
                    * RENDER_SCALE
                )

                # Source image pixels per physical meter.
                source_px_per_meter = (
                    RENDER_SCALE
                    / resolution
                )

                camera_scale = (
                    pixels_per_meter
                    / source_px_per_meter
                )

                viewport_painter.translate(
                    center_x,
                    center_y
                )

                # yaw=0 points right in the map image. Rotate it -90 degrees
                # so the rover's forward direction points toward screen-up.
                viewport_painter.rotate(
                    math.degrees(rover_yaw)
                    - 90.0
                )

                viewport_painter.scale(
                    camera_scale,
                    camera_scale
                )

                viewport_painter.translate(
                    -rover_render_x,
                    -rover_render_y
                )

                viewport_painter.drawPixmap(
                    0,
                    0,
                    source_pixmap
                )

                viewport_painter.resetTransform()
                viewport_painter.setClipping(
                    False
                )

                # Border + tiny mode label so it is obvious that the camera is
                # locked to the rover instead of the normal north-up map.
                follow_pen = QPen(
                    QColor(AMBER)
                )
                follow_pen.setWidthF(
                    1.0
                )
                viewport_painter.setPen(
                    follow_pen
                )
                viewport_painter.setBrush(
                    Qt.BrushStyle.NoBrush
                )
                viewport_painter.drawRect(
                    int(round(view_left)),
                    int(round(view_top)),
                    max(
                        1,
                        int(round(view_side)) - 1
                    ),
                    max(
                        1,
                        int(round(view_side)) - 1
                    )
                )

                viewport_painter.setPen(
                    QColor(TEXT)
                )

                follow_font = QFont(
                    "Segoe UI"
                )
                follow_font.setBold(
                    True
                )
                follow_font.setPointSizeF(
                    9.0
                )
                viewport_painter.setFont(
                    follow_font
                )

                viewport_painter.drawText(
                    int(round(view_left + 10)),
                    int(round(view_top + 20)),
                    "FOLLOW ROVER   •   5 m × 5 m   •   HEADING UP"
                )

        else:

            # Normal north-up camera with mouse-wheel zoom and right-drag pan.
            scale, offset_x, offset_y = (
                self.map_fit(msg)
            )

            displayed_w = max(
                1,
                int(round(raw_width * scale))
            )

            displayed_h = max(
                1,
                int(round(raw_height * scale))
            )

            scaled_map = (
                source_pixmap
                .scaled(
                    displayed_w,
                    displayed_h,
                    Qt.AspectRatioMode.IgnoreAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )

            viewport_painter.drawPixmap(
                int(round(offset_x)),
                int(round(offset_y)),
                scaled_map
            )

        viewport_painter.end()

        self.map_view.setPixmap(
            viewport
        )

    # ========================================================
    # HIGH-RES OCCUPANCY GRID
    # ========================================================

    def _grid_to_image(
        self,
        msg,
        width,
        height
    ):

        data = np.array(
            msg.data,
            dtype=np.int16
        ).reshape(
            (height, width)
        )

        # ----------------------------------------------------
        # RVIZ-LIKE MAP APPEARANCE
        # ----------------------------------------------------
        #
        # RViz's normal occupancy-map view is probability based:
        #   free     -> white
        #   occupied -> black
        #   unknown  -> mid gray
        #
        # Keeping Cartographer's intermediate probabilities visible gives
        # walls much more of the familiar RViz appearance than converting
        # every cell >= 50 directly to the same dark value.
        # ----------------------------------------------------

        gray = np.full(
            (height, width),
            128,
            dtype=np.uint8
        )

        known = (
            data >= 0
        )

        if np.any(known):
            occupancy = np.clip(
                data[known],
                0,
                100
            ).astype(np.float32)

            gray[known] = np.clip(
                255.0
                - occupancy * 2.55,
                0,
                255
            ).astype(np.uint8)

        # ROS OccupancyGrid row 0 is the bottom of the map;
        # image row 0 is the top.
        gray = np.flipud(
            gray
        )

        # Render internally at higher resolution first so overlays, rover
        # footprint, path and headings stay clean.
        gray = np.repeat(
            gray,
            RENDER_SCALE,
            axis=0
        )

        gray = np.repeat(
            gray,
            RENDER_SCALE,
            axis=1
        )

        rgb = np.repeat(
            gray[:, :, None],
            3,
            axis=2
        )

        rgb = np.ascontiguousarray(
            rgb
        )

        self._image_buffer = rgb

        render_height = (
            height * RENDER_SCALE
        )

        render_width = (
            width * RENDER_SCALE
        )

        image = QImage(
            self._image_buffer.data,
            render_width,
            render_height,
            render_width * 3,
            QImage.Format.Format_RGB888
        )

        return image.copy()

    # ========================================================
    # MAP WALL PRIORITY OVERLAY
    # ========================================================

    def _draw_map_walls_overlay(
        self,
        painter,
        msg,
        occupied_threshold=65
    ):
        """
        Redraw strongly occupied Cartographer cells above the Nav2 costmap
        overlays so real mapped walls stay crisp and unmistakable.

        The base occupancy map is already drawn underneath. This pass only
        redraws high-confidence occupied cells; free/unknown/intermediate
        cells remain untouched so the global/local cost gradients are still
        clearly visible around obstacles.
        """

        width = msg.info.width
        height = msg.info.height

        if width <= 0 or height <= 0:
            return

        data = np.array(
            msg.data,
            dtype=np.int16
        ).reshape(
            (height, width)
        )

        ys, xs = np.where(
            data >= occupied_threshold
        )

        if len(xs) == 0:
            return

        painter.save()
        painter.setPen(
            Qt.PenStyle.NoPen
        )
        painter.setBrush(
            QColor(18, 18, 18, 245)
        )

        cell_px = RENDER_SCALE

        for row, col in zip(
            ys.tolist(),
            xs.tolist()
        ):
            render_row = (
                height - 1 - row
            )

            painter.drawRect(
                col * RENDER_SCALE,
                render_row * RENDER_SCALE,
                cell_px,
                cell_px
            )

        painter.restore()

    # ========================================================
    # COSTMAP OVERLAYS
    # ========================================================

    def _draw_costmap_overlay(
        self,
        painter,
        costmap_msg,
        base_map_msg,
        gradient,
        alpha_min=18,
        alpha_max=160,
    ):

        if costmap_msg is None:
            return

        width = costmap_msg.info.width
        height = costmap_msg.info.height

        if width <= 0 or height <= 0:
            return

        data = np.array(
            costmap_msg.data,
            dtype=np.int16
        ).reshape(
            (height, width)
        )

        resolution = costmap_msg.info.resolution
        origin = costmap_msg.info.origin

        # ----------------------------------------------------
        # FRAME ALIGNMENT -- THIS FIXES THE LOCAL COSTMAP OFFSET
        # ----------------------------------------------------
        #
        # Global costmap:
        #     frame_id = map
        #
        # Local rolling costmap in this rover:
        #     frame_id = odom
        #
        # The old UI treated both grids as if their origin coordinates were
        # already expressed in map. RViz does not do that -- it transforms
        # the odom-frame local costmap into map through TF.
        # ----------------------------------------------------

        base_frame = (
            base_map_msg.header.frame_id
            or "map"
        ).lstrip("/")

        costmap_frame = (
            costmap_msg.header.frame_id
            or base_frame
        ).lstrip("/")

        frame_tf = self.ros_node.get_frame_transform_2d(
            base_frame,
            costmap_frame,
            costmap_msg.header.stamp
        )

        if frame_tf is None:
            # Never draw a grid in the wrong frame. If TF is briefly missing,
            # simply skip this overlay for this refresh.
            return

        frame_tx, frame_ty, frame_yaw = frame_tf

        frame_cos = math.cos(
            frame_yaw
        )
        frame_sin = math.sin(
            frame_yaw
        )

        # OccupancyGrid origin can also carry its own rotation inside the
        # source frame.
        grid_yaw = quaternion_to_yaw(
            origin.orientation
        )

        grid_cos = math.cos(
            grid_yaw
        )
        grid_sin = math.sin(
            grid_yaw
        )

        total_yaw = (
            frame_yaw + grid_yaw
        )

        base_origin_x = (
            base_map_msg.info.origin.position.x
        )
        base_origin_y = (
            base_map_msg.info.origin.position.y
        )
        base_resolution = (
            base_map_msg.info.resolution
        )
        base_height = (
            base_map_msg.info.height
        )

        def source_to_base(
            source_x,
            source_y
        ):
            """Apply map<-costmap_frame TF."""
            return (
                frame_tx
                + source_x * frame_cos
                - source_y * frame_sin,
                frame_ty
                + source_x * frame_sin
                + source_y * frame_cos,
            )

        def grid_to_source(
            grid_x,
            grid_y
        ):
            """Apply the OccupancyGrid origin pose."""
            return (
                origin.position.x
                + grid_x * grid_cos
                - grid_y * grid_sin,
                origin.position.y
                + grid_x * grid_sin
                + grid_y * grid_cos,
            )

        def base_to_render(
            x,
            y
        ):
            return QPointF(
                (
                    (x - base_origin_x)
                    / base_resolution
                    * RENDER_SCALE
                ),
                (
                    (
                        base_height - 1
                        - (y - base_origin_y)
                        / base_resolution
                    )
                    * RENDER_SCALE
                )
            )

        painter.save()
        painter.setPen(
            Qt.PenStyle.NoPen
        )

        # Nav2 OccupancyGrid costmap messages use -1 for unknown,
        # 0 for free, and positive values for increasing cost.
        # Leave unknown/free transparent, matching the way the overlay is
        # normally used on top of a map in RViz.
        ys, xs = np.where(
            data > 0
        )

        # If there is essentially no rotation between the costmap grid and
        # the map, draw fast axis-aligned cells. Global costmaps normally
        # take this path. The rolling local costmap can take the polygon path
        # when map<-odom contains a heading correction.
        axis_aligned = (
            abs(math.sin(total_yaw))
            < 1.0e-5
        )

        cell_px = max(
            1.0,
            resolution
            / base_resolution
            * RENDER_SCALE
        )

        for row, col in zip(
            ys.tolist(),
            xs.tolist()
        ):

            cost = int(
                data[row, col]
            )

            normalized = max(
                0.0,
                min(
                    1.0,
                    cost / 100.0
                )
            )

            # Visual hierarchy:
            #   Cartographer map walls > local costmap > global costmap.
            # Each overlay supplies its own alpha range so the global layer
            # stays subdued while the local rolling costmap stands out.
            alpha = int(
                alpha_min
                + (alpha_max - alpha_min)
                * (normalized ** 0.85)
            )

            color = gradient_color(
                normalized,
                gradient
            )
            color.setAlpha(
                alpha
            )
            painter.setBrush(
                color
            )

            center_grid_x = (
                (col + 0.5)
                * resolution
            )
            center_grid_y = (
                (row + 0.5)
                * resolution
            )

            source_x, source_y = grid_to_source(
                center_grid_x,
                center_grid_y
            )

            base_x, base_y = source_to_base(
                source_x,
                source_y
            )

            center = base_to_render(
                base_x,
                base_y
            )

            if axis_aligned:
                painter.drawRect(
                    int(round(center.x() - cell_px / 2.0)),
                    int(round(center.y() - cell_px / 2.0)),
                    int(math.ceil(cell_px)),
                    int(math.ceil(cell_px)),
                )
                continue

            # For a rotated odom->map transform, transform the actual four
            # cell corners. This prevents the local rolling costmap from
            # looking shifted/skewed when SLAM's map->odom transform rotates.
            x0 = col * resolution
            x1 = (col + 1) * resolution
            y0 = row * resolution
            y1 = (row + 1) * resolution

            polygon = []

            for gx, gy in (
                (x0, y0),
                (x1, y0),
                (x1, y1),
                (x0, y1),
            ):
                sx, sy = grid_to_source(
                    gx,
                    gy
                )

                bx, by = source_to_base(
                    sx,
                    sy
                )

                polygon.append(
                    base_to_render(
                        bx,
                        by
                    )
                )

            painter.drawPolygon(
                QPolygonF(polygon)
            )

        painter.restore()

    # ========================================================
    # STOP OVERLAY
    # ========================================================

    def _draw_stop_overlay(
        self,
        painter,
        width,
        height
    ):

        painter.save()

        # Slightly dim the map without hiding the rover/costmap context.
        painter.fillRect(
            0,
            0,
            width,
            height,
            QColor(
                120,
                0,
                0,
                34
            )
        )

        border_pen = QPen(
            QColor("#FF625A")
        )
        border_pen.setWidthF(
            1.5 * RENDER_SCALE
        )
        painter.setPen(
            border_pen
        )
        painter.setBrush(
            Qt.BrushStyle.NoBrush
        )
        inset = int(
            1.5 * RENDER_SCALE
        )
        painter.drawRect(
            inset,
            inset,
            width - 2 * inset,
            height - 2 * inset
        )

        banner_h = int(
            11 * RENDER_SCALE
        )
        painter.fillRect(
            0,
            0,
            width,
            banner_h,
            QColor(
                105,
                15,
                15,
                225
            )
        )

        font = QFont(
            "Segoe UI"
        )
        font.setBold(
            True
        )
        font.setPointSizeF(
            10.5
        )
        painter.setFont(
            font
        )
        painter.setPen(
            QColor("#FFF3F2")
        )

        painter.drawText(
            0,
            0,
            width,
            banner_h,
            int(
                Qt.AlignmentFlag.AlignCenter
            ),
            "■  STOPPED   •   ZERO VELOCITY HOLD"
        )

        painter.restore()

    # ========================================================
    # GLOBAL PATH
    # ========================================================

    def _draw_path(
        self,
        painter,
        origin_x,
        origin_y,
        resolution,
        raw_height
    ):

        path = (
            self.ros_node.latest_path
        )

        if (
            path is None
            or len(path.poses) < 2
        ):

            return

        pen = QPen(
            QColor("#43D17A")
        )

        pen.setWidthF(
            1.2 * RENDER_SCALE
        )

        pen.setCapStyle(
            Qt.PenCapStyle.RoundCap
        )

        pen.setJoinStyle(
            Qt.PenJoinStyle.RoundJoin
        )

        painter.setPen(
            pen
        )

        points = []

        for pose_stamped in path.poses:

            px = (
                (
                    pose_stamped.pose.position.x
                    - origin_x
                )
                / resolution
                * RENDER_SCALE
            )

            py = (
                (
                    raw_height - 1
                    - (
                        pose_stamped.pose.position.y
                        - origin_y
                    ) / resolution
                )
                * RENDER_SCALE
            )

            points.append(
                QPointF(
                    px,
                    py
                )
            )

        for a, b in zip(
            points,
            points[1:]
        ):

            painter.drawLine(
                a,
                b
            )

    # ========================================================
    # ROVER
    # TRUE 16 cm x 16 cm
    # ========================================================

    def _draw_rover(
        self,
        painter,
        origin_x,
        origin_y,
        resolution,
        raw_height
    ):

        pose = (
            self.ros_node.get_rover_pose()
        )

        if pose is None:

            return

        rover_x, rover_y, yaw = (
            pose
        )

        px = (
            (
                rover_x - origin_x
            )
            / resolution
            * RENDER_SCALE
        )

        py = (
            (
                raw_height - 1
                - (
                    rover_y - origin_y
                ) / resolution
            )
            * RENDER_SCALE
        )

        # True physical rover footprint
        rover_size_px = (
            0.16
            / resolution
            * RENDER_SCALE
        )

        half = (
            rover_size_px / 2.0
        )

        local_corners = [
            (half, half),
            (half, -half),
            (-half, -half),
            (-half, half),
        ]

        corners = []

        for (
            local_x,
            local_y
        ) in local_corners:

            rx = (
                local_x * math.cos(yaw)
                - local_y * math.sin(yaw)
            )

            ry = (
                local_x * math.sin(yaw)
                + local_y * math.cos(yaw)
            )

            corners.append(
                QPointF(
                    px + rx,
                    py - ry
                )
            )

        # ----------------------------
        # RED BODY
        # ----------------------------

        painter.setBrush(
            QColor("#E53935")
        )

        body_pen = QPen(
            QColor("#8E1717")
        )

        body_pen.setWidthF(
            0.6 * RENDER_SCALE
        )

        painter.setPen(
            body_pen
        )

        painter.drawPolygon(
            QPolygonF(corners)
        )

        # ----------------------------
        # DARK HEADING ARROW
        # ----------------------------

        heading_length = (
            rover_size_px * 1.35
        )

        arrow_end_x = (
            px
            + heading_length
            * math.cos(yaw)
        )

        arrow_end_y = (
            py
            - heading_length
            * math.sin(yaw)
        )

        heading_pen = QPen(
            QColor("#111318")
        )

        heading_pen.setWidthF(
            0.90 * RENDER_SCALE
        )

        heading_pen.setCapStyle(
            Qt.PenCapStyle.RoundCap
        )

        painter.setPen(
            heading_pen
        )

        painter.drawLine(
            QPointF(
                px,
                py
            ),
            QPointF(
                arrow_end_x,
                arrow_end_y
            )
        )

        # ----------------------------
        # ARROW HEAD
        # ----------------------------

        head_length = (
            rover_size_px * 0.38
        )

        head_angle = (
            math.radians(28)
        )

        left_x = (
            arrow_end_x
            - head_length
            * math.cos(
                yaw - head_angle
            )
        )

        left_y = (
            arrow_end_y
            + head_length
            * math.sin(
                yaw - head_angle
            )
        )

        right_x = (
            arrow_end_x
            - head_length
            * math.cos(
                yaw + head_angle
            )
        )

        right_y = (
            arrow_end_y
            + head_length
            * math.sin(
                yaw + head_angle
            )
        )

        painter.drawLine(
            QPointF(
                arrow_end_x,
                arrow_end_y
            ),
            QPointF(
                left_x,
                left_y
            )
        )

        painter.drawLine(
            QPointF(
                arrow_end_x,
                arrow_end_y
            ),
            QPointF(
                right_x,
                right_y
            )
        )

    # ========================================================
    # GOAL
    # ========================================================

    def _draw_goal(
        self,
        painter,
        origin_x,
        origin_y,
        resolution,
        raw_height
    ):

        if self.goal_pose is None:

            return

        (
            goal_x,
            goal_y,
            goal_yaw
        ) = self.goal_pose

        gx = (
            (
                goal_x - origin_x
            )
            / resolution
            * RENDER_SCALE
        )

        gy = (
            (
                raw_height - 1
                - (
                    goal_y - origin_y
                ) / resolution
            )
            * RENDER_SCALE
        )

        # ----------------------------
        # GOAL DOT
        # ----------------------------

        goal_radius = (
            1.25 * RENDER_SCALE
        )

        painter.setBrush(
            QColor("#2979FF")
        )

        goal_pen = QPen(
            QColor("#90CAF9")
        )

        goal_pen.setWidthF(
            0.5 * RENDER_SCALE
        )

        painter.setPen(
            goal_pen
        )

        painter.drawEllipse(
            QPointF(
                gx,
                gy
            ),
            goal_radius,
            goal_radius
        )

        # ----------------------------
        # GOAL HEADING
        # ----------------------------

        arrow_length = (
            6.0 * RENDER_SCALE
        )

        end_x = (
            gx
            + arrow_length
            * math.cos(goal_yaw)
        )

        end_y = (
            gy
            - arrow_length
            * math.sin(goal_yaw)
        )

        heading_pen = QPen(
            QColor("#2979FF")
        )

        heading_pen.setWidthF(
            0.75 * RENDER_SCALE
        )

        heading_pen.setCapStyle(
            Qt.PenCapStyle.RoundCap
        )

        painter.setPen(
            heading_pen
        )

        painter.drawLine(
            QPointF(
                gx,
                gy
            ),
            QPointF(
                end_x,
                end_y
            )
        )

        # ----------------------------
        # GOAL ARROW HEAD
        # ----------------------------

        head_length = (
            2.0 * RENDER_SCALE
        )

        head_angle = (
            math.radians(30)
        )

        left_x = (
            end_x
            - head_length
            * math.cos(
                goal_yaw - head_angle
            )
        )

        left_y = (
            end_y
            + head_length
            * math.sin(
                goal_yaw - head_angle
            )
        )

        right_x = (
            end_x
            - head_length
            * math.cos(
                goal_yaw + head_angle
            )
        )

        right_y = (
            end_y
            + head_length
            * math.sin(
                goal_yaw + head_angle
            )
        )

        painter.drawLine(
            QPointF(
                end_x,
                end_y
            ),
            QPointF(
                left_x,
                left_y
            )
        )

        painter.drawLine(
            QPointF(
                end_x,
                end_y
            ),
            QPointF(
                right_x,
                right_y
            )
        )

    # ========================================================
    # DRAG PREVIEW
    # ========================================================

    def _draw_drag_preview(
        self,
        painter,
        msg
    ):

        if not (
            self.map_view.dragging
            and self.map_view.drag_start
            and self.map_view.drag_end
        ):

            return

        start_world = self._screen_to_world(
            self.map_view.drag_start,
            msg
        )

        end_world = self._screen_to_world(
            self.map_view.drag_end,
            msg
        )

        if (
            start_world is None
            or end_world is None
        ):
            return

        start = self._world_to_render(
            start_world[0],
            start_world[1],
            msg
        )

        end = self._world_to_render(
            end_world[0],
            end_world[1],
            msg
        )

        pen = QPen(
            Qt.GlobalColor.cyan
        )

        pen.setWidthF(
            0.75 * RENDER_SCALE
        )

        pen.setCapStyle(
            Qt.PenCapStyle.RoundCap
        )

        painter.setPen(
            pen
        )

        painter.setBrush(
            Qt.GlobalColor.cyan
        )

        painter.drawEllipse(
            start,
            1.5 * RENDER_SCALE,
            1.5 * RENDER_SCALE
        )

        painter.drawLine(
            start,
            end
        )

    # ========================================================
    # STALE MAP OVERLAY
    # ========================================================

    def _draw_stale_overlay(
        self,
        painter,
        width,
        height
    ):

        painter.fillRect(
            0,
            0,
            width,
            height,
            QColor(
                20,
                20,
                20,
                155
            )
        )

        font = QFont(
            "Segoe UI"
        )

        font.setBold(
            True
        )

        font.setPointSizeF(
            14
        )

        painter.setFont(
            font
        )

        painter.setPen(
            QColor("#FFD54F")
        )

        painter.drawText(
            0,
            0,
            width,
            height,
            int(
                Qt.AlignmentFlag.AlignCenter
            ),
            "MAP DATA STALE"
        )


# ============================================================
# MAIN
# ============================================================

def main(
    args=None
):

    rclpy.init(
        args=args
    )

    node = RoverUINode()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    ros_thread.start()

    app = QApplication(
        sys.argv
    )

    app.setFont(
        QFont(
            "Segoe UI",
            10
        )
    )

    window = MainWindow(
        node
    )

    window.show()

    exit_code = (
        app.exec()
    )

    rclpy.shutdown()

    ros_thread.join(
        timeout=1.0
    )

    node.destroy_node()

    sys.exit(
        exit_code
    )


if __name__ == "__main__":
    main()
