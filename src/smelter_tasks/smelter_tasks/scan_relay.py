"""Normalize Fortress scan frames and enforce scan freshness on Humble."""

from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ScanRelay(Node):
    def __init__(self):
        super().__init__("scan_relay")
        self._scan_at = None
        self._velocity_at = None
        self._requested = Twist()
        self._drop_scan = False
        self._inhibit_motion = False
        self._scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self._velocity = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(LaserScan, "/scan_raw", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_requested", self._on_velocity, 10)
        self.create_subscription(Bool, "/sim/drop_scan", self._on_fault, 10)
        self.create_subscription(Bool, "/sim/inhibit_motion", self._on_inhibit, 10)
        self.create_timer(0.05, self._publish_velocity)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, message):
        if self._drop_scan or not message.ranges:
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if stamp <= 0 or not any(message.range_min <= r <= message.range_max or r == float("inf")
                                for r in message.ranges):
            return
        message.header.frame_id = "laser_link"
        self._scan_at = stamp
        self._scan.publish(message)

    def _on_velocity(self, message):
        if isfinite(message.linear.x) and isfinite(message.angular.z):
            self._requested = message
            self._velocity_at = self._now()
        else:
            self._velocity_at = None

    def _on_fault(self, message):
        self._drop_scan = message.data
        if self._drop_scan:
            self._scan_at = None

    def _on_inhibit(self, message):
        self._inhibit_motion = message.data

    def _publish_velocity(self):
        now = self._now()
        fresh_scan = self._scan_at is not None and -0.05 <= now - self._scan_at < 0.35
        fresh_command = self._velocity_at is not None and 0 <= now - self._velocity_at < 0.25
        allow_motion = fresh_scan and fresh_command and not self._inhibit_motion
        self._velocity.publish(self._requested if allow_motion else Twist())


def main():
    rclpy.init()
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
