"""Execute the mission with Nav2 actions and record reproducible simulation results."""

import json
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
from time import monotonic

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import FollowPath, NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener

from smelter_gazebo.geometry import Pose, angle_difference, clearance, distance_to_polygon
from .cart import CartDriver
from .config import straight_path
from .metrics import RunMetrics


def _yaw(rotation):
    return atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z))


def _pose(transform):
    return Pose(transform.translation.x, transform.translation.y, _yaw(transform.rotation))


class InspectionMission(Node):
    def __init__(self):
        super().__init__("mission_runner")
        runtime_file = Path(self.declare_parameter("runtime_file", "").value)
        self.config = json.loads(runtime_file.read_text(encoding="utf-8"))
        self.metrics = RunMetrics(runtime_file.parent, self.config)
        self.pre = Pose(**self.config["pre"])
        self.goal = Pose(**self.config["goal"])
        self.docking = self.config["docking"]
        self.cart = CartDriver(self.config["cart"]) if self.config["cart"] else None
        self.scenario = self.config["scenario"]

        self.state = "waiting_for_stack"
        self.phase = "navigating"
        self.started = None
        self.wall_started = monotonic()
        self.last_tick_wall = self.wall_started
        self.ready_since = None
        self.bt_active = False
        self.bt_state_query = None
        self.last_sample = None
        self.scan_at = None
        self.blocked = False
        self.blocked_since = None
        self.clear_since = None
        self.resume_after = None
        self.scan_clearance = None
        self.odom_speed = None
        self.odom_angular = None
        self.requested = 0.0
        self.safe_velocity = 0.0
        self.truth = {}
        self.truth_at = {}
        self.goal_handle = None
        self.sending = False
        self.settling_since = None
        self.settling_started = None
        self.fault_at = None
        self.passed = None

        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.follow = ActionClient(self, FollowPath, "/follow_path")
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.cart_pub = self.create_publisher(Twist, "/cart/cmd_vel", 10) if self.cart else None
        self.fault_pub = self.create_publisher(Bool, "/sim/drop_scan", 10)
        self.inhibit_pub = self.create_publisher(Bool, "/sim/inhibit_motion", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/sim/poses", self._truth, qos_profile_sensor_data)
        self.create_subscription(Contacts, "/sim/contacts", self._contacts, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_requested", self._requested, 10)
        self.create_subscription(Twist, "/cmd_vel", self._safe, 10)
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_timer(0.05, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self, now):
        return max(0.0, now - self.started) if self.started is not None else 0.0

    def _event(self, name, detail=""):
        self.metrics.event(self._elapsed(self._now()), name, detail)
        self.get_logger().info(f"{name} {detail}".strip())

    def _set_state(self, state, detail=""):
        self.state = state
        self._event(state, detail)

    def _scan(self, message):
        self.scan_at = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        front = max(x for x, _ in self.metrics.footprint) + 0.48
        side = max(y for _, y in self.metrics.footprint) + 0.18
        hits = 0
        distance = None
        for index, value in enumerate(message.ranges):
            if not isfinite(value) or not message.range_min <= value <= message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            x, y = value * cos(angle), value * sin(angle)
            hits += int(0 < x < front and abs(y) < side)
            if value <= 3.0:
                current = distance_to_polygon((x, y), self.metrics.footprint)
                distance = current if distance is None else min(distance, current)
        self.scan_clearance = distance
        self.blocked = hits > 3

    def _odom(self, message):
        self.odom_speed = message.twist.twist.linear.x
        self.odom_angular = message.twist.twist.angular.z

    def _truth(self, message):
        for transform in message.transforms:
            name = transform.child_frame_id.strip("/").split("::")[-1]
            if name in ("x30", "slag_cart"):
                self.truth[name] = _pose(transform.transform)
                self.truth_at[name] = self._now()

    def _contacts(self, message):
        if self.started is None or self.passed is not None:
            return
        for contact in message.contacts:
            first, second = contact.collision1, contact.collision2
            self.metrics.contact(self._elapsed(self._now()), first.name or str(first.id), second.name or str(second.id))

    def _requested(self, message):
        self.requested = message.linear.x

    def _safe(self, message):
        self.safe_velocity = message.linear.x

    def _plan(self, message):
        if self.started is not None and message.poses:
            self.metrics.plan_count += 1

    def _localized(self):
        try:
            return _pose(self.buffer.lookup_transform("map", "base_footprint", Time()).transform)
        except TransformException:
            return None

    def _pose_message(self, pose):
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = sin(pose.yaw / 2)
        message.pose.orientation.w = cos(pose.yaw / 2)
        return message

    def _send(self, current):
        self._set_state(self.phase)
        self.sending = True
        if self.phase == "navigating":
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_message(self.pre)
            future = self.navigate.send_goal_async(goal)
        else:
            goal = FollowPath.Goal()
            goal.controller_id = "Dock"
            goal.goal_checker_id = "dock_goal_checker"
            goal.path.header = self._pose_message(self.goal).header
            goal.path.poses = [self._pose_message(pose) for pose in straight_path(current, self.goal, 0.05)]
            future = self.follow.send_goal_async(goal)
        future.add_done_callback(self._accepted)

    def _accepted(self, future):
        self.sending = False
        handle = future.result()
        if not handle.accepted:
            if self.state != "waiting":
                self.finish(False, "Nav2 rejected the action goal")
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._completed)
        if self.state == "waiting" or self.passed is not None:
            handle.cancel_goal_async()

    def _completed(self, future):
        self.goal_handle = None
        if self.passed is not None or self.state == "waiting" or self.fault_at is not None:
            return
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            if self.cart and self.blocked:
                self._pause("Nav2 blocked by an obstacle")
            else:
                self.finish(False, f"{self.phase} action ended with status {status}")
            return
        if self.phase == "navigating":
            current = self._localized()
            if current is None:
                self.finish(False, "Map pose unavailable at the pre-dock goal")
                return
            self.phase = "docking"
            self._send(current)
        else:
            self.settling_started = self._now()
            self._set_state("settling")

    def _pause(self, reason):
        self.clear_since = None
        self.resume_after = self._now() + 0.35
        self._set_state("waiting", reason)
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

    def _publish_cart(self, linear, angular):
        if self.cart_pub:
            command = Twist()
            command.linear.x = linear
            command.angular.z = angular
            self.cart_pub.publish(command)

    def _ready(self, now, current, fresh_scan):
        if self.bt_state_query is not None and self.bt_state_query.done():
            response = self.bt_state_query.result()
            self.bt_active = response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE
            self.bt_state_query = None
        if not self.bt_active and self.bt_state_query is None and self.bt_state.service_is_ready():
            self.bt_state_query = self.bt_state.call_async(GetState.Request())
        ready = (self.bt_active and current is not None and fresh_scan and self.odom_speed is not None
                 and now - self.truth_at.get("x30", -10.0) < 0.3
                 and (self.cart is None or now - self.truth_at.get("slag_cart", -10.0) < 0.3)
                 and self.navigate.server_is_ready() and self.follow.server_is_ready())
        if not ready:
            self.ready_since = None
        elif self.ready_since is None:
            self.ready_since = now
        elif now - self.ready_since >= 0.50:
            self.started = now
            self._send(current)

    def _settle(self, now, current, dog):
        if now - self.settling_started > 4.0:
            self.finish(False, "Docking did not settle within the clearance, heading and speed limits")
            return
        if current is None or dog is None:
            return
        distance = clearance(current.polygon(self.metrics.footprint), self.metrics.target)
        within_limits = (self.docking["min_clearance"] <= distance <= self.docking["max_clearance"]
                         and abs(angle_difference(self.goal.yaw, current.yaw)) <= self.docking["yaw_tolerance"]
                         and abs(self.odom_speed) <= self.docking["max_stopped_speed"]
                         and abs(self.odom_angular) < 0.07)
        if not within_limits:
            self.settling_since = None
        elif self.settling_since is None:
            self.settling_since = now
        elif now - self.settling_since >= self.docking["settled_time"]:
            truth_distance = clearance(dog.polygon(self.metrics.footprint), self.metrics.target)
            truth_yaw_error = abs(angle_difference(self.goal.yaw, dog.yaw))
            if (self.docking["min_clearance"] <= truth_distance <= self.docking["max_clearance"]
                    and truth_yaw_error <= self.docking["yaw_tolerance"]):
                self.finish(True, "Docked and stationary with verified Gazebo clearance")
            else:
                self.finish(False, f"Gazebo docking error: clearance {truth_distance:.3f} m, yaw {truth_yaw_error:.3f} rad")

    def _check_fault_scenario(self, now):
        if self.fault_at is None and self.metrics.distance >= 0.35 and abs(self.odom_speed) > 0.08:
            self.fault_at = now
            self.fault_pub.publish(Bool(data=True))
            self._set_state("scan_loss_injected")
        if self.fault_at is None:
            return
        if now - self.fault_at > 2.0:
            self.finish(False, "Base did not come to a verified stop after scan loss")
        elif (now - self.fault_at > 0.65 and now - self.scan_at > 0.40
              and abs(self.safe_velocity) < 0.001 and abs(self.odom_speed) < 0.025
              and abs(self.odom_angular) < 0.05):
            self.finish(True, "Laser scan outage stopped the moving base")

    def _tick(self):
        self.last_tick_wall = monotonic()
        if self.passed is not None:
            return
        now = self._now()
        current = self._localized()
        fresh_scan = self.scan_at is not None and -0.05 <= now - self.scan_at <= 0.35
        self.state_pub.publish(String(data=self.state))
        if self.started is None:
            self._ready(now, current, fresh_scan)
            return
        elapsed = self._elapsed(now)
        dog = self.truth.get("x30")
        cart = self.truth.get("slag_cart")
        if now - self.truth_at.get("x30", -10.0) > 1.5:
            self.finish(False, "Gazebo pose observations stopped")
            return
        if self.cart:
            if now - self.truth_at.get("slag_cart", -10.0) > 0.5:
                self.finish(False, "Cart pose observations stopped")
                return
            v, omega, event = self.cart.update(elapsed, cart)
            self._publish_cart(v, omega)
            if event:
                self._event(event)
        if dog is not None and (self.last_sample is None or now - self.last_sample >= 0.095):
            self.metrics.sample(elapsed, self.state, dog, cart, self.scan_clearance,
                                self.odom_speed, self.requested, self.safe_velocity)
            self.last_sample = now
        if self.metrics.contacts or self.metrics.min_cart == 0.0:
            self.finish(False, "Collision or dog/cart footprint overlap")
            return
        if elapsed > self.config["timeout"]:
            self.finish(False, f"Mission timed out in state {self.state}")
            return
        if self.scenario == "scan_loss":
            self._check_fault_scenario(now)
            if self.fault_at is not None:
                return
        if not fresh_scan and now - self.scan_at > 1.0:
            self.finish(False, "Unexpected laser scan outage")
            return
        if self.state == "settling":
            self._settle(now, current, dog)
            return
        if self.state == "waiting":
            if not fresh_scan or self.blocked:
                self.clear_since = None
            elif self.clear_since is None:
                self.clear_since = now
            elif (now - self.clear_since > 0.55 and now >= self.resume_after and not self.sending
                  and self.goal_handle is None and current is not None):
                self._event("resuming", self.phase)
                self._send(current)
            return
        if self.cart and self.scenario != "detour" and fresh_scan and self.blocked:
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > 0.23:
                self._pause("Laser points in the forward stop zone")
        else:
            self.blocked_since = None

    def wall_watchdog(self):
        wall = monotonic()
        if self.started is None and wall - self.wall_started > 90.0:
            self.finish(False, "Startup timed out waiting for scan, odometry, world poses and Nav2")
        elif self.started is not None and wall - self.last_tick_wall > 15.0:
            self.finish(False, "Simulation clock stalled")

    def finish(self, passed, reason):
        if self.passed is not None:
            return
        self.passed = passed
        self._set_state("succeeded" if passed else "failed", reason)
        if rclpy.ok():
            self.inhibit_pub.publish(Bool(data=True))
            self._publish_cart(0.0, 0.0)
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.state_pub.publish(String(data=self.state))
        self.metrics.finish(passed, reason, self._elapsed(self._now()),
                            monotonic() - self.wall_started, self.truth.get("x30"))


def main():
    rclpy.init()
    node = InspectionMission()
    try:
        while rclpy.ok() and node.passed is None:
            rclpy.spin_once(node, timeout_sec=0.20)
            node.wall_watchdog()
        if node.passed is not None and rclpy.ok():
            stop_deadline = monotonic() + 0.35
            while monotonic() < stop_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node.passed is None:
            node.finish(False, "Simulation interrupted")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if node.passed else 1
