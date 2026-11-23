"""Record independent Gazebo poses, contact events, and mission outcomes."""

import csv
import json
from math import hypot
from pathlib import Path

from smelter_gazebo.geometry import Pose, angle_difference, clearance, rectangle
from .config import target_outline


class RunMetrics:
    def __init__(self, directory: Path, runtime: dict):
        self.directory = directory
        self.scenario = runtime["scenario"]
        self.label = runtime["label"]
        self.footprint = tuple(tuple(point) for point in runtime["footprint"])
        self.target = target_outline(runtime["target"])
        self.goal = Pose(**runtime["goal"])
        self.cart_footprint = rectangle(*runtime["cart"]["size"][:2]) if runtime["cart"] else None
        self.events = []
        self.contacts = []
        self._last_contact = {}
        self._last_row = None
        self.distance = 0.0
        self.wait_time = 0.0
        self.plan_count = 0
        self.min_cart = None
        self.min_target = None
        self.min_scan = None
        self.max_lateral_deviation = 0.0
        self.track_origin = Pose(**runtime["start"])
        self.samples = 0
        self._stream = (directory / "trajectory.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._stream)
        self._writer.writerow(("sim_s", "state", "dog_x", "dog_y", "dog_yaw", "cart_x", "cart_y",
                               "cart_yaw", "target_clearance_m", "cart_clearance_m",
                               "scan_clearance_m", "odom_linear_mps", "requested_mps", "safe_mps"))

    def event(self, stamp, name, detail=""):
        self.events.append({"sim_s": round(stamp, 3), "event": name, "detail": detail})

    def contact(self, stamp, first, second):
        pair = tuple(sorted((first, second)))
        previous = self._last_contact.get(pair, -10.0)
        self._last_contact[pair] = stamp
        if stamp - previous > 0.25:
            self.contacts.append({"sim_s": round(stamp, 3), "entities": pair})

    def sample(self, stamp, state, dog, cart, scan_distance, odom_speed, requested, safe):
        dog_outline = dog.polygon(self.footprint)
        target_distance = clearance(dog_outline, self.target)
        cart_distance = clearance(dog_outline, cart.polygon(self.cart_footprint)) if cart else None
        self.min_target = target_distance if self.min_target is None else min(target_distance, self.min_target)
        if cart_distance is not None:
            self.min_cart = cart_distance if self.min_cart is None else min(cart_distance, self.min_cart)
        if scan_distance is not None:
            self.min_scan = scan_distance if self.min_scan is None else min(scan_distance, self.min_scan)
        _, cross_track = self.track_origin.inverse_transform((dog.x, dog.y))
        self.max_lateral_deviation = max(abs(cross_track), self.max_lateral_deviation)
        if self._last_row:
            last_stamp, last_pose = self._last_row
            self.distance += hypot(dog.x - last_pose.x, dog.y - last_pose.y)
            if state == "waiting" and abs(odom_speed) < 0.03:
                self.wait_time += stamp - last_stamp
        self._last_row = stamp, dog
        self.samples += 1
        values = (stamp, state, dog.x, dog.y, dog.yaw,
                  cart.x if cart else None, cart.y if cart else None, cart.yaw if cart else None,
                  target_distance, cart_distance, scan_distance, odom_speed, requested, safe)
        self._writer.writerow(round(value, 6) if isinstance(value, float) else value for value in values)

    def finish(self, passed, reason, sim_seconds, wall_seconds, dog):
        self._stream.close()
        result = {
            "passed": passed,
            "reason": reason,
            "mission": self.label,
            "scenario": self.scenario,
            "sim_seconds": round(sim_seconds, 3),
            "wall_seconds": round(wall_seconds, 3),
            "path_length_m": round(self.distance, 4),
            "wait_seconds": round(self.wait_time, 3),
            "global_plans": self.plan_count,
            "global_replans": max(0, self.plan_count - 1),
            "collision_events": len(self.contacts),
            "min_scan_clearance_m": self.min_scan,
            "min_target_clearance_m": self.min_target,
            "min_cart_clearance_m": self.min_cart,
            "max_lateral_deviation_m": self.max_lateral_deviation,
            "final_target_clearance_m": clearance(dog.polygon(self.footprint), self.target) if dog else None,
            "final_yaw_error_rad": abs(angle_difference(self.goal.yaw, dog.yaw)) if dog else None,
            "samples": self.samples,
            "events": self.events,
            "contacts": self.contacts,
        }
        staging = self.directory / "metrics.json.tmp"
        staging.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        staging.replace(self.directory / "metrics.json")
