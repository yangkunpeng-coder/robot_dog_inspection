"""Follow configured cart checkpoints using simulation time and Gazebo poses."""

from math import hypot

from smelter_gazebo.geometry import Pose, angle_difference, heading


class CartDriver:
    def __init__(self, config):
        self.config = config
        self.index = 0
        self.held_until = None

    @property
    def finished(self):
        return self.index >= len(self.config["checkpoints"])

    def update(self, now: float, pose: Pose | None) -> tuple[float, float, str | None]:
        if pose is None or self.finished or now < self.config["trigger_delay"]:
            return 0.0, 0.0, None
        if self.held_until is not None:
            if now < self.held_until:
                return 0.0, 0.0, None
            self.held_until = None
            self.index += 1
            return 0.0, 0.0, "cart_departed"

        checkpoint = self.config["checkpoints"][self.index]
        destination = tuple(checkpoint["position"])
        distance = hypot(destination[0] - pose.x, destination[1] - pose.y)
        if distance < 0.115:
            if checkpoint["hold"] > 0:
                self.held_until = now + checkpoint["hold"]
                return 0.0, 0.0, "cart_holding"
            self.index += 1
            return 0.0, 0.0, "cart_finished" if self.finished else "cart_crossed"

        error = angle_difference(heading((pose.x, pose.y), destination), pose.yaw)
        angular = max(-0.45, min(0.45, 1.8 * error))
        linear = 0.0 if abs(error) > 0.18 else min(self.config["speed"], max(0.10, distance * 1.45))
        return linear, angular, None
