"""Resolve mission inputs and validate them against the generated map."""

from dataclasses import asdict
from math import ceil, hypot, isfinite
from pathlib import Path

import yaml

from smelter_gazebo.geometry import Pose, angle_difference, circle, clearance, heading, rectangle
from smelter_gazebo.grid import Grid

SCENARIOS = ("static", "crossing", "blocking", "detour", "scan_loss")


def read_yaml(path: Path) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"Expected a YAML mapping: {path}")
    return config


def _values(values, count, label):
    result = tuple(float(value) for value in values)
    if len(result) != count or not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must contain {count} finite numbers")
    return result


def _positive(value, label):
    value = float(value)
    if not isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be positive")
    return value


def target_outline(target):
    if target["shape"] == "circle":
        outline = circle(_positive(target["radius"], "target.radius"))
    elif target["shape"] == "rectangle":
        outline = rectangle(*(_positive(value, "target.size") for value in _values(target["size"], 2, "target.size")))
    else:
        raise ValueError("target.shape must be circle or rectangle")
    return Pose(*_values(target["pose"], 3, "target.pose")).polygon(outline)


def docking_poses(target, footprint, desired, yaw, offset):
    """Find the desired edge-to-edge distance on the configured approach line."""
    center = Pose(target["pose"][0], target["pose"][1], yaw)
    outline = target_outline(target)
    target_radius = max(hypot(x - center.x, y - center.y) for x, y in outline)
    dog_radius = max(hypot(x, y) for x, y in footprint)
    lo, hi = 0.0, target_radius + dog_radius + desired + 0.05
    for _ in range(32):
        mid = (lo + hi) / 2
        distance = clearance(center.offset(-mid).polygon(footprint), outline)
        if distance < desired:
            lo = mid
        else:
            hi = mid
    goal = center.offset(-(lo + hi) / 2)
    return goal.offset(-offset), goal


def straight_path(first: Pose, second: Pose, step: float):
    steps = max(1, ceil(hypot(second.x - first.x, second.y - first.y) / step))
    for index in range(steps + 1):
        fraction = index / steps
        yield Pose(first.x + fraction * (second.x - first.x),
                   first.y + fraction * (second.y - first.y), second.yaw)


def _require_free(grid, poses, footprint, label):
    for pose in poses:
        if not grid.is_free(pose.polygon(footprint)):
            raise ValueError(f"{label} overlaps occupied or unknown space near ({pose.x:.2f}, {pose.y:.2f})")


def _cart_config(grid, source, scenario, start, dog_footprint):
    cart = {**source, **source.get("scenarios", {}).get(scenario, {})}
    cart.pop("scenarios", None)
    cart["size"] = _values(cart["size"], 3, "cart.size")
    cart["start"] = _values(cart["start"], 3, "cart.start")
    cart["speed"] = _positive(cart["speed"], "cart.speed")
    cart["trigger_delay"] = float(cart.get("trigger_delay", 0.0))
    if not isfinite(cart["trigger_delay"]) or cart["trigger_delay"] < 0:
        raise ValueError("cart.trigger_delay must be finite and non-negative")
    if not all(value > 0 for value in cart["size"]) or not cart["checkpoints"]:
        raise ValueError("Provide positive cart dimensions and at least one checkpoint")

    footprint = rectangle(*cart["size"][:2])
    pose = Pose(*cart["start"])
    checkpoints = []
    if clearance(pose.polygon(footprint), start.polygon(dog_footprint)) < 0.20:
        raise ValueError("The cart and dog spawn poses are too close")
    for checkpoint in cart["checkpoints"]:
        x, y = _values(checkpoint["position"], 2, "cart.checkpoint.position")
        if hypot(x - pose.x, y - pose.y) < 0.2:
            raise ValueError("Adjacent cart checkpoints must be at least 0.2 m apart")
        hold = float(checkpoint.get("hold", {}).get(scenario, 0.0))
        if not isfinite(hold) or hold < 0:
            raise ValueError("Cart hold times must be finite and non-negative")
        yaw = heading((pose.x, pose.y), (x, y))
        rotation = angle_difference(yaw, pose.yaw)
        steps = max(1, ceil(abs(rotation) / 0.035))
        _require_free(grid, (Pose(pose.x, pose.y, pose.yaw + i * rotation / steps)
                             for i in range(steps + 1)), footprint, "Cart turn")
        end = Pose(x, y, yaw)
        _require_free(grid, straight_path(pose, end, grid.resolution / 2), footprint, "Cart route")
        checkpoints.append({"position": (x, y), "hold": hold})
        pose = end
    cart["checkpoints"] = checkpoints
    return cart


def resolve_mission(grid: Grid, config: dict, cart_config: dict, scenario: str, length: float, width: float):
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario {scenario!r}. Choose one of {', '.join(SCENARIOS)}")
    start = Pose(*_values(config["start"], 3, "start"))
    footprint = rectangle(length, width)
    target = dict(config["target"])
    target["pose"] = _values(target["pose"], 3, "target.pose")
    target["height"] = _positive(target["height"], "target.height")
    if not isinstance(target["add_model"], bool):
        raise ValueError("target.add_model must be true or false")
    if target["shape"] == "rectangle":
        target["size"] = _values(target["size"], 2, "target.size")
    elif target["shape"] == "circle":
        target["radius"] = _positive(target["radius"], "target.radius")
    outline = target_outline(target)
    if target["add_model"]:
        grid = grid.with_obstacle(outline)
    docking = {name: _positive(config["docking"][name], f"docking.{name}") for name in
               ("desired_clearance", "min_clearance", "max_clearance", "pre_offset",
                "yaw_tolerance", "settled_time", "max_stopped_speed")}
    yaw = float(config["docking"]["approach_yaw"])
    if not isfinite(yaw) or not docking["min_clearance"] < docking["desired_clearance"] < docking["max_clearance"]:
        raise ValueError("Specify a finite approach_yaw and min < desired < max clearance")
    pre, goal = docking_poses(target, footprint, docking["desired_clearance"], yaw, docking["pre_offset"])
    _require_free(grid, (start,), footprint, "Dog spawn")
    _require_free(grid, straight_path(pre, goal, grid.resolution / 2), footprint, "Docking approach")
    cart = (_cart_config(grid, cart_config, scenario, start, footprint)
            if scenario in ("crossing", "blocking", "detour") else None)

    return grid, {
        "label": str(config["label"]),
        "scenario": scenario,
        "start": asdict(start),
        "pre": asdict(pre),
        "goal": asdict(goal),
        "target": target,
        "docking": docking,
        "footprint": footprint,
        "cart": cart,
        "timeout": _positive(config["timeout"], "timeout"),
    }
