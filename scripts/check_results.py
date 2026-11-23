"""Check that the recorded mission satisfied the expected scenario behavior."""

import argparse
import json
from pathlib import Path


def check(path):
    result = json.loads(path.read_text(encoding="utf-8"))
    if not result["passed"]:
        raise ValueError(result["reason"])
    if result["collision_events"] != 0:
        raise ValueError("Gazebo recorded a collision")
    runtime = json.loads(path.with_name("mission.json").read_text(encoding="utf-8"))
    scenario = result["scenario"]
    events = {entry["event"] for entry in result["events"]}
    if scenario != "scan_loss":
        docking = runtime["docking"]
        distance = result["final_target_clearance_m"]
        if distance is None or not docking["min_clearance"] <= distance <= docking["max_clearance"]:
            raise ValueError("Final target clearance is out of bounds")
        yaw_error = result["final_yaw_error_rad"]
        if yaw_error is None or yaw_error > docking["yaw_tolerance"]:
            raise ValueError("Final heading error exceeds the configured tolerance")
    if scenario in ("crossing", "blocking", "detour"):
        if result["min_cart_clearance_m"] is None or result["min_cart_clearance_m"] <= 0:
            raise ValueError("The dog/cart clearance was not recorded or there was overlap")
    if scenario == "crossing" and "cart_crossed" not in events:
        raise ValueError("The crossing checkpoint was not recorded")
    if scenario == "blocking" and (
        not {"waiting", "resuming", "cart_holding", "cart_departed"} <= events
        or result["wait_seconds"] <= 0
    ):
        raise ValueError("Expected a cart hold, a dog stop and navigation recovery")
    if scenario == "detour" and result["max_lateral_deviation_m"] < 0.75:
        raise ValueError("The dog did not take the alternate route in the demo")
    if scenario == "scan_loss" and "scan_loss_injected" not in events:
        raise ValueError("The expected scan outage was not recorded")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", nargs="+", type=Path)
    args = parser.parse_args()
    passed = True
    for path in args.metrics:
        try:
            result = check(path)
        except (OSError, ValueError, KeyError) as exc:
            print(f"FAIL {path}: {exc}")
            passed = False
        else:
            print(f"PASS {result['scenario']}: {result['reason']} ({result['sim_seconds']:.2f} sim s)")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
