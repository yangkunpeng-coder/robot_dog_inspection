"""Run the factory demo on ROS 2 Humble and Gazebo Fortress."""

from datetime import datetime
import json
import os
from pathlib import Path
from platform import release

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from smelter_description.model import build_model
from smelter_gazebo.grid import Grid
from smelter_gazebo.world import build_world
from smelter_tasks.config import SCENARIOS, read_yaml, resolve_mission


class _RosParamsDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        # Humble's rcl parameter parser rejects YAML aliases.
        return True


def _share(name):
    return Path(get_package_share_directory(name))


def _flag(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def _nav_config(navigation, run_dir, map_path, runtime, robot):
    params = read_yaml(navigation / "config" / "nav2.yaml")
    params["map_server"]["ros__parameters"]["yaml_filename"] = str(map_path)
    params["amcl"]["ros__parameters"]["initial_pose"] = {**runtime["start"], "z": 0.0}
    params["bt_navigator"]["ros__parameters"].update({
        "default_nav_to_pose_bt_xml": str(navigation / "config" / "navigate_to_pose.xml"),
        "default_nav_through_poses_bt_xml": str(navigation / "config" / "navigate_through_poses.xml"),
    })
    dock_goal = params["controller_server"]["ros__parameters"]["dock_goal_checker"]
    dock_goal["yaw_goal_tolerance"] = min(0.04, runtime["docking"]["yaw_tolerance"] * 0.4)
    for name in ("local_costmap", "global_costmap"):
        params[name][name]["ros__parameters"]["footprint"] = json.dumps(runtime["footprint"])
    global_obstacles = params["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    global_obstacles["enabled"] = runtime["scenario"] == "detour"
    # The global map tracks the cart only in the detour scenario. Other runs use the
    # same static global plan so that waiting and resuming can be measured consistently.
    monitor = params["collision_monitor"]["ros__parameters"]
    for name, front, rear, side in (("Stop", 0.48, 0.17, 0.18), ("Slow", 0.95, 0.24, 0.30)):
        x0, x1 = -robot.length / 2 - rear, robot.length / 2 + front
        y = robot.width / 2 + side
        monitor[name]["points"] = [x0, -y, x1, -y, x1, y, x0, y]
    path = run_dir / "nav2.yaml"
    path.write_text(yaml.dump(params, Dumper=_RosParamsDumper, sort_keys=False), encoding="utf-8")
    return path


def _bridge(cart, video):
    mappings = [
        ("/world/smelter/clock", "rosgraph_msgs/msg/Clock", "[", "Clock", "/clock"),
        ("/x30/lidar", "sensor_msgs/msg/LaserScan", "[", "LaserScan", "/scan_raw"),
        ("/world/smelter/pose/info", "tf2_msgs/msg/TFMessage", "[", "Pose_V", "/sim/poses"),
        ("/x30/contact", "ros_gz_interfaces/msg/Contacts", "[", "Contacts", "/sim/contacts"),
    ]
    if cart:
        mappings.append(("/model/slag_cart/cmd_vel", "geometry_msgs/msg/Twist", "]", "Twist", "/cart/cmd_vel"))
    if video:
        mappings.append(("/smelter/demo_image", "sensor_msgs/msg/Image", "[", "Image", "/sim/demo_image"))
    return Node(package="ros_gz_bridge", executable="parameter_bridge", name="sim_bridge",
                arguments=[f"{topic}@{ros_type}{direction}ignition.msgs.{gz_type}"
                           for topic, ros_type, direction, gz_type, _ in mappings],
                remappings=[(topic, ros_topic) for topic, _, _, _, ros_topic in mappings],
                output="screen")


def _controller_spawner(name):
    return Node(package="controller_manager", executable="spawner", name=f"spawn_{name}",
                arguments=[name, "--controller-manager", "/controller_manager",
                           "--controller-manager-timeout", "45"], output="screen")


def _after_success(action, next_action, label):
    def complete(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason=f"{label} failed (exit {event.returncode})"))]
        return [next_action] if next_action else []

    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=complete))


def _launch(context):
    scenario = LaunchConfiguration("scenario").perform(context)
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario {scenario}. Expected {', '.join(SCENARIOS)}")
    output_dir = LaunchConfiguration("output_dir").perform(context)
    directory = (Path(output_dir).expanduser() if output_dir else
                 Path.cwd() / "artifacts" / f"{scenario}-{datetime.now():%Y%m%d-%H%M%S}")
    directory = directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if any(directory.iterdir()):
        raise ValueError(f"Choose an empty output directory: {directory}")

    navigation = _share("smelter_navigation")
    mission = read_yaml(Path(LaunchConfiguration("mission").perform(context)))
    route = read_yaml(Path(LaunchConfiguration("cart_route").perform(context)))
    robot = build_model(_share("smelter_description") / "x30" / "X30.urdf", directory,
                        float(mission["robot"]["length"]), float(mission["robot"]["width"]))
    grid = Grid.from_yaml(Path(LaunchConfiguration("map").perform(context)))
    nav_grid, runtime = resolve_mission(grid, mission, route, scenario, robot.length, robot.width)
    runtime["robot_model"] = {"length": robot.length, "width": robot.width,
                              "height": robot.height, "scale": robot.scale}
    partition = os.environ.get("IGN_PARTITION", f"smelter_{os.getpid()}")
    runtime["ign_partition"] = partition
    runtime_file = directory / "mission.json"
    runtime_file.write_text(json.dumps(runtime, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    video = _flag(context, "video")
    camera_points = []
    if video:
        camera_points = [(runtime["start"]["x"], runtime["start"]["y"]), runtime["target"]["pose"][:2]]
        if runtime["cart"]:
            camera_points.extend([runtime["cart"]["start"][:2],
                                  *(point["position"] for point in runtime["cart"]["checkpoints"])])
    world = build_world(grid, directory, runtime["target"], runtime["cart"], camera_points)
    nav_params = _nav_config(navigation, directory, nav_grid.write(directory), runtime, robot)
    use_sim_time = {"use_sim_time": True}

    server_args = ["ign", "gazebo", "-r", "-s", "-v", "2"]
    if _flag(context, "headless_rendering"):
        server_args.append("--headless-rendering")
    server = ExecuteProcess(cmd=[*server_args, str(world)], name="gazebo_server", output="screen")
    state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                           parameters=[use_sim_time, {"robot_description": ParameterValue(
                               robot.urdf.read_text(encoding="utf-8"), value_type=str)}],
                           output="screen")
    initial = runtime["start"]
    spawn = Node(package="ros_gz_sim", executable="create", name="spawn_x30",
                 arguments=["-world", "smelter", "-name", "x30", "-file", str(robot.urdf),
                            "-x", str(initial["x"]), "-y", str(initial["y"]), "-Y", str(initial["yaw"])],
                 output="screen")
    broadcaster = _controller_spawner("joint_state_broadcaster")
    drive = _controller_spawner("x30_base_controller")
    node_configs = [
        ("nav2_map_server", "map_server", "map_server"),
        ("nav2_amcl", "amcl", "amcl"),
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_collision_monitor", "collision_monitor", "collision_monitor"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
    ]
    nav_nodes = [
        Node(package=package, executable=executable, name=name, parameters=[str(nav_params)],
             remappings=[("cmd_vel", "/cmd_vel_requested")] if name == "controller_server" else [],
             output="screen")
        for package, executable, name in node_configs
    ]
    lifecycle = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                     name="lifecycle_manager", parameters=[use_sim_time, {
                         "autostart": True, "node_names": [name for _, _, name in node_configs],
                         "bond_timeout": 8.0,
                     }], output="screen")
    relay = Node(package="smelter_tasks", executable="scan_relay", parameters=[use_sim_time], output="screen")
    runner = Node(package="smelter_tasks", executable="mission_runner",
                  parameters=[use_sim_time, {"runtime_file": str(runtime_file)}], output="screen")

    actions = [
        SetEnvironmentVariable("IGN_PARTITION", partition),
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", os.pathsep.join(filter(None, (
            str(Path(get_package_prefix("gz_ros2_control")) / "lib"),
            os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH"),
        )))),
        _after_success(server, None, "Gazebo server"),
        _after_success(spawn, broadcaster, "X30 spawn"),
        _after_success(broadcaster, drive, "joint state broadcaster"),
        _after_success(drive, None, "X30 drive"),
    ]
    if "microsoft" in release().lower():
        defaults = {"ROS_DOMAIN_ID": "53", "ROS_LOCALHOST_ONLY": "1",
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
        actions[:0] = [SetEnvironmentVariable(name, value) for name, value in defaults.items()
                       if not os.environ.get(name)]
    actions.extend(_after_success(node, None, name) for node, (_, _, name) in zip(nav_nodes, node_configs))
    if _flag(context, "shutdown_on_result"):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[
            EmitEvent(event=Shutdown(reason="Inspection run finished. See metrics.json.")),
        ])))
    actions.extend([server, state_publisher, _bridge(runtime["cart"], video), relay,
                    *nav_nodes, lifecycle, runner, spawn])
    if video:
        actions.append(Node(package="smelter_tasks", executable="demo_recorder", parameters=[
            use_sim_time, {"video_path": str(directory / "inspection.mp4"), "scenario": scenario},
        ], output="screen"))
    if _flag(context, "gui"):
        actions.extend([
            ExecuteProcess(cmd=["ign", "gazebo", "-g", "-v", "2"], name="gazebo_gui", output="screen"),
            Node(package="rviz2", executable="rviz2", arguments=["-d", str(navigation / "rviz" / "smelter.rviz")],
                 parameters=[use_sim_time], output="screen"),
        ])
    if _flag(context, "record"):
        actions.append(ExecuteProcess(cmd=[
            "ros2", "bag", "record", "-o", str(directory / "bag"), "/clock", "/map", "/tf", "/tf_static",
            "/scan", "/odom", "/cmd_vel_requested", "/cmd_vel_nav", "/cmd_vel", "/plan",
            "/sim/poses", "/sim/contacts", "/mission/state", "/cart/cmd_vel",
        ], name="rosbag", output="screen"))
    return actions


def generate_launch_description():
    navigation = _share("smelter_navigation")
    tasks = _share("smelter_tasks")
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="static", choices=list(SCENARIOS)),
        DeclareLaunchArgument("mission", default_value=str(tasks / "config" / "mission_points.yaml")),
        DeclareLaunchArgument("cart_route", default_value=str(tasks / "config" / "cart_route.yaml")),
        DeclareLaunchArgument("map", default_value=str(navigation / "maps" / "jueying.yaml")),
        DeclareLaunchArgument("output_dir", default_value=""),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("headless_rendering", default_value="true"),
        DeclareLaunchArgument("shutdown_on_result", default_value="true"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("video", default_value="false"),
        OpaqueFunction(function=_launch),
    ])
