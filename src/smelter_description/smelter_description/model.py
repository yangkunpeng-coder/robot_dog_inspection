"""Bake the supplied X30 meshes in a standing pose and add a simulated drive."""

from dataclasses import dataclass
from math import cos, isfinite, pi, sin, sqrt
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import yaml

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
ZERO = (0.0, 0.0, 0.0)
WHEEL_RADIUS = 0.060
WHEEL_SEPARATION = 0.24
HULL_BOTTOM = 0.023
TRIANGLE = struct.Struct("<12fH")


def _vector(node, key, default=ZERO):
    return tuple(map(float, node.attrib[key].split())) if node is not None and key in node.attrib else default


def _rotate(matrix, vector):
    x, y, z = vector
    return tuple(a * x + b * y + c * z for a, b, c in matrix)


def _multiply(first, second):
    return tuple(tuple(sum(a * b for a, b in zip(row, column))
                       for column in zip(*second)) for row in first)


def _add(first, second):
    return tuple(a + b for a, b in zip(first, second))


def _axis_rotation(axis, angle):
    length = sqrt(sum(a * a for a in axis))
    x, y, z = (a / length for a in axis)
    c, s = cos(angle), sin(angle)
    u = 1.0 - c
    return ((c + x*x*u, x*y*u - z*s, x*z*u + y*s),
            (y*x*u + z*s, c + y*y*u, y*z*u - x*s),
            (z*x*u - y*s, z*y*u + x*s, c + z*z*u))


def _rpy(roll, pitch, yaw):
    return _multiply(
        _axis_rotation((0, 0, 1), yaw),
        _multiply(_axis_rotation((0, 1, 0), pitch), _axis_rotation((1, 0, 0), roll)),
    )


def _origin(parent):
    origin = parent.find("origin")
    return _rpy(*_vector(origin, "rpy")), _vector(origin, "xyz")


def _standing_rotation(joint):
    name = joint.attrib["name"]
    leg, role, _ = name.split("_")
    angle = {"HipX": 0.13 if leg.endswith("L") else -0.13,
             "HipY": -0.70, "Knee": 1.40}[role]
    limit = joint.find("limit")
    if not float(limit.attrib["lower"]) <= angle <= float(limit.attrib["upper"]):
        raise ValueError(f"Standing pose outside the range of {name}")
    return _axis_rotation(_vector(joint.find("axis"), "xyz"), angle)


def _visual_meshes(source):
    robot = ET.parse(source).getroot()
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children = {name: [] for name in links}
    for joint in robot.findall("joint"):
        children[joint.find("parent").attrib["link"]].append(joint)

    def walk(name, rotation, translation):
        for visual in links[name].findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                visual_rotation, visual_translation = _origin(visual)
                yield (source.parent / mesh.attrib["filename"], _vector(mesh, "scale", (1, 1, 1)),
                       _multiply(rotation, visual_rotation),
                       _add(translation, _rotate(rotation, visual_translation)))
        for joint in children[name]:
            child_rotation, child_translation = _origin(joint)
            if joint.attrib["type"] == "revolute":
                child_rotation = _multiply(child_rotation, _standing_rotation(joint))
            yield from walk(joint.find("child").attrib["link"], _multiply(rotation, child_rotation),
                            _add(translation, _rotate(rotation, child_translation)))

    return walk("TORSO", IDENTITY, ZERO)


def _bake_visual(source, destination, nominal_length):
    from array import array

    transformed = array("f")
    low, high = [float("inf")] * 3, [float("-inf")] * 3
    meshes = {}
    for mesh, scale, rotation, translation in _visual_meshes(source):
        if mesh not in meshes:
            data = mesh.read_bytes()
            count, = struct.unpack_from("<I", data, 80)
            if len(data) != 84 + count * TRIANGLE.size:
                raise ValueError(f"Expected a binary STL: {mesh}")
            meshes[mesh] = data[84:]
        order = (3, 9, 6) if scale[0] * scale[1] * scale[2] < 0 else (3, 6, 9)
        for triangle in TRIANGLE.iter_unpack(meshes[mesh]):
            for index in order:
                point = _add(_rotate(rotation, tuple(
                    value * factor for value, factor in zip(triangle[index:index + 3], scale)
                )), translation)
                for axis, value in enumerate(point):
                    low[axis] = min(low[axis], value)
                    high[axis] = max(high[axis], value)
                transformed.extend(point)

    scale = nominal_length / (high[0] - low[0])
    offsets = ((high[0] + low[0]) / 2, (high[1] + low[1]) / 2, low[2])
    count = len(transformed) // 9
    with destination.open("wb") as stream:
        stream.write(b"X30 fixed stance".ljust(80, b" "))
        stream.write(struct.pack("<I", count))
        for index in range(0, len(transformed), 9):
            vertices = [(value - offsets[axis % 3]) * scale + (HULL_BOTTOM if axis % 3 == 2 else 0)
                        for axis, value in enumerate(transformed[index:index + 9])]
            ax, ay, az = (vertices[axis + 3] - vertices[axis] for axis in range(3))
            bx, by, bz = (vertices[axis + 6] - vertices[axis] for axis in range(3))
            nx, ny, nz = ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx
            length = sqrt(nx*nx + ny*ny + nz*nz) or 1.0
            stream.write(TRIANGLE.pack(nx/length, ny/length, nz/length, *vertices, 0))

    width = (high[1] - low[1]) * scale
    top = HULL_BOTTOM + (high[2] - low[2]) * scale
    torso = (-offsets[0] * scale, -offsets[1] * scale, HULL_BOTTOM - offsets[2] * scale)
    return width, top, torso, scale


@dataclass(frozen=True)
class RobotModel:
    urdf: Path
    length: float
    width: float
    height: float
    scale: float


def _number(values):
    return " ".join(f"{value:.8g}" for value in values)


def _link_with_inertia(robot, name, mass, diagonal):
    link = ET.SubElement(robot, "link", name=name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(inertial, "inertia", ixx=str(diagonal[0]), ixy="0", ixz="0",
                  iyy=str(diagonal[1]), iyz="0", izz=str(diagonal[2]))
    return link


def _joint(robot, name, parent, child, position, kind="fixed"):
    joint = ET.SubElement(robot, "joint", name=name, type=kind)
    ET.SubElement(joint, "origin", xyz=_number(position))
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    return joint


def _shape(link, kind, name, position, shape, rpy=ZERO, **dimensions):
    element = ET.SubElement(link, kind, name=name)
    ET.SubElement(element, "origin", xyz=_number(position), rpy=_number(rpy))
    ET.SubElement(ET.SubElement(element, "geometry"), shape, **dimensions)
    return element


def _controllers(path):
    config = {
        "controller_manager": {"ros__parameters": {
            "update_rate": 100,
            "use_sim_time": True,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "x30_base_controller": {"type": "diff_drive_controller/DiffDriveController"},
        }},
        "x30_base_controller": {"ros__parameters": {
            "use_sim_time": True,
            "left_wheel_names": ["left_wheel_joint"],
            "right_wheel_names": ["right_wheel_joint"],
            "wheel_separation": WHEEL_SEPARATION,
            "wheel_radius": WHEEL_RADIUS,
            "use_stamped_vel": False,
            "open_loop": False,
            "position_feedback": True,
            "publish_rate": 40.0,
            "odom_frame_id": "odom",
            "base_frame_id": "base_footprint",
            "tf_frame_prefix_enable": False,
            "enable_odom_tf": True,
            "cmd_vel_timeout": 0.4,
            "linear.x.has_velocity_limits": True,
            "linear.x.max_velocity": 0.35,
            "linear.x.min_velocity": -0.15,
            "linear.x.has_acceleration_limits": True,
            "linear.x.max_acceleration": 0.65,
            "linear.x.min_acceleration": -0.85,
            "angular.z.has_velocity_limits": True,
            "angular.z.max_velocity": 0.8,
            "angular.z.min_velocity": -0.8,
        }},
    }
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def build_model(source: Path, directory: Path, nominal_length: float, nominal_width: float) -> RobotModel:
    if not all(isfinite(value) and value > 0 for value in (nominal_length, nominal_width)):
        raise ValueError("Robot length and width must be positive and finite")
    visual_path = directory / "x30_standing.stl"
    visual_width, top, torso, scale = _bake_visual(source, visual_path, nominal_length)
    width = max(nominal_width, visual_width)
    laser_height = top + 0.025
    robot = ET.Element("robot", name="x30")
    ET.SubElement(robot, "link", name="base_footprint")
    body = _link_with_inertia(robot, "base_link", 16.0, (0.25, 0.85, 0.85))
    _joint(robot, "base_joint", "base_footprint", "base_link", (0, 0, WHEEL_RADIUS))
    visual = _shape(body, "visual", "x30_standing", (0, 0, -WHEEL_RADIUS), "mesh",
                    filename=f"file://{visual_path.resolve().as_posix()}")
    ET.SubElement(ET.SubElement(visual, "material", name="x30_grey"), "color", rgba="0.24 0.27 0.31 1")
    _shape(body, "collision", "hull", (0, 0, (HULL_BOTTOM + top) / 2 - WHEEL_RADIUS), "box",
           size=_number((nominal_length, width, top - HULL_BOTTOM)))
    ET.SubElement(robot, "link", name="TORSO")
    _joint(robot, "torso_joint", "base_link", "TORSO", (torso[0], torso[1], torso[2] - WHEEL_RADIUS))
    ET.SubElement(robot, "link", name="laser_link")
    _joint(robot, "laser_joint", "base_link", "laser_link", (0, 0, laser_height - WHEEL_RADIUS))

    for side, sign in (("left", 1), ("right", -1)):
        name = f"{side}_wheel"
        wheel = _link_with_inertia(robot, name, 0.35, (0.001, 0.001, 0.001))
        _shape(wheel, "collision", name, ZERO, "cylinder", rpy=(pi / 2, 0, 0),
               radius=str(WHEEL_RADIUS), length="0.035")
        joint = _joint(robot, f"{name}_joint", "base_link", name,
                       (0, sign * WHEEL_SEPARATION / 2, 0), "continuous")
        ET.SubElement(joint, "axis", xyz="0 1 0")
        ET.SubElement(joint, "limit", effort="50", velocity="25")

    for side, sign in (("front", 1), ("rear", -1)):
        name = f"{side}_caster"
        caster = _link_with_inertia(robot, name, 0.05, (0.0001, 0.0001, 0.0001))
        _joint(robot, f"{name}_joint", "base_link", name, (sign * nominal_length * 0.37, 0, 0.025 - WHEEL_RADIUS))
        _shape(caster, "collision", name, ZERO, "sphere", radius="0.025")
        caster_gazebo = ET.SubElement(robot, "gazebo", reference=name)
        ET.SubElement(caster_gazebo, "mu1").text = "0.015"
        ET.SubElement(caster_gazebo, "mu2").text = "0.015"

    body_gazebo = ET.SubElement(robot, "gazebo", reference="base_link")
    sensor = ET.SubElement(body_gazebo, "sensor", name="x30_lidar", type="gpu_lidar")
    ET.SubElement(sensor, "pose").text = _number((0, 0, laser_height - WHEEL_RADIUS, 0, 0, 0))
    ET.SubElement(sensor, "topic").text = "/x30/lidar"
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = "10"
    lidar = ET.SubElement(sensor, "lidar")
    horizontal = ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal")
    for key, value in (("samples", "720"), ("resolution", "1"), ("min_angle", str(-pi)), ("max_angle", str(pi))):
        ET.SubElement(horizontal, key).text = value
    range_node = ET.SubElement(lidar, "range")
    for key, value in (("min", "0.09"), ("max", "10.0"), ("resolution", "0.01")):
        ET.SubElement(range_node, key).text = value
    touch = ET.SubElement(body_gazebo, "sensor", name="hull_contact", type="contact")
    ET.SubElement(touch, "always_on").text = "true"
    contact = ET.SubElement(touch, "contact")
    # Fortress's URDF converter merges base_link into base_footprint.
    ET.SubElement(contact, "collision").text = "base_footprint_fixed_joint_lump__hull_collision"
    # The Fortress Contact system reads the topic from this nested element.
    ET.SubElement(contact, "topic").text = "/x30/contact"

    control = ET.SubElement(robot, "ros2_control", name="X30GazeboSystem", type="system")
    ET.SubElement(ET.SubElement(control, "hardware"), "plugin").text = "gz_ros2_control/GazeboSimSystem"
    for side in ("left", "right"):
        joint = ET.SubElement(control, "joint", name=f"{side}_wheel_joint")
        ET.SubElement(joint, "command_interface", name="velocity")
        ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(joint, "state_interface", name="velocity")
    plugin = ET.SubElement(ET.SubElement(robot, "gazebo"), "plugin",
                           filename="libgz_ros2_control-system.so",
                           name="gz_ros2_control::GazeboSimROS2ControlPlugin")
    config = directory / "controllers.yaml"
    _controllers(config)
    ET.SubElement(plugin, "parameters").text = str(config.resolve())
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "remapping").text = "/x30_base_controller/cmd_vel_unstamped:=/cmd_vel"
    ET.SubElement(ros, "remapping").text = "/x30_base_controller/odom:=/odom"

    urdf = directory / "x30.urdf"
    ET.indent(robot)
    ET.ElementTree(robot).write(urdf, encoding="utf-8", xml_declaration=True)
    return RobotModel(urdf, nominal_length, width, top, scale)
