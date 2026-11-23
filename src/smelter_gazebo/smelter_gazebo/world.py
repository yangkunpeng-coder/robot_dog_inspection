"""Generate a Gazebo Fortress world and the temporary cart model."""

from math import atan2, hypot, pi
from pathlib import Path
import xml.etree.ElementTree as ET

from .grid import OCCUPIED, Grid
from .geometry import Pose

TILE_SIZE = 96
OBSTACLE_HEIGHT = 1.25


def _child(node, name, value, **attributes):
    ET.SubElement(node, name, **attributes).text = str(value)


def _values(values):
    return " ".join(f"{v:.8g}" for v in values)


def _pose(node, xyz, rpy=(0, 0, 0)):
    _child(node, "pose", _values((*xyz, *rpy)))


def _box(size):
    return "box", {"size": _values(size)}


def _body(link, name, xyz, shape, color, rpy=(0, 0, 0), friction=None):
    kinds = ("collision", "visual") if color else ("collision",)
    for kind in kinds:
        element = ET.SubElement(link, kind, name=f"{name}_{kind}")
        _pose(element, xyz, rpy)
        tag, values = shape
        geometry = ET.SubElement(ET.SubElement(element, "geometry"), tag)
        for field, value in values.items():
            _child(geometry, field, value)
        if kind == "visual":
            material = ET.SubElement(element, "material")
            _child(material, "ambient", color)
            _child(material, "diffuse", color)
            _child(element, "cast_shadows", "false")
        elif friction is not None:
            ode = ET.SubElement(ET.SubElement(ET.SubElement(element, "surface"), "friction"), "ode")
            _child(ode, "mu", friction)
            _child(ode, "mu2", friction)


def _runs(grid, row, start, end):
    cells = grid.cells
    offset = grid.width * row
    col = start
    while col < end:
        if cells[offset + col] != OCCUPIED:
            col += 1
            continue
        first = col
        col += 1
        while col < end and cells[offset + col] == OCCUPIED:
            col += 1
        yield first, col


def tile_rectangles(grid: Grid, u0: int, v0: int, u1: int, v1: int):
    """Merge identical horizontal runs vertically without filling any empty cell."""
    active = {}
    for row in range(v0, v1):
        runs = set(_runs(grid, row, u0, u1))
        for run in sorted(active.keys() - runs):
            yield run[0], active.pop(run), run[1], row
        for run in sorted(runs):
            active.setdefault(run, row)
    for run, first_row in sorted(active.items()):
        yield run[0], first_row, run[1], v1


def _write_mesh(path, rectangles, grid):
    # Closed boxes, tiled into a small number of meshes to keep rendering inexpensive.
    faces = ((1, 4, 3, 2), (5, 6, 7, 8), (1, 2, 6, 5),
             (2, 3, 7, 6), (3, 4, 8, 7), (4, 1, 5, 8))
    r = grid.resolution
    with path.open("w", encoding="ascii", newline="\n") as stream:
        for normal in ((0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0)):
            stream.write(f"vn {_values(normal)}\n")
        for index, (u0, v0, u1, v1) in enumerate(rectangles):
            x0, x1 = u0 * r, u1 * r
            y0, y1 = (grid.height - v1) * r, (grid.height - v0) * r
            for z in (0.0, OBSTACLE_HEIGHT):
                for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    stream.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
            offset = index * 8
            for normal, face in enumerate(faces, 1):
                indices = " ".join(f"{i + offset}//{normal}" for i in face)
                stream.write(f"f {indices}\n")


def _static_model(world, name, xyz, yaw=0.0):
    model = ET.SubElement(world, "model", name=name)
    _child(model, "static", "true")
    _pose(model, xyz, (0, 0, yaw))
    return ET.SubElement(model, "link", name="body")


def _inertial(link, mass, size):
    x, y, z = size
    inertial = ET.SubElement(link, "inertial")
    _child(inertial, "mass", mass)
    inertia = ET.SubElement(inertial, "inertia")
    for field, value in (("ixx", mass*(y*y + z*z)/12), ("iyy", mass*(x*x + z*z)/12),
                         ("izz", mass*(x*x + y*y)/12), ("ixy", 0), ("ixz", 0), ("iyz", 0)):
        _child(inertia, field, value)


def _add_cart(world, pose: Pose, size, speed):
    length, width, height = size
    model = ET.SubElement(world, "model", name="slag_cart")
    _pose(model, (pose.x, pose.y, 0.0), (0, 0, pose.yaw))
    chassis = ET.SubElement(model, "link", name="chassis")
    _inertial(chassis, 160.0, size)
    bottom = 0.13
    _body(chassis, "cart_hull", (0, 0, (height + bottom) / 2),
          _box((length, width, height - bottom)), "0.81 0.42 0.13 1")

    radius, track = 0.23, width * 0.65
    for side, sign in (("left", 1), ("right", -1)):
        name = f"{side}_wheel"
        wheel = ET.SubElement(model, "link", name=name)
        _pose(wheel, (0, sign * track / 2, radius))
        _inertial(wheel, 3.0, (radius * 2, 0.20, radius * 2))
        _body(wheel, name, (0, 0, 0), ("cylinder", {"radius": radius, "length": 0.20}),
              "0.12 0.12 0.12 1", rpy=(pi / 2, 0, 0), friction=1.0)
        joint = ET.SubElement(model, "joint", name=f"{name}_joint", type="revolute")
        _child(joint, "parent", "chassis")
        _child(joint, "child", name)
        axis = ET.SubElement(joint, "axis")
        _child(axis, "xyz", "0 1 0")
        limit = ET.SubElement(axis, "limit")
        _child(limit, "lower", -1e16)
        _child(limit, "upper", 1e16)

    for side, sign in (("front", 1), ("rear", -1)):
        _body(chassis, f"{side}_caster", (sign * length * 0.39, 0, 0.16),
              ("sphere", {"radius": 0.16}), None, friction=0.02)

    drive = ET.SubElement(model, "plugin", filename="ignition-gazebo-diff-drive-system",
                          name="ignition::gazebo::systems::DiffDrive")
    for key, value in (("left_joint", "left_wheel_joint"), ("right_joint", "right_wheel_joint"),
                       ("wheel_radius", radius), ("wheel_separation", track),
                       ("topic", "/model/slag_cart/cmd_vel"), ("max_linear_velocity", speed),
                       ("max_linear_acceleration", 0.8), ("min_linear_acceleration", -0.8)):
        _child(drive, key, value)


def _add_demo_camera(world, points):
    xs, ys = zip(*points)
    center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
    span = max(max(xs) - min(xs), max(ys) - min(ys), 7.0)
    dx, dy, z = 0.48 * span, 0.48 * span, 1.42 * span
    link = _static_model(world, "demo_camera", (center[0] + dx, center[1] + dy, z))
    sensor = ET.SubElement(link, "sensor", name="inspection_camera", type="camera")
    _pose(sensor, (0, 0, 0), (0, atan2(z - 0.2, hypot(dx, dy)), atan2(-dy, -dx)))
    _child(sensor, "topic", "/smelter/demo_image")
    _child(sensor, "always_on", "true")
    _child(sensor, "update_rate", 10)
    camera = ET.SubElement(sensor, "camera")
    _child(camera, "horizontal_fov", 0.97)
    image = ET.SubElement(camera, "image")
    _child(image, "width", 960)
    _child(image, "height", 540)
    _child(image, "format", "R8G8B8")
    clip = ET.SubElement(camera, "clip")
    _child(clip, "near", 0.15)
    _child(clip, "far", 300)


def build_world(grid: Grid, directory: Path, target: dict, cart: dict | None, camera_points=()) -> Path:
    mesh_dir = directory / "world_meshes"
    mesh_dir.mkdir(exist_ok=True)
    sdf = ET.Element("sdf", version="1.8")
    world = ET.SubElement(sdf, "world", name="smelter")
    physics = ET.SubElement(world, "physics", name="default", type="ignored")
    _child(physics, "max_step_size", 0.005)
    _child(physics, "real_time_factor", 1.0)
    _child(physics, "real_time_update_rate", 200)
    for name, filename in (("Physics", "physics"), ("UserCommands", "user-commands"),
                           ("SceneBroadcaster", "scene-broadcaster"), ("Contact", "contact"),
                           ("Sensors", "sensors")):
        plugin = ET.SubElement(world, "plugin", filename=f"ignition-gazebo-{filename}-system",
                               name=f"ignition::gazebo::systems::{name}")
        if name == "Sensors":
            _child(plugin, "render_engine", "ogre2")

    scene = ET.SubElement(world, "scene")
    _child(scene, "ambient", "0.65 0.65 0.65 1")
    _child(scene, "background", "0.79 0.82 0.87 1")
    _child(scene, "shadows", "false")
    light = ET.SubElement(world, "light", name="overhead", type="directional")
    _pose(light, (0, 0, 8))
    _child(light, "diffuse", "0.75 0.75 0.75 1")
    _child(light, "direction", "-0.4 0.2 -1")

    size_x, size_y = grid.width * grid.resolution, grid.height * grid.resolution
    floor_xy = grid.origin.transform((size_x / 2, size_y / 2))
    floor = _static_model(world, "floor", (*floor_xy, -0.075), grid.origin.yaw)
    _body(floor, "ground", (0, 0, 0), _box((size_x, size_y, 0.15)), "0.53 0.55 0.56 1", friction=1.0)
    obstacles = _static_model(world, "map_obstacles", (grid.origin.x, grid.origin.y, 0), grid.origin.yaw)
    for v in range(0, grid.height, TILE_SIZE):
        for u in range(0, grid.width, TILE_SIZE):
            rectangles = list(tile_rectangles(grid, u, v, min(u + TILE_SIZE, grid.width),
                                              min(v + TILE_SIZE, grid.height)))
            if rectangles:
                name = f"tile_{u}_{v}"
                mesh = mesh_dir / f"{name}.obj"
                _write_mesh(mesh, rectangles, grid)
                _body(obstacles, name, (0, 0, 0), ("mesh", {"uri": f"file://{mesh.resolve().as_posix()}"}),
                      "0.29 0.33 0.39 1")

    if target["add_model"]:
        pose = Pose(*target["pose"])
        height = target["height"]
        pot = _static_model(world, "target_slag_pot", (pose.x, pose.y, 0), pose.yaw)
        shape = (("cylinder", {"radius": target["radius"], "length": height})
                 if target["shape"] == "circle" else _box((*target["size"], height)))
        _body(pot, "pot", (0, 0, height / 2), shape, "0.75 0.25 0.16 1")
    if cart:
        _add_cart(world, Pose(*cart["start"]), cart["size"], cart["speed"])
    if camera_points:
        _add_demo_camera(world, camera_points)
    path = directory / "smelter.sdf"
    ET.indent(sdf)
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)
    return path
