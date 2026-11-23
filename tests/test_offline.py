"""Run with python3 -m unittest discover -s tests -v from the project root."""

from collections import Counter
from math import pi
from pathlib import Path
from shutil import copyfile
import struct
import sys
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
for package in ("smelter_gazebo", "smelter_tasks", "smelter_description"):
    sys.path.insert(0, str(ROOT / "src" / package))

from smelter_description.model import build_model
from smelter_gazebo.geometry import Pose, clearance, rectangle
from smelter_gazebo.grid import FREE, OCCUPIED, UNKNOWN, Grid
from smelter_gazebo.world import TILE_SIZE, build_world, tile_rectangles
from smelter_tasks.cart import CartDriver
from smelter_tasks.config import SCENARIOS, docking_poses, read_yaml, resolve_mission, target_outline


class FactoryAssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temporary = TemporaryDirectory()
        cls.addClassCleanup(temporary.cleanup)
        cls.directory = Path(temporary.name)
        map_file = cls.directory / "jueying.yaml"
        copyfile(ROOT / "doc/maps/jueying.yaml", map_file)
        copyfile(ROOT / "doc/maps/jueying.pgm", cls.directory / "jueying.pgm")
        cls.grid = Grid.from_yaml(map_file)
        cls.model = build_model(ROOT / "doc/models/x30/X30.urdf", cls.directory, 0.70, 0.40)
        config = ROOT / "src/smelter_tasks/config"
        cls.mission = read_yaml(config / "mission_points.yaml")
        cls.route = read_yaml(config / "cart_route.yaml")

    def test_map_coordinates_and_round_trip(self):
        grid = self.grid
        self.assertEqual((grid.width, grid.height), (1152, 715))
        self.assertEqual(Counter(grid.cells), {FREE: 393159, UNKNOWN: 413561, OCCUPIED: 16960})
        self.assertAlmostEqual(grid.cell_center(0, 0)[0], -62.95)
        self.assertAlmostEqual(grid.cell_center(0, 0)[1], 19.95)
        self.assertAlmostEqual(grid.cell_center(1151, 714)[1], -51.45)
        self.assertEqual(Grid.from_yaml(grid.write(self.directory)), grid)

    def test_mesh_rectangles_cover_exactly_the_occupied_cells(self):
        grid = self.grid
        visited = bytearray(len(grid.cells))
        for v in range(0, grid.height, TILE_SIZE):
            for u in range(0, grid.width, TILE_SIZE):
                for u0, v0, u1, v1 in tile_rectangles(
                    grid, u, v, min(u + TILE_SIZE, grid.width), min(v + TILE_SIZE, grid.height)
                ):
                    for row in range(v0, v1):
                        start = row * grid.width + u0
                        for i in range(start, start + u1 - u0):
                            self.assertEqual(visited[i], 0)
                            visited[i] = 1
        self.assertEqual(visited, bytes(value == OCCUPIED for value in grid.cells))

    def test_standing_model_and_all_scenarios(self):
        model = self.model
        mesh = (self.directory / "x30_standing.stl").read_bytes()
        triangles, = struct.unpack_from("<I", mesh, 80)
        self.assertEqual(len(mesh), 84 + 50 * triangles)
        self.assertGreater(triangles, 200000)
        self.assertAlmostEqual(model.length, 0.70)
        self.assertAlmostEqual(model.width, 0.40)
        robot = ET.parse(model.urdf)
        self.assertEqual(robot.find(".//sensor[@type='gpu_lidar']/topic").text, "/x30/lidar")
        self.assertEqual(robot.find(".//sensor[@type='contact']/contact/topic").text, "/x30/contact")
        for scenario in SCENARIOS:
            with self.subTest(scenario=scenario):
                nav_grid, config = resolve_mission(
                    self.grid, self.mission, self.route, scenario, model.length, model.width
                )
                goal = Pose(**config["goal"])
                target = target_outline(config["target"])
                self.assertAlmostEqual(clearance(goal.polygon(config["footprint"]), target), 1.50, places=5)
                self.assertGreater(nav_grid.cells.count(OCCUPIED), self.grid.cells.count(OCCUPIED))
                if scenario == "blocking":
                    world = ET.parse(build_world(self.grid, self.directory, config["target"], config["cart"]))
                    self.assertIsNotNone(world.find(".//model[@name='slag_cart']"))
                    self.assertIsNotNone(world.find(".//model[@name='target_slag_pot']"))

    def test_cart_rejects_unknown_sweep(self):
        invalid_route = {**self.route, "start": [-60.0, -50.0, 0.0]}
        with self.assertRaisesRegex(ValueError, "Cart (route|turn)"):
            resolve_mission(self.grid, self.mission, invalid_route, "crossing", 0.70, 0.40)


class GeometryAndCartTest(unittest.TestCase):
    def test_clearance_counts_containment_and_rotated_edges(self):
        hull = rectangle(0.70, 0.40)
        target = Pose(3.0, 0.0, pi / 2).polygon(rectangle(4.0, 2.0))
        self.assertAlmostEqual(clearance(hull, target), 1.65)
        self.assertEqual(clearance(hull, Pose(0, 0, 0.7).polygon(rectangle(0.12, 0.08))), 0.0)

    def test_docking_on_a_rotated_target(self):
        target = {"shape": "rectangle", "pose": (5.0, 2.0, 0.53), "size": (3.0, 2.0)}
        footprint = rectangle(0.70, 0.40)
        pre, goal = docking_poses(target, footprint, 1.50, 0.28, 0.6)
        self.assertAlmostEqual(clearance(goal.polygon(footprint), target_outline(target)), 1.50, places=6)
        self.assertGreater(clearance(pre.polygon(footprint), target_outline(target)), 1.50)

    def test_cart_holds_then_departs_in_simulation_time(self):
        cart = CartDriver({"trigger_delay": 0.1, "speed": 0.68, "checkpoints": [
            {"position": (1.0, 0.0), "hold": 8.0}, {"position": (5.0, 0.0), "hold": 0.0},
        ]})
        self.assertEqual(cart.update(0.0, Pose(0.0, 0.0)), (0.0, 0.0, None))
        self.assertEqual(cart.update(1.0, None), (0.0, 0.0, None))
        self.assertEqual(cart.update(2.0, Pose(1.0, 0.0))[2], "cart_holding")
        self.assertEqual(cart.update(9.9, Pose(1.0, 0.0)), (0.0, 0.0, None))
        self.assertEqual(cart.update(10.0, Pose(1.0, 0.0))[2], "cart_departed")
        self.assertGreater(cart.update(10.1, Pose(1.0, 0.0))[0], 0)
        self.assertEqual(cart.update(15.0, Pose(5.0, 0.0))[2], "cart_finished")
