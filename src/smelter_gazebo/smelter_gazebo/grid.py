"""Read, validate and export the trinary map used by Gazebo and Nav2."""

from dataclasses import dataclass, replace
from io import BytesIO
from math import floor, isfinite
from pathlib import Path

import yaml

from .geometry import EPSILON, Polygon, Pose, intersects

OCCUPIED = 0
UNKNOWN = 128
FREE = 255


def _token(stream: BytesIO) -> bytes:
    char = stream.read(1)
    while char and (char.isspace() or char == b"#"):
        if char == b"#":
            stream.readline()
        char = stream.read(1)
    value = bytearray()
    while char and not char.isspace() and char != b"#":
        value.extend(char)
        char = stream.read(1)
    if char == b"#":
        stream.readline()
    elif char == b"\r" and stream.read(1) != b"\n":
        stream.seek(-1, 1)
    if not value:
        raise ValueError("Incomplete PGM header")
    return bytes(value)


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin: Pose
    cells: bytes

    @classmethod
    def from_yaml(cls, path: Path) -> "Grid":
        config = yaml.safe_load(path.read_text(encoding="utf-8"))
        if config.get("mode", "trinary") != "trinary":
            raise ValueError("The simulator requires a trinary occupancy map")
        image = path.parent / config["image"]
        stream = BytesIO(image.read_bytes())
        if _token(stream) != b"P5":
            raise ValueError(f"{image} is not a binary PGM (P5)")
        width, height, max_value = (int(_token(stream)) for _ in range(3))
        pixels = stream.read()
        if max_value != 255 or width <= 0 or height <= 0 or len(pixels) != width * height:
            raise ValueError(f"Invalid PGM dimensions or bit depth: {image}")
        resolution = float(config["resolution"])
        origin = Pose(*map(float, config["origin"]))
        free, occupied = float(config["free_thresh"]), float(config["occupied_thresh"])
        if (not isfinite(resolution) or resolution <= 0
                or not all(isfinite(v) for v in (origin.x, origin.y, origin.yaw))
                or not 0 <= free < occupied <= 1 or config["negate"] not in (0, 1)):
            raise ValueError(f"Invalid map metadata: {path}")
        probabilities = (i / 255 if config["negate"] else (255 - i) / 255 for i in range(256))
        labels = bytes(OCCUPIED if p > occupied else FREE if p < free else UNKNOWN
                       for p in probabilities)
        return cls(width, height, resolution, origin, pixels.translate(labels))

    def write(self, directory: Path) -> Path:
        image = directory / "map.pgm"
        image.write_bytes(f"P5\n{self.width} {self.height}\n255\n".encode("ascii") + self.cells)
        path = directory / "map.yaml"
        path.write_text(yaml.safe_dump({
            "image": image.name,
            "mode": "trinary",
            "resolution": self.resolution,
            "origin": [self.origin.x, self.origin.y, self.origin.yaw],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }, sort_keys=False), encoding="utf-8")
        return path

    def cell_center(self, u: int, v: int) -> tuple[float, float]:
        r = self.resolution
        return self.origin.transform(((u + 0.5) * r, (self.height - v - 0.5) * r))

    def _bounds(self, polygon: Polygon):
        r = self.resolution
        xs, ys = zip(*polygon)
        return (floor((min(xs) - EPSILON) / r), floor((max(xs) + EPSILON) / r),
                floor((min(ys) - EPSILON) / r), floor((max(ys) + EPSILON) / r))

    def _square(self, col: int, row: int) -> Polygon:
        r = self.resolution
        return ((col * r, row * r), ((col + 1) * r, row * r),
                ((col + 1) * r, (row + 1) * r), (col * r, (row + 1) * r))

    def _index(self, col: int, row: int) -> int:
        return (self.height - 1 - row) * self.width + col

    def is_free(self, polygon: Polygon) -> bool:
        local = tuple(self.origin.inverse_transform(point) for point in polygon)
        x0, x1, y0, y1 = self._bounds(local)
        if x0 < 0 or y0 < 0 or x1 >= self.width or y1 >= self.height:
            return False
        for row in range(y0, y1 + 1):
            for col in range(x0, x1 + 1):
                if self.cells[self._index(col, row)] != FREE and intersects(local, self._square(col, row)):
                    return False
        return True

    def with_obstacle(self, polygon: Polygon) -> "Grid":
        local = tuple(self.origin.inverse_transform(point) for point in polygon)
        x0, x1, y0, y1 = self._bounds(local)
        if x0 < 0 or y0 < 0 or x1 >= self.width or y1 >= self.height:
            raise ValueError("The target footprint crosses the map boundary")
        cells = bytearray(self.cells)
        for row in range(y0, y1 + 1):
            for col in range(x0, x1 + 1):
                if intersects(local, self._square(col, row)):
                    cells[self._index(col, row)] = OCCUPIED
        return replace(self, cells=bytes(cells))
