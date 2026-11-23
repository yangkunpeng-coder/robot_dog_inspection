"""Geometry shared by the map, cart, docking controller and experiment runner."""

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin

Point = tuple[float, float]
Polygon = tuple[Point, ...]
EPSILON = 1e-9


def angle_difference(target: float, current: float) -> float:
    return (target - current + pi) % (2 * pi) - pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float = 0.0

    def transform(self, point: Point) -> Point:
        x, y = point
        c, s = cos(self.yaw), sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def inverse_transform(self, point: Point) -> Point:
        x, y = point[0] - self.x, point[1] - self.y
        c, s = cos(self.yaw), sin(self.yaw)
        return c * x + s * y, -s * x + c * y

    def polygon(self, footprint: Polygon) -> Polygon:
        return tuple(self.transform(point) for point in footprint)

    def offset(self, distance: float) -> "Pose":
        x, y = self.transform((distance, 0.0))
        return Pose(x, y, self.yaw)


def rectangle(length: float, width: float) -> Polygon:
    x, y = length / 2, width / 2
    return ((-x, -y), (x, -y), (x, y), (-x, y))


def circle(radius: float, sides: int = 48) -> Polygon:
    return tuple((radius * cos(2 * pi * i / sides), radius * sin(2 * pi * i / sides))
                 for i in range(sides))


def edges(polygon: Polygon):
    return zip(polygon, polygon[1:] + polygon[:1])


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    offset_x, offset_y = point[0] - start[0], point[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= EPSILON ** 2:
        return hypot(offset_x, offset_y)
    ratio = max(0.0, min(1.0, (offset_x * dx + offset_y * dy) / length_squared))
    return hypot(offset_x - ratio * dx, offset_y - ratio * dy)


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    if any(max(min(a[i], b[i]), min(c[i], d[i])) >
           min(max(a[i], b[i]), max(c[i], d[i])) + EPSILON for i in (0, 1)):
        return False
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    return (min(ab_c, ab_d) <= EPSILON and max(ab_c, ab_d) >= -EPSILON
            and min(cd_a, cd_b) <= EPSILON and max(cd_a, cd_b) >= -EPSILON)


def contains(polygon: Polygon, point: Point) -> bool:
    x, y = point
    inside = False
    for a, b in edges(polygon):
        if _point_segment_distance(point, a, b) <= EPSILON:
            return True
        if (a[1] > y) != (b[1] > y):
            intersection_x = a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if intersection_x > x:
                inside = not inside
    return inside


def intersects(first: Polygon, second: Polygon) -> bool:
    return (contains(first, second[0]) or contains(second, first[0])
            or any(_segments_intersect(a, b, c, d)
                   for a, b in edges(first) for c, d in edges(second)))


def distance_to_polygon(point: Point, polygon: Polygon) -> float:
    if contains(polygon, point):
        return 0.0
    return min(_point_segment_distance(point, a, b) for a, b in edges(polygon))


def clearance(first: Polygon, second: Polygon) -> float:
    """Shortest horizontal boundary distance. Contact and overlap both give zero."""
    if intersects(first, second):
        return 0.0
    return min(
        min(_point_segment_distance(point, a, b)
            for point in first for a, b in edges(second)),
        min(_point_segment_distance(point, a, b)
            for point in second for a, b in edges(first)),
    )


def heading(start: Point, end: Point) -> float:
    return atan2(end[1] - start[1], end[0] - start[0])
