from __future__ import annotations
from dataclasses import dataclass
from math import hypot, atan2


@dataclass
class Vector2:
    x: float
    y: float

    def __add__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2") -> "Vector2":
        return Vector2(self.x - other.x, self.y - other.y)

    def norm(self) -> float:
        return hypot(self.x, self.y)

    def angle(self) -> float:
        return atan2(self.y, self.x)


@dataclass
class Pose:
    position: Vector2
    theta: float  


@dataclass
class Ball:
    position: Vector2


@dataclass
class Robot:
    id: int
    pose: Pose
