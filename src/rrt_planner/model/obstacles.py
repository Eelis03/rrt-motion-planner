"""Obstacle primitives and exact segment intersection tests.

Every obstacle is treated as a closed set, so a segment that touches a boundary
without entering the interior counts as a collision. Edge validity is decided by
a closed-form intersection test rather than by sampling points along the edge, so
no obstacle can be stepped over regardless of its size relative to the step size.

Boxes and convex polygons are both intersections of half-spaces, so both use the
same parametric clipping routine, :func:`halfspaces_intersect_segment`. Circles
use the exact point-to-segment distance instead.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np
from numpy.typing import NDArray

from rrt_planner.model.space import Vector, as_vector

__all__ = [
    "Box",
    "Circle",
    "ConvexPolygon",
    "Obstacle",
    "ObstacleSet",
    "halfspaces_intersect_segment",
]

# Tolerance used to decide whether a segment runs parallel to a half-space
# boundary. Normals are unit vectors, so the dot product with the segment
# direction is bounded by the segment length and the tolerance scales with it.
_PARALLEL_EPS = 1e-12


@runtime_checkable
class Obstacle(Protocol):
    """A closed obstacle region embedded in a configuration space."""

    @property
    def dimension(self) -> int:
        """Dimension of the space the obstacle lives in."""
        ...

    def contains(self, point: Vector) -> bool:
        """True when ``point`` lies in the obstacle, boundary included."""
        ...

    def intersects_segment(self, start: Vector, end: Vector) -> bool:
        """True when the closed segment from ``start`` to ``end`` meets the obstacle."""
        ...


def halfspaces_intersect_segment(
    normals: NDArray[np.float64],
    offsets: NDArray[np.float64],
    start: Vector,
    end: Vector,
) -> bool:
    """Decide whether a segment meets the convex region ``{x : normals @ x <= offsets}``.

    The routine is the Cyrus and Beck parametric clipping test. Writing the segment
    as ``p(t) = start + t * (end - start)`` for ``t`` in ``[0, 1]``, each half-space
    contributes the linear constraint ``t * (n . d) <= b - n . start``. Constraints
    with a positive coefficient bound ``t`` from above, those with a negative
    coefficient bound it from below, and those with a zero coefficient either hold
    for every ``t`` or for none. The segment meets the region exactly when the
    resulting interval is non-empty. A degenerate segment with ``start == end``
    reduces to a point containment test, which is the correct limiting behaviour.
    """
    direction = end - start
    denominators = normals @ direction
    numerators = offsets - normals @ start
    scale = max(float(np.linalg.norm(direction)), 1.0)
    tolerance = _PARALLEL_EPS * scale

    parallel = np.abs(denominators) <= tolerance
    if bool(np.any(parallel & (numerators < 0.0))):
        return False

    lower = 0.0
    upper = 1.0
    entering = ~parallel & (denominators < 0.0)
    leaving = ~parallel & (denominators > 0.0)
    if bool(np.any(entering)):
        lower = max(lower, float(np.max(numerators[entering] / denominators[entering])))
    if bool(np.any(leaving)):
        upper = min(upper, float(np.min(numerators[leaving] / denominators[leaving])))
    return lower <= upper


@dataclass(frozen=True, slots=True, eq=False)
class Circle:
    """A closed ball of radius ``radius`` about ``center``.

    In the plane this is a disc. The same definition and the same intersection
    test apply unchanged in higher dimensions, where the region is a hyperball.
    """

    center: Vector
    radius: float

    def __post_init__(self) -> None:
        center = as_vector(self.center)
        radius = float(self.radius)
        if radius <= 0.0:
            raise ValueError("radius must be positive")
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "radius", radius)

    @property
    def dimension(self) -> int:
        """Dimension of the space the obstacle lives in."""
        return int(self.center.size)

    def contains(self, point: Vector) -> bool:
        """True when ``point`` lies in the ball, boundary included."""
        return bool(np.linalg.norm(point - self.center) <= self.radius)

    def intersects_segment(self, start: Vector, end: Vector) -> bool:
        """True when the closed segment meets the ball.

        The squared distance from the centre to a point of the segment is a convex
        quadratic in the parameter ``t``, so its minimum over ``[0, 1]`` is attained
        at the clamped stationary point. Comparing that minimum with the radius is
        exact.
        """
        direction = end - start
        offset = start - self.center
        squared_length = float(direction @ direction)
        if squared_length <= 0.0:
            closest = 0.0
        else:
            closest = min(1.0, max(0.0, -float(offset @ direction) / squared_length))
        nearest = offset + closest * direction
        return bool(float(np.linalg.norm(nearest)) <= self.radius)


@dataclass(frozen=True, slots=True, eq=False)
class Box:
    """A closed axis-aligned box, valid in any dimension."""

    lower: Vector
    upper: Vector

    def __post_init__(self) -> None:
        lower = as_vector(self.lower)
        upper = as_vector(self.upper)
        if lower.shape != upper.shape:
            raise ValueError(f"bound shapes differ: {lower.shape} and {upper.shape}")
        if not np.all(upper > lower):
            raise ValueError("every upper bound must exceed its lower bound")
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @property
    def dimension(self) -> int:
        """Dimension of the space the obstacle lives in."""
        return int(self.lower.size)

    @property
    def halfspaces(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """The box as ``2d`` half-spaces ``normals @ x <= offsets``."""
        identity = np.eye(self.dimension, dtype=np.float64)
        normals = np.vstack((identity, -identity))
        offsets = np.concatenate((self.upper, -self.lower))
        return normals, offsets

    def contains(self, point: Vector) -> bool:
        """True when ``point`` lies in the box, boundary included."""
        return bool(np.all(point >= self.lower) and np.all(point <= self.upper))

    def intersects_segment(self, start: Vector, end: Vector) -> bool:
        """True when the closed segment meets the box."""
        normals, offsets = self.halfspaces
        return halfspaces_intersect_segment(normals, offsets, start, end)


@dataclass(frozen=True, slots=True, eq=False)
class ConvexPolygon:
    """A closed convex polygon in the plane, given by its vertices.

    Vertices may be supplied in clockwise or counter-clockwise order. The
    constructor normalises them to counter-clockwise order, rejects fewer than
    three vertices, and rejects any vertex sequence that is not convex.
    """

    vertices: NDArray[np.float64]

    def __post_init__(self) -> None:
        vertices = np.array(self.vertices, dtype=np.float64, copy=True)
        if vertices.ndim != 2 or vertices.shape[1] != 2:
            raise ValueError(f"expected an (n, 2) vertex array, got shape {vertices.shape}")
        if vertices.shape[0] < 3:
            raise ValueError("a polygon needs at least three vertices")
        if not np.all(np.isfinite(vertices)):
            raise ValueError("vertex coordinates must be finite")
        if _signed_area(vertices) < 0.0:
            vertices = vertices[::-1].copy()
        _require_convex(vertices)
        vertices.setflags(write=False)
        object.__setattr__(self, "vertices", vertices)

    @property
    def dimension(self) -> int:
        """Dimension of the space the obstacle lives in."""
        return 2

    @property
    def halfspaces(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """The polygon as one outward half-space per edge, ``normals @ x <= offsets``."""
        edges = np.roll(self.vertices, -1, axis=0) - self.vertices
        outward = np.column_stack((edges[:, 1], -edges[:, 0]))
        lengths = np.linalg.norm(outward, axis=1)
        normals = outward / lengths[:, None]
        offsets = np.einsum("ij,ij->i", normals, self.vertices)
        return normals, np.asarray(offsets, dtype=np.float64)

    def contains(self, point: Vector) -> bool:
        """True when ``point`` lies in the polygon, boundary included."""
        normals, offsets = self.halfspaces
        return bool(np.all(normals @ point <= offsets))

    def intersects_segment(self, start: Vector, end: Vector) -> bool:
        """True when the closed segment meets the polygon."""
        normals, offsets = self.halfspaces
        return halfspaces_intersect_segment(normals, offsets, start, end)


def _signed_area(vertices: NDArray[np.float64]) -> float:
    following = np.roll(vertices, -1, axis=0)
    cross = vertices[:, 0] * following[:, 1] - following[:, 0] * vertices[:, 1]
    return 0.5 * float(np.sum(cross))


def _require_convex(vertices: NDArray[np.float64]) -> None:
    edges = np.roll(vertices, -1, axis=0) - vertices
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0] * following[:, 1] - following[:, 0] * edges[:, 1]
    if not np.all(cross > 0.0):
        raise ValueError("vertices must describe a convex polygon without repeated points")


@dataclass(frozen=True, slots=True, eq=False)
class ObstacleSet:
    """An immutable collection of obstacles sharing one dimension."""

    obstacles: tuple[Obstacle, ...]

    def __post_init__(self) -> None:
        obstacles = tuple(self.obstacles)
        dimensions = {obstacle.dimension for obstacle in obstacles}
        if len(dimensions) > 1:
            raise ValueError(f"obstacles span several dimensions: {sorted(dimensions)}")
        object.__setattr__(self, "obstacles", obstacles)

    @classmethod
    def empty(cls) -> ObstacleSet:
        """An obstacle set containing nothing."""
        return cls(obstacles=())

    def __len__(self) -> int:
        return len(self.obstacles)

    @property
    def dimension(self) -> int | None:
        """Shared dimension of the obstacles, or ``None`` when the set is empty."""
        if not self.obstacles:
            return None
        return self.obstacles[0].dimension

    def is_free(self, point: Vector) -> bool:
        """True when ``point`` lies in no obstacle."""
        return not any(obstacle.contains(point) for obstacle in self.obstacles)

    def segment_is_free(self, start: Vector, end: Vector) -> bool:
        """True when the closed segment from ``start`` to ``end`` meets no obstacle."""
        return not any(obstacle.intersects_segment(start, end) for obstacle in self.obstacles)
