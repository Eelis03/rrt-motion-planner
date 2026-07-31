"""The planner interface, the steering function, and the nearest neighbour index."""

from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

import numpy as np
from scipy.spatial import cKDTree

from rrt_planner.model.problem import PlanningProblem, PlanResult
from rrt_planner.model.space import Vector, as_vector

__all__ = ["NearestNeighbourIndex", "Planner", "steer"]


@runtime_checkable
class Planner(Protocol):
    """A single-query motion planner.

    Implementations are configuration objects: all mutable state belongs to one
    call of :meth:`plan`, so the same instance may be reused across problems and
    seeds without any run influencing another.
    """

    @property
    def name(self) -> str:
        """Human-readable planner name used in reports and figures."""
        ...

    def plan(self, problem: PlanningProblem, seed: int) -> PlanResult:
        """Solve ``problem``. The same problem and seed must give the same result."""
        ...


def steer(origin: Vector, target: Vector, step_size: float) -> Vector:
    """Return the point on the ray from ``origin`` to ``target`` at most ``step_size`` away.

    When the target is already within reach it is returned unchanged, so a planner
    can connect exactly to sampled configurations and to the goal.
    """
    if step_size <= 0.0:
        raise ValueError("step size must be positive")
    direction = target - origin
    distance = float(np.linalg.norm(direction))
    if distance <= step_size:
        return as_vector(target)
    return as_vector(origin + direction * (step_size / distance))


class NearestNeighbourIndex:
    """Incremental Euclidean nearest neighbour queries backed by ``scipy.spatial.cKDTree``.

    A k-d tree is a static structure: inserting a point means rebuilding it. Rebuilding
    on every insertion would cost ``O(n log n)`` per planner iteration, which dominates
    the run. This index therefore keeps a tree over the first ``m`` points and a small
    buffer of later ones that is scanned linearly, rebuilding once the buffer reaches
    ``sqrt(m)`` entries. Queries then cost ``O(log m + sqrt(m))`` and the amortised
    rebuild cost per insertion is ``O(sqrt(m) log m)``.

    The rebuild schedule depends only on how many points have been inserted, never on
    their values, so a given insertion sequence always produces the same answers. Ties
    are resolved in favour of the lower index.
    """

    __slots__ = ("_dimension", "_minimum_batch", "_points", "_tree", "_tree_size")

    def __init__(self, dimension: int, minimum_batch: int = 16) -> None:
        if dimension < 1:
            raise ValueError("dimension must be at least one")
        if minimum_batch < 1:
            raise ValueError("minimum batch must be at least one")
        self._dimension = dimension
        self._minimum_batch = minimum_batch
        self._points: list[Vector] = []
        self._tree: cKDTree | None = None
        self._tree_size = 0

    def __len__(self) -> int:
        return len(self._points)

    @property
    def points(self) -> tuple[Vector, ...]:
        """Every inserted point, in insertion order."""
        return tuple(self._points)

    def add(self, point: Vector) -> int:
        """Insert ``point`` and return the index assigned to it."""
        if point.size != self._dimension:
            raise ValueError(f"expected a point of dimension {self._dimension}")
        index = len(self._points)
        self._points.append(as_vector(point))
        pending = len(self._points) - self._tree_size
        threshold = max(self._minimum_batch, math.isqrt(max(self._tree_size, 0)))
        if pending >= threshold:
            self._rebuild()
        return index

    def _rebuild(self) -> None:
        self._tree = cKDTree(np.vstack(self._points))
        self._tree_size = len(self._points)

    def nearest(self, point: Vector) -> int:
        """Return the index of the inserted point closest to ``point``."""
        if not self._points:
            raise ValueError("the index is empty")
        best_index = -1
        best_distance = math.inf
        if self._tree is not None and self._tree_size > 0:
            distance, candidate = self._tree.query(point, k=1)
            best_index = int(candidate)
            best_distance = float(distance)
        for offset in range(self._tree_size, len(self._points)):
            distance = float(np.linalg.norm(self._points[offset] - point))
            if distance < best_distance:
                best_distance = distance
                best_index = offset
        return best_index

    def within_radius(self, point: Vector, radius: float) -> list[int]:
        """Return the indices of every inserted point within ``radius``, in ascending order."""
        found: list[int] = []
        if self._tree is not None and self._tree_size > 0:
            found.extend(int(index) for index in self._tree.query_ball_point(point, radius))
        for offset in range(self._tree_size, len(self._points)):
            if float(np.linalg.norm(self._points[offset] - point)) <= radius:
                found.append(offset)
        found.sort()
        return found
