"""Problem definitions and planner results."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field

import numpy as np

from rrt_planner.model.graph import Roadmap, SearchTree
from rrt_planner.model.obstacles import ObstacleSet
from rrt_planner.model.space import ConfigurationSpace, Vector, as_vector

__all__ = ["PlanResult", "PlanningProblem", "path_cost"]


def path_cost(path: Sequence[Vector]) -> float:
    """Return the Euclidean length of a polyline, or zero for fewer than two points."""
    if len(path) < 2:
        return 0.0
    return float(
        sum(float(np.linalg.norm(path[i + 1] - path[i])) for i in range(len(path) - 1))
    )


@dataclass(frozen=True, slots=True, eq=False)
class PlanningProblem:
    """A single-query planning problem: where to start, where to finish, what to avoid.

    The start and the goal are exact configurations. A returned path must begin at
    ``start`` and end at ``goal``, so success is not diluted by a goal tolerance
    that would make different planners comparable only up to that tolerance.
    """

    name: str
    space: ConfigurationSpace
    obstacles: ObstacleSet
    start: Vector
    goal: Vector

    def __post_init__(self) -> None:
        start = as_vector(self.start)
        goal = as_vector(self.goal)
        dimension = self.space.dimension
        if start.size != dimension or goal.size != dimension:
            raise ValueError(
                f"start and goal must have dimension {dimension}, "
                f"got {start.size} and {goal.size}"
            )
        obstacle_dimension = self.obstacles.dimension
        if obstacle_dimension is not None and obstacle_dimension != dimension:
            raise ValueError(
                f"obstacles have dimension {obstacle_dimension}, space has {dimension}"
            )
        if not self.space.contains(start):
            raise ValueError("the start configuration lies outside the space")
        if not self.space.contains(goal):
            raise ValueError("the goal configuration lies outside the space")
        if not self.obstacles.is_free(start):
            raise ValueError("the start configuration lies inside an obstacle")
        if not self.obstacles.is_free(goal):
            raise ValueError("the goal configuration lies inside an obstacle")
        object.__setattr__(self, "start", start)
        object.__setattr__(self, "goal", goal)

    @property
    def dimension(self) -> int:
        """Dimension of the configuration space."""
        return self.space.dimension

    @property
    def straight_line_cost(self) -> float:
        """Distance from start to goal, a lower bound on any feasible path cost."""
        return float(np.linalg.norm(self.goal - self.start))

    def path_is_valid(self, path: Sequence[Vector], tolerance: float = 1e-9) -> bool:
        """True when ``path`` starts at the start, ends at the goal, and is collision free."""
        if len(path) < 2:
            return False
        if float(np.linalg.norm(path[0] - self.start)) > tolerance:
            return False
        if float(np.linalg.norm(path[-1] - self.goal)) > tolerance:
            return False
        return all(
            self.space.contains(point) for point in path
        ) and all(
            self.obstacles.segment_is_free(path[i], path[i + 1]) for i in range(len(path) - 1)
        )


@dataclass(frozen=True, slots=True, eq=False)
class PlanResult:
    """The outcome of one planner run on one problem with one seed.

    ``tree`` and ``roadmap`` expose the structure the planner built. They are kept
    so that the visualisation exporter and the convergence figures can read it, and
    are ignored by the benchmark metrics.

    ``point_checks`` and ``segment_checks`` count the collision queries the run
    asked. They are the machine-independent measure of planner effort, so unlike
    wall time they can be recorded, compared across platforms, and regression
    tested exactly.
    """

    planner: str
    problem: str
    seed: int
    success: bool
    path: tuple[Vector, ...] = ()
    cost: float = math.inf
    node_count: int = 0
    iterations: int = 0
    point_checks: int = 0
    segment_checks: int = 0
    cost_history: tuple[tuple[int, float], ...] = ()
    rewires: tuple[tuple[int, int, int], ...] = ()
    tree: SearchTree | None = field(default=None, repr=False)
    roadmap: Roadmap | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if self.success and len(self.path) < 2:
            raise ValueError("a successful result must carry a path of at least two points")
        if not self.success and math.isfinite(self.cost):
            raise ValueError("a failed result must carry an infinite cost")
        if self.point_checks < 0 or self.segment_checks < 0:
            raise ValueError("collision check counts cannot be negative")

    @property
    def collision_checks(self) -> int:
        """Total collision queries the run asked, of either kind."""
        return self.point_checks + self.segment_checks
