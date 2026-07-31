"""The standard problem set used by the benchmark.

Every planar problem uses the same ten by ten square and the same start and goal, so
one step size and one set of planner parameters are meaningful across all of them and
the differences in the results come from the obstacles alone. The last problem is a
three-dimensional cube, included so that the dimension-independent parts of the code
are exercised by the benchmark and not only by the unit tests.
"""

from __future__ import annotations

import numpy as np

from rrt_planner.model.obstacles import Box, Circle, ConvexPolygon, ObstacleSet
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.model.space import ConfigurationSpace

__all__ = [
    "PROBLEM_BUILDERS",
    "cluttered_problem",
    "empty_problem",
    "maze_problem",
    "narrow_passage_problem",
    "polygon_field_problem",
    "standard_problems",
    "three_dimensional_problem",
]

_PLANE = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0]))
_START = np.array([0.5, 0.5])
_GOAL = np.array([9.5, 9.5])


def empty_problem() -> PlanningProblem:
    """A ten by ten square with no obstacles: the reference case for path quality."""
    return PlanningProblem(
        name="empty",
        space=_PLANE,
        obstacles=ObstacleSet.empty(),
        start=_START,
        goal=_GOAL,
    )


def cluttered_problem() -> PlanningProblem:
    """A regular grid of discs, which leaves many homotopy classes open.

    Three of the nine discs sit on the straight line between the start and the goal,
    so the straight-line cost is a strict lower bound that cannot be attained.
    """
    centres = [(x + 0.5, y + 0.5) for x in range(2, 9, 3) for y in range(2, 9, 3)]
    obstacles = tuple(Circle(center=np.array(centre), radius=1.0) for centre in centres)
    return PlanningProblem(
        name="cluttered",
        space=_PLANE,
        obstacles=ObstacleSet(obstacles),
        start=_START,
        goal=_GOAL,
    )


def narrow_passage_problem() -> PlanningProblem:
    """A wall pierced by one gap, the classical hard case for sampling-based planning.

    The gap is 0.4 wide in a wall 1.0 thick, so the corridor covers 0.4 percent of the
    sampling domain, and it is placed away from the straight line between the start and
    the goal so that a planner has to find it rather than pass through it by chance.
    """
    obstacles = (
        Box(lower=np.array([4.5, 0.0]), upper=np.array([5.5, 7.6])),
        Box(lower=np.array([4.5, 8.0]), upper=np.array([5.5, 10.0])),
    )
    return PlanningProblem(
        name="narrow_passage",
        space=_PLANE,
        obstacles=ObstacleSet(obstacles),
        start=_START,
        goal=_GOAL,
    )


def maze_problem() -> PlanningProblem:
    """Three staggered walls that force a long detour in each direction."""
    obstacles = (
        Box(lower=np.array([2.0, 0.0]), upper=np.array([2.8, 7.5])),
        Box(lower=np.array([5.0, 2.5]), upper=np.array([5.8, 10.0])),
        Box(lower=np.array([8.0, 0.0]), upper=np.array([8.8, 7.5])),
    )
    return PlanningProblem(
        name="maze",
        space=_PLANE,
        obstacles=ObstacleSet(obstacles),
        start=_START,
        goal=_GOAL,
    )


def polygon_field_problem() -> PlanningProblem:
    """Convex polygons of several shapes, which exercise the half-space clipping test."""
    obstacles = (
        ConvexPolygon(vertices=np.array([[2.0, 1.0], [4.0, 1.5], [3.0, 4.0]])),
        ConvexPolygon(vertices=np.array([[5.5, 4.0], [8.0, 3.0], [8.5, 5.5], [6.0, 6.5]])),
        ConvexPolygon(vertices=np.array([[1.0, 6.0], [3.5, 5.5], [3.0, 8.5], [1.5, 8.0]])),
        ConvexPolygon(vertices=np.array([[6.0, 8.0], [9.0, 7.5], [8.0, 9.5]])),
        Circle(center=np.array([5.0, 1.5]), radius=0.8),
        Box(lower=np.array([0.5, 3.0]), upper=np.array([1.5, 4.5])),
    )
    return PlanningProblem(
        name="polygon_field",
        space=_PLANE,
        obstacles=ObstacleSet(obstacles),
        start=_START,
        goal=_GOAL,
    )


def three_dimensional_problem() -> PlanningProblem:
    """A cube containing balls and boxes, used to keep the code dimension-independent.

    The cube is five units on a side rather than ten, so that the same step size gives
    a comparable sample density to the planar problems despite the extra dimension.
    """
    space = ConfigurationSpace(
        lower=np.array([0.0, 0.0, 0.0]), upper=np.array([5.0, 5.0, 5.0])
    )
    obstacles = (
        Circle(center=np.array([1.5, 1.5, 1.5]), radius=0.8),
        Circle(center=np.array([2.5, 2.5, 2.5]), radius=0.7),
        Circle(center=np.array([3.5, 3.5, 3.5]), radius=0.8),
        Box(lower=np.array([1.0, 3.0, 0.0]), upper=np.array([1.5, 4.0, 3.0])),
        Box(lower=np.array([3.0, 1.0, 2.0]), upper=np.array([4.0, 1.5, 5.0])),
    )
    return PlanningProblem(
        name="cube_3d",
        space=space,
        obstacles=ObstacleSet(obstacles),
        start=np.array([0.25, 0.25, 0.25]),
        goal=np.array([4.75, 4.75, 4.75]),
    )


PROBLEM_BUILDERS = (
    empty_problem,
    cluttered_problem,
    narrow_passage_problem,
    maze_problem,
    polygon_field_problem,
    three_dimensional_problem,
)


def standard_problems() -> tuple[PlanningProblem, ...]:
    """Build the standard problem set, in the order the benchmark reports it."""
    return tuple(builder() for builder in PROBLEM_BUILDERS)
