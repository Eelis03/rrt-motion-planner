"""Fixtures shared by the test tiers.

Planner budgets here are deliberately small. The tiers that check correctness do not
need a large sample count, and the whole suite is required to finish in well under two
minutes.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.algorithm.base import Planner
from rrt_planner.model.obstacles import Box, Circle, ObstacleSet
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.model.space import ConfigurationSpace

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="session")
def plane() -> ConfigurationSpace:
    """The ten by ten square used by most of the tests."""
    return ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0]))


@pytest.fixture(scope="session")
def free_problem(plane: ConfigurationSpace) -> PlanningProblem:
    """A problem with no obstacles at all."""
    return PlanningProblem(
        name="free",
        space=plane,
        obstacles=ObstacleSet.empty(),
        start=np.array([1.0, 1.0]),
        goal=np.array([9.0, 9.0]),
    )


@pytest.fixture(scope="session")
def blocked_problem(plane: ConfigurationSpace) -> PlanningProblem:
    """A problem whose direct route is blocked by a wall and a disc."""
    obstacles = ObstacleSet(
        (
            Box(lower=np.array([4.0, 0.0]), upper=np.array([5.0, 7.0])),
            Circle(center=np.array([7.0, 8.5]), radius=1.2),
        )
    )
    return PlanningProblem(
        name="blocked",
        space=plane,
        obstacles=obstacles,
        start=np.array([1.0, 1.0]),
        goal=np.array([9.0, 9.0]),
    )


@pytest.fixture(scope="session")
def sealed_problem(plane: ConfigurationSpace) -> PlanningProblem:
    """A problem whose goal is walled off, so no planner can succeed."""
    obstacles = ObstacleSet(
        (
            Box(lower=np.array([7.0, 6.0]), upper=np.array([7.5, 10.0])),
            Box(lower=np.array([7.0, 6.0]), upper=np.array([10.0, 6.5])),
        )
    )
    return PlanningProblem(
        name="sealed",
        space=plane,
        obstacles=obstacles,
        start=np.array([1.0, 1.0]),
        goal=np.array([9.0, 9.0]),
    )


def small_planners() -> tuple[Planner, ...]:
    """The three planners under budgets small enough for a fast test run."""
    return (
        RRT(max_samples=1500, step_size=0.6),
        RRTStar(max_samples=600, step_size=0.6),
        PRM(milestones=250, neighbours=8),
    )


@pytest.fixture(params=small_planners(), ids=lambda planner: planner.name)
def planner(request: pytest.FixtureRequest) -> Planner:
    """Each planner in turn, so that invariants are checked against all three."""
    planner_under_test: Planner = request.param
    return planner_under_test
