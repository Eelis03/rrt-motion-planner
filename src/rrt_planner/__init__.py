"""Sampling-based motion planning with RRT, RRT star, and PRM.

The package is arranged in layers, each depending only on the ones before it:

``rrt_planner.model``
    Value types: configuration spaces, obstacles, search trees and roadmaps, problems
    and results. No input or output, no algorithms.
``rrt_planner.algorithm``
    The three planners and the ``Planner`` protocol they share, plus the steering
    function and the nearest neighbour index. No plotting, no file access.
``rrt_planner.pipeline``
    The standard problem set, the seeded benchmark runner, and trace export.
``rrt_planner.analysis``
    Aggregation of run traces into comparison metrics, tables, and figures.

The example scripts under ``examples/`` are wiring only and hold no logic.
"""

from rrt_planner.algorithm import PRM, RRT, Planner, RRTStar
from rrt_planner.model import (
    Box,
    Circle,
    ConfigurationSpace,
    ConvexPolygon,
    ObstacleSet,
    PlanningProblem,
    PlanResult,
)
from rrt_planner.pipeline import run_benchmark, standard_problems

__all__ = [
    "PRM",
    "RRT",
    "Box",
    "Circle",
    "ConfigurationSpace",
    "ConvexPolygon",
    "ObstacleSet",
    "PlanResult",
    "Planner",
    "PlanningProblem",
    "RRTStar",
    "__version__",
    "run_benchmark",
    "standard_problems",
]

__version__ = "0.1.0"
