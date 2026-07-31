"""Export of a planner run to the JSON trace consumed by the visualisation layer.

The document is deliberately flat and free of NaN or infinity, so that it survives a
round trip through ``JSON.parse`` in a browser without special cases. Nothing in the
Python package reads it back; it exists only so that ``viz/`` can be developed and run
without a Python runtime.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from rrt_planner.model.obstacles import Box, Circle, ConvexPolygon, Obstacle
from rrt_planner.model.problem import PlanningProblem, PlanResult

__all__ = ["TRACE_FORMAT", "TRACE_VERSION", "obstacle_document", "trace_document", "write_trace"]

TRACE_FORMAT = "rrt-planner-trace"
TRACE_VERSION = 1


def obstacle_document(obstacle: Obstacle) -> dict[str, Any]:
    """Represent one obstacle as a JSON-compatible dictionary."""
    if isinstance(obstacle, Circle):
        return {
            "kind": "circle",
            "center": [float(value) for value in obstacle.center],
            "radius": float(obstacle.radius),
        }
    if isinstance(obstacle, Box):
        return {
            "kind": "box",
            "lower": [float(value) for value in obstacle.lower],
            "upper": [float(value) for value in obstacle.upper],
        }
    if isinstance(obstacle, ConvexPolygon):
        return {
            "kind": "polygon",
            "vertices": [[float(x), float(y)] for x, y in obstacle.vertices],
        }
    raise TypeError(f"no trace representation for {type(obstacle).__name__}")


def trace_document(problem: PlanningProblem, result: PlanResult) -> dict[str, Any]:
    """Build the trace document for one planner run on one planar problem."""
    if problem.dimension != 2:
        raise ValueError("only planar problems can be exported to the visualisation trace")
    structure = _structure_document(result)
    return {
        "format": TRACE_FORMAT,
        "version": TRACE_VERSION,
        "planner": result.planner,
        "problem": result.problem,
        "seed": result.seed,
        "success": result.success,
        "cost": float(result.cost) if result.success else None,
        "nodeCount": result.node_count,
        "iterations": result.iterations,
        "bounds": {
            "lower": [float(value) for value in problem.space.lower],
            "upper": [float(value) for value in problem.space.upper],
        },
        "start": [float(value) for value in problem.start],
        "goal": [float(value) for value in problem.goal],
        "obstacles": [obstacle_document(obstacle) for obstacle in problem.obstacles.obstacles],
        "structure": structure,
        "path": [[float(point[0]), float(point[1])] for point in result.path],
        "costHistory": [[int(step), float(cost)] for step, cost in result.cost_history],
    }


def _structure_document(result: PlanResult) -> dict[str, Any]:
    """Represent the tree or the roadmap the planner built."""
    if result.tree is not None:
        tree = result.tree
        return {
            "kind": "tree",
            "vertices": [[float(q[0]), float(q[1])] for q in tree.configurations],
            "parents": list(tree.insertion_parents),
            "edges": [],
            "rewires": [
                [int(step), int(node), int(parent)]
                for step, node, parent in result.rewires
            ],
        }
    if result.roadmap is not None:
        roadmap = result.roadmap
        return {
            "kind": "roadmap",
            "vertices": [[float(q[0]), float(q[1])] for q in roadmap.configurations],
            "parents": [],
            "edges": [[int(first), int(second)] for first, second in roadmap.edges()],
            "rewires": [],
        }
    raise ValueError("the result carries neither a tree nor a roadmap")


def write_trace(path: Path, problem: PlanningProblem, result: PlanResult) -> Path:
    """Write the trace document for one run and return the path written."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(trace_document(problem, result), indent=1), encoding="utf-8"
    )
    return path
