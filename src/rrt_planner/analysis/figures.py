"""Matplotlib figures built from problems, results, and aggregated metrics.

The non-interactive Agg backend is selected before ``pyplot`` is imported, so importing
this module never requires a display and the example scripts behave identically under
continuous integration.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Circle as CirclePatch
from matplotlib.patches import Polygon as PolygonPatch
from matplotlib.patches import Rectangle

from rrt_planner.analysis.metrics import Summary
from rrt_planner.model.obstacles import Box, Circle, ConvexPolygon
from rrt_planner.model.problem import PlanningProblem, PlanResult

__all__ = [
    "convergence_figure",
    "draw_problem",
    "save_figure",
    "solution_figure",
    "summary_figure",
]

_OBSTACLE_STYLE: dict[str, Any] = {
    "facecolor": "0.75",
    "edgecolor": "0.35",
    "linewidth": 0.8,
}


def draw_problem(axes: Axes, problem: PlanningProblem) -> None:
    """Draw the bounds, the obstacles, the start, and the goal of a planar problem."""
    if problem.dimension != 2:
        raise ValueError("only planar problems can be drawn")
    for obstacle in problem.obstacles.obstacles:
        if isinstance(obstacle, Circle):
            axes.add_patch(
                CirclePatch(
                    (float(obstacle.center[0]), float(obstacle.center[1])),
                    float(obstacle.radius),
                    **_OBSTACLE_STYLE,
                )
            )
        elif isinstance(obstacle, Box):
            width, height = obstacle.upper - obstacle.lower
            axes.add_patch(
                Rectangle(
                    (float(obstacle.lower[0]), float(obstacle.lower[1])),
                    float(width),
                    float(height),
                    **_OBSTACLE_STYLE,
                )
            )
        elif isinstance(obstacle, ConvexPolygon):
            axes.add_patch(PolygonPatch(obstacle.vertices, closed=True, **_OBSTACLE_STYLE))
    axes.plot(*problem.start, marker="o", color="tab:green", markersize=7, linestyle="none")
    axes.plot(*problem.goal, marker="*", color="tab:red", markersize=12, linestyle="none")
    axes.set_xlim(float(problem.space.lower[0]), float(problem.space.upper[0]))
    axes.set_ylim(float(problem.space.lower[1]), float(problem.space.upper[1]))
    axes.set_aspect("equal")


def _draw_structure(axes: Axes, result: PlanResult) -> None:
    """Draw the tree or the roadmap the planner built."""
    segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    if result.tree is not None:
        points = result.tree.configurations
        segments = [
            ((float(points[a][0]), float(points[a][1])), (float(points[b][0]), float(points[b][1])))
            for a, b in result.tree.edges()
        ]
    elif result.roadmap is not None:
        points = result.roadmap.configurations
        segments = [
            ((float(points[a][0]), float(points[a][1])), (float(points[b][0]), float(points[b][1])))
            for a, b in result.roadmap.edges()
        ]
    # Solid rather than semi-transparent: where a tree is dense, overlapping strokes at
    # partial opacity blend into a single dark mass and stop showing the branching that
    # is the point of drawing the structure at all. A light solid tone keeps the tree
    # behind the path and keeps the number of distinct tones in the image small.
    for (x0, y0), (x1, y1) in segments:
        axes.plot([x0, x1], [y0, y1], color="#8fb8dc", linewidth=0.3, zorder=1)


def solution_figure(
    problem: PlanningProblem, results: Sequence[PlanResult], columns: int = 3
) -> Figure:
    """Draw one panel per result: the search structure and the path it produced."""
    count = max(len(results), 1)
    columns = min(columns, count)
    rows = (count + columns - 1) // columns
    figure, axes_grid = plt.subplots(
        rows, columns, figsize=(4.2 * columns, 4.4 * rows), squeeze=False
    )
    flat = [axes for row in axes_grid for axes in row]
    for axes, result in zip(flat, results, strict=False):
        draw_problem(axes, problem)
        _draw_structure(axes, result)
        if result.success:
            path = np.vstack(result.path)
            axes.plot(path[:, 0], path[:, 1], color="tab:orange", linewidth=2.0, zorder=3)
            title = f"{result.planner}: cost {result.cost:.2f}, {result.node_count} nodes"
        else:
            title = f"{result.planner}: no path, {result.node_count} nodes"
        axes.set_title(title, fontsize=10)
    for axes in flat[len(results) :]:
        axes.set_axis_off()
    figure.suptitle(f"Problem: {problem.name}")
    figure.tight_layout()
    return figure


def convergence_figure(results: Sequence[PlanResult]) -> Figure:
    """Plot the cost of the incumbent solution against the iteration that produced it."""
    figure, axes = plt.subplots(figsize=(6.4, 4.2))
    for result in results:
        if not result.cost_history:
            continue
        iterations = [step for step, _ in result.cost_history]
        costs = [cost for _, cost in result.cost_history]
        iterations.append(result.iterations)
        costs.append(costs[-1])
        axes.step(iterations, costs, where="post", label=f"{result.planner}, seed {result.seed}")
    axes.set_xlabel("iteration")
    axes.set_ylabel("cost of the incumbent solution")
    axes.set_title("Solution cost against sampling effort")
    axes.grid(True, linewidth=0.3, alpha=0.6)
    axes.legend(fontsize=8)
    figure.tight_layout()
    return figure


def summary_figure(summaries: Sequence[Summary]) -> Figure:
    """Draw grouped bars for success rate, cost, collision checks, and wall time."""
    problems = list(dict.fromkeys(summary.problem for summary in summaries))
    planners = list(dict.fromkeys(summary.planner for summary in summaries))
    lookup = {(s.problem, s.planner): s for s in summaries}

    figure, axes_row = plt.subplots(1, 4, figsize=(17.5, 4.2))
    positions = np.arange(len(problems), dtype=float)
    width = 0.8 / max(len(planners), 1)

    panels: tuple[tuple[str, Callable[[Summary], float], Callable[[Summary], float] | None], ...]
    panels = (
        ("success rate", lambda summary: summary.success_rate, None),
        ("mean path cost", lambda summary: summary.cost_mean, lambda summary: summary.cost_std),
        (
            "mean collision checks",
            lambda summary: summary.collision_check_mean,
            lambda summary: summary.collision_check_std,
        ),
        (
            "mean wall time (s)",
            lambda summary: summary.wall_time_mean,
            lambda summary: summary.wall_time_std,
        ),
    )
    for axes, (label, value_of, error_of) in zip(axes_row, panels, strict=True):
        for offset, planner in enumerate(planners):
            values = [
                value_of(lookup[(problem, planner)]) if (problem, planner) in lookup else np.nan
                for problem in problems
            ]
            errors = None
            if error_of is not None:
                errors = [
                    error_of(lookup[(problem, planner)])
                    if (problem, planner) in lookup
                    else np.nan
                    for problem in problems
                ]
            axes.bar(
                positions + offset * width,
                values,
                width=width,
                yerr=errors,
                capsize=2.5,
                label=planner,
            )
        axes.set_xticks(positions + width * (len(planners) - 1) / 2.0)
        axes.set_xticklabels(problems, rotation=30, ha="right", fontsize=8)
        axes.set_ylabel(label)
        axes.grid(True, axis="y", linewidth=0.3, alpha=0.6)
    axes_row[0].legend(fontsize=8)
    figure.suptitle("Benchmark summary")
    figure.tight_layout()
    return figure


def save_figure(figure: Figure, path: Path, dpi: int = 150) -> Path:
    """Write ``figure`` to ``path``, creating the directory, and close it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=dpi)
    plt.close(figure)
    return path
