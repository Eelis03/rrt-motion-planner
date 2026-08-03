"""Aggregation of run traces into comparison metrics.

Cost is averaged over successful runs only, because the cost of a failed run is not a
number on the same scale as the others and treating it as a large finite value would
silently mix two different things. Node count, collision checks, and wall time are
averaged over every run, since all three are spent whether or not a path is found.
Where a statistic is undefined, for instance the cost of a planner that never
succeeded, it is reported as ``nan`` rather than replaced by a placeholder.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from rrt_planner.pipeline.benchmark import RunTrace

__all__ = ["Summary", "mean_and_deviation", "summarise"]


@dataclass(frozen=True, slots=True)
class Summary:
    """Aggregated metrics for one planner on one problem."""

    planner: str
    problem: str
    runs: int
    successes: int
    cost_mean: float
    cost_std: float
    node_count_mean: float
    node_count_std: float
    collision_check_mean: float
    collision_check_std: float
    wall_time_mean: float
    wall_time_std: float

    @property
    def success_rate(self) -> float:
        """Fraction of runs that returned a valid path."""
        return self.successes / self.runs if self.runs else math.nan


def mean_and_deviation(values: Sequence[float]) -> tuple[float, float]:
    """Return the mean and the sample standard deviation of ``values``.

    The deviation uses the Bessel-corrected denominator ``n - 1``, which is the right
    choice when the runs are a sample of the seed distribution rather than the whole of
    it. A single value has a mean and no spread, so its deviation is reported as zero.
    An empty sequence has neither, and both are reported as ``nan``.
    """
    count = len(values)
    if count == 0:
        return math.nan, math.nan
    mean = math.fsum(values) / count
    if count == 1:
        return mean, 0.0
    variance = math.fsum((value - mean) ** 2 for value in values) / (count - 1)
    return mean, math.sqrt(variance)


def summarise(traces: Iterable[RunTrace]) -> tuple[Summary, ...]:
    """Group traces by problem and planner, preserving first-seen order."""
    grouped: dict[tuple[str, str], list[RunTrace]] = {}
    for trace in traces:
        grouped.setdefault((trace.problem, trace.planner), []).append(trace)

    summaries: list[Summary] = []
    for (problem, planner), group in grouped.items():
        successful = [trace for trace in group if trace.success]
        cost_mean, cost_std = mean_and_deviation([trace.cost for trace in successful])
        node_mean, node_std = mean_and_deviation([float(t.node_count) for t in group])
        check_mean, check_std = mean_and_deviation([float(t.collision_checks) for t in group])
        time_mean, time_std = mean_and_deviation([t.wall_time_s for t in group])
        summaries.append(
            Summary(
                planner=planner,
                problem=problem,
                runs=len(group),
                successes=len(successful),
                cost_mean=cost_mean,
                cost_std=cost_std,
                node_count_mean=node_mean,
                node_count_std=node_std,
                collision_check_mean=check_mean,
                collision_check_std=check_std,
                wall_time_mean=time_mean,
                wall_time_std=time_std,
            )
        )
    return tuple(summaries)
