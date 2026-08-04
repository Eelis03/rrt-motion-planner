"""Aggregation of run traces into comparison metrics.

Cost is averaged over successful runs only, because the cost of a failed run is not a
number on the same scale as the others and treating it as a large finite value would
silently mix two different things. Node count, collision checks, and wall time are
averaged over every run, since all three are spent whether or not a path is found.
Where a statistic is undefined, for instance the cost of a planner that never
succeeded, it is reported as ``nan`` rather than replaced by a placeholder.

:func:`summarise` describes one planner at a time and discards the seed. The benchmark
gives every planner the same seed sequence, so the seed can carry more than that:
:func:`compare_paired` keeps it and reports the difference between two planners run by
run, whose spread is the spread of the difference rather than the sum of two spreads.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from rrt_planner.pipeline.benchmark import RunTrace

__all__ = ["PairedComparison", "Summary", "compare_paired", "mean_and_deviation", "summarise"]


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


@dataclass(frozen=True, slots=True)
class PairedComparison:
    """Two planners differenced seed by seed on one problem.

    Every difference is ``planner_a`` minus ``planner_b``, so a negative cost
    difference means the first planner returned the cheaper path. Costs are
    differenced over the seeds both planners solved, because the cost of a failed run
    is not a number on the same scale as the others. Collision checks are differenced
    over every shared seed, because a failed run spends them too.

    The seeds left out of the cost difference are reported rather than dropped, since
    a planner that fails on the seeds it finds hard would otherwise be flattered by
    exactly the runs it lost.
    """

    planner_a: str
    planner_b: str
    problem: str
    seeds: int
    both_succeeded: int
    a_only_succeeded: int
    b_only_succeeded: int
    cost_difference_mean: float
    cost_difference_std: float
    a_cheaper: int
    b_cheaper: int
    check_difference_mean: float
    check_difference_std: float

    @property
    def neither_succeeded(self) -> int:
        """Shared seeds on which both planners failed."""
        return self.seeds - self.both_succeeded - self.a_only_succeeded - self.b_only_succeeded


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


def compare_paired(
    traces: Iterable[RunTrace], planner_a: str, planner_b: str
) -> tuple[PairedComparison, ...]:
    """Difference two planners run by run, one row per problem, in first-seen order.

    Only the seeds both planners ran are compared, so a problem the two do not share
    is left out of the result rather than reported as a row of ``nan``. A planner
    named here but absent from ``traces`` is an error, because a misspelt name would
    otherwise be indistinguishable from a pair with nothing in common.
    """
    if planner_a == planner_b:
        raise ValueError("a paired comparison needs two different planners")

    runs: dict[tuple[str, str], dict[int, RunTrace]] = {}
    observed: set[str] = set()
    for trace in traces:
        runs.setdefault((trace.problem, trace.planner), {})[trace.seed] = trace
        observed.add(trace.planner)
    for name in (planner_a, planner_b):
        if name not in observed:
            raise ValueError(f"no runs recorded for planner {name}")

    comparisons: list[PairedComparison] = []
    for problem in dict.fromkeys(problem for problem, _ in runs):
        first = runs.get((problem, planner_a), {})
        second = runs.get((problem, planner_b), {})
        shared = sorted(first.keys() & second.keys())
        if not shared:
            continue
        pairs = [(first[seed], second[seed]) for seed in shared]
        solved = [(a, b) for a, b in pairs if a.success and b.success]
        cost_mean, cost_std = mean_and_deviation([a.cost - b.cost for a, b in solved])
        check_mean, check_std = mean_and_deviation(
            [float(a.collision_checks - b.collision_checks) for a, b in pairs]
        )
        comparisons.append(
            PairedComparison(
                planner_a=planner_a,
                planner_b=planner_b,
                problem=problem,
                seeds=len(pairs),
                both_succeeded=len(solved),
                a_only_succeeded=sum(1 for a, b in pairs if a.success and not b.success),
                b_only_succeeded=sum(1 for a, b in pairs if b.success and not a.success),
                cost_difference_mean=cost_mean,
                cost_difference_std=cost_std,
                a_cheaper=sum(1 for a, b in solved if a.cost < b.cost),
                b_cheaper=sum(1 for a, b in solved if b.cost < a.cost),
                check_difference_mean=check_mean,
                check_difference_std=check_std,
            )
        )
    return tuple(comparisons)
