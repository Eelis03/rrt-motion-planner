"""The benchmark runner.

One run is one planner applied to one problem under one seed. Each run produces a
:class:`RunTrace`, a flat record with no references to the structures the planner
built, so a whole benchmark can be held in memory, serialised, and compared against a
stored reference without carrying trees and roadmaps along with it.

Seeds are shared across planners. Run ``i`` of a problem uses the same seed for every
planner, so the comparison is paired and the seed-to-seed variance that all planners
face is the same variance.
"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

from rrt_planner.algorithm.base import Planner
from rrt_planner.model.problem import PlanningProblem

__all__ = ["RunTrace", "load_traces", "run_benchmark", "save_traces", "seeds_for"]


@dataclass(frozen=True, slots=True)
class RunTrace:
    """The structured record of one planner run.

    Every field except ``wall_time_s`` is fixed by the seed, so the whole record is
    comparable across machines. The collision check counts are the effort measure
    that has that property, which is why they are recorded alongside wall time
    rather than instead of the node count.
    """

    planner: str
    problem: str
    seed: int
    success: bool
    cost: float
    node_count: int
    iterations: int
    point_checks: int
    segment_checks: int
    wall_time_s: float
    path_length: int

    @property
    def collision_checks(self) -> int:
        """Total collision queries the run asked, of either kind."""
        return self.point_checks + self.segment_checks


def seeds_for(repeats: int, base_seed: int) -> tuple[int, ...]:
    """Return the seed sequence used for ``repeats`` runs of every planner."""
    if repeats < 1:
        raise ValueError("repeats must be at least one")
    return tuple(base_seed + offset for offset in range(repeats))


def run_benchmark(
    planners: Sequence[Planner],
    problems: Sequence[PlanningProblem],
    *,
    repeats: int = 10,
    base_seed: int = 0,
) -> tuple[RunTrace, ...]:
    """Run every planner on every problem for every seed, in a fixed order."""
    if not planners:
        raise ValueError("at least one planner is required")
    if not problems:
        raise ValueError("at least one problem is required")
    seeds = seeds_for(repeats, base_seed)
    traces: list[RunTrace] = []
    for problem in problems:
        for planner in planners:
            for seed in seeds:
                traces.append(_run_once(planner, problem, seed))
    return tuple(traces)


def _run_once(planner: Planner, problem: PlanningProblem, seed: int) -> RunTrace:
    """Execute one run and time it with a monotonic clock."""
    started = time.perf_counter()
    result = planner.plan(problem, seed)
    elapsed = time.perf_counter() - started
    if result.success and not problem.path_is_valid(result.path):
        raise RuntimeError(
            f"{planner.name} returned an invalid path on {problem.name} with seed {seed}"
        )
    return RunTrace(
        planner=result.planner,
        problem=result.problem,
        seed=seed,
        success=result.success,
        cost=result.cost,
        node_count=result.node_count,
        iterations=result.iterations,
        point_checks=result.point_checks,
        segment_checks=result.segment_checks,
        wall_time_s=elapsed,
        path_length=len(result.path),
    )


def save_traces(path: Path, traces: Iterable[RunTrace]) -> None:
    """Write run traces as JSON. Infinite costs are stored as ``null``."""
    payload = []
    for trace in traces:
        record = asdict(trace)
        record["cost"] = None if not math.isfinite(trace.cost) else trace.cost
        payload.append(record)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_traces(path: Path) -> tuple[RunTrace, ...]:
    """Read run traces written by :func:`save_traces`."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    traces: list[RunTrace] = []
    for record in payload:
        cost = math.inf if record["cost"] is None else float(record["cost"])
        traces.append(
            RunTrace(
                planner=str(record["planner"]),
                problem=str(record["problem"]),
                seed=int(record["seed"]),
                success=bool(record["success"]),
                cost=cost,
                node_count=int(record["node_count"]),
                iterations=int(record["iterations"]),
                point_checks=int(record["point_checks"]),
                segment_checks=int(record["segment_checks"]),
                wall_time_s=float(record["wall_time_s"]),
                path_length=int(record["path_length"]),
            )
        )
    return tuple(traces)
