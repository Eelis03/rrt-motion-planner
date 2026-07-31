"""Tier two: a recorded reference result, compared with a numeric tolerance.

The reference in ``tests/data/reference_benchmark.json`` was produced by
:func:`reference_traces` and is the behaviour this repository claims. A change to
sampling, to steering, to the nearest neighbour index, or to any obstacle test moves
these numbers, so an unexplained difference here is a behavioural change and not a
tolerance problem.

Wall time is not compared: it is the one recorded quantity that depends on the machine.
Costs are compared with a relative tolerance of ``1e-6``, which absorbs the last-place
floating point differences between platforms while remaining far tighter than any real
change in planner behaviour, and discrete counts are compared exactly.
"""

from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path

import pytest

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.algorithm.base import Planner
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.pipeline.benchmark import RunTrace, run_benchmark, save_traces
from rrt_planner.pipeline.suite import standard_problems

REFERENCE_PATH = Path(__file__).parent / "data" / "reference_benchmark.json"
REFERENCE_PROBLEMS = ("cluttered", "narrow_passage")
REFERENCE_REPEATS = 3
REFERENCE_BASE_SEED = 0
COST_TOLERANCE = 1e-6


def reference_planners() -> tuple[Planner, ...]:
    """The exact planner configuration the reference was recorded with."""
    return (
        RRT(max_samples=1200, step_size=0.6, goal_bias=0.05),
        RRTStar(max_samples=600, step_size=0.6, goal_bias=0.05, gamma_scale=1.1),
        PRM(milestones=250, neighbours=8),
    )


def reference_problems() -> tuple[PlanningProblem, ...]:
    """The problems the reference was recorded on."""
    by_name = {problem.name: problem for problem in standard_problems()}
    return tuple(by_name[name] for name in REFERENCE_PROBLEMS)


def reference_traces() -> tuple[RunTrace, ...]:
    """Re-run the recorded configuration."""
    return run_benchmark(
        reference_planners(),
        reference_problems(),
        repeats=REFERENCE_REPEATS,
        base_seed=REFERENCE_BASE_SEED,
    )


def record_reference(destination: Path = REFERENCE_PATH) -> Path:
    """Rewrite the reference file. Run this deliberately, never as part of a test.

    Wall time is stored as zero rather than as measured, so that nothing in the file
    invites comparison against a quantity that depends on the machine.
    """
    timeless = tuple(replace(trace, wall_time_s=0.0) for trace in reference_traces())
    save_traces(destination, timeless)
    return destination


def load_reference() -> list[dict[str, object]]:
    """Read the recorded reference as plain dictionaries."""
    payload = json.loads(REFERENCE_PATH.read_text(encoding="utf-8"))
    assert isinstance(payload, list)
    return payload


class TestReferenceFile:
    """The recorded file itself is well formed."""

    def test_covers_every_planner_problem_and_seed(self) -> None:
        records = load_reference()
        assert len(records) == 3 * len(REFERENCE_PROBLEMS) * REFERENCE_REPEATS
        assert {record["planner"] for record in records} == {"RRT", "RRT star", "PRM"}
        assert {record["problem"] for record in records} == set(REFERENCE_PROBLEMS)
        assert {record["seed"] for record in records} == {0, 1, 2}

    def test_records_a_successful_run_for_every_planner(self) -> None:
        successes = {record["planner"] for record in load_reference() if record["success"]}
        assert successes == {"RRT", "RRT star", "PRM"}


class TestRegression:
    """The current code reproduces the recorded numbers."""

    def test_reproduces_the_recorded_run(self) -> None:
        recorded = load_reference()
        produced = reference_traces()
        assert len(produced) == len(recorded)

        for expected, actual in zip(recorded, produced, strict=True):
            label = f"{actual.planner} on {actual.problem} with seed {actual.seed}"
            assert actual.planner == expected["planner"], label
            assert actual.problem == expected["problem"], label
            assert actual.seed == expected["seed"], label
            assert actual.success == expected["success"], label
            assert actual.node_count == expected["node_count"], label
            assert actual.iterations == expected["iterations"], label
            assert actual.path_length == expected["path_length"], label
            if expected["cost"] is None:
                assert math.isinf(actual.cost), label
            else:
                assert actual.cost == pytest.approx(
                    float(expected["cost"]), rel=COST_TOLERANCE
                ), label

    def test_the_reference_run_is_itself_repeatable(self) -> None:
        first = reference_traces()
        second = reference_traces()
        assert [(t.planner, t.cost, t.node_count) for t in first] == [
            (t.planner, t.cost, t.node_count) for t in second
        ]
