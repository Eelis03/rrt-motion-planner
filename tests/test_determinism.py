"""Tier one: a seed fixes the result exactly.

Determinism is what makes the benchmark meaningful and the regression tier possible, so
it is checked structure by structure rather than through the summary numbers alone.
"""

from __future__ import annotations

import numpy as np
import pytest

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.algorithm.base import Planner
from rrt_planner.model.graph import SearchTree
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.pipeline.benchmark import run_benchmark


def assert_trees_are_identical(first: SearchTree | None, second: SearchTree | None) -> None:
    """Compare two search trees vertex by vertex."""
    assert first is not None
    assert second is not None
    assert first.size == second.size
    assert first.parents == second.parents
    assert first.insertion_parents == second.insertion_parents
    assert first.costs == second.costs
    for left, right in zip(first.configurations, second.configurations, strict=True):
        assert np.array_equal(left, right)


class TestRepeatability:
    """The same inputs give byte-identical structures."""

    def test_the_same_seed_produces_the_same_result(
        self, planner: Planner, blocked_problem: PlanningProblem
    ) -> None:
        first = planner.plan(blocked_problem, seed=42)
        second = planner.plan(blocked_problem, seed=42)
        assert first.success == second.success
        assert first.cost == second.cost
        assert first.node_count == second.node_count
        assert first.iterations == second.iterations
        assert len(first.path) == len(second.path)
        for a, b in zip(first.path, second.path, strict=True):
            assert np.array_equal(a, b)

    @pytest.mark.parametrize("factory", [RRT, RRTStar])
    def test_the_same_seed_produces_the_same_tree(
        self, factory: type[RRT] | type[RRTStar], blocked_problem: PlanningProblem
    ) -> None:
        planner = factory(max_samples=500, step_size=0.6)
        first = planner.plan(blocked_problem, seed=7)
        second = planner.plan(blocked_problem, seed=7)
        assert_trees_are_identical(first.tree, second.tree)

    def test_the_same_seed_produces_the_same_roadmap(
        self, blocked_problem: PlanningProblem
    ) -> None:
        prm = PRM(milestones=200, neighbours=8)
        first, _ = prm.build(blocked_problem.space, blocked_problem.obstacles, seed=7)
        second, _ = prm.build(blocked_problem.space, blocked_problem.obstacles, seed=7)
        assert first.edges() == second.edges()
        for a, b in zip(first.configurations, second.configurations, strict=True):
            assert np.array_equal(a, b)

    def test_different_seeds_explore_differently(
        self, blocked_problem: PlanningProblem
    ) -> None:
        planner = RRT(max_samples=500, step_size=0.6)
        first = planner.plan(blocked_problem, seed=1)
        second = planner.plan(blocked_problem, seed=2)
        assert first.cost != second.cost

    def test_a_planner_instance_is_not_disturbed_by_earlier_runs(
        self, planner: Planner, blocked_problem: PlanningProblem, free_problem: PlanningProblem
    ) -> None:
        reference = planner.plan(blocked_problem, seed=13)
        planner.plan(free_problem, seed=99)
        planner.plan(blocked_problem, seed=1)
        repeated = planner.plan(blocked_problem, seed=13)
        assert repeated.cost == reference.cost
        assert repeated.node_count == reference.node_count

    def test_the_benchmark_is_reproducible(self, blocked_problem: PlanningProblem) -> None:
        planners = (RRT(max_samples=400, step_size=0.6), PRM(milestones=120, neighbours=6))
        first = run_benchmark(planners, (blocked_problem,), repeats=3, base_seed=5)
        second = run_benchmark(planners, (blocked_problem,), repeats=3, base_seed=5)
        assert [(t.planner, t.seed, t.success, t.cost) for t in first] == [
            (t.planner, t.seed, t.success, t.cost) for t in second
        ]
