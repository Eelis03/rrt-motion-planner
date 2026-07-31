"""Tier one: the properties that separate RRT star from RRT.

The published guarantee is asymptotic: the cost of the RRT star solution converges to
the optimum almost surely as the sample count grows. A test cannot check a limit, so it
checks the finite-sample consequence that makes the limit possible, namely that the
incumbent cost never rises when the budget is increased, and it checks the rewiring
radius against the formula it is supposed to implement.
"""

from __future__ import annotations

import math
from itertools import pairwise

import numpy as np
import pytest

from rrt_planner.algorithm import RRT, RRTStar
from rrt_planner.algorithm.rrt_star import unit_ball_volume
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.model.space import ConfigurationSpace


class TestUnitBallVolume:
    """Hand-computed values of ``zeta_d``."""

    @pytest.mark.parametrize(
        ("dimension", "expected"),
        [
            (1, 2.0),
            (2, math.pi),
            (3, 4.0 * math.pi / 3.0),
            (4, math.pi**2 / 2.0),
            (5, 8.0 * math.pi**2 / 15.0),
        ],
    )
    def test_matches_the_closed_form(self, dimension: int, expected: float) -> None:
        assert unit_ball_volume(dimension) == pytest.approx(expected, rel=1e-12)


class TestRewiringRadius:
    """The radius follows Karaman and Frazzoli, equation for ``r(n)``."""

    def test_gamma_exceeds_the_published_threshold(self) -> None:
        space = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0]))
        planner = RRTStar(gamma_scale=1.1)
        threshold = (
            2.0 * (1.0 + 1.0 / 2.0) ** (1.0 / 2.0) * (100.0 / math.pi) ** (1.0 / 2.0)
        )
        assert planner.gamma(space) > threshold
        assert planner.gamma(space) == pytest.approx(1.1 * threshold, rel=1e-12)

    def test_radius_matches_the_formula(self) -> None:
        space = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0]))
        planner = RRTStar(step_size=10.0, gamma_scale=1.1)
        expected = planner.gamma(space) * math.sqrt(math.log(100.0) / 100.0)
        assert planner.rewiring_radius(space, 100) == pytest.approx(expected, rel=1e-12)
        # 1.1 * 2 * sqrt(1.5) * sqrt(100 / pi) * sqrt(ln 100 / 100) = 3.26224
        assert planner.rewiring_radius(space, 100) == pytest.approx(3.26224, abs=1e-5)

    def test_radius_shrinks_as_the_tree_grows(self) -> None:
        space = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0]))
        planner = RRTStar(step_size=10.0)
        radii = [planner.rewiring_radius(space, count) for count in (100, 1000, 10000)]
        assert radii[0] > radii[1] > radii[2]

    def test_radius_never_exceeds_the_step_size(self) -> None:
        space = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0]))
        planner = RRTStar(step_size=0.5)
        for count in (2, 10, 100, 1000, 100000):
            assert planner.rewiring_radius(space, count) <= 0.5


class TestCostBehaviour:
    """Cost against sampling effort."""

    @pytest.mark.parametrize("seed", [0, 1, 2])
    def test_cost_does_not_rise_when_the_budget_grows(
        self, seed: int, blocked_problem: PlanningProblem
    ) -> None:
        costs = [
            RRTStar(max_samples=budget, step_size=0.6).plan(blocked_problem, seed).cost
            for budget in (250, 500, 1000)
        ]
        assert costs[0] >= costs[1] >= costs[2]
        assert math.isfinite(costs[-1])

    def test_cost_history_decreases_strictly(self, blocked_problem: PlanningProblem) -> None:
        result = RRTStar(max_samples=1000, step_size=0.6).plan(blocked_problem, seed=3)
        assert len(result.cost_history) >= 2
        iterations = [step for step, _ in result.cost_history]
        costs = [cost for _, cost in result.cost_history]
        assert iterations == sorted(iterations)
        assert all(later < earlier for earlier, later in pairwise(costs))
        assert costs[-1] == pytest.approx(result.cost, rel=1e-12)

    def test_rewiring_improves_on_plain_rrt(self, blocked_problem: PlanningProblem) -> None:
        # Averaged over seeds rather than asserted per seed: RRT star is better in
        # expectation, and a single seed can always be unlucky.
        budget = 1000
        seeds = range(8)
        plain = [RRT(max_samples=budget, step_size=0.6).plan(blocked_problem, s) for s in seeds]
        starred = [
            RRTStar(max_samples=budget, step_size=0.6).plan(blocked_problem, s) for s in seeds
        ]
        assert all(result.success for result in plain)
        assert all(result.success for result in starred)
        mean_plain = sum(result.cost for result in plain) / len(plain)
        mean_starred = sum(result.cost for result in starred) / len(starred)
        assert mean_starred < mean_plain

    def test_approaches_the_straight_line_in_open_space(
        self, free_problem: PlanningProblem
    ) -> None:
        result = RRTStar(max_samples=2000, step_size=0.6).plan(free_problem, seed=5)
        assert result.success is True
        excess = result.cost / free_problem.straight_line_cost - 1.0
        assert 0.0 <= excess < 0.05
