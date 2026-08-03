"""Tier one: invariants that every planner must satisfy."""

from __future__ import annotations

import math
from itertools import pairwise

import numpy as np
import pytest

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.algorithm.base import Planner
from rrt_planner.model.graph import NO_PARENT
from rrt_planner.model.obstacles import CountingChecker
from rrt_planner.model.problem import PlanningProblem, PlanResult, path_cost


def assert_path_is_admissible(problem: PlanningProblem, path: tuple[np.ndarray, ...]) -> None:
    """Check the path invariants directly, without relying on the helper under test."""
    assert len(path) >= 2
    assert np.allclose(path[0], problem.start)
    assert np.allclose(path[-1], problem.goal)
    for point in path:
        assert problem.space.contains(point)
        assert problem.obstacles.is_free(point)
    for first, second in pairwise(path):
        assert problem.obstacles.segment_is_free(first, second)


class TestFeasibility:
    """What a planner must return when a path exists, and when none does."""

    def test_finds_a_path_in_obstacle_free_space(
        self, planner: Planner, free_problem: PlanningProblem
    ) -> None:
        result = planner.plan(free_problem, seed=0)
        assert result.success is True
        assert_path_is_admissible(free_problem, result.path)

    def test_finds_a_path_around_obstacles(
        self, planner: Planner, blocked_problem: PlanningProblem
    ) -> None:
        result = planner.plan(blocked_problem, seed=1)
        assert result.success is True
        assert_path_is_admissible(blocked_problem, result.path)

    def test_reports_failure_when_the_goal_is_walled_off(
        self, planner: Planner, sealed_problem: PlanningProblem
    ) -> None:
        result = planner.plan(sealed_problem, seed=2)
        assert result.success is False
        assert result.path == ()
        assert math.isinf(result.cost)

    def test_reported_cost_is_the_length_of_the_reported_path(
        self, planner: Planner, blocked_problem: PlanningProblem
    ) -> None:
        result = planner.plan(blocked_problem, seed=3)
        assert result.success is True
        assert result.cost == pytest.approx(path_cost(result.path), rel=1e-9, abs=1e-9)

    def test_cost_is_at_least_the_straight_line_distance(
        self, planner: Planner, blocked_problem: PlanningProblem
    ) -> None:
        result = planner.plan(blocked_problem, seed=4)
        assert result.cost >= blocked_problem.straight_line_cost - 1e-9

    def test_every_planner_satisfies_the_protocol(self, planner: Planner) -> None:
        assert isinstance(planner, Planner)
        assert isinstance(planner.name, str) and planner.name


class TestTreeInvariants:
    """Properties of the structures the tree planners build."""

    @pytest.mark.parametrize("factory", [RRT, RRTStar])
    def test_every_tree_edge_respects_the_step_size(
        self, factory: type[RRT] | type[RRTStar], blocked_problem: PlanningProblem
    ) -> None:
        planner = factory(max_samples=400, step_size=0.6)
        tree = planner.plan(blocked_problem, seed=5).tree
        assert tree is not None
        for parent, child in tree.edges():
            length = float(
                np.linalg.norm(tree.configurations[child] - tree.configurations[parent])
            )
            assert length <= planner.step_size + 1e-9

    @pytest.mark.parametrize("factory", [RRT, RRTStar])
    def test_every_tree_edge_is_collision_free(
        self, factory: type[RRT] | type[RRTStar], blocked_problem: PlanningProblem
    ) -> None:
        planner = factory(max_samples=400, step_size=0.6)
        tree = planner.plan(blocked_problem, seed=6).tree
        assert tree is not None
        for parent, child in tree.edges():
            assert blocked_problem.obstacles.segment_is_free(
                tree.configurations[parent], tree.configurations[child]
            )

    @pytest.mark.parametrize("factory", [RRT, RRTStar])
    def test_the_tree_is_rooted_at_the_start_and_acyclic(
        self, factory: type[RRT] | type[RRTStar], blocked_problem: PlanningProblem
    ) -> None:
        planner = factory(max_samples=400, step_size=0.6)
        tree = planner.plan(blocked_problem, seed=7).tree
        assert tree is not None
        assert np.allclose(tree.configurations[0], blocked_problem.start)
        assert tree.parents[0] == NO_PARENT
        for index in range(tree.size):
            visited: set[int] = set()
            cursor = index
            while cursor != NO_PARENT:
                assert cursor not in visited, "the parent chain revisits a vertex"
                visited.add(cursor)
                cursor = tree.parents[cursor]
            assert 0 in visited, "the parent chain does not reach the root"

    def test_stored_costs_equal_the_tree_path_lengths(
        self, blocked_problem: PlanningProblem
    ) -> None:
        # Rewiring must leave every stored cost exact, not merely close.
        result = RRTStar(max_samples=500, step_size=0.6).plan(blocked_problem, seed=8)
        tree = result.tree
        assert tree is not None
        for index in range(tree.size):
            expected = path_cost(tree.path_to(index))
            assert tree.costs[index] == pytest.approx(expected, rel=1e-9, abs=1e-9)


class TestRoadmap:
    """The build phase and the query phase of PRM are separable."""

    def test_one_roadmap_answers_several_queries(
        self, blocked_problem: PlanningProblem
    ) -> None:
        prm = PRM(milestones=250, neighbours=8)
        roadmap, attempts = prm.build(blocked_problem.space, blocked_problem.obstacles, seed=9)
        assert roadmap.size == 250
        assert attempts >= roadmap.size
        assert roadmap.edge_count > 0

        first, cost_first = prm.query(
            roadmap, blocked_problem.obstacles, blocked_problem.start, blocked_problem.goal
        )
        second, cost_second = prm.query(
            roadmap, blocked_problem.obstacles, blocked_problem.goal, blocked_problem.start
        )
        assert first and second
        assert cost_first == pytest.approx(cost_second, rel=1e-9)
        assert_path_is_admissible(blocked_problem, first)

    def test_a_query_does_not_modify_the_roadmap(
        self, blocked_problem: PlanningProblem
    ) -> None:
        prm = PRM(milestones=150, neighbours=6)
        roadmap, _ = prm.build(blocked_problem.space, blocked_problem.obstacles, seed=10)
        size_before, edges_before = roadmap.size, roadmap.edge_count
        prm.query(
            roadmap, blocked_problem.obstacles, blocked_problem.start, blocked_problem.goal
        )
        assert (roadmap.size, roadmap.edge_count) == (size_before, edges_before)

    def test_every_milestone_is_collision_free(self, blocked_problem: PlanningProblem) -> None:
        prm = PRM(milestones=150, neighbours=6)
        roadmap, _ = prm.build(blocked_problem.space, blocked_problem.obstacles, seed=11)
        assert all(blocked_problem.obstacles.is_free(q) for q in roadmap.configurations)

    def test_radius_connection_is_available(self, blocked_problem: PlanningProblem) -> None:
        prm = PRM(milestones=200, connection_radius=1.5)
        result = prm.plan(blocked_problem, seed=12)
        assert result.success is True
        assert_path_is_admissible(blocked_problem, result.path)


class TestCollisionAccounting:
    """Every planner reports the collision queries it asked."""

    def test_a_result_reports_the_queries_that_built_it(
        self, planner: Planner, blocked_problem: PlanningProblem
    ) -> None:
        result = planner.plan(blocked_problem, seed=20)
        assert result.point_checks >= 0
        assert result.segment_checks > 0
        assert result.collision_checks == result.point_checks + result.segment_checks
        # Every vertex beyond the root was admitted by at least one segment query, so
        # the count can never be smaller than the structure it produced.
        assert result.segment_checks >= result.node_count - 1

    def test_a_tree_planner_asks_at_most_two_segment_queries_per_iteration(
        self, blocked_problem: PlanningProblem
    ) -> None:
        # One for the extension, one for the attempt to reach the goal from it.
        result = RRT(max_samples=500, step_size=0.6).plan(blocked_problem, seed=21)
        assert result.point_checks == 0
        assert result.segment_checks <= 2 * result.iterations

    def test_rewiring_is_what_the_extra_queries_are_spent_on(
        self, sealed_problem: PlanningProblem
    ) -> None:
        # The sealed problem has no solution, so both planners spend the whole budget
        # and the counts are comparable. RRT star pays for parent selection and
        # rewiring on top of the single extension query RRT makes.
        budget = 400
        feasible = RRT(max_samples=budget, step_size=0.6).plan(sealed_problem, seed=22)
        optimal = RRTStar(max_samples=budget, step_size=0.6).plan(sealed_problem, seed=22)
        assert feasible.iterations == optimal.iterations == budget
        assert optimal.segment_checks > feasible.segment_checks

    def test_a_roadmap_tests_every_sample_before_it_tests_any_edge(
        self, blocked_problem: PlanningProblem
    ) -> None:
        prm = PRM(milestones=150, neighbours=6)
        result = prm.plan(blocked_problem, seed=23)
        assert result.roadmap is not None
        assert result.point_checks >= result.roadmap.size
        assert result.point_checks == result.iterations
        assert result.segment_checks >= result.roadmap.edge_count

    def test_counting_does_not_change_what_the_roadmap_becomes(
        self, blocked_problem: PlanningProblem
    ) -> None:
        prm = PRM(milestones=150, neighbours=6)
        plain, plain_attempts = prm.build(
            blocked_problem.space, blocked_problem.obstacles, seed=24
        )
        counter = CountingChecker(blocked_problem.obstacles)
        counted, counted_attempts = prm.build(blocked_problem.space, counter, seed=24)
        assert plain_attempts == counted_attempts
        assert plain.edges() == counted.edges()
        for left, right in zip(plain.configurations, counted.configurations, strict=True):
            assert np.array_equal(left, right)
        assert counter.total_checks > 0

    def test_the_counts_are_fixed_by_the_seed(
        self, planner: Planner, blocked_problem: PlanningProblem
    ) -> None:
        first = planner.plan(blocked_problem, seed=25)
        second = planner.plan(blocked_problem, seed=25)
        assert (first.point_checks, first.segment_checks) == (
            second.point_checks,
            second.segment_checks,
        )

    def test_a_result_rejects_a_negative_count(self) -> None:
        with pytest.raises(ValueError, match="negative"):
            PlanResult(
                planner="RRT",
                problem="blocked",
                seed=0,
                success=False,
                segment_checks=-1,
            )


class TestConfigurationValidation:
    """Planner and problem construction rejects nonsense early."""

    @pytest.mark.parametrize("factory", [RRT, RRTStar])
    def test_rejects_a_non_positive_step_size(
        self, factory: type[RRT] | type[RRTStar]
    ) -> None:
        with pytest.raises(ValueError, match="step_size"):
            factory(step_size=0.0)

    @pytest.mark.parametrize("factory", [RRT, RRTStar])
    def test_rejects_an_out_of_range_goal_bias(
        self, factory: type[RRT] | type[RRTStar]
    ) -> None:
        with pytest.raises(ValueError, match="goal_bias"):
            factory(goal_bias=1.0)

    def test_rejects_a_gamma_scale_that_breaks_the_optimality_condition(self) -> None:
        with pytest.raises(ValueError, match="gamma_scale"):
            RRTStar(gamma_scale=1.0)

    def test_rejects_a_roadmap_that_is_too_small(self) -> None:
        with pytest.raises(ValueError, match="milestones"):
            PRM(milestones=1)

    def test_rejects_a_start_inside_an_obstacle(
        self, blocked_problem: PlanningProblem
    ) -> None:
        with pytest.raises(ValueError, match="start"):
            PlanningProblem(
                name="bad",
                space=blocked_problem.space,
                obstacles=blocked_problem.obstacles,
                start=np.array([4.5, 3.0]),
                goal=blocked_problem.goal,
            )

    def test_rejects_a_goal_outside_the_space(
        self, blocked_problem: PlanningProblem
    ) -> None:
        with pytest.raises(ValueError, match="goal"):
            PlanningProblem(
                name="bad",
                space=blocked_problem.space,
                obstacles=blocked_problem.obstacles,
                start=blocked_problem.start,
                goal=np.array([11.0, 11.0]),
            )
