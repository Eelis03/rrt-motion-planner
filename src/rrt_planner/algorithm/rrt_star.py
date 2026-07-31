"""RRT star, the asymptotically optimal variant of RRT.

The rewiring radius is the one derived by Karaman and Frazzoli, namely

    r(n) = min( gamma * (log n / n) ** (1 / d), eta )

with

    gamma > gamma_star = 2 * (1 + 1 / d) ** (1 / d) * (mu(X_free) / zeta_d) ** (1 / d),

where ``d`` is the dimension of the configuration space, ``mu(X_free)`` the measure
of the free space, ``zeta_d`` the volume of the unit ball in ``d`` dimensions, and
``eta`` the steering step size. The measure of the free space is not known to the
planner, so the measure of the whole configuration space is used instead. That is an
over-estimate, it can only raise ``gamma``, and the convergence result requires
``gamma`` to be above the threshold, so the substitution is safe. It is paid for in
larger near-neighbour sets than strictly necessary.

References
----------
S. Karaman and E. Frazzoli, "Sampling-based algorithms for optimal motion planning",
The International Journal of Robotics Research 30(7), 2011, pages 846 to 894.
DOI 10.1177/0278364911406761.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from rrt_planner.algorithm.base import NearestNeighbourIndex, steer
from rrt_planner.algorithm.rrt import GOAL_EPSILON
from rrt_planner.model.graph import SearchTree
from rrt_planner.model.problem import PlanningProblem, PlanResult
from rrt_planner.model.space import ConfigurationSpace, Vector

__all__ = ["RRTStar", "unit_ball_volume"]

# Costs are compared with this margin so that floating point noise cannot trigger a
# rewiring that does not actually improve anything.
_COST_EPSILON = 1e-12


def unit_ball_volume(dimension: int) -> float:
    """Volume ``zeta_d`` of the unit ball in ``dimension`` dimensions."""
    if dimension < 1:
        raise ValueError("dimension must be at least one")
    return float(math.pi ** (dimension / 2.0) / math.gamma(dimension / 2.0 + 1.0))


@dataclass(frozen=True, slots=True)
class RRTStar:
    """RRT star with near-neighbour parent selection and rewiring.

    Each accepted extension is attached to whichever vertex in its near-neighbour
    ball yields the lowest cost-to-come by a collision-free segment, and every other
    vertex in that ball is then reconnected through the new vertex when doing so is
    cheaper. Costs of the affected subtrees are propagated, so every stored cost is
    the exact length of the tree path to that vertex at all times.

    The planner runs the full sample budget rather than stopping at the first
    solution, which is what makes the cost of the incumbent solution decrease.
    """

    max_samples: int = 2000
    step_size: float = 0.5
    goal_bias: float = 0.05
    gamma_scale: float = 1.1

    def __post_init__(self) -> None:
        if self.max_samples < 1:
            raise ValueError("max_samples must be at least one")
        if self.step_size <= 0.0:
            raise ValueError("step_size must be positive")
        if not 0.0 <= self.goal_bias < 1.0:
            raise ValueError("goal_bias must lie in [0, 1)")
        if self.gamma_scale <= 1.0:
            raise ValueError("gamma_scale must exceed one so that gamma exceeds gamma star")

    @property
    def name(self) -> str:
        """Human-readable planner name used in reports and figures."""
        return "RRT star"

    def gamma(self, space: ConfigurationSpace) -> float:
        """The constant ``gamma`` of the rewiring radius for ``space``."""
        dimension = space.dimension
        threshold = float(
            2.0
            * (1.0 + 1.0 / dimension) ** (1.0 / dimension)
            * (space.volume / unit_ball_volume(dimension)) ** (1.0 / dimension)
        )
        return self.gamma_scale * threshold

    def rewiring_radius(self, space: ConfigurationSpace, vertex_count: int) -> float:
        """The radius ``r(n)`` used to collect near neighbours at ``vertex_count`` vertices."""
        count = max(vertex_count, 2)
        dimension = space.dimension
        radius = float(self.gamma(space) * (math.log(count) / count) ** (1.0 / dimension))
        return min(radius, self.step_size)

    def plan(self, problem: PlanningProblem, seed: int) -> PlanResult:
        """Run the full sample budget, returning the best path found."""
        rng = np.random.default_rng(seed)
        tree = SearchTree.rooted_at(problem.start)
        index = NearestNeighbourIndex(problem.dimension)
        index.add(problem.start)

        goal_index: int | None = None
        cost_history: list[tuple[int, float]] = []
        rewires: list[tuple[int, int, int]] = []

        for iteration in range(1, self.max_samples + 1):
            if goal_index is None and rng.random() < self.goal_bias:
                target = problem.goal
            else:
                target = problem.space.sample(rng)

            nearest = index.nearest(target)
            origin = tree.configurations[nearest]
            candidate = steer(origin, target, self.step_size)
            step = float(np.linalg.norm(candidate - origin))
            if step <= GOAL_EPSILON:
                continue
            if not problem.obstacles.segment_is_free(origin, candidate):
                continue

            radius = self.rewiring_radius(problem.space, tree.size)
            neighbours = index.within_radius(candidate, radius)
            parent, cost = self._choose_parent(problem, tree, neighbours, candidate, nearest, step)
            new_index = tree.add_node(candidate, parent, cost)
            index.add(candidate)

            self._rewire(problem, tree, neighbours, new_index, rewires)

            if goal_index is None:
                goal_index = self._connect_goal(problem, tree, index, new_index)
            if goal_index is not None:
                best = tree.costs[goal_index]
                if not cost_history or best < cost_history[-1][1]:
                    cost_history.append((iteration, best))

        if goal_index is None:
            return PlanResult(
                planner=self.name,
                problem=problem.name,
                seed=seed,
                success=False,
                node_count=tree.size,
                iterations=self.max_samples,
                rewires=tuple(rewires),
                tree=tree,
            )

        return PlanResult(
            planner=self.name,
            problem=problem.name,
            seed=seed,
            success=True,
            path=tree.path_to(goal_index),
            cost=tree.costs[goal_index],
            node_count=tree.size,
            iterations=self.max_samples,
            cost_history=tuple(cost_history),
            rewires=tuple(rewires),
            tree=tree,
        )

    def _choose_parent(
        self,
        problem: PlanningProblem,
        tree: SearchTree,
        neighbours: list[int],
        candidate: Vector,
        nearest: int,
        step: float,
    ) -> tuple[int, float]:
        """Return the cheapest collision-free parent for ``candidate`` and its cost."""
        best_parent = nearest
        best_cost = tree.costs[nearest] + step
        for neighbour in neighbours:
            if neighbour == nearest:
                continue
            distance = float(np.linalg.norm(candidate - tree.configurations[neighbour]))
            cost = tree.costs[neighbour] + distance
            if cost >= best_cost - _COST_EPSILON:
                continue
            if not problem.obstacles.segment_is_free(tree.configurations[neighbour], candidate):
                continue
            best_parent = neighbour
            best_cost = cost
        return best_parent, best_cost

    def _rewire(
        self,
        problem: PlanningProblem,
        tree: SearchTree,
        neighbours: list[int],
        new_index: int,
        rewires: list[tuple[int, int, int]],
    ) -> None:
        """Reconnect near neighbours through the new vertex wherever that is cheaper."""
        candidate = tree.configurations[new_index]
        for neighbour in neighbours:
            if neighbour == tree.parents[new_index]:
                continue
            distance = float(np.linalg.norm(candidate - tree.configurations[neighbour]))
            cost = tree.costs[new_index] + distance
            if cost >= tree.costs[neighbour] - _COST_EPSILON:
                continue
            if not problem.obstacles.segment_is_free(candidate, tree.configurations[neighbour]):
                continue
            tree.reparent(neighbour, new_index, cost)
            _propagate_costs(tree, neighbour)
            rewires.append((tree.size, neighbour, new_index))

    def _connect_goal(
        self,
        problem: PlanningProblem,
        tree: SearchTree,
        index: NearestNeighbourIndex,
        new_index: int,
    ) -> int | None:
        """Insert the goal as an ordinary vertex when the new vertex can reach it directly.

        The goal enters the nearest neighbour index alongside the tree so that the two
        share one indexing scheme, and so that later rewiring can lower its cost. It is
        an ordinary vertex from that point on: acquiring children changes neither its own
        cost nor the path to it.
        """
        candidate = tree.configurations[new_index]
        distance = float(np.linalg.norm(problem.goal - candidate))
        if distance <= GOAL_EPSILON:
            return new_index
        if distance > self.step_size:
            return None
        if not problem.obstacles.segment_is_free(candidate, problem.goal):
            return None
        goal_index = tree.add_node(problem.goal, new_index, tree.costs[new_index] + distance)
        index.add(problem.goal)
        return goal_index


def _propagate_costs(tree: SearchTree, root: int) -> None:
    """Refresh the cost-to-come of every descendant of ``root`` after a rewiring."""
    stack = list(tree.children[root])
    while stack:
        node = stack.pop()
        parent = tree.parents[node]
        distance = float(
            np.linalg.norm(tree.configurations[node] - tree.configurations[parent])
        )
        tree.costs[node] = tree.costs[parent] + distance
        stack.extend(tree.children[node])
