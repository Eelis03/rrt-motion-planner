"""Rapidly-exploring random tree, following LaValle and Kuffner.

References
----------
S. M. LaValle, "Rapidly-exploring random trees: a new tool for path planning",
Technical Report TR 98-11, Computer Science Department, Iowa State University, 1998.

S. M. LaValle and J. J. Kuffner, "Randomized kinodynamic planning",
The International Journal of Robotics Research 20(5), 2001, pages 378 to 400.
DOI 10.1177/02783640122067453.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rrt_planner.algorithm.base import NearestNeighbourIndex, steer
from rrt_planner.model.graph import SearchTree
from rrt_planner.model.problem import PlanningProblem, PlanResult, path_cost

__all__ = ["GOAL_EPSILON", "RRT"]

# Two configurations closer than this are treated as the same vertex, which keeps
# a goal-biased sample that lands exactly on the goal from being inserted twice.
GOAL_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class RRT:
    """Single-query RRT with goal biasing and a bounded extension length.

    Each iteration draws a configuration, which is the goal itself with probability
    ``goal_bias``, extends the tree from its nearest vertex by at most ``step_size``
    towards that configuration, and keeps the new vertex when the connecting segment
    is collision free. The search stops at the first vertex from which the goal can
    be reached by one collision-free segment, so the tree is a feasible-path search
    and makes no optimality claim.
    """

    max_samples: int = 2000
    step_size: float = 0.5
    goal_bias: float = 0.05

    def __post_init__(self) -> None:
        if self.max_samples < 1:
            raise ValueError("max_samples must be at least one")
        if self.step_size <= 0.0:
            raise ValueError("step_size must be positive")
        if not 0.0 <= self.goal_bias < 1.0:
            raise ValueError("goal_bias must lie in [0, 1)")

    @property
    def name(self) -> str:
        """Human-readable planner name used in reports and figures."""
        return "RRT"

    def plan(self, problem: PlanningProblem, seed: int) -> PlanResult:
        """Grow a tree from the start until the goal is reachable or the budget runs out."""
        rng = np.random.default_rng(seed)
        tree = SearchTree.rooted_at(problem.start)
        index = NearestNeighbourIndex(problem.dimension)
        index.add(problem.start)

        goal_index: int | None = None
        iterations = 0
        for iteration in range(1, self.max_samples + 1):
            iterations = iteration
            biased = rng.random() < self.goal_bias
            target = problem.goal if biased else problem.space.sample(rng)

            nearest = index.nearest(target)
            origin = tree.configurations[nearest]
            candidate = steer(origin, target, self.step_size)
            if not problem.obstacles.segment_is_free(origin, candidate):
                continue

            step = float(np.linalg.norm(candidate - origin))
            if step <= GOAL_EPSILON:
                continue
            new_index = tree.add_node(candidate, nearest, tree.costs[nearest] + step)
            index.add(candidate)

            goal_index = self._try_goal(problem, tree, new_index)
            if goal_index is not None:
                break

        if goal_index is None:
            return PlanResult(
                planner=self.name,
                problem=problem.name,
                seed=seed,
                success=False,
                node_count=tree.size,
                iterations=iterations,
                tree=tree,
            )

        path = tree.path_to(goal_index)
        cost = path_cost(path)
        return PlanResult(
            planner=self.name,
            problem=problem.name,
            seed=seed,
            success=True,
            path=path,
            cost=cost,
            node_count=tree.size,
            iterations=iterations,
            cost_history=((iterations, cost),),
            tree=tree,
        )

    def _try_goal(
        self, problem: PlanningProblem, tree: SearchTree, new_index: int
    ) -> int | None:
        """Return the goal vertex index when the goal is reachable from ``new_index``."""
        candidate = tree.configurations[new_index]
        distance = float(np.linalg.norm(problem.goal - candidate))
        if distance <= GOAL_EPSILON:
            return new_index
        if distance > self.step_size:
            return None
        if not problem.obstacles.segment_is_free(candidate, problem.goal):
            return None
        return tree.add_node(problem.goal, new_index, tree.costs[new_index] + distance)
