"""Probabilistic roadmap, following Kavraki, Svestka, Latombe and Overmars.

The roadmap is built once and answers any number of queries in the same
environment. That is the property that distinguishes PRM from the tree planners:
the sampling cost is paid once and amortised over queries, at the price of not
being able to exploit knowledge of a particular start and goal while sampling.

References
----------
L. E. Kavraki, P. Svestka, J.-C. Latombe and M. H. Overmars, "Probabilistic roadmaps
for path planning in high-dimensional configuration spaces", IEEE Transactions on
Robotics and Automation 12(4), 1996, pages 566 to 580. DOI 10.1109/70.508439.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from rrt_planner.model.graph import Roadmap
from rrt_planner.model.obstacles import ObstacleSet
from rrt_planner.model.problem import PlanningProblem, PlanResult, path_cost
from rrt_planner.model.space import ConfigurationSpace, Vector

__all__ = ["PRM"]

# Sampling gives up after this many rejected draws per requested milestone, which
# stops a fully blocked environment from looping forever.
_MAX_ATTEMPTS_PER_SAMPLE = 50


@dataclass(frozen=True, slots=True)
class PRM:
    """Probabilistic roadmap with a build phase and a query phase.

    ``connection_radius`` selects the connection strategy. When it is ``None`` each
    milestone is connected to its ``neighbours`` nearest milestones, which keeps the
    roadmap sparse and its degree bounded. When it is a number, each milestone is
    connected to every milestone within that distance instead.
    """

    milestones: int = 500
    neighbours: int = 10
    connection_radius: float | None = None

    def __post_init__(self) -> None:
        if self.milestones < 2:
            raise ValueError("a roadmap needs at least two milestones")
        if self.neighbours < 1:
            raise ValueError("neighbours must be at least one")
        if self.connection_radius is not None and self.connection_radius <= 0.0:
            raise ValueError("connection_radius must be positive when given")

    @property
    def name(self) -> str:
        """Human-readable planner name used in reports and figures."""
        return "PRM"

    def build(
        self, space: ConfigurationSpace, obstacles: ObstacleSet, seed: int
    ) -> tuple[Roadmap, int]:
        """Build a roadmap over the free part of ``space``.

        Returns the roadmap and the number of samples drawn, including the rejected
        ones, so that the caller can report the sampling effort separately from the
        number of milestones that survived.
        """
        rng = np.random.default_rng(seed)
        roadmap = Roadmap()
        attempts = 0
        budget = self.milestones * _MAX_ATTEMPTS_PER_SAMPLE
        while roadmap.size < self.milestones and attempts < budget:
            attempts += 1
            candidate = space.sample(rng)
            if obstacles.is_free(candidate):
                roadmap.add_vertex(candidate)
        self._connect(roadmap, obstacles)
        return roadmap, attempts

    def _connect(self, roadmap: Roadmap, obstacles: ObstacleSet) -> None:
        """Add every admissible edge between milestones already in ``roadmap``."""
        if roadmap.size < 2:
            return
        points = roadmap.configuration_array()
        tree = cKDTree(points)
        for first, second in self._candidate_pairs(tree, points):
            configurations = roadmap.configurations
            if obstacles.segment_is_free(configurations[first], configurations[second]):
                distance = float(np.linalg.norm(points[second] - points[first]))
                roadmap.add_edge(first, second, distance)

    def _candidate_pairs(
        self, tree: cKDTree, points: np.ndarray
    ) -> list[tuple[int, int]]:
        """Return each unordered milestone pair that the connection strategy proposes.

        The k-nearest relation is not symmetric, so the pairs are collected into a set
        and sorted. Every milestone therefore has at least its ``k`` nearest neighbours
        as candidates, and the result does not depend on iteration order.
        """
        pairs: set[tuple[int, int]] = set()
        if self.connection_radius is not None:
            groups = tree.query_ball_point(points, self.connection_radius)
        else:
            count = min(self.neighbours + 1, len(points))
            _, indices = tree.query(points, k=count)
            groups = list(np.atleast_2d(indices))
        for first, group in enumerate(groups):
            for candidate in group:
                second = int(candidate)
                if second != first:
                    pairs.add((min(first, second), max(first, second)))
        return sorted(pairs)

    def query(
        self,
        roadmap: Roadmap,
        obstacles: ObstacleSet,
        start: Vector,
        goal: Vector,
    ) -> tuple[tuple[Vector, ...], float]:
        """Answer a single query against a prebuilt roadmap.

        The roadmap is not modified. The start and the goal are attached to a private
        copy, so the same roadmap can serve any number of queries.
        """
        augmented = Roadmap(
            configurations=list(roadmap.configurations),
            adjacency=[list(neighbours) for neighbours in roadmap.adjacency],
        )
        start_index = augmented.add_vertex(start)
        goal_index = augmented.add_vertex(goal)
        for index, configuration in ((start_index, start), (goal_index, goal)):
            self._attach(augmented, obstacles, roadmap.size, index, configuration)

        sequence, cost = augmented.shortest_path(start_index, goal_index)
        if not sequence:
            return (), math.inf
        return tuple(augmented.configurations[i] for i in sequence), cost

    def _attach(
        self,
        augmented: Roadmap,
        obstacles: ObstacleSet,
        milestone_count: int,
        index: int,
        configuration: Vector,
    ) -> None:
        """Connect one query configuration to the roadmap milestones near it."""
        if milestone_count == 0:
            return
        points = np.vstack(augmented.configurations[:milestone_count])
        distances = np.linalg.norm(points - configuration, axis=1)
        if self.connection_radius is not None:
            order = np.flatnonzero(distances <= self.connection_radius)
            order = order[np.argsort(distances[order], kind="stable")]
        else:
            count = min(self.neighbours, milestone_count)
            order = np.argsort(distances, kind="stable")[:count]
        for milestone in order:
            target = int(milestone)
            if obstacles.segment_is_free(configuration, augmented.configurations[target]):
                augmented.add_edge(index, target, float(distances[target]))

    def plan(self, problem: PlanningProblem, seed: int) -> PlanResult:
        """Build a roadmap for ``problem`` and answer its single query."""
        roadmap, attempts = self.build(problem.space, problem.obstacles, seed)
        path, _ = self.query(roadmap, problem.obstacles, problem.start, problem.goal)
        if not path:
            return PlanResult(
                planner=self.name,
                problem=problem.name,
                seed=seed,
                success=False,
                node_count=roadmap.size,
                iterations=attempts,
                roadmap=roadmap,
            )
        exact_cost = path_cost(path)
        return PlanResult(
            planner=self.name,
            problem=problem.name,
            seed=seed,
            success=True,
            path=path,
            cost=exact_cost,
            node_count=roadmap.size,
            iterations=attempts,
            cost_history=((attempts, exact_cost),),
            roadmap=roadmap,
        )
