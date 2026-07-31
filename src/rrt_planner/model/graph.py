"""Search structures shared by the planners: a rooted tree and an undirected roadmap."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from rrt_planner.model.space import Vector, as_vector

__all__ = ["NO_PARENT", "Roadmap", "SearchTree", "TreeNode"]

NO_PARENT = -1


@dataclass(frozen=True, slots=True, eq=False)
class TreeNode:
    """One vertex of a search tree, as a value."""

    index: int
    configuration: Vector
    parent: int
    cost: float


@dataclass(slots=True, eq=False)
class SearchTree:
    """A rooted tree of configurations with cost-to-come bookkeeping.

    Parallel lists are used rather than linked node objects because the planners
    address vertices by integer index, which is also what the nearest neighbour
    index returns. ``insertion_parents`` keeps the parent each vertex had when it
    was first added, so a rewiring history can be replayed later without storing a
    copy of the tree per iteration.
    """

    configurations: list[Vector] = field(default_factory=list)
    parents: list[int] = field(default_factory=list)
    insertion_parents: list[int] = field(default_factory=list)
    costs: list[float] = field(default_factory=list)
    children: list[list[int]] = field(default_factory=list)

    @classmethod
    def rooted_at(cls, configuration: Vector) -> SearchTree:
        """Return a tree holding ``configuration`` as its root, at zero cost."""
        tree = cls()
        tree.configurations.append(as_vector(configuration))
        tree.parents.append(NO_PARENT)
        tree.insertion_parents.append(NO_PARENT)
        tree.costs.append(0.0)
        tree.children.append([])
        return tree

    @property
    def size(self) -> int:
        """Number of vertices."""
        return len(self.configurations)

    def add_node(self, configuration: Vector, parent: int, cost: float) -> int:
        """Append a vertex under ``parent`` and return its index."""
        if not 0 <= parent < self.size:
            raise IndexError(f"parent index out of range: {parent}")
        index = self.size
        self.configurations.append(as_vector(configuration))
        self.parents.append(parent)
        self.insertion_parents.append(parent)
        self.costs.append(float(cost))
        self.children.append([])
        self.children[parent].append(index)
        return index

    def reparent(self, index: int, parent: int, cost: float) -> None:
        """Move vertex ``index`` under ``parent`` and set its new cost-to-come."""
        if index == parent:
            raise ValueError("a vertex cannot be its own parent")
        previous = self.parents[index]
        if previous != NO_PARENT:
            self.children[previous].remove(index)
        self.parents[index] = parent
        self.costs[index] = float(cost)
        self.children[parent].append(index)

    def node(self, index: int) -> TreeNode:
        """Return vertex ``index`` as a value object."""
        return TreeNode(
            index=index,
            configuration=self.configurations[index],
            parent=self.parents[index],
            cost=self.costs[index],
        )

    def path_to(self, index: int) -> tuple[Vector, ...]:
        """Return the configurations from the root to vertex ``index`` inclusive."""
        reversed_path: list[Vector] = []
        cursor = index
        while cursor != NO_PARENT:
            reversed_path.append(self.configurations[cursor])
            cursor = self.parents[cursor]
        return tuple(reversed(reversed_path))

    def edges(self) -> tuple[tuple[int, int], ...]:
        """Return every ``(parent, child)`` pair currently in the tree."""
        return tuple(
            (parent, index) for index, parent in enumerate(self.parents) if parent != NO_PARENT
        )


@dataclass(slots=True, eq=False)
class Roadmap:
    """An undirected weighted graph over configurations, as built by PRM."""

    configurations: list[Vector] = field(default_factory=list)
    adjacency: list[list[tuple[int, float]]] = field(default_factory=list)

    @property
    def size(self) -> int:
        """Number of vertices."""
        return len(self.configurations)

    @property
    def edge_count(self) -> int:
        """Number of undirected edges."""
        return sum(len(neighbours) for neighbours in self.adjacency) // 2

    def add_vertex(self, configuration: Vector) -> int:
        """Append an isolated vertex and return its index."""
        index = self.size
        self.configurations.append(as_vector(configuration))
        self.adjacency.append([])
        return index

    def add_edge(self, first: int, second: int, weight: float) -> None:
        """Add an undirected edge of the given weight."""
        if first == second:
            raise ValueError("self loops are not permitted in a roadmap")
        self.adjacency[first].append((second, float(weight)))
        self.adjacency[second].append((first, float(weight)))

    def edges(self) -> tuple[tuple[int, int], ...]:
        """Return every undirected edge once, as ordered index pairs."""
        return tuple(
            (index, other)
            for index, neighbours in enumerate(self.adjacency)
            for other, _ in neighbours
            if index < other
        )

    def shortest_path(self, source: int, target: int) -> tuple[tuple[int, ...], float]:
        """Return the cheapest vertex sequence from ``source`` to ``target`` and its cost.

        Dijkstra's algorithm over a binary heap. Ties are broken by vertex index so
        that the result depends only on the graph, never on heap ordering accidents.
        Returns an empty sequence and infinite cost when the target is unreachable.
        """
        distances = [math.inf] * self.size
        previous = [NO_PARENT] * self.size
        settled = [False] * self.size
        distances[source] = 0.0
        queue: list[tuple[float, int]] = [(0.0, source)]

        while queue:
            distance, vertex = heapq.heappop(queue)
            if settled[vertex]:
                continue
            settled[vertex] = True
            if vertex == target:
                break
            for neighbour, weight in sorted(self.adjacency[vertex]):
                if settled[neighbour]:
                    continue
                candidate = distance + weight
                if candidate < distances[neighbour]:
                    distances[neighbour] = candidate
                    previous[neighbour] = vertex
                    heapq.heappush(queue, (candidate, neighbour))

        if not math.isfinite(distances[target]):
            return (), math.inf

        sequence: list[int] = []
        cursor = target
        while cursor != NO_PARENT:
            sequence.append(cursor)
            cursor = previous[cursor]
        return tuple(reversed(sequence)), distances[target]

    def configuration_array(self) -> np.ndarray:
        """Return the vertices stacked into an ``(n, d)`` array."""
        if not self.configurations:
            return np.zeros((0, 0), dtype=np.float64)
        return np.vstack(self.configurations)
