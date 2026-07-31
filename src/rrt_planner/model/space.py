"""Configuration space with axis-aligned bounds."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

__all__ = ["ConfigurationSpace", "Vector"]

Vector = NDArray[np.float64]


def as_vector(values: object) -> Vector:
    """Return ``values`` as an immutable one-dimensional float64 array."""
    array = np.array(values, dtype=np.float64, copy=True)
    if array.ndim != 1:
        raise ValueError(f"expected a one-dimensional vector, got shape {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ValueError("vector entries must be finite")
    array.setflags(write=False)
    return array


@dataclass(frozen=True, slots=True, eq=False)
class ConfigurationSpace:
    """An axis-aligned box in ``R^d`` from which configurations are drawn.

    The space is the sampling domain of every planner in this package. It carries
    no obstacle information: free space is the set difference between this box and
    the obstacle set supplied alongside it in a :class:`~rrt_planner.model.problem.PlanningProblem`.
    """

    lower: Vector
    upper: Vector

    def __post_init__(self) -> None:
        lower = as_vector(self.lower)
        upper = as_vector(self.upper)
        if lower.shape != upper.shape:
            raise ValueError(f"bound shapes differ: {lower.shape} and {upper.shape}")
        if lower.size == 0:
            raise ValueError("a configuration space needs at least one dimension")
        if not np.all(upper > lower):
            raise ValueError("every upper bound must exceed its lower bound")
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    @property
    def dimension(self) -> int:
        """Number of configuration variables."""
        return int(self.lower.size)

    @property
    def extent(self) -> Vector:
        """Side length of the bounding box along each axis."""
        return as_vector(self.upper - self.lower)

    @property
    def volume(self) -> float:
        """Lebesgue measure of the bounding box."""
        return float(np.prod(self.upper - self.lower))

    def contains(self, point: Vector) -> bool:
        """True when ``point`` lies inside the bounds, boundary included."""
        return bool(np.all(point >= self.lower) and np.all(point <= self.upper))

    def sample(self, rng: np.random.Generator) -> Vector:
        """Draw one configuration uniformly from the bounding box."""
        drawn = rng.uniform(self.lower, self.upper)
        return as_vector(drawn)

    def sample_batch(self, rng: np.random.Generator, count: int) -> NDArray[np.float64]:
        """Draw ``count`` configurations uniformly, returned as a ``(count, d)`` array."""
        if count < 0:
            raise ValueError("count must not be negative")
        drawn = rng.uniform(self.lower, self.upper, size=(count, self.dimension))
        return np.asarray(drawn, dtype=np.float64)
