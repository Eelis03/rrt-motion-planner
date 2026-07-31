"""Tier one: configuration spaces, steering, and the nearest neighbour index."""

from __future__ import annotations

import numpy as np
import pytest

from rrt_planner.algorithm.base import NearestNeighbourIndex, steer
from rrt_planner.model.space import ConfigurationSpace


class TestConfigurationSpace:
    """Bounds, measures, and sampling."""

    def test_reports_dimension_extent_and_volume(self) -> None:
        space = ConfigurationSpace(
            lower=np.array([0.0, -1.0, 2.0]), upper=np.array([2.0, 1.0, 5.0])
        )
        assert space.dimension == 3
        assert np.allclose(space.extent, [2.0, 2.0, 3.0])
        assert space.volume == pytest.approx(12.0)

    def test_containment_includes_the_boundary(self) -> None:
        space = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([1.0, 1.0]))
        assert space.contains(np.array([0.0, 1.0])) is True
        assert space.contains(np.array([0.5, 0.5])) is True
        assert space.contains(np.array([1.0 + 1e-9, 0.5])) is False

    def test_samples_stay_inside_the_bounds(self) -> None:
        space = ConfigurationSpace(lower=np.array([-3.0, 4.0]), upper=np.array([-1.0, 9.0]))
        rng = np.random.default_rng(11)
        batch = space.sample_batch(rng, 500)
        assert batch.shape == (500, 2)
        assert np.all(batch >= space.lower)
        assert np.all(batch <= space.upper)

    def test_sampling_is_reproducible(self) -> None:
        space = ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([1.0, 1.0]))
        first = [space.sample(np.random.default_rng(3)) for _ in range(2)]
        second = space.sample(np.random.default_rng(3))
        assert np.array_equal(first[0], first[1])
        assert np.array_equal(first[0], second)

    def test_bounds_are_immutable(self) -> None:
        lower = np.array([0.0, 0.0])
        space = ConfigurationSpace(lower=lower, upper=np.array([1.0, 1.0]))
        lower[0] = -5.0
        assert space.lower[0] == 0.0
        with pytest.raises(ValueError):
            space.lower[0] = -5.0

    def test_rejects_degenerate_bounds(self) -> None:
        with pytest.raises(ValueError, match="upper bound"):
            ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([1.0, 0.0]))

    def test_rejects_a_non_finite_bound(self) -> None:
        with pytest.raises(ValueError, match="finite"):
            ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([1.0, np.inf]))


class TestSteer:
    """The bounded extension operator."""

    def test_returns_the_target_when_it_is_within_reach(self) -> None:
        origin = np.array([0.0, 0.0])
        target = np.array([0.3, 0.4])
        assert np.allclose(steer(origin, target, 0.5), target)

    def test_truncates_a_distant_target_to_the_step_size(self) -> None:
        origin = np.array([1.0, 1.0])
        target = np.array([4.0, 5.0])
        moved = steer(origin, target, 1.0)
        assert float(np.linalg.norm(moved - origin)) == pytest.approx(1.0)
        assert np.allclose(moved, [1.6, 1.8])

    def test_never_exceeds_the_step_size(self) -> None:
        rng = np.random.default_rng(5)
        for _ in range(200):
            origin, target = rng.uniform(-5.0, 5.0, size=(2, 4))
            moved = steer(origin, target, 0.75)
            assert float(np.linalg.norm(moved - origin)) <= 0.75 + 1e-12

    def test_rejects_a_non_positive_step(self) -> None:
        with pytest.raises(ValueError, match="step size"):
            steer(np.array([0.0]), np.array([1.0]), 0.0)


class TestNearestNeighbourIndex:
    """Agreement with brute force across the batching and rebuild schedule."""

    def test_matches_brute_force_nearest(self) -> None:
        rng = np.random.default_rng(17)
        points = rng.uniform(0.0, 10.0, size=(300, 3))
        index = NearestNeighbourIndex(3)
        for step, point in enumerate(points):
            index.add(point)
            if step % 7 != 0:
                continue
            query = rng.uniform(0.0, 10.0, size=3)
            distances = np.linalg.norm(points[: step + 1] - query, axis=1)
            assert index.nearest(query) == int(np.argmin(distances))

    def test_matches_brute_force_radius_queries(self) -> None:
        rng = np.random.default_rng(23)
        points = rng.uniform(0.0, 10.0, size=(250, 2))
        index = NearestNeighbourIndex(2)
        for point in points:
            index.add(point)
        for _ in range(25):
            query = rng.uniform(0.0, 10.0, size=2)
            radius = float(rng.uniform(0.5, 3.0))
            distances = np.linalg.norm(points - query, axis=1)
            expected = sorted(int(i) for i in np.flatnonzero(distances <= radius))
            assert index.within_radius(query, radius) == expected

    def test_assigns_indices_in_insertion_order(self) -> None:
        index = NearestNeighbourIndex(2)
        assigned = [index.add(np.array([float(i), 0.0])) for i in range(40)]
        assert assigned == list(range(40))
        assert len(index) == 40
        assert index.points[7][0] == 7.0

    def test_ties_resolve_to_the_lower_index(self) -> None:
        index = NearestNeighbourIndex(2)
        for _ in range(30):
            index.add(np.array([1.0, 1.0]))
        assert index.nearest(np.array([2.0, 2.0])) == 0

    def test_rejects_queries_against_an_empty_index(self) -> None:
        with pytest.raises(ValueError, match="empty"):
            NearestNeighbourIndex(2).nearest(np.array([0.0, 0.0]))

    def test_rejects_a_point_of_the_wrong_dimension(self) -> None:
        with pytest.raises(ValueError, match="dimension"):
            NearestNeighbourIndex(2).add(np.array([0.0, 0.0, 0.0]))
