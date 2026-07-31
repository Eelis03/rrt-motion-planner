"""Tier one: hand-computed collision cases.

Every expected value in this module was worked out by hand from the geometry, not
recorded from a previous run of the code. Obstacles are closed sets throughout, so a
segment that only touches a boundary is a collision.
"""

from __future__ import annotations

import numpy as np
import pytest

from rrt_planner.model.obstacles import (
    Box,
    Circle,
    ConvexPolygon,
    ObstacleSet,
    halfspaces_intersect_segment,
)


def vector(*values: float) -> np.ndarray:
    """Shorthand for a float64 configuration."""
    return np.array(values, dtype=np.float64)


class TestCircle:
    """The unit disc centred at the origin."""

    disc = Circle(center=vector(0.0, 0.0), radius=1.0)

    @pytest.mark.parametrize(
        ("start", "end", "expected", "reason"),
        [
            ((-2.0, 0.0), (2.0, 0.0), True, "diameter crossing"),
            ((-2.0, 1.0), (2.0, 1.0), True, "tangent at (0, 1), a boundary touch"),
            ((-2.0, 1.5), (2.0, 1.5), False, "clears the disc by 0.5"),
            ((2.0, 0.0), (3.0, 0.0), False, "wholly outside, closest point at distance 2"),
            ((0.0, 0.0), (0.5, 0.0), True, "wholly inside"),
            ((-2.0, -2.0), (2.0, 2.0), True, "diagonal through the centre"),
            ((1.0, 1.0), (2.0, 2.0), False, "closest point (1, 1) is at distance sqrt(2)"),
            ((-3.0, 0.0), (-1.0, 0.0), True, "endpoint exactly on the boundary"),
        ],
    )
    def test_segment_cases(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        expected: bool,
        reason: str,
    ) -> None:
        assert self.disc.intersects_segment(vector(*start), vector(*end)) is expected, reason

    def test_degenerate_segment_is_a_point_test(self) -> None:
        assert self.disc.intersects_segment(vector(0.5, 0.5), vector(0.5, 0.5)) is True
        assert self.disc.intersects_segment(vector(2.0, 2.0), vector(2.0, 2.0)) is False

    def test_closest_point_is_interior_to_the_segment(self) -> None:
        # The segment from (-1, 0.9) to (1, 0.9) has both endpoints outside the disc at
        # distance sqrt(1 + 0.81) = 1.345, yet passes within 0.9 of the centre.
        assert float(np.linalg.norm(vector(-1.0, 0.9))) > 1.0
        assert self.disc.intersects_segment(vector(-1.0, 0.9), vector(1.0, 0.9)) is True

    def test_works_in_three_dimensions(self) -> None:
        ball = Circle(center=vector(0.0, 0.0, 0.0), radius=1.0)
        assert ball.intersects_segment(vector(-2.0, 0.0, 0.5), vector(2.0, 0.0, 0.5)) is True
        assert ball.intersects_segment(vector(-2.0, 0.0, 1.5), vector(2.0, 0.0, 1.5)) is False

    def test_rejects_a_non_positive_radius(self) -> None:
        with pytest.raises(ValueError, match="radius"):
            Circle(center=vector(0.0, 0.0), radius=0.0)


class TestBox:
    """The unit square with corners (0, 0) and (1, 1)."""

    square = Box(lower=vector(0.0, 0.0), upper=vector(1.0, 1.0))

    @pytest.mark.parametrize(
        ("start", "end", "expected", "reason"),
        [
            ((-1.0, 0.5), (2.0, 0.5), True, "crosses horizontally through the middle"),
            ((-1.0, 2.0), (2.0, 2.0), False, "passes one unit above the top edge"),
            ((0.5, -1.0), (0.5, -0.5), False, "stops half a unit below the bottom edge"),
            ((-1.0, 1.0), (1.0, -1.0), True, "the line x + y = 0 touches the corner (0, 0)"),
            ((-1.0, 0.9), (1.0, -1.1), False, "the parallel line x + y = -0.1 clears the corner"),
            ((0.25, 0.25), (0.75, 0.75), True, "wholly inside"),
            ((-1.0, 0.0), (2.0, 0.0), True, "runs along the bottom edge"),
            ((2.0, 2.0), (3.0, 3.0), False, "wholly outside"),
        ],
    )
    def test_segment_cases(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        expected: bool,
        reason: str,
    ) -> None:
        assert self.square.intersects_segment(vector(*start), vector(*end)) is expected, reason

    def test_a_long_segment_cannot_step_over_a_thin_box(self) -> None:
        # A wall 0.01 wide. Sampling the segment at any spacing coarser than 0.01 would
        # miss it; the parametric test cannot.
        wall = Box(lower=vector(0.995, 0.0), upper=vector(1.005, 10.0))
        assert wall.intersects_segment(vector(0.0, 5.0), vector(10.0, 5.0)) is True

    def test_works_in_three_dimensions(self) -> None:
        cube = Box(lower=vector(0.0, 0.0, 0.0), upper=vector(1.0, 1.0, 1.0))
        assert cube.intersects_segment(vector(-1.0, 0.5, 0.5), vector(2.0, 0.5, 0.5)) is True
        assert cube.intersects_segment(vector(-1.0, 0.5, 2.0), vector(2.0, 0.5, 2.0)) is False

    def test_rejects_inverted_bounds(self) -> None:
        with pytest.raises(ValueError, match="upper bound"):
            Box(lower=vector(1.0, 1.0), upper=vector(0.0, 2.0))


class TestConvexPolygon:
    """The triangle with vertices (0, 0), (2, 0) and (0, 2), whose hypotenuse is x + y = 2."""

    triangle = ConvexPolygon(vertices=np.array([[0.0, 0.0], [2.0, 0.0], [0.0, 2.0]]))

    @pytest.mark.parametrize(
        ("start", "end", "expected", "reason"),
        [
            ((-1.0, 1.0), (1.0, 1.0), True, "enters at (0, 1) and ends on the hypotenuse"),
            ((1.5, 1.5), (3.0, 3.0), False, "beyond the hypotenuse throughout"),
            ((1.0, 1.0), (3.0, 3.0), True, "starts exactly on the hypotenuse"),
            ((0.5, 0.5), (0.6, 0.6), True, "wholly inside"),
            ((-1.0, -1.0), (-0.5, -0.5), False, "wholly outside, past the right angle"),
            ((2.0, 0.0), (4.0, 0.0), True, "starts at the vertex (2, 0)"),
            ((-1.0, 3.0), (3.0, -1.0), True, "the line x + y = 2 contains the hypotenuse"),
            ((-1.0, 3.2), (3.2, -1.0), False, "the parallel line x + y = 2.2 misses"),
        ],
    )
    def test_segment_cases(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        expected: bool,
        reason: str,
    ) -> None:
        assert self.triangle.intersects_segment(vector(*start), vector(*end)) is expected, reason

    def test_contains_matches_hand_computed_points(self) -> None:
        assert self.triangle.contains(vector(0.5, 0.5)) is True
        assert self.triangle.contains(vector(1.0, 1.0)) is True
        assert self.triangle.contains(vector(1.01, 1.0)) is False
        assert self.triangle.contains(vector(-0.01, 0.5)) is False

    def test_vertex_order_does_not_matter(self) -> None:
        clockwise = ConvexPolygon(vertices=np.array([[0.0, 0.0], [0.0, 2.0], [2.0, 0.0]]))
        assert clockwise.contains(vector(0.5, 0.5)) is True
        assert clockwise.intersects_segment(vector(-1.0, 1.0), vector(1.0, 1.0)) is True

    def test_square_as_a_polygon_agrees_with_the_box(self) -> None:
        polygon = ConvexPolygon(
            vertices=np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        )
        box = Box(lower=vector(0.0, 0.0), upper=vector(1.0, 1.0))
        cases = [
            (vector(-1.0, 0.5), vector(2.0, 0.5)),
            (vector(-1.0, 2.0), vector(2.0, 2.0)),
            (vector(-1.0, 1.0), vector(1.0, -1.0)),
            (vector(0.25, 0.25), vector(0.75, 0.75)),
        ]
        for start, end in cases:
            assert polygon.intersects_segment(start, end) == box.intersects_segment(start, end)

    def test_rejects_a_non_convex_outline(self) -> None:
        dart = np.array([[0.0, 0.0], [2.0, 0.0], [1.0, 1.0], [2.0, 2.0], [0.0, 2.0]])
        with pytest.raises(ValueError, match="convex"):
            ConvexPolygon(vertices=dart)

    def test_rejects_too_few_vertices(self) -> None:
        with pytest.raises(ValueError, match="three vertices"):
            ConvexPolygon(vertices=np.array([[0.0, 0.0], [1.0, 1.0]]))


class TestHalfspaceClipping:
    """The primitive shared by boxes and polygons."""

    def test_empty_constraint_set_always_intersects(self) -> None:
        normals = np.zeros((0, 2), dtype=np.float64)
        offsets = np.zeros((0,), dtype=np.float64)
        assert halfspaces_intersect_segment(normals, offsets, vector(0.0, 0.0), vector(1.0, 1.0))

    def test_single_halfplane(self) -> None:
        # The region x <= 0.
        normals = np.array([[1.0, 0.0]])
        offsets = np.array([0.0])
        assert halfspaces_intersect_segment(normals, offsets, vector(-1.0, 0.0), vector(1.0, 0.0))
        assert halfspaces_intersect_segment(normals, offsets, vector(0.0, 0.0), vector(1.0, 0.0))
        assert not halfspaces_intersect_segment(
            normals, offsets, vector(0.5, 0.0), vector(1.0, 0.0)
        )

    def test_segment_parallel_to_a_constraint(self) -> None:
        # The slab 0 <= y <= 1, tested with a segment along y = 2.
        normals = np.array([[0.0, 1.0], [0.0, -1.0]])
        offsets = np.array([1.0, 0.0])
        assert not halfspaces_intersect_segment(
            normals, offsets, vector(-5.0, 2.0), vector(5.0, 2.0)
        )
        assert halfspaces_intersect_segment(
            normals, offsets, vector(-5.0, 0.5), vector(5.0, 0.5)
        )


class TestObstacleSet:
    """Composition of obstacles."""

    def test_empty_set_is_free_everywhere(self) -> None:
        empty = ObstacleSet.empty()
        assert empty.is_free(vector(0.0, 0.0)) is True
        assert empty.segment_is_free(vector(-5.0, 0.0), vector(5.0, 0.0)) is True
        assert empty.dimension is None
        assert len(empty) == 0

    def test_a_segment_is_free_only_when_every_obstacle_clears_it(self) -> None:
        obstacles = ObstacleSet(
            (
                Circle(center=vector(0.0, 0.0), radius=1.0),
                Box(lower=vector(3.0, -1.0), upper=vector(4.0, 1.0)),
            )
        )
        assert obstacles.segment_is_free(vector(-5.0, 2.0), vector(5.0, 2.0)) is True
        assert obstacles.segment_is_free(vector(1.5, 0.0), vector(2.5, 0.0)) is True
        assert obstacles.segment_is_free(vector(1.5, 0.0), vector(3.5, 0.0)) is False
        assert obstacles.segment_is_free(vector(-2.0, 0.0), vector(2.0, 0.0)) is False

    def test_rejects_mixed_dimensions(self) -> None:
        with pytest.raises(ValueError, match="dimensions"):
            ObstacleSet(
                (
                    Circle(center=vector(0.0, 0.0), radius=1.0),
                    Circle(center=vector(0.0, 0.0, 0.0), radius=1.0),
                )
            )
