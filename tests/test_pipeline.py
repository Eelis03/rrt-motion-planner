"""Tier one and tier two: the problem suite, the benchmark runner, and trace export."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.pipeline.benchmark import (
    RunTrace,
    load_traces,
    run_benchmark,
    save_traces,
    seeds_for,
)
from rrt_planner.pipeline.suite import standard_problems
from rrt_planner.pipeline.trace import TRACE_FORMAT, trace_document, write_trace


class TestStandardProblems:
    """The suite is well formed and solvable."""

    def test_names_are_unique_and_stable(self) -> None:
        problems = standard_problems()
        names = [problem.name for problem in problems]
        assert names == [
            "empty",
            "cluttered",
            "narrow_passage",
            "maze",
            "polygon_field",
            "cube_3d",
        ]
        assert len(set(names)) == len(names)

    def test_endpoints_are_valid_in_every_problem(self) -> None:
        for problem in standard_problems():
            assert problem.space.contains(problem.start)
            assert problem.space.contains(problem.goal)
            assert problem.obstacles.is_free(problem.start)
            assert problem.obstacles.is_free(problem.goal)

    def test_the_suite_covers_two_and_three_dimensions(self) -> None:
        dimensions = {problem.dimension for problem in standard_problems()}
        assert dimensions == {2, 3}

    def test_only_the_empty_problem_admits_the_straight_line(self) -> None:
        for problem in standard_problems():
            direct = problem.obstacles.segment_is_free(problem.start, problem.goal)
            assert direct == (problem.name == "empty")

    @pytest.mark.parametrize("name", ["empty", "cluttered", "narrow_passage", "polygon_field"])
    def test_each_problem_is_solvable_within_a_small_budget(self, name: str) -> None:
        problem = next(p for p in standard_problems() if p.name == name)
        result = RRT(max_samples=4000, step_size=0.5).plan(problem, seed=0)
        assert result.success is True
        assert problem.path_is_valid(result.path)


class TestBenchmarkRunner:
    """Ordering, seeding, and record contents."""

    def test_seeds_are_consecutive_from_the_base(self) -> None:
        assert seeds_for(4, 100) == (100, 101, 102, 103)
        with pytest.raises(ValueError, match="repeats"):
            seeds_for(0, 0)

    def test_produces_one_record_per_planner_problem_and_seed(
        self, free_problem: PlanningProblem, blocked_problem: PlanningProblem
    ) -> None:
        planners = (RRT(max_samples=400, step_size=0.6), PRM(milestones=120, neighbours=6))
        traces = run_benchmark(
            planners, (free_problem, blocked_problem), repeats=3, base_seed=10
        )
        assert len(traces) == 2 * 2 * 3
        assert [trace.problem for trace in traces[:6]] == ["free"] * 6
        assert [trace.planner for trace in traces[:3]] == ["RRT"] * 3
        assert [trace.seed for trace in traces[:3]] == [10, 11, 12]

    def test_planners_share_the_same_seeds(self, blocked_problem: PlanningProblem) -> None:
        planners = (RRT(max_samples=400, step_size=0.6), RRTStar(max_samples=200, step_size=0.6))
        traces = run_benchmark(planners, (blocked_problem,), repeats=4, base_seed=7)
        by_planner: dict[str, list[int]] = {}
        for trace in traces:
            by_planner.setdefault(trace.planner, []).append(trace.seed)
        seed_lists = list(by_planner.values())
        assert seed_lists[0] == seed_lists[1]

    def test_records_carry_positive_timings_and_node_counts(
        self, blocked_problem: PlanningProblem
    ) -> None:
        traces = run_benchmark(
            (RRT(max_samples=400, step_size=0.6),), (blocked_problem,), repeats=2, base_seed=0
        )
        for trace in traces:
            assert trace.wall_time_s > 0.0
            assert trace.node_count > 0
            assert trace.iterations > 0
            assert (trace.path_length >= 2) == trace.success

    def test_records_carry_the_collision_check_counts(
        self, blocked_problem: PlanningProblem
    ) -> None:
        traces = run_benchmark(
            (RRT(max_samples=400, step_size=0.6), PRM(milestones=120, neighbours=6)),
            (blocked_problem,),
            repeats=2,
            base_seed=0,
        )
        for record in traces:
            assert record.segment_checks > 0
            assert record.collision_checks == record.point_checks + record.segment_checks
            # A tree planner never tests a configuration on its own: it only ever asks
            # whether an extension is admissible. A roadmap rejects sampled milestones
            # by point query before it connects anything.
            if record.planner == "RRT":
                assert record.point_checks == 0
            else:
                assert record.point_checks >= 120

    def test_failure_is_recorded_with_an_infinite_cost(
        self, sealed_problem: PlanningProblem
    ) -> None:
        traces = run_benchmark(
            (RRT(max_samples=300, step_size=0.6),), (sealed_problem,), repeats=1, base_seed=0
        )
        assert traces[0].success is False
        assert math.isinf(traces[0].cost)
        assert traces[0].path_length == 0

    def test_rejects_empty_inputs(self, free_problem: PlanningProblem) -> None:
        with pytest.raises(ValueError, match="planner"):
            run_benchmark((), (free_problem,))
        with pytest.raises(ValueError, match="problem"):
            run_benchmark((RRT(),), ())


class TestTracePersistence:
    """Run traces survive a round trip through JSON."""

    def test_round_trip_preserves_every_field(
        self, tmp_path: Path, blocked_problem: PlanningProblem, sealed_problem: PlanningProblem
    ) -> None:
        traces = run_benchmark(
            (RRT(max_samples=300, step_size=0.6),),
            (blocked_problem, sealed_problem),
            repeats=2,
            base_seed=0,
        )
        destination = tmp_path / "nested" / "traces.json"
        save_traces(destination, traces)
        restored = load_traces(destination)
        assert restored == traces

    def test_an_infinite_cost_is_stored_as_null(
        self, tmp_path: Path, sealed_problem: PlanningProblem
    ) -> None:
        traces = (
            RunTrace(
                planner="RRT",
                problem=sealed_problem.name,
                seed=0,
                success=False,
                cost=math.inf,
                node_count=12,
                iterations=300,
                point_checks=0,
                segment_checks=298,
                wall_time_s=0.5,
                path_length=0,
            ),
        )
        destination = tmp_path / "traces.json"
        save_traces(destination, traces)
        payload = json.loads(destination.read_text(encoding="utf-8"))
        assert payload[0]["cost"] is None
        assert math.isinf(load_traces(destination)[0].cost)


class TestVisualisationTrace:
    """The JSON document the visualisation layer consumes."""

    def test_document_describes_the_problem_and_the_tree(self) -> None:
        problem = next(p for p in standard_problems() if p.name == "polygon_field")
        result = RRTStar(max_samples=400, step_size=0.5).plan(problem, seed=0)
        document = trace_document(problem, result)

        assert document["format"] == TRACE_FORMAT
        assert document["planner"] == "RRT star"
        assert document["bounds"] == {"lower": [0.0, 0.0], "upper": [10.0, 10.0]}
        assert document["start"] == [0.5, 0.5]
        kinds = {obstacle["kind"] for obstacle in document["obstacles"]}
        assert kinds == {"polygon", "circle", "box"}

        structure = document["structure"]
        assert structure["kind"] == "tree"
        assert len(structure["vertices"]) == result.node_count
        assert len(structure["parents"]) == result.node_count
        assert structure["parents"][0] == -1
        assert all(parent < index for index, parent in enumerate(structure["parents"]))

    def test_document_describes_a_roadmap_when_prm_was_used(self) -> None:
        problem = next(p for p in standard_problems() if p.name == "cluttered")
        result = PRM(milestones=120, neighbours=6).plan(problem, seed=0)
        assert result.roadmap is not None
        structure = trace_document(problem, result)["structure"]
        assert structure["kind"] == "roadmap"
        assert structure["parents"] == []
        assert len(structure["edges"]) == result.roadmap.edge_count
        assert all(0 <= a < 120 and 0 <= b < 120 for a, b in structure["edges"])

    def test_document_is_json_serialisable_and_finite(self, tmp_path: Path) -> None:
        problem = next(p for p in standard_problems() if p.name == "maze")
        result = RRT(max_samples=1500, step_size=0.5).plan(problem, seed=0)
        written = write_trace(tmp_path / "trace.json", problem, result)
        payload = json.loads(written.read_text(encoding="utf-8"))
        assert payload["success"] is True
        assert payload["cost"] == pytest.approx(result.cost)
        flat = json.dumps(payload)
        assert "Infinity" not in flat
        assert "NaN" not in flat

    def test_rewires_reference_valid_vertices(self) -> None:
        problem = next(p for p in standard_problems() if p.name == "cluttered")
        result = RRTStar(max_samples=400, step_size=0.5).plan(problem, seed=0)
        document = trace_document(problem, result)
        count = len(document["structure"]["vertices"])
        assert document["structure"]["rewires"]
        for step, node, parent in document["structure"]["rewires"]:
            assert 0 < step <= count
            assert 0 <= node < count
            assert 0 <= parent < count
            assert node != parent

    def test_a_three_dimensional_problem_cannot_be_exported(self) -> None:
        problem = next(p for p in standard_problems() if p.name == "cube_3d")
        result = RRT(max_samples=200, step_size=0.5).plan(problem, seed=0)
        with pytest.raises(ValueError, match="planar"):
            trace_document(problem, result)

    def test_the_exported_path_matches_the_result(self) -> None:
        problem = next(p for p in standard_problems() if p.name == "empty")
        result = RRT(max_samples=500, step_size=0.5).plan(problem, seed=0)
        exported = np.array(trace_document(problem, result)["path"])
        assert np.allclose(exported, np.vstack(result.path))
