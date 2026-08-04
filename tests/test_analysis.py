"""Tier one: aggregation, tables, and figures."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.analysis.figures import (
    convergence_figure,
    save_figure,
    solution_figure,
    summary_figure,
)
from rrt_planner.analysis.metrics import compare_paired, mean_and_deviation, summarise
from rrt_planner.analysis.report import format_paired_table, format_summary_table
from rrt_planner.model.problem import PlanningProblem
from rrt_planner.pipeline.benchmark import RunTrace, run_benchmark


def trace(planner: str, problem: str, seed: int, success: bool, cost: float) -> RunTrace:
    """Build a run trace with the fields the metrics depend on."""
    return RunTrace(
        planner=planner,
        problem=problem,
        seed=seed,
        success=success,
        cost=cost,
        node_count=100 + seed,
        iterations=500,
        point_checks=10 * (seed + 1),
        segment_checks=1000 + seed,
        wall_time_s=0.1 * (seed + 1),
        path_length=4 if success else 0,
    )


class TestMeanAndDeviation:
    """Hand-computed statistics."""

    def test_matches_hand_computed_values(self) -> None:
        mean, deviation = mean_and_deviation([2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0])
        # Mean 5, squared deviations 9 + 1 + 1 + 1 + 0 + 0 + 4 + 16 = 32, over n - 1 = 7.
        assert mean == pytest.approx(5.0)
        assert deviation == pytest.approx(math.sqrt(32.0 / 7.0))

    def test_a_single_value_has_no_spread(self) -> None:
        assert mean_and_deviation([3.5]) == (3.5, 0.0)

    def test_an_empty_sample_is_undefined(self) -> None:
        mean, deviation = mean_and_deviation([])
        assert math.isnan(mean)
        assert math.isnan(deviation)


class TestSummarise:
    """Grouping and the treatment of failures."""

    def test_groups_by_problem_and_planner(self) -> None:
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, True, 20.0),
            trace("PRM", "a", 0, True, 15.0),
            trace("RRT", "b", 0, True, 30.0),
        ]
        summaries = summarise(traces)
        assert [(s.problem, s.planner) for s in summaries] == [
            ("a", "RRT"),
            ("a", "PRM"),
            ("b", "RRT"),
        ]
        assert summaries[0].runs == 2
        assert summaries[0].cost_mean == pytest.approx(15.0)
        assert summaries[0].cost_std == pytest.approx(math.sqrt(50.0))

    def test_cost_averages_over_successful_runs_only(self) -> None:
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, False, math.inf),
            trace("RRT", "a", 2, True, 20.0),
        ]
        summary = summarise(traces)[0]
        assert summary.runs == 3
        assert summary.successes == 2
        assert summary.success_rate == pytest.approx(2.0 / 3.0)
        assert summary.cost_mean == pytest.approx(15.0)
        assert math.isfinite(summary.node_count_mean)

    def test_a_planner_that_never_succeeds_reports_an_undefined_cost(self) -> None:
        summary = summarise([trace("RRT", "a", 0, False, math.inf)])[0]
        assert summary.success_rate == 0.0
        assert math.isnan(summary.cost_mean)
        assert math.isnan(summary.cost_std)

    def test_node_count_and_time_average_over_all_runs(self) -> None:
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, False, math.inf),
        ]
        summary = summarise(traces)[0]
        assert summary.node_count_mean == pytest.approx(100.5)
        assert summary.wall_time_mean == pytest.approx(0.15)

    def test_collision_checks_average_over_all_runs_including_failures(self) -> None:
        # Seed 0 asks 10 + 1000 queries, seed 1 asks 20 + 1001, so the mean is 1015.5
        # and the sample deviation is sqrt(((1010 - 1015.5)^2 + (1021 - 1015.5)^2) / 1).
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, False, math.inf),
        ]
        summary = summarise(traces)[0]
        assert summary.collision_check_mean == pytest.approx(1015.5)
        assert summary.collision_check_std == pytest.approx(math.sqrt(2.0 * 5.5**2))

    def test_the_reported_total_is_the_sum_of_the_two_kinds(self) -> None:
        record = trace("RRT", "a", 3, True, 10.0)
        assert record.point_checks == 40
        assert record.segment_checks == 1003
        assert record.collision_checks == 1043


class TestComparePaired:
    """The seed-by-seed comparison the shared seed sequence makes available."""

    def test_differences_are_taken_seed_by_seed(self) -> None:
        # Per-seed differences 1, 3 and -1, so the mean is 1 and the squared deviations
        # are 0 + 4 + 4 over n - 1 = 2. The first planner is dearer on average and
        # cheaper on one seed of the three, which the two counts report separately.
        traces = [
            trace("RRT star", "a", 0, True, 10.0),
            trace("RRT star", "a", 1, True, 20.0),
            trace("RRT star", "a", 2, True, 30.0),
            trace("PRM", "a", 0, True, 9.0),
            trace("PRM", "a", 1, True, 17.0),
            trace("PRM", "a", 2, True, 31.0),
        ]
        comparison = compare_paired(traces, "RRT star", "PRM")[0]
        assert (comparison.planner_a, comparison.planner_b) == ("RRT star", "PRM")
        assert comparison.seeds == 3
        assert comparison.both_succeeded == 3
        assert comparison.cost_difference_mean == pytest.approx(1.0)
        assert comparison.cost_difference_std == pytest.approx(2.0)
        assert (comparison.a_cheaper, comparison.b_cheaper) == (1, 2)

    def test_only_a_seed_both_planners_solved_enters_the_cost_difference(self) -> None:
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, True, 20.0),
            trace("RRT", "a", 2, False, math.inf),
            trace("PRM", "a", 0, True, 8.0),
            trace("PRM", "a", 1, False, math.inf),
            trace("PRM", "a", 2, True, 15.0),
        ]
        comparison = compare_paired(traces, "RRT", "PRM")[0]
        assert comparison.seeds == 3
        assert comparison.both_succeeded == 1
        assert (comparison.a_only_succeeded, comparison.b_only_succeeded) == (1, 1)
        assert comparison.cost_difference_mean == pytest.approx(2.0)
        assert comparison.cost_difference_std == 0.0

    def test_a_seed_neither_planner_solved_is_counted_and_not_averaged(self) -> None:
        traces = [
            trace("RRT", "a", 0, False, math.inf),
            trace("PRM", "a", 0, False, math.inf),
        ]
        comparison = compare_paired(traces, "RRT", "PRM")[0]
        assert comparison.neither_succeeded == 1
        assert math.isnan(comparison.cost_difference_mean)
        assert math.isnan(comparison.cost_difference_std)
        assert (comparison.a_cheaper, comparison.b_cheaper) == (0, 0)

    def test_check_differences_include_the_seeds_that_failed(self) -> None:
        # Seed 0 asks 10 + 1000 against 10 + 900, a difference of 100. Seed 1 fails for
        # both planners and asks 20 + 1001 against 20 + 800, a difference of 201.
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, False, math.inf),
            replace(trace("PRM", "a", 0, True, 9.0), segment_checks=900),
            replace(trace("PRM", "a", 1, False, math.inf), segment_checks=800),
        ]
        comparison = compare_paired(traces, "RRT", "PRM")[0]
        assert comparison.both_succeeded == 1
        assert comparison.check_difference_mean == pytest.approx(150.5)
        assert comparison.check_difference_std == pytest.approx(math.sqrt(2.0 * 50.5**2))

    def test_a_seed_only_one_planner_ran_is_not_compared(self) -> None:
        traces = [
            trace("RRT", "a", 0, True, 10.0),
            trace("RRT", "a", 1, True, 20.0),
            trace("PRM", "a", 1, True, 18.0),
            trace("PRM", "a", 2, True, 12.0),
        ]
        comparison = compare_paired(traces, "RRT", "PRM")[0]
        assert comparison.seeds == 1
        assert comparison.cost_difference_mean == pytest.approx(2.0)

    def test_problems_keep_first_seen_order_and_need_a_shared_seed(self) -> None:
        traces = [
            trace("RRT", "b", 0, True, 10.0),
            trace("PRM", "b", 0, True, 9.0),
            trace("RRT", "a", 0, True, 10.0),
            trace("PRM", "a", 0, True, 9.0),
            trace("RRT", "c", 0, True, 10.0),
        ]
        assert [c.problem for c in compare_paired(traces, "RRT", "PRM")] == ["b", "a"]

    def test_one_planner_cannot_be_compared_with_itself(self) -> None:
        with pytest.raises(ValueError, match="two different planners"):
            compare_paired([trace("RRT", "a", 0, True, 10.0)], "RRT", "RRT")

    def test_a_planner_absent_from_the_traces_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="no runs recorded"):
            compare_paired([trace("RRT", "a", 0, True, 10.0)], "RRT", "PRM")


class TestReport:
    """The Markdown table pasted into the README."""

    def test_table_has_a_header_a_rule_and_one_row_per_summary(self) -> None:
        summaries = summarise(
            [
                trace("RRT", "a", 0, True, 10.0),
                trace("PRM", "a", 0, False, math.inf),
            ]
        )
        lines = format_summary_table(summaries).splitlines()
        assert len(lines) == 4
        assert lines[0].startswith("| Problem")
        assert set(lines[1]) <= {"|", "-", " "}
        assert lines[2].count("|") == lines[0].count("|")
        assert "1/1" in lines[2]

    def test_an_undefined_statistic_is_written_as_not_available(self) -> None:
        summaries = summarise([trace("PRM", "a", 0, False, math.inf)])
        assert "n/a" in format_summary_table(summaries)

    def test_an_empty_input_still_renders_a_header(self) -> None:
        lines = format_summary_table([]).splitlines()
        assert len(lines) == 2

    def test_the_paired_table_has_one_row_per_problem_compared(self) -> None:
        comparisons = compare_paired(
            [
                trace("RRT star", "a", 0, True, 10.0),
                trace("PRM", "a", 0, True, 12.5),
                trace("RRT star", "b", 0, True, 10.0),
                trace("PRM", "b", 0, True, 9.0),
            ],
            "RRT star",
            "PRM",
        )
        lines = format_paired_table(comparisons).splitlines()
        assert len(lines) == 4
        assert lines[0].startswith("| Problem")
        assert set(lines[1]) <= {"|", "-", " "}
        assert "-2.50" in lines[2]
        assert "1.00" in lines[3]

    def test_the_paired_table_writes_an_undefined_difference_as_not_available(self) -> None:
        comparisons = compare_paired(
            [
                trace("RRT", "a", 0, False, math.inf),
                trace("PRM", "a", 0, True, 12.0),
            ],
            "RRT",
            "PRM",
        )
        assert "n/a" in format_paired_table(comparisons)

    def test_an_empty_paired_input_still_renders_a_header(self) -> None:
        lines = format_paired_table([]).splitlines()
        assert len(lines) == 2


class TestFigures:
    """Figures are produced without a display and written to disk."""

    def test_solution_figure_covers_every_result(
        self, tmp_path: Path, blocked_problem: PlanningProblem
    ) -> None:
        results = [
            RRT(max_samples=400, step_size=0.6).plan(blocked_problem, 0),
            RRTStar(max_samples=300, step_size=0.6).plan(blocked_problem, 0),
            PRM(milestones=120, neighbours=6).plan(blocked_problem, 0),
        ]
        figure = solution_figure(blocked_problem, results)
        assert len(figure.axes) >= len(results)
        written = save_figure(figure, tmp_path / "figures" / "solutions.png")
        assert written.exists() and written.stat().st_size > 0

    def test_convergence_figure_uses_the_cost_history(
        self, tmp_path: Path, blocked_problem: PlanningProblem
    ) -> None:
        results = [
            RRTStar(max_samples=500, step_size=0.6).plan(blocked_problem, seed)
            for seed in (0, 1)
        ]
        written = save_figure(convergence_figure(results), tmp_path / "convergence.png")
        assert written.exists() and written.stat().st_size > 0

    def test_summary_figure_draws_four_panels(
        self, tmp_path: Path, blocked_problem: PlanningProblem
    ) -> None:
        traces = run_benchmark(
            (RRT(max_samples=300, step_size=0.6), PRM(milestones=100, neighbours=6)),
            (blocked_problem,),
            repeats=2,
            base_seed=0,
        )
        figure = summary_figure(summarise(traces))
        assert len(figure.axes) == 4
        written = save_figure(figure, tmp_path / "summary.png")
        assert written.exists() and written.stat().st_size > 0

    def test_a_three_dimensional_problem_cannot_be_drawn(self) -> None:
        import matplotlib.pyplot as plt

        from rrt_planner.analysis.figures import draw_problem
        from rrt_planner.pipeline.suite import three_dimensional_problem

        figure, axes = plt.subplots()
        try:
            with pytest.raises(ValueError, match="planar"):
                draw_problem(axes, three_dimensional_problem())
        finally:
            plt.close(figure)
