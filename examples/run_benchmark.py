"""Run the comparative benchmark and write its table, traces, and figures.

    uv run python examples/run_benchmark.py --repeats 10 --samples 3000

The table printed by this script is the one reproduced in the Results section of the
README.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.analysis.figures import convergence_figure, save_figure, summary_figure
from rrt_planner.analysis.metrics import compare_paired, summarise
from rrt_planner.analysis.report import format_paired_table, format_summary_table
from rrt_planner.pipeline.benchmark import run_benchmark, save_traces
from rrt_planner.pipeline.suite import standard_problems


def parse_arguments() -> argparse.Namespace:
    """Read the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--milestones", type=int, default=500)
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("outputs"))
    return parser.parse_args()


def main() -> None:
    """Run every planner on every standard problem and report the aggregate."""
    arguments = parse_arguments()
    planners = (
        RRT(max_samples=arguments.samples, step_size=arguments.step_size),
        RRTStar(max_samples=arguments.samples, step_size=arguments.step_size),
        PRM(milestones=arguments.milestones, neighbours=10),
    )
    problems = standard_problems()

    traces = run_benchmark(
        planners, problems, repeats=arguments.repeats, base_seed=arguments.base_seed
    )
    summaries = summarise(traces)

    print(
        f"planners: RRT and RRT star with {arguments.samples} samples at step "
        f"{arguments.step_size}, PRM with {arguments.milestones} milestones and k = 10"
    )
    print(f"repeats: {arguments.repeats} seeds from {arguments.base_seed}")
    print()
    print(format_summary_table(summaries))
    print()
    print("paired differences per seed, RRT star minus PRM")
    print(format_paired_table(compare_paired(traces, "RRT star", "PRM")))

    save_traces(arguments.output / "benchmark_traces.json", traces)
    save_figure(summary_figure(summaries), arguments.output / "benchmark_summary.png")

    convergence = [
        RRTStar(max_samples=arguments.samples, step_size=arguments.step_size).plan(
            problems[1], seed
        )
        for seed in range(min(arguments.repeats, 5))
    ]
    save_figure(convergence_figure(convergence), arguments.output / "convergence.png")
    print()
    print(f"traces and figures written to {arguments.output}")


if __name__ == "__main__":
    main()
