"""Solve one standard problem with all three planners and draw the result.

    uv run python examples/plan_single_query.py --problem maze --samples 3000
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.analysis.figures import save_figure, solution_figure
from rrt_planner.pipeline.suite import standard_problems

PROBLEM_NAMES = tuple(problem.name for problem in standard_problems())


def parse_arguments() -> argparse.Namespace:
    """Read the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--problem", default="maze", choices=PROBLEM_NAMES)
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--milestones", type=int, default=500)
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("outputs"))
    return parser.parse_args()


def main() -> None:
    """Plan once with each planner and report the outcome."""
    arguments = parse_arguments()
    problem = next(p for p in standard_problems() if p.name == arguments.problem)
    planners = (
        RRT(max_samples=arguments.samples, step_size=arguments.step_size),
        RRTStar(max_samples=arguments.samples, step_size=arguments.step_size),
        PRM(milestones=arguments.milestones, neighbours=10),
    )

    results = [planner.plan(problem, arguments.seed) for planner in planners]
    print(f"problem: {problem.name}, seed {arguments.seed}")
    print(f"straight-line lower bound: {problem.straight_line_cost:.3f}")
    for result in results:
        outcome = f"cost {result.cost:.3f}" if result.success else "no path"
        print(
            f"  {result.planner:9s} {outcome:>13s}  nodes {result.node_count:5d}"
            f"  checks {result.collision_checks:6d}"
            f" ({result.point_checks} point, {result.segment_checks} segment)"
        )

    if problem.dimension == 2:
        written = save_figure(
            solution_figure(problem, results),
            arguments.output / f"{problem.name}_single_query.png",
        )
        print(f"figure written to {written}")


if __name__ == "__main__":
    main()
