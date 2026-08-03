"""Regenerate the three figures committed under docs/figures.

    uv run python examples/make_figures.py

Everything here is wiring. The drawing lives in ``rrt_planner.analysis.figures``,
which is the same code the other examples use, so the committed figures cannot drift
away from what the library actually produces.

The seeds and budgets below are fixed rather than taken from the command line for the
defaults that matter, so that re-running this reproduces the same pictures. Matplotlib
output is not byte reproducible across platforms, so the files will differ bit for bit
between machines even when the planning is identical.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.analysis.figures import convergence_figure, save_figure, solution_figure
from rrt_planner.pipeline.suite import standard_problems

# Seed 4 is one of the two seeds in 0 to 9 on which PRM fails to connect the narrow
# corridor at 500 milestones, which is what the figure exists to show.
COMPARISON_SEED = 0
FAILURE_SEED = 4
CONVERGENCE_SEEDS = (0, 1, 2, 3, 4)

# The published figures share a 250 KiB budget for the whole repository, and the two
# that draw thousands of tree edges are what spends it. Their resolution is therefore
# chosen against that budget rather than for print. The convergence plot is five lines
# on white, so it costs almost nothing and is drawn at a resolution that keeps its axis
# labels crisp.
SOLUTION_DPI = 72
CONVERGENCE_DPI = 86
BUDGET_KIB = 250.0


def parse_arguments() -> argparse.Namespace:
    """Read the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=3000)
    parser.add_argument("--milestones", type=int, default=500)
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--dpi", type=int, default=SOLUTION_DPI)
    parser.add_argument("--output", type=Path, default=Path("docs") / "figures")
    return parser.parse_args()


def main() -> None:
    """Write the three published figures and report their sizes against the budget."""
    arguments = parse_arguments()
    by_name = {problem.name: problem for problem in standard_problems()}
    rrt = RRT(max_samples=arguments.samples, step_size=arguments.step_size)
    rrt_star = RRTStar(max_samples=arguments.samples, step_size=arguments.step_size)
    prm = PRM(milestones=arguments.milestones, neighbours=10)

    maze = by_name["maze"]
    passage = by_name["narrow_passage"]

    written: list[Path] = []

    # One: what rewiring does to the tree and to the path it returns.
    written.append(
        save_figure(
            solution_figure(
                maze,
                [rrt.plan(maze, COMPARISON_SEED), rrt_star.plan(maze, COMPARISON_SEED)],
                columns=2,
            ),
            arguments.output / "rrt-vs-rrt-star-maze.png",
            dpi=arguments.dpi,
        )
    )

    # Two: why the roadmap is the only planner that fails on the narrow passage.
    written.append(
        save_figure(
            solution_figure(
                passage,
                [rrt.plan(passage, FAILURE_SEED), prm.plan(passage, FAILURE_SEED)],
                columns=2,
            ),
            arguments.output / "narrow-passage-tree-vs-roadmap.png",
            dpi=arguments.dpi,
        )
    )

    # Three: the incumbent cost falling as the tree fills the free space.
    written.append(
        save_figure(
            convergence_figure([rrt_star.plan(maze, seed) for seed in CONVERGENCE_SEEDS]),
            arguments.output / "rrt-star-convergence.png",
            dpi=CONVERGENCE_DPI,
        )
    )

    total = 0
    for path in written:
        size = path.stat().st_size
        total += size
        print(f"{path} {size / 1024:.1f} KiB")
    print(f"total {total / 1024:.1f} KiB of the {BUDGET_KIB:.1f} KiB budget")


if __name__ == "__main__":
    main()
