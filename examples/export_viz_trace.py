"""Export JSON traces for the browser visualisation in viz/.

    uv run python examples/export_viz_trace.py --samples 1500

The files are written into viz/traces/ by default, which is where the page looks for
them. The Python package never reads them back.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rrt_planner.algorithm import PRM, RRT, RRTStar
from rrt_planner.algorithm.base import Planner
from rrt_planner.pipeline.suite import standard_problems
from rrt_planner.pipeline.trace import write_trace

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent


def parse_arguments() -> argparse.Namespace:
    """Read the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=2500)
    parser.add_argument("--milestones", type=int, default=400)
    parser.add_argument("--step-size", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=REPOSITORY_ROOT / "viz" / "traces")
    return parser.parse_args()


def main() -> None:
    """Write one trace per planner and planar problem combination of interest."""
    arguments = parse_arguments()
    by_name = {problem.name: problem for problem in standard_problems()}
    planners: dict[str, Planner] = {
        "rrt": RRT(max_samples=arguments.samples, step_size=arguments.step_size),
        "rrt_star": RRTStar(max_samples=arguments.samples, step_size=arguments.step_size),
        "prm": PRM(milestones=arguments.milestones, neighbours=10),
    }
    selection = (
        ("cluttered", "rrt"),
        ("cluttered", "rrt_star"),
        ("cluttered", "prm"),
        ("maze", "rrt_star"),
        ("narrow_passage", "rrt"),
        ("polygon_field", "rrt_star"),
    )

    written: list[Path] = []
    for problem_name, planner_key in selection:
        problem = by_name[problem_name]
        result = planners[planner_key].plan(problem, arguments.seed)
        destination = arguments.output / f"{problem_name}_{planner_key}.json"
        written.append(write_trace(destination, problem, result))
        outcome = f"cost {result.cost:.3f}" if result.success else "no path"
        print(f"{problem_name:15s} {result.planner:9s} {outcome:>14s}  {destination.name}")

    index = arguments.output / "index.json"
    index.write_text(
        "[\n" + ",\n".join(f'  "{path.name}"' for path in written) + "\n]\n",
        encoding="utf-8",
    )
    print(f"index written to {index}")


if __name__ == "__main__":
    main()
