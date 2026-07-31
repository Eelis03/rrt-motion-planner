# Rrt Motion Planner

Sampling-based motion planning with RRT, RRT star, and PRM, including a comparative benchmark harness.

[![CI](https://github.com/Eelis03/rrt-motion-planner/actions/workflows/ci.yml/badge.svg)](https://github.com/Eelis03/rrt-motion-planner/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.12-blue)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

## Overview

Three sampling-based motion planners, RRT, RRT star, and PRM, implemented against one
configuration space abstraction and one exact collision checker, together with a seeded
benchmark harness that measures what separates them. The planners work in any number of
dimensions, share a common interface, and are deterministic given a seed. The intended
reader is someone choosing a planner for a robot, who needs to know what each one costs
and what each one returns rather than only how each one is defined.

## Problem

A robot with `d` degrees of freedom occupies a configuration space in `R^d`. Given a
start configuration, a goal configuration, and a set of obstacles, find a continuous
collision-free path between them. The space is not enumerable, so the free region is
never constructed explicitly: it is probed one configuration and one straight-line
motion at a time.

Three properties of a planner matter for this problem, and they trade against each other:

1. Does it find a path when one exists, and how often, given a fixed sampling budget.
2. How good is the path it returns, measured against the length of the shortest path.
3. What does the answer cost in samples, in collision checks, and in wall clock time.

The comparison is only meaningful when the collision checker is exact. A checker that
samples points along an edge can miss a thin obstacle and report a path through a wall,
which turns a planner comparison into a comparison of collision-checking resolutions.
This repository therefore decides edge validity in closed form.

## Approach

The two tree planners follow LaValle and Kuffner: draw a configuration, extend the tree
from the vertex nearest to it by at most one step, and keep the extension when the
connecting segment is collision free. RRT stops at the first extension from which the
goal is reachable. RRT star, from Karaman and Frazzoli, spends its whole budget instead,
attaching each new vertex to the cheapest parent inside a shrinking ball and reconnecting
the other vertices of that ball through the new vertex when that lowers their cost. The
ball radius is the published one,

```
r(n) = min( gamma * (log n / n) ** (1 / d), eta ),
```

with `gamma` above the threshold `2 * (1 + 1 / d) ** (1 / d) * (mu(X_free) / zeta_d) ** (1 / d)`,
which is the condition under which the solution cost converges to the optimum. PRM,
from Kavraki and colleagues, separates a build phase, which samples collision-free
milestones and connects each to its k nearest neighbours, from a query phase, which
attaches a start and a goal to the roadmap and runs Dijkstra's algorithm over it.

Collision checking is exact and shared by all three. A box in any dimension and a convex
polygon in the plane are both intersections of half-spaces, so both are tested by the same
parametric clipping routine, which is the method of Cyrus and Beck: the segment is written
as `p(t) = a + t (b - a)`, each half-space becomes a linear constraint on `t`, and the
segment meets the region exactly when the resulting interval is not empty. A ball is tested
by minimising a convex quadratic in `t` over `[0, 1]` in closed form. Nearest neighbour and
radius queries go through `scipy.spatial.cKDTree`, wrapped in an index that rebuilds the
tree on a schedule fixed by the number of insertions, so queries stay fast without making
the result depend on anything but the seed.

`docs/design-notes.md` records the alternatives that were considered and rejected, and the
conditions under which this implementation gives poor results.

## Installation

Requires Python 3.12 or later.

```bash
git clone https://github.com/Eelis03/rrt-motion-planner.git
cd rrt-motion-planner
uv sync
```

Using pip instead of uv:

```bash
python -m venv .venv
.venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
```

## Usage

```python
import numpy as np

from rrt_planner import Box, Circle, ConfigurationSpace, ObstacleSet, PlanningProblem, RRTStar

problem = PlanningProblem(
    name="demo",
    space=ConfigurationSpace(lower=np.array([0.0, 0.0]), upper=np.array([10.0, 10.0])),
    obstacles=ObstacleSet(
        (
            Box(lower=np.array([4.0, 0.0]), upper=np.array([5.0, 7.0])),
            Circle(center=np.array([7.0, 8.5]), radius=1.2),
        )
    ),
    start=np.array([1.0, 1.0]),
    goal=np.array([9.0, 9.0]),
)

result = RRTStar(max_samples=2000, step_size=0.5).plan(problem, seed=0)
print(result.success, round(result.cost, 3), result.node_count)
# True 13.662 1755
```

Swapping in `RRT(max_samples=2000, step_size=0.5)` or `PRM(milestones=500, neighbours=10)`
requires no other change: all three satisfy the same `Planner` protocol.

Runnable examples live in `examples/`:

```bash
uv run python examples/plan_single_query.py --problem maze --samples 3000
uv run python examples/run_benchmark.py --repeats 10 --samples 3000
uv run python examples/export_viz_trace.py
```

The first solves one problem with all three planners and writes a figure. The second
produces the table below. The third writes the JSON traces read by the browser animation
in `viz/`, which is described in `viz/README.md`.

## Results

Produced by `uv run python examples/run_benchmark.py --repeats 10 --samples 3000`, on
Python 3.12.10 with numpy 2.5.1 and scipy 1.18.0, on one core of an AMD64 desktop under
Windows 11. RRT and RRT star use 3000 samples with a step size of 0.5; PRM uses 500
milestones connected to their 10 nearest neighbours. Every planner sees the same ten
seeds, 0 to 9, on each problem. Cost is averaged over successful runs, node count and
wall time over all runs, and the deviations are sample standard deviations over the ten
seeds. The whole run takes about 65 seconds.

| Problem        | Planner  | Success | Cost mean | Cost sd | Nodes mean | Nodes sd | Time mean (s) | Time sd (s) |
| -------------- | -------- | ------- | --------- | ------- | ---------- | -------- | ------------- | ----------- |
| empty          | RRT      | 10/10   | 14.98     | 0.80    | 124.0      | 28.6     | 0.004         | 0.001       |
| empty          | RRT star | 10/10   | 12.96     | 0.07    | 3002.0     | 0.0      | 0.413         | 0.034       |
| empty          | PRM      | 10/10   | 13.17     | 0.20    | 500.0      | 0.0      | 0.015         | 0.004       |
| cluttered      | RRT      | 10/10   | 17.14     | 1.85    | 290.5      | 111.5    | 0.038         | 0.017       |
| cluttered      | RRT star | 10/10   | 13.74     | 0.18    | 2135.2     | 22.8     | 0.761         | 0.077       |
| cluttered      | PRM      | 10/10   | 14.12     | 0.27    | 500.0      | 0.0      | 0.132         | 0.023       |
| narrow_passage | RRT      | 10/10   | 17.09     | 1.15    | 245.1      | 148.0    | 0.027         | 0.021       |
| narrow_passage | RRT star | 10/10   | 13.89     | 0.09    | 2549.8     | 172.3    | 0.698         | 0.060       |
| narrow_passage | PRM      | 8/10    | 14.25     | 0.27    | 500.0      | 0.0      | 0.118         | 0.011       |
| maze           | RRT      | 10/10   | 31.01     | 1.84    | 376.2      | 71.6     | 0.059         | 0.013       |
| maze           | RRT star | 10/10   | 23.64     | 0.35    | 2152.5     | 69.1     | 0.752         | 0.033       |
| maze           | PRM      | 10/10   | 24.60     | 0.79    | 500.0      | 0.0      | 0.164         | 0.004       |
| polygon_field  | RRT      | 10/10   | 17.02     | 0.95    | 210.4      | 74.2     | 0.048         | 0.020       |
| polygon_field  | RRT star | 10/10   | 13.75     | 0.09    | 2353.8     | 32.9     | 1.504         | 0.035       |
| polygon_field  | PRM      | 10/10   | 14.01     | 0.22    | 500.0      | 0.0      | 0.383         | 0.007       |
| cube_3d        | RRT      | 10/10   | 10.98     | 0.80    | 137.5      | 41.8     | 0.012         | 0.004       |
| cube_3d        | RRT star | 10/10   | 9.43      | 0.22    | 2766.6     | 25.5     | 0.721         | 0.024       |
| cube_3d        | PRM      | 10/10   | 8.86      | 0.18    | 500.0      | 0.0      | 0.151         | 0.003       |

The five planar problems all run from `(0.5, 0.5)` to `(9.5, 9.5)` in a ten by ten square,
so the straight-line lower bound on cost is 12.728 in every one of them. The three
dimensional problem runs across a cube five units on a side, where the bound is 7.794.

What the numbers say:

- Cost. RRT star is the best planner on every planar problem, and its 12.96 on the
  obstacle-free problem is 1.8 percent above the straight-line bound. RRT is between 16
  and 31 percent worse than RRT star on the planar problems, and its cost deviation is five
  to thirteen times larger there, because a feasible-path search returns whichever homotopy
  class it stumbles into first.
- Effort. RRT pays for that with two orders of magnitude less work: 0.004 to 0.059
  seconds against 0.41 to 1.50 for RRT star. RRT stops at the first solution, so its node
  count reports how hard the problem was, and 376 nodes on the maze against 124 on the
  empty square measures that directly.
- Robustness. PRM is the only planner that fails, on the narrow passage, twice in ten
  runs. Its 500 milestones are spread uniformly, so about two of them fall in the corridor
  that covers 0.4 percent of the domain, and two milestones do not always connect through
  it. The tree planners never fail there, because a sample beyond the wall pulls the tree
  towards the corridor even when the sample itself is unreachable.
- Dimension. On the three dimensional problem PRM returns the cheapest path, 8.86 against
  9.43 for RRT star, reversing the planar result. The near-neighbour radius is clamped to
  the step size, and at these vertex counts a ball of radius 0.5 in three dimensions holds
  few enough vertices that rewiring rarely finds an improvement, while the roadmap keeps
  connecting each milestone to ten neighbours whatever the dimension.

`outputs/benchmark_summary.png` shows the same table as grouped bars, and
`outputs/convergence.png` plots the incumbent cost of RRT star against iteration for five
seeds, which is the finite-sample form of the asymptotic optimality result: the cost falls
monotonically and flattens as the tree fills the free space.

## Architecture

| Module | Responsibility |
| --- | --- |
| `src/rrt_planner/model/space.py` | `ConfigurationSpace`: axis-aligned bounds in any dimension, measure, and uniform sampling. |
| `src/rrt_planner/model/obstacles.py` | `Circle`, `Box`, `ConvexPolygon`, `ObstacleSet`, and the exact segment intersection tests. |
| `src/rrt_planner/model/graph.py` | `SearchTree` with cost bookkeeping and rewiring history, `Roadmap` with Dijkstra's algorithm. |
| `src/rrt_planner/model/problem.py` | `PlanningProblem`, `PlanResult`, and path validation. |
| `src/rrt_planner/algorithm/base.py` | The `Planner` protocol, the steering function, and the k-d tree backed neighbour index. |
| `src/rrt_planner/algorithm/rrt.py` | RRT with goal biasing and a bounded extension length. |
| `src/rrt_planner/algorithm/rrt_star.py` | RRT star: near-neighbour parent selection, rewiring, cost propagation, published radius. |
| `src/rrt_planner/algorithm/prm.py` | PRM: roadmap build phase, query phase, k-nearest or radius connection. |
| `src/rrt_planner/pipeline/suite.py` | The six standard problems the benchmark reports. |
| `src/rrt_planner/pipeline/benchmark.py` | The seeded runner and the `RunTrace` record it produces per run. |
| `src/rrt_planner/pipeline/trace.py` | Export of one run to the JSON document the visualisation reads. |
| `src/rrt_planner/analysis/metrics.py` | Aggregation of run traces into per-planner summaries. |
| `src/rrt_planner/analysis/report.py` | Rendering of summaries as the Markdown table above. |
| `src/rrt_planner/analysis/figures.py` | Solution, convergence, and summary figures. |
| `examples/` | Wiring scripts, no logic. |
| `viz/` | Additive TypeScript and canvas animation, imported by nothing in `src/` or `tests/`. |

Each layer depends only on the ones above it. The model layer performs no input or output
and knows nothing about planners; the algorithm layer draws nothing and writes nothing.

## Testing

```bash
uv run pytest
uv run ruff check .
uv run mypy
```

The suite has three tiers: property and invariant tests covering the mathematics,
regression tests pinning recorded behaviour, and integration tests running each
example script under a reduced iteration count.

167 tests run in about 15 seconds. The first tier checks the collision checker against
hand-computed segment cases including boundary touches and corner grazes, checks that
every returned path starts at the start, ends at the goal, and is collision free, checks
that a seed reproduces a tree exactly, and checks that the RRT star cost does not rise
when the sample budget grows. The second tier compares 18 recorded runs in
`tests/data/reference_benchmark.json` against a fresh run, exactly on the discrete counts
and within a relative tolerance of 1e-6 on cost. The third tier runs each script in
`examples/` as a subprocess under reduced budgets, writing into a temporary directory.

## References

Algorithms:

- S. M. LaValle, "Rapidly-exploring random trees: a new tool for path planning",
  Technical Report TR 98-11, Computer Science Department, Iowa State University, 1998.
  <https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf>
- S. M. LaValle and J. J. Kuffner, "Randomized kinodynamic planning", The International
  Journal of Robotics Research 20(5), 2001, pages 378 to 400.
  DOI [10.1177/02783640122067453](https://doi.org/10.1177/02783640122067453)
- S. Karaman and E. Frazzoli, "Sampling-based algorithms for optimal motion planning",
  The International Journal of Robotics Research 30(7), 2011, pages 846 to 894.
  DOI [10.1177/0278364911406761](https://doi.org/10.1177/0278364911406761)
- L. E. Kavraki, P. Svestka, J.-C. Latombe and M. H. Overmars, "Probabilistic roadmaps for
  path planning in high-dimensional configuration spaces", IEEE Transactions on Robotics
  and Automation 12(4), 1996, pages 566 to 580.
  DOI [10.1109/70.508439](https://doi.org/10.1109/70.508439)
- M. Cyrus and J. Beck, "Generalized two- and three-dimensional clipping", Computers and
  Graphics 3(1), 1978, pages 23 to 28.
  DOI [10.1016/0097-8493(78)90021-3](https://doi.org/10.1016/0097-8493(78)90021-3)
- E. W. Dijkstra, "A note on two problems in connexion with graphs", Numerische Mathematik
  1, 1959, pages 269 to 271. DOI [10.1007/BF01386390](https://doi.org/10.1007/BF01386390)
- J. L. Bentley, "Multidimensional binary search trees used for associative searching",
  Communications of the ACM 18(9), 1975, pages 509 to 517.
  DOI [10.1145/361002.361007](https://doi.org/10.1145/361002.361007)
- S. M. LaValle, "Planning Algorithms", Cambridge University Press, 2006, chapter 5.
  DOI [10.1017/CBO9780511546877](https://doi.org/10.1017/CBO9780511546877)

Dependencies:

- [numpy](https://numpy.org/) (BSD 3-Clause). Array arithmetic, the geometric predicates,
  and the seeded PCG64 generator that makes every run reproducible.
- [scipy](https://scipy.org/) (BSD 3-Clause). `scipy.spatial.cKDTree` for nearest
  neighbour and radius queries in the planners and in the roadmap build phase.
- [matplotlib](https://matplotlib.org/) (matplotlib license, a BSD-style permissive
  license). The figures in the analysis layer.
- [pytest](https://docs.pytest.org/) (MIT), [ruff](https://docs.astral.sh/ruff/) (MIT),
  and [mypy](https://mypy-lang.org/) (MIT). Development only: test running, linting, and
  type checking.
- [TypeScript](https://www.typescriptlang.org/) (Apache-2.0). Build time only, and only
  for `viz/`. The published page loads no library at runtime.

## License

Released under the MIT license. See [LICENSE](LICENSE).
