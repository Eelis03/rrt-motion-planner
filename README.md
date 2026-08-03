# Rrt Motion Planner

A seeded benchmark of three sampling-based motion planners, RRT, RRT star, and PRM,
built against one configuration space, one exact collision checker, and one set of
problems, so that every difference it reports is a difference between the algorithms.

[![CI](https://github.com/Eelis03/rrt-motion-planner/actions/workflows/ci.yml/badge.svg)](https://github.com/Eelis03/rrt-motion-planner/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/python-3.12-blue)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

![RRT and RRT star on the maze problem from the same seed, drawn side by side. RRT stops at its first solution and returns a path that wanders back on itself at cost 31.71 from 499 nodes. RRT star spends the whole budget, fills the free space with 2024 nodes, and its rewired path runs almost straight through both gaps at cost 23.96.](docs/figures/rrt-vs-rrt-star-maze.png)

That picture is the whole question in one frame. Rewiring costs RRT star thirteen times
the wall time of RRT on this problem and returns a path a quarter shorter. Whether that
is a good trade depends on the problem, and the table below is an attempt to say where it
is and where it is not.

## Results

Produced by `uv run python examples/run_benchmark.py --repeats 10 --samples 3000`, on
Python 3.12.10 with numpy 2.5.1 and scipy 1.18.0, on one core of an AMD64 desktop under
Windows 11. RRT and RRT star use 3000 samples with a step size of 0.5; PRM uses 500
milestones connected to their 10 nearest neighbours. Every planner sees the same ten
seeds, 0 to 9, on each problem. Cost is averaged over successful runs; node count,
collision checks, and wall time over all runs. Deviations are sample standard deviations
over the ten seeds. A check is one question put to the collision checker, either whether
a configuration is free or whether a straight segment is. The whole run takes about 80
seconds.

| Problem        | Planner  | Success | Cost mean | Cost sd | Nodes mean | Nodes sd | Checks mean | Checks sd | Time mean (s) | Time sd (s) |
| -------------- | -------- | ------- | --------- | ------- | ---------- | -------- | ----------- | --------- | ------------- | ----------- |
| empty          | RRT      | 10/10   | 14.98     | 0.80    | 124.0      | 28.6     | 123         | 29        | 0.005         | 0.001       |
| empty          | RRT star | 10/10   | 12.96     | 0.07    | 3002.0     | 0.0      | 11625       | 224       | 0.518         | 0.088       |
| empty          | PRM      | 10/10   | 13.17     | 0.20    | 500.0      | 0.0      | 3406        | 14        | 0.017         | 0.001       |
| cluttered      | RRT      | 10/10   | 17.14     | 1.85    | 290.5      | 111.5    | 438         | 168       | 0.041         | 0.017       |
| cluttered      | RRT star | 10/10   | 13.74     | 0.18    | 2135.2     | 22.8     | 8613        | 185       | 0.894         | 0.037       |
| cluttered      | PRM      | 10/10   | 14.12     | 0.27    | 500.0      | 0.0      | 3625        | 36        | 0.176         | 0.006       |
| narrow_passage | RRT      | 10/10   | 17.09     | 1.15    | 245.1      | 148.0    | 450         | 344       | 0.048         | 0.037       |
| narrow_passage | RRT star | 10/10   | 13.89     | 0.09    | 2549.8     | 172.3    | 9839        | 664       | 0.998         | 0.110       |
| narrow_passage | PRM      | 8/10    | 14.25     | 0.27    | 500.0      | 0.0      | 3487        | 20        | 0.150         | 0.012       |
| maze           | RRT      | 10/10   | 31.01     | 1.84    | 376.2      | 71.6     | 781         | 148       | 0.069         | 0.014       |
| maze           | RRT star | 10/10   | 23.64     | 0.35    | 2152.5     | 69.1     | 8387        | 260       | 0.932         | 0.056       |
| maze           | PRM      | 10/10   | 24.60     | 0.79    | 500.0      | 0.0      | 3566        | 19        | 0.191         | 0.005       |
| polygon_field  | RRT      | 10/10   | 17.02     | 0.95    | 210.4      | 74.2     | 298         | 102       | 0.054         | 0.019       |
| polygon_field  | RRT star | 10/10   | 13.75     | 0.09    | 2353.8     | 32.9     | 9687        | 261       | 1.912         | 0.115       |
| polygon_field  | PRM      | 10/10   | 14.01     | 0.22    | 500.0      | 0.0      | 3575        | 34        | 0.464         | 0.045       |
| cube_3d        | RRT      | 10/10   | 10.98     | 0.80    | 137.5      | 41.8     | 165         | 47        | 0.014         | 0.005       |
| cube_3d        | RRT star | 10/10   | 9.43      | 0.22    | 2766.6     | 25.5     | 8649        | 199       | 0.848         | 0.029       |
| cube_3d        | PRM      | 10/10   | 8.86      | 0.18    | 500.0      | 0.0      | 3516        | 28        | 0.169         | 0.007       |

The five planar problems all run from `(0.5, 0.5)` to `(9.5, 9.5)` in a ten by ten
square, so the straight-line lower bound on cost is 12.728 in every one of them. The
three dimensional problem crosses a cube five units on a side, where the bound is 7.794.
Both are printed by `examples/plan_single_query.py`.

### Where RRT star earns its cost, and where it does not

**It earns it on the maze.** 23.64 against RRT's 31.01 and PRM's 24.60, for 0.932 seconds
and 8387 checks against RRT's 0.069 and 781. A path a quarter shorter is worth an order
of magnitude more planning on any problem where the path is driven more than once.

**It earns it on the narrow passage, for a different reason.** 13.89 at 10 successes out
of 10, against PRM's 14.25 at 8 out of 10. The gain there is not the 0.36 in cost, it is
that the tree planners never failed and the roadmap did.

**It does not earn it on the empty square.** 12.96 against PRM's 13.17 is a gain of 0.21
in exchange for 0.518 seconds against 0.017, and 11625 checks against 3406. When the free
space is convex, a roadmap is already close to optimal and rewiring has almost nothing
left to find.

**It does not earn it on the cube.** PRM returns 8.86 against RRT star's 9.43 and takes a
fifth of the time doing it. RRT star loses outright, and the reason is in the radius:
the near-neighbour radius is clamped to the step size of 0.5, and a ball of that radius
in three dimensions holds few enough vertices that rewiring rarely finds an improvement,
while the roadmap keeps connecting each milestone to ten neighbours whatever the
dimension.

### What else the table shows

**Cost variance separates a feasible search from an optimal one.** RRT's cost deviation
is five to thirteen times larger than RRT star's on every planar problem, because a
feasible-path search returns whichever homotopy class it stumbles into first. RRT star's
0.07 on the empty square is a planner that returns essentially the same answer every
time, 1.8 percent above the straight-line bound.

**Effort in checks, not seconds, is what compares across machines.** RRT asks between 123
and 781 collision questions, RRT star between 8387 and 11625, PRM between 3406 and 3625.
On the maze RRT star asks about eleven times what RRT does; on the empty square the gap
is close to a hundredfold, because RRT finishes almost at once there while RRT star never
finishes early at all.
PRM's count barely moves across problems, spanning under seven percent, because its
budget is fixed at 500 milestones and ten neighbours whatever the obstacles look like.

**A check is portable, but its price is not.** RRT star asks 9687 checks on
`polygon_field` and 8649 on `cube_3d`, within twelve percent of each other, and spends
1.912 seconds against 0.848. A convex polygon rebuilds its half-space representation on
every query while a box in three dimensions has six fixed planes. This is why the table
keeps both columns rather than replacing wall time with checks.

### Why PRM is the only planner that fails

![The narrow passage problem under RRT and under PRM on seed 4, side by side. The RRT tree threads the 0.4 wide gap in the wall and returns a path at cost 16.30 from 115 nodes. The PRM roadmap covers both halves of the domain densely with 500 milestones, but no roadmap edge crosses the gap, the two halves stay disconnected, and PRM returns no path.](docs/figures/narrow-passage-tree-vs-roadmap.png)

The corridor is 0.4 wide in a wall 1.0 thick, so it covers 0.4 percent of the sampling
domain and about two of PRM's 500 milestones land inside it. Two milestones do not always
connect through, and on 2 of the 10 seeds they did not. The right-hand panel above shows
exactly that: a roadmap that is dense everywhere and severed at the one place that
matters.

The tree planners never fail here, and the left-hand panel shows why. A sample drawn
beyond the wall is unreachable, but the vertex nearest to it is the vertex closest to the
gap, so the tree is pulled towards the corridor by samples that can never be reached. The
roadmap has no equivalent mechanism, because it samples before it knows what it is trying
to connect.

## Why the three planners differ

The two tree planners follow LaValle and Kuffner: draw a configuration, extend the tree
from the vertex nearest to it by at most one step, and keep the extension when the
connecting segment is collision free. The Voronoi bias in that rule, that the vertex
nearest a uniform sample is the vertex whose Voronoi region is largest, is what makes the
tree grow outward rather than thicken where it already is.

**RRT** stops at the first extension from which the goal is reachable. That single
decision is the whole of its cost profile: it is the cheapest planner here by two orders
of magnitude, it makes no optimality claim, and its node count reports how hard the
problem was rather than how large its budget was. 376 nodes on the maze against 124 on
the empty square measures the difficulty of the maze directly.

**RRT star**, from Karaman and Frazzoli, spends the whole budget instead. Each new vertex
is attached to the cheapest parent inside a shrinking ball, and the other vertices of
that ball are reconnected through it whenever that lowers their cost. The ball radius is
the published one,

```
r(n) = min( gamma * (log n / n) ** (1 / d), eta ),
```

with `gamma` above the threshold `2 * (1 + 1 / d) ** (1 / d) * (mu(X_free) / zeta_d) ** (1 / d)`,
which is the condition under which the solution cost converges to the optimum. The
convergence is not only asymptotic in principle, it is visible in a single run:

![Incumbent RRT star solution cost against iteration on the maze, for seeds 0 to 4. Each curve begins where that seed first reached the goal, between iteration 750 and 900 at a cost between 28 and 31, then falls in discrete steps as rewiring finds improvements, and every curve has flattened between 23.7 and 24.1 by iteration 3000.](docs/figures/rrt-star-convergence.png)

The goal is inserted as an ordinary tree vertex the first time it becomes reachable, so
it takes part in rewiring like any other vertex and its cost, which is the solution cost,
can only fall. That is why every curve is monotone.

**PRM**, from Kavraki and colleagues, separates a build phase, which samples
collision-free milestones and connects each to its k nearest neighbours, from a query
phase, which attaches a start and a goal to the roadmap and runs Dijkstra's algorithm
over it. It is the only multi-query member of the three, and the implementation keeps
that visible: `build` returns a roadmap, `query` answers against one without modifying
it, and a test asserts that a query leaves it unchanged. The benchmark charges it for a
fresh roadmap on every run, which is the fair thing to do for a single-query comparison
and the wrong thing to do if you have many queries in one environment.

## Why these numbers can be trusted

**The collision checker is exact.** A checker that samples points along an edge can step
over a thin obstacle and report a path through a wall, which turns a planner comparison
into a comparison of collision-checking resolutions. Edge validity here is decided in
closed form. A box in any dimension and a convex polygon in the plane are both
intersections of half-spaces, so both go through one parametric clipping routine, the
method of Cyrus and Beck: the segment is written as `p(t) = a + t (b - a)`, each
half-space becomes a linear constraint on `t`, and the segment meets the region exactly
when the resulting interval is not empty. A ball is decided by minimising a convex
quadratic in `t` over `[0, 1]`. A test drives a segment through a box 0.01 wide that any
point sampling at a coarser spacing would miss.

**The comparison is paired.** Run `i` of a problem uses the same seed for every planner,
so the seed-to-seed variance all three face is the same variance.

**Every run is reproducible from its seed.** Each planner builds its own
`numpy.random.Generator`, nothing reads a global random state, and no planner mutates
itself during a run. A test compares two runs of one seed vertex by vertex, parent by
parent, and cost by cost.

**Recorded behaviour is pinned.** `tests/data/reference_benchmark.json` holds 18 recorded
runs that a test reproduces from scratch: exactly on every discrete count, including the
collision check counts, and within a relative tolerance of 1e-6 on cost. Wall time is the
one field not compared, and it is stored as zero so that nothing invites the comparison.

**The committed figures are snapshots, not build output.** They were written by
`uv run python examples/make_figures.py` and are checked in so the page renders without a
build step. CI does not compare them byte for byte, because matplotlib output is not byte
reproducible across platforms: the same planning produces different PNG bytes on a
different matplotlib build or operating system. What CI does check is that the generator
still runs, that every figure it names is committed, and that the three together stay
inside the 250 KiB budget this repository is held to.

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

The package ships `py.typed`, so anything that installs it type checks against real
annotations rather than against `Any`.

## Reproducing every number on this page

| Command | What it produces |
| --- | --- |
| `uv run python examples/run_benchmark.py --repeats 10 --samples 3000` | The results table above, the trace JSON, and the summary figures. About 80 seconds. |
| `uv run python examples/plan_single_query.py --problem maze --samples 3000` | One problem solved by all three planners, with costs, node counts, and check counts. |
| `uv run python examples/make_figures.py` | The three figures committed under `docs/figures/`. |
| `uv run python examples/export_viz_trace.py` | The JSON traces read by the browser animation in `viz/`. |

Calling the library directly takes no more than this, and swapping in
`RRT(max_samples=2000, step_size=0.5)` or `PRM(milestones=500, neighbours=10)` requires
no other change, because all three satisfy the same `Planner` protocol:

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
print(result.success, round(result.cost, 3), result.node_count, result.collision_checks)
# True 13.662 1755 5787
```

Checks, lint, and types:

```bash
uv run pytest -q
uv run ruff check .
uv run mypy
uv run pytest --cov=src/rrt_planner --cov-report=term-missing
```

196 tests run in about 30 seconds. Statement coverage of `src/rrt_planner` is 96 percent,
1105 statements with 44 uncovered, reported by the fourth command as 96.02; CI runs that
command with `--cov-fail-under=94` in the Python test job, and nowhere else. The suite has three tiers: property and
invariant tests covering the mathematics, regression tests pinning recorded behaviour,
and integration tests running each script in `examples/` as a subprocess under a reduced
budget, writing into a temporary directory.

## How the code is arranged

Each layer depends only on the ones above it. The model layer performs no input or output
and knows nothing about planners; the algorithm layer draws nothing and writes nothing.

| Module | Responsibility |
| --- | --- |
| `src/rrt_planner/model/space.py` | `ConfigurationSpace`: axis-aligned bounds in any dimension, measure, and uniform sampling. |
| `src/rrt_planner/model/obstacles.py` | `Circle`, `Box`, `ConvexPolygon`, `ObstacleSet`, the exact segment intersection tests, and `CountingChecker`, which records the queries a run makes. |
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

Nearest neighbour and radius queries go through `scipy.spatial.cKDTree`, wrapped in an
index that rebuilds the tree on a schedule fixed by the number of insertions, so queries
stay fast without making the result depend on anything but the seed.

## What it does not do

An edge is a straight segment, so the planners cover the holonomic case; anything with
motion constraints needs `steer` in `algorithm/base.py` replaced by a solver for the
local boundary value problem. Distance is the L2 norm, so a revolute joint that wraps at
`2 pi` is not represented. Obstacles must be convex, and a non-convex one has to be
supplied as a union of pieces. `ConvexPolygon` is planar, so `cube_3d` appears in the
benchmark but not in the browser animation, which `trace_document` restricts to two
dimensions. Success rates come from ten seeds, so 8 out of 10 carries a wide confidence
interval.

`docs/design-notes.md` records these in full, along with the alternatives that were
considered and rejected, and one limitation that has since been closed: collision checks
were not counted, and now they are, which is where the `Checks` columns above came from.

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
- [pytest](https://docs.pytest.org/) and [pytest-cov](https://pytest-cov.readthedocs.io/)
  (both MIT), [ruff](https://docs.astral.sh/ruff/) (MIT), and
  [mypy](https://mypy-lang.org/) (MIT). Development only: test running, coverage
  measurement, linting, and type checking.
- [TypeScript](https://www.typescriptlang.org/) (Apache-2.0). Build time only, and only
  for `viz/`. The published page loads no library at runtime.

## License

Released under the MIT license. See [LICENSE](LICENSE).
