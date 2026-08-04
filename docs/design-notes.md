# Design notes for Rrt Motion Planner

## Method selection

### Three planners rather than one

The subject of this repository is the comparison, so all three planners are implemented
behind one `Planner` protocol, over one configuration space abstraction and one collision
checker. Any difference the benchmark reports is then a difference between the algorithms
and not between two collision checkers or two distance metrics. The protocol is narrow on
purpose: a name, and a `plan(problem, seed)` that returns a `PlanResult`. Everything else,
including the sample budget and the step size, belongs to the planner instance, which is a
frozen dataclass, so a planner instance carries no state between runs.

### RRT, following LaValle and Kuffner

RRT is the feasibility baseline. The Voronoi bias of the extension rule, that the vertex
nearest to a uniform sample is the vertex whose Voronoi region is largest, is what makes
the tree grow towards unexplored regions rather than thicken where it already is, and it
is what lets the tree planners solve the narrow passage that PRM sometimes fails on. Goal
biasing is included at the usual 5 percent, and the search stops at the first vertex from
which the goal is reachable by one collision-free segment. This makes RRT the cheapest of
the three by two orders of magnitude, and the worst by path cost. The assumption it
depends on is that a path exists with clearance large enough that a uniform sample lands
near it within the budget. The maze problem, whose best solutions here need about 50 steps
of length 0.5, sits near that limit and is the reason it was included.

### RRT star, following Karaman and Frazzoli

The rewiring radius is taken from the paper rather than tuned:

```
r(n) = min( gamma * (log n / n) ** (1 / d), eta ),
gamma > 2 * (1 + 1 / d) ** (1 / d) * (mu(X_free) / zeta_d) ** (1 / d).
```

Two implementation decisions follow from it.

First, the measure of the free space `mu(X_free)` is not available to a planner that only
probes the space, so the measure of the whole configuration space is substituted. That is
an over-estimate, it can only raise `gamma`, and the convergence result needs `gamma` to
be above the threshold, so the substitution is safe. It is paid for in near-neighbour sets
that are larger than strictly necessary, which is wasted collision checking, not a wrong
answer. The multiplier on the threshold is exposed as `gamma_scale` and is validated to be
above 1, so the guarantee cannot be broken by a configuration mistake.

Second, at the vertex counts used here the radius is always clamped to the step size `eta`.
In the ten by ten planar problems, `gamma` is about 15.2, so the unclamped radius only
falls below the step size of 0.5 beyond about 8300 vertices. This is the normal regime for
RRT star, and it is the reason the three dimensional result differs: a ball of radius 0.5
in three dimensions holds far fewer vertices than in two, so rewiring finds fewer
improvements, and PRM overtakes RRT star on `cube_3d`.

Costs are propagated to the whole affected subtree after every rewiring, so a stored cost
is always the exact length of the tree path to that vertex. A test asserts exactly that
over every vertex, rather than only over the vertex on the solution path, because the
cheaper alternative, updating the rewired vertex and leaving its descendants stale, is a
mistake that stays invisible until it silently degrades the returned path.

The goal is inserted as an ordinary tree vertex the first time it becomes reachable. It
then takes part in rewiring like any other vertex, so its cost, which is the solution
cost, falls monotonically for the rest of the run. Rewiring can never create a cycle: an
ancestor `a` of a new vertex `x` satisfies `cost(x) >= cost(a) + d(a, x)` by the triangle
inequality along the tree path, so `cost(x) + d(x, a) > cost(a)` and the strict improvement
test that guards a reparenting can never select an ancestor.

### PRM, following Kavraki, Svestka, Latombe and Overmars

PRM is the multi-query member of the set, and the implementation keeps that visible:
`build` returns a roadmap, `query` answers against a roadmap without modifying it, and
`plan` composes the two so that PRM still satisfies the single-query protocol the benchmark
needs. A query copies the adjacency, attaches the start and the goal to their k nearest
milestones, and runs Dijkstra's algorithm, so the same roadmap answers any number of
queries and a test asserts that a query leaves it unchanged.

The k-nearest relation is not symmetric, so candidate pairs are collected into a set and
sorted before edges are tested. Without that, an edge would be created or not depending on
which endpoint happened to be visited first.

### Exact collision checking

Edge validity is decided in closed form, never by sampling points along the edge. Sampling
is the common shortcut and it is unsound: an obstacle thinner than the sampling interval is
stepped over, and the planner then returns a path through a wall. A test in
`tests/test_geometry.py` drives a segment through a box 0.01 wide, which any point sampling
at a coarser spacing would miss.

Boxes in any dimension and convex polygons in the plane are both intersections of
half-spaces, so both are decided by one routine, the parametric clipping test of Cyrus and
Beck. Writing the segment as `p(t) = a + t (b - a)`, each half-space `n . x <= b` becomes
`t (n . d) <= b - n . a`, which bounds `t` from above, from below, or not at all when the
segment runs parallel to the boundary. The segment meets the region exactly when the
resulting interval intersects `[0, 1]`. A degenerate segment with `a == b` falls out of the
same code as a point containment test, which is the correct limiting behaviour and needs no
special case. Balls are handled separately, by minimising a convex quadratic in `t` over
`[0, 1]`, which is exact and dimension-independent.

Obstacles are closed sets throughout, so a segment that touches a boundary without entering
the interior counts as a collision. The alternative, treating them as open, admits paths
that graze an obstacle exactly, which is not a useful answer for a physical robot.

### Nearest neighbour queries

A k-d tree is static: inserting a point means rebuilding it. Rebuilding on every planner
iteration would cost `O(n log n)` per iteration and would dominate the run. The index in
`algorithm/base.py` therefore keeps a `cKDTree` over the first `m` points and a buffer of
later ones that is scanned linearly, rebuilding when the buffer reaches `sqrt(m)` entries.
Queries cost `O(log m + sqrt(m))` and the amortised rebuild cost per insertion is
`O(sqrt(m) log m)`. The rebuild schedule depends only on the number of insertions, never on
their values, so the index cannot make a run depend on anything but the seed, and ties are
resolved in favour of the lower index for the same reason.

### Determinism

Every planner takes a seed and constructs its own `numpy.random.Generator` from it. Nothing
is read from a global random state, no planner mutates itself during a run, and the
benchmark gives every planner the same seed sequence so that the comparison is paired.
Determinism is what makes the regression tier possible at all, and it is checked directly:
a test compares two runs of the same seed vertex by vertex, parent by parent, and cost by
cost, rather than comparing summary numbers that could agree by accident. The collision
check counts inherit the same property, which is what allows them to be pinned exactly
rather than within a tolerance.

### Differencing the runs, not only averaging them

The benchmark hands every planner the same seed sequence, and the analysis layer used to
throw the seed away: `summarise` groups by problem and planner and reports two means that
a reader compares by eye. `compare_paired` keeps the seed and subtracts the runs, which
answers what two means cannot, namely on how many of the ten seeds one planner was
actually cheaper. On the empty square the means are 12.96 and 13.17 with deviations of
0.07 and 0.20, and RRT star is nonetheless the cheaper of the two on all ten seeds.

Costs are differenced only over the seeds both planners solved, since the cost of a failed
run is not a number, and the seeds that this excludes are reported beside the difference
rather than dropped. Two of the ten narrow passage seeds are solved by RRT star and not by
PRM, so they cannot enter a difference of costs, and a mean that omitted them silently
would credit PRM for exactly the runs it lost. Collision checks are differenced over every
shared seed, because a failed run spends them too.

The spread of a difference is not narrower than the spreads it came from, and was not
expected to be. Two planners handed the same seed consume it differently, so their
departures from the seed mean are uncorrelated and the difference deviation comes out at
the quadrature sum: 0.21 on the empty square against `sqrt(0.07 ** 2 + 0.20 ** 2)`, which
is 0.212. The shared sequence makes the comparison fair; it does not make it a paired
experiment in the sense that would shrink the interval. What the differencing buys is the
per-seed win count and the explicit account of the seeds one planner failed.

## Rejected alternatives

### Point sampling along edges for collision checking

Cost: a subdivision count or a resolution parameter, and a scan of that many point tests
per edge. Benefit: it works for any obstacle that can answer a point containment query,
including implicit and non-convex shapes, and it is simple to write. Rejected because it is
unsound at any finite resolution, and because the whole benchmark rests on the claim that a
returned path is collision free. Exact tests cost no more here: a closed-form ball test and
a parametric half-space clip are each cheaper than a few dozen point tests.

### Rebuilding the k-d tree on every insertion

Cost: `O(n log n)` per planner iteration, which measurement put well above everything else
in the loop. Benefit: no buffer and no linear scan, so about twenty fewer lines. Rejected
on cost. The alternative in the other direction, an incremental structure such as a
balanced box-decomposition tree or a cover tree, was also rejected: it would have to be
written from scratch, since the fixed dependency set does not include one, and the
batched k-d tree already makes neighbour queries a minor share of the run.

### A goal region with a tolerance instead of an exact goal

Cost: none, it is the easier option. Benefit: a planner can stop as soon as it enters a
ball around the goal, which raises success rates and shortens runs. Rejected because it
makes the comparison depend on the tolerance: two planners returning paths that end at
different points inside the tolerance ball are not returning comparable costs. Here a
returned path ends at the goal configuration exactly, and `path_is_valid` checks it.

### Path shortcutting after planning

Cost: a post-processing pass, plus a decision about how much of the budget it may spend.
Benefit: a large and cheap reduction in RRT path cost, which is what most practical
systems do. Rejected because it would confuse the comparison being made. RRT with
shortcutting is a different algorithm from RRT, and reporting it under the name RRT would
hide the property being demonstrated, that the cost gap between RRT and RRT star comes from
rewiring rather than from smoothing. The gap in the results table is the honest one.

### RRT connect, bidirectional search

Cost: a second tree, a connection heuristic, and a swap rule. Benefit: it is markedly
faster than RRT on problems like the maze. Rejected on scope. The comparison here is
feasible against optimal against multi-query, and a fourth planner that is a faster
feasible planner would not add a dimension to it. It is the first thing worth adding if the
scope grows.

### Informed RRT star and other sampling improvements

Cost: an ellipsoidal sampler, and a rejection or direct-sampling routine for it. Benefit:
much faster convergence once a first solution exists. Rejected because the point of
including RRT star here is to show the published algorithm and its published radius
behaving as the theory says, which a modified sampler would obscure.

### A scene description file format

Cost: a schema, a parser, and validation. Benefit: problems could be defined without
touching Python. Rejected because the six standard problems are built by six functions in
`pipeline/suite.py`, which is less code than a parser, is type checked, and cannot go out
of step with the model it constructs.

### Bundling the visualisation with a module loader or a framework

Cost: a runtime dependency, and a build that has to be trusted by anyone opening the page.
Benefit: the usual conveniences. Rejected because the visualisation must run with no
network access and no package installation. It compiles to one classic script with
`module: none` and `outFile`, and the compiled bundle is committed, so the page needs
nothing but a static file server.

## Closed limitations

### Collision checks are counted and reported

This was recorded below as a limitation. The benchmark measured planner effort in wall
time, which is a property of the machine that ran it, and in node counts, which describe
the structure a planner built rather than the work it did to build it. Collision checks
are the machine-independent measure the literature reports, and they were not counted.

They are now. `CollisionChecker` in `model/obstacles.py` is the protocol through which a
planner asks its two questions, `ObstacleSet` implements it directly, and
`CountingChecker` wraps an obstacle set and records what was asked. Each planner
constructs one counter per run and reports `point_checks` and `segment_checks` on the
`PlanResult`; the benchmark carries both into `RunTrace`; the table gained a `Checks
mean` and a `Checks sd` column. The two kinds are counted apart because the planners do
not use them in the same proportion: a tree planner asks nothing but segment queries,
while a roadmap tests every drawn sample for freedom before it tests a single edge.

What it cost:

- One object per run and one attribute increment per query. The counter touches no
  geometry, so the arithmetic underneath is unchanged, and every cost and node count in
  `tests/data/reference_benchmark.json` stayed where it was when the counter was
  introduced. That is the evidence that the wrapper is transparent rather than an
  assertion that it is.
- Four private methods changed signature. `RRT._try_goal`, `RRTStar._choose_parent`,
  `RRTStar._rewire` and `RRTStar._connect_goal` now take the checker instead of the
  problem, which is honest about what they were using the problem for.
- Two public signatures widened. `PRM.build` and `PRM.query` accept any
  `CollisionChecker` where they previously named `ObstacleSet`. A widening breaks no
  caller, and a caller that wants no counting still passes the obstacle set itself.
- The obstacle set stayed frozen, which was the point of putting the counter outside it.
  A counter inside `ObstacleSet` was rejected: the benchmark shares one problem object
  across every run of every planner, so a count living in the obstacles would be a count
  of the whole benchmark, and resetting it between runs would make the obstacle set
  mutable in precisely the way the rest of this design avoids.

What it bought beyond a column in the table: the counts are fixed by the seed and by
nothing else, so the regression tier now pins them exactly, next to the node counts. A
change to parent selection or to the rewiring test that happened to leave the returned
path unchanged would have passed that tier before. It does not now.

What remains: wall time is still reported and is still machine-specific. It is kept
rather than replaced, because a collision check is not equally expensive on every
problem. RRT star asks 9687 checks on `polygon_field` and 8649 on `cube_3d`, within
twelve percent of each other, and spends 1.912 seconds against 0.848. A convex polygon
rebuilds its half-space representation on every query, while a box in three dimensions
has six fixed planes. The two columns answer different questions and neither one
replaces the other.

## Known limitations

- **Straight-line motion only.** An edge is a straight segment in configuration space, so
  the planners cover the holonomic case. A car, a quadrotor, or anything else whose motion
  is constrained would need the steering function replaced by a solver for the local
  boundary value problem, and `steer` in `algorithm/base.py` is the single place that would
  change.
- **Euclidean metric only.** Distance is the L2 norm on configurations. A revolute joint
  that wraps at `2 pi`, or a mixed translation and rotation space where the two components
  need different weights, is not represented. Adding one means a metric object threaded
  through the neighbour index, which currently relies on the k-d tree's Euclidean queries.
- **Convex obstacles only.** A non-convex obstacle has to be supplied as a union of convex
  pieces. Nothing detects that a supplied polygon was meant to be non-convex, beyond the
  constructor rejecting a vertex sequence that is not convex.
- **Polygons are planar.** `Circle` and `Box` work in any dimension, `ConvexPolygon` does
  not. The half-space clipping routine it uses is dimension-independent, so a general
  convex polytope would only need a constructor that takes half-spaces directly.
- **The free-space measure is over-estimated.** As described above, `gamma` is computed
  from the measure of the whole space. In an environment that is mostly occupied, this
  inflates the near-neighbour radius and wastes collision checks. Estimating the free
  measure from the rejection rate during sampling would fix it, at the cost of making the
  radius depend on the run.
- **Wall time is still machine-specific.** Collision checks are now reported beside it and
  are the portable measure, but the table keeps a wall time column that cannot be compared
  against a run on another machine. The recorded reference in
  `tests/data/reference_benchmark.json` stores wall time as zero, so that nothing in the
  file invites that comparison, while every other field in it is compared exactly.
- **A collision check is not a unit of time.** The count is portable and the cost of one
  count is not. Half-space representations are rebuilt on each query rather than cached in
  the obstacle, so a polygon-heavy problem pays more per check than a box-heavy one.
  Caching the half-spaces on the obstacle would narrow the gap and is the obvious next
  thing to measure.
- **The visualisation is planar.** `trace_document` rejects a problem that is not two
  dimensional. The three dimensional problem appears in the benchmark but not in the
  browser animation.
- **Success rates come from ten seeds.** A rate of 8 out of 10 has a wide confidence
  interval. The number of repeats is a command line argument, and a claim that needed
  tighter intervals would raise it.
