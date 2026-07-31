# Visualisation layer

A browser animation of planner tree growth, written in TypeScript and drawn on a 2D
canvas. It is additive: the Python package and its test suite neither import it nor
depend on it, and `src/` and `tests/` contain no reference to this directory.

## Running it

The compiled bundle `dist/viz.js` is committed, so no build step is needed to view the
page. Traces are fetched over HTTP, so serve the directory rather than opening the file
from disk:

```bash
uv run python examples/export_viz_trace.py     # writes viz/traces/*.json
cd viz
python -m http.server 8000
```

Then open `http://localhost:8000/`. Opening `index.html` directly from the file system
also works, with one difference: the browser blocks the trace index fetch, so the page
asks for a trace file to be chosen from disk instead.

## Building it

TypeScript is a build-time dependency only. The page loads one plain script, fetches
nothing from a content delivery network, and uses no runtime library.

```bash
cd viz
npm ci           # installs typescript, the only dependency, and only for the build
npm run build    # tsc -p tsconfig.json, writes dist/viz.js
npm run check    # type check without emitting
```

Continuous integration runs the type check and then rebuilds the bundle and compares it
with the committed one, so the file in `dist/` cannot drift away from the sources in
`src/`.

The compiler is configured with `module: none` and `outFile`, so the five source files
are concatenated into one classic script under a single `Viz` namespace. That keeps the
page working without module resolution, and keeps the bundle free of a loader.

## Layout

| File | Responsibility |
| --- | --- |
| `src/types.ts` | The shape of the trace document, mirroring the Python exporter. |
| `src/trace.ts` | Strict parsing and validation, and replay of recorded rewirings. |
| `src/view.ts` | Configuration space to canvas mapping, preserving the aspect ratio. |
| `src/render.ts` | Drawing of one frame: obstacles, structure, path, endpoints. |
| `src/app.ts` | Page wiring, the animation loop, and the controls. |
| `index.html` | Markup, styling, and the controls the wiring expects. |
| `traces/` | Trace documents written by `examples/export_viz_trace.py`. |

## What the animation shows

A frame is identified by the number of vertices revealed. For a tree, each vertex is
drawn joined to the parent it had at that point in the run, and the rewirings recorded
after that point are not yet applied. RRT star therefore shows branches detaching and
reattaching to cheaper parents as the replay proceeds, which is the behaviour that
separates it from RRT. For a roadmap, the milestones and then the edges appear, since
PRM builds the whole roadmap before any query is answered. The solution path is drawn
once the replay reaches the end of the run.

## Trace format

One JSON object per run, version 1, written by `rrt_planner.pipeline.trace`. Costs are
`null` rather than infinite when a run failed, so that the document contains no value
that JSON cannot carry. The parser rejects a document that is missing a field or that
refers to a vertex outside the vertex list, naming the field at fault.
