/**
 * Drawing of one animation frame onto a 2D canvas context.
 *
 * A frame is identified by the number of vertices revealed so far. Nothing is cached
 * between frames: the whole scene is redrawn from the trace each time, which keeps the
 * renderer free of state that could drift out of step with the slider.
 */
namespace Viz {
  const COLOURS = {
    background: "#101418",
    obstacleFill: "#2b3138",
    obstacleEdge: "#59636e",
    edge: "#3d7ea6",
    recentEdge: "#7fd1ff",
    roadmapEdge: "#4a6f8a",
    path: "#f2994a",
    start: "#5cc98b",
    goal: "#e05c5c",
    frame: "#39424b",
  };

  const RECENT_VERTICES = 60;

  function drawObstacles(
    context: CanvasRenderingContext2D,
    view: Viewport,
    obstacles: readonly Obstacle[]
  ): void {
    context.fillStyle = COLOURS.obstacleFill;
    context.strokeStyle = COLOURS.obstacleEdge;
    context.lineWidth = 1;
    for (const obstacle of obstacles) {
      context.beginPath();
      if (obstacle.kind === "circle") {
        context.arc(
          view.x(obstacle.center[0]),
          view.y(obstacle.center[1]),
          view.length(obstacle.radius),
          0,
          Math.PI * 2
        );
      } else if (obstacle.kind === "box") {
        const left = view.x(obstacle.lower[0]);
        const top = view.y(obstacle.upper[1]);
        context.rect(
          left,
          top,
          view.length(obstacle.upper[0] - obstacle.lower[0]),
          view.length(obstacle.upper[1] - obstacle.lower[1])
        );
      } else {
        obstacle.vertices.forEach((vertex, index) => {
          const px = view.x(vertex[0]);
          const py = view.y(vertex[1]);
          if (index === 0) {
            context.moveTo(px, py);
          } else {
            context.lineTo(px, py);
          }
        });
        context.closePath();
      }
      context.fill();
      context.stroke();
    }
  }

  function drawTree(
    context: CanvasRenderingContext2D,
    view: Viewport,
    structure: Structure,
    revealed: number
  ): void {
    const parents = parentsAfter(structure, revealed);
    const recentFrom = Math.max(1, revealed - RECENT_VERTICES);

    context.lineWidth = 1;
    context.strokeStyle = COLOURS.edge;
    context.beginPath();
    for (let index = 1; index < recentFrom; index += 1) {
      const parent = parents[index];
      if (parent < 0) {
        continue;
      }
      context.moveTo(view.x(structure.vertices[parent][0]), view.y(structure.vertices[parent][1]));
      context.lineTo(view.x(structure.vertices[index][0]), view.y(structure.vertices[index][1]));
    }
    context.stroke();

    context.strokeStyle = COLOURS.recentEdge;
    context.beginPath();
    for (let index = recentFrom; index < revealed; index += 1) {
      const parent = parents[index];
      if (parent < 0) {
        continue;
      }
      context.moveTo(view.x(structure.vertices[parent][0]), view.y(structure.vertices[parent][1]));
      context.lineTo(view.x(structure.vertices[index][0]), view.y(structure.vertices[index][1]));
    }
    context.stroke();
  }

  function drawRoadmap(
    context: CanvasRenderingContext2D,
    view: Viewport,
    structure: Structure,
    revealed: number
  ): void {
    context.lineWidth = 0.7;
    context.strokeStyle = COLOURS.roadmapEdge;
    context.beginPath();
    for (const edge of structure.edges) {
      if (edge[0] >= revealed || edge[1] >= revealed) {
        continue;
      }
      context.moveTo(view.x(structure.vertices[edge[0]][0]), view.y(structure.vertices[edge[0]][1]));
      context.lineTo(view.x(structure.vertices[edge[1]][0]), view.y(structure.vertices[edge[1]][1]));
    }
    context.stroke();

    context.fillStyle = COLOURS.recentEdge;
    for (let index = 0; index < revealed; index += 1) {
      context.beginPath();
      context.arc(
        view.x(structure.vertices[index][0]),
        view.y(structure.vertices[index][1]),
        1.6,
        0,
        Math.PI * 2
      );
      context.fill();
    }
  }

  function drawPath(
    context: CanvasRenderingContext2D,
    view: Viewport,
    path: ReadonlyArray<readonly number[]>
  ): void {
    if (path.length < 2) {
      return;
    }
    context.strokeStyle = COLOURS.path;
    context.lineWidth = 2.5;
    context.lineJoin = "round";
    context.beginPath();
    context.moveTo(view.x(path[0][0]), view.y(path[0][1]));
    for (let index = 1; index < path.length; index += 1) {
      context.lineTo(view.x(path[index][0]), view.y(path[index][1]));
    }
    context.stroke();
  }

  function drawEndpoints(context: CanvasRenderingContext2D, view: Viewport, trace: Trace): void {
    context.fillStyle = COLOURS.start;
    context.beginPath();
    context.arc(view.x(trace.start[0]), view.y(trace.start[1]), 5, 0, Math.PI * 2);
    context.fill();

    context.fillStyle = COLOURS.goal;
    context.beginPath();
    context.arc(view.x(trace.goal[0]), view.y(trace.goal[1]), 5, 0, Math.PI * 2);
    context.fill();
  }

  function drawFrame(context: CanvasRenderingContext2D, view: Viewport, bounds: Bounds): void {
    context.strokeStyle = COLOURS.frame;
    context.lineWidth = 1;
    context.strokeRect(
      view.x(bounds.lower[0]),
      view.y(bounds.upper[1]),
      view.length(bounds.upper[0] - bounds.lower[0]),
      view.length(bounds.upper[1] - bounds.lower[1])
    );
  }

  /** Redraw the whole scene with the first `revealed` vertices of the structure present. */
  export function drawScene(
    context: CanvasRenderingContext2D,
    view: Viewport,
    trace: Trace,
    revealed: number
  ): void {
    const total = trace.structure.vertices.length;
    const shown = clampCount(revealed, total);

    context.fillStyle = COLOURS.background;
    context.fillRect(0, 0, view.canvasWidth, view.canvasHeight);
    drawFrame(context, view, trace.bounds);
    drawObstacles(context, view, trace.obstacles);

    if (trace.structure.kind === "tree") {
      drawTree(context, view, trace.structure, shown);
    } else {
      drawRoadmap(context, view, trace.structure, shown);
    }

    if (trace.success && shown >= total) {
      drawPath(context, view, trace.path);
    }
    drawEndpoints(context, view, trace);
  }
}
