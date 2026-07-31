/**
 * Parsing and validation of a trace document.
 *
 * The parser is deliberately strict. A trace that is missing a field, or that carries a
 * vertex index pointing outside the vertex list, is rejected with a message naming the
 * problem rather than producing a drawing that is quietly wrong.
 */
namespace Viz {
  export const TRACE_FORMAT = "rrt-planner-trace";
  export const TRACE_VERSION = 1;

  function fail(message: string): never {
    throw new Error(`malformed trace: ${message}`);
  }

  function asRecord(value: unknown, field: string): Record<string, unknown> {
    if (typeof value !== "object" || value === null || Array.isArray(value)) {
      fail(`${field} must be an object`);
    }
    return value as Record<string, unknown>;
  }

  function asNumber(value: unknown, field: string): number {
    if (typeof value !== "number" || !Number.isFinite(value)) {
      fail(`${field} must be a finite number`);
    }
    return value;
  }

  function asString(value: unknown, field: string): string {
    if (typeof value !== "string") {
      fail(`${field} must be a string`);
    }
    return value;
  }

  function asBoolean(value: unknown, field: string): boolean {
    if (typeof value !== "boolean") {
      fail(`${field} must be a boolean`);
    }
    return value;
  }

  function asArray(value: unknown, field: string): unknown[] {
    if (!Array.isArray(value)) {
      fail(`${field} must be an array`);
    }
    return value;
  }

  function asPoint(value: unknown, field: string): number[] {
    const entries = asArray(value, field);
    if (entries.length !== 2) {
      fail(`${field} must hold two coordinates, found ${entries.length}`);
    }
    return [asNumber(entries[0], `${field}[0]`), asNumber(entries[1], `${field}[1]`)];
  }

  function asPoints(value: unknown, field: string): number[][] {
    return asArray(value, field).map((entry, index) => asPoint(entry, `${field}[${index}]`));
  }

  function parseObstacle(value: unknown, field: string): Obstacle {
    const record = asRecord(value, field);
    const kind = asString(record["kind"], `${field}.kind`);
    if (kind === "circle") {
      return {
        kind: "circle",
        center: asPoint(record["center"], `${field}.center`),
        radius: asNumber(record["radius"], `${field}.radius`),
      };
    }
    if (kind === "box") {
      return {
        kind: "box",
        lower: asPoint(record["lower"], `${field}.lower`),
        upper: asPoint(record["upper"], `${field}.upper`),
      };
    }
    if (kind === "polygon") {
      const vertices = asPoints(record["vertices"], `${field}.vertices`);
      if (vertices.length < 3) {
        fail(`${field}.vertices needs at least three entries`);
      }
      return { kind: "polygon", vertices: vertices };
    }
    return fail(`${field}.kind is not a known obstacle: ${kind}`);
  }

  function parseStructure(value: unknown, field: string): Structure {
    const record = asRecord(value, field);
    const kind = asString(record["kind"], `${field}.kind`);
    if (kind !== "tree" && kind !== "roadmap") {
      fail(`${field}.kind must be tree or roadmap, found ${kind}`);
    }
    const vertices = asPoints(record["vertices"], `${field}.vertices`);
    const count = vertices.length;

    const parents = asArray(record["parents"], `${field}.parents`).map((entry, index) => {
      const parent = asNumber(entry, `${field}.parents[${index}]`);
      if (parent < -1 || parent >= count) {
        fail(`${field}.parents[${index}] is out of range: ${parent}`);
      }
      return parent;
    });
    if (kind === "tree" && parents.length !== count) {
      fail(`${field}.parents must hold one entry per vertex`);
    }

    const edges = asArray(record["edges"], `${field}.edges`).map((entry, index) => {
      const pair = asPoint(entry, `${field}.edges[${index}]`);
      if (pair[0] < 0 || pair[0] >= count || pair[1] < 0 || pair[1] >= count) {
        fail(`${field}.edges[${index}] refers to a vertex that does not exist`);
      }
      return pair;
    });

    const rewires = asArray(record["rewires"], `${field}.rewires`).map((entry, index) => {
      const triple = asArray(entry, `${field}.rewires[${index}]`);
      if (triple.length !== 3) {
        fail(`${field}.rewires[${index}] must hold three entries`);
      }
      return {
        step: asNumber(triple[0], `${field}.rewires[${index}][0]`),
        node: asNumber(triple[1], `${field}.rewires[${index}][1]`),
        parent: asNumber(triple[2], `${field}.rewires[${index}][2]`),
      };
    });

    return { kind: kind, vertices: vertices, parents: parents, edges: edges, rewires: rewires };
  }

  /** Turn a parsed JSON value into a trace, or throw explaining why it is not one. */
  export function parseTrace(value: unknown): Trace {
    const record = asRecord(value, "trace");
    const format = asString(record["format"], "trace.format");
    if (format !== TRACE_FORMAT) {
      fail(`unknown format ${format}, expected ${TRACE_FORMAT}`);
    }
    const version = asNumber(record["version"], "trace.version");
    if (version !== TRACE_VERSION) {
      fail(`unsupported version ${version}, expected ${TRACE_VERSION}`);
    }
    const bounds = asRecord(record["bounds"], "trace.bounds");
    const cost = record["cost"];
    return {
      planner: asString(record["planner"], "trace.planner"),
      problem: asString(record["problem"], "trace.problem"),
      seed: asNumber(record["seed"], "trace.seed"),
      success: asBoolean(record["success"], "trace.success"),
      cost: cost === null ? null : asNumber(cost, "trace.cost"),
      nodeCount: asNumber(record["nodeCount"], "trace.nodeCount"),
      iterations: asNumber(record["iterations"], "trace.iterations"),
      bounds: {
        lower: asPoint(bounds["lower"], "trace.bounds.lower"),
        upper: asPoint(bounds["upper"], "trace.bounds.upper"),
      },
      start: asPoint(record["start"], "trace.start"),
      goal: asPoint(record["goal"], "trace.goal"),
      obstacles: asArray(record["obstacles"], "trace.obstacles").map((entry, index) =>
        parseObstacle(entry, `trace.obstacles[${index}]`)
      ),
      structure: parseStructure(record["structure"], "trace.structure"),
      path: asPoints(record["path"], "trace.path"),
      costHistory: asArray(record["costHistory"], "trace.costHistory").map((entry, index) =>
        asPoint(entry, `trace.costHistory[${index}]`)
      ),
    };
  }

  /**
   * The parent of every vertex once the first `revealed` vertices exist.
   *
   * Replaying the rewirings recorded up to that point is what makes the animation show
   * RRT star reorganising its tree rather than only extending it. The cost is linear in
   * the number of vertices and rewirings, which is the same order as drawing the frame.
   */
  export function parentsAfter(structure: Structure, revealed: number): Int32Array {
    const count = clampCount(revealed, structure.vertices.length);
    const parents = new Int32Array(count);
    for (let index = 0; index < count; index += 1) {
      parents[index] = structure.parents[index] ?? -1;
    }
    for (const rewire of structure.rewires) {
      if (rewire.step > count) {
        break;
      }
      if (rewire.node < count && rewire.parent < count) {
        parents[rewire.node] = rewire.parent;
      }
    }
    return parents;
  }

  /**
   * Reduce a requested reveal position to a whole number of vertices within range.
   *
   * The animation advances by a fractional number of vertices per frame, because it is
   * driven by elapsed time rather than by frame count, and the slider hands back a
   * string. Everything downstream indexes arrays with the result, so it is made a whole
   * number in one place instead of at each use.
   */
  export function clampCount(requested: number, total: number): number {
    if (!Number.isFinite(requested)) {
      return total;
    }
    return Math.max(0, Math.min(Math.floor(requested), total));
  }
}
