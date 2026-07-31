/**
 * Shape of the JSON trace written by rrt_planner.pipeline.trace.
 *
 * These declarations mirror the Python exporter exactly. They are the only contract
 * between the two halves of the repository, and nothing in the Python package reads
 * the trace back, so a change on either side has to be made on both.
 */
namespace Viz {
  export interface Bounds {
    readonly lower: readonly number[];
    readonly upper: readonly number[];
  }

  export interface CircleObstacle {
    readonly kind: "circle";
    readonly center: readonly number[];
    readonly radius: number;
  }

  export interface BoxObstacle {
    readonly kind: "box";
    readonly lower: readonly number[];
    readonly upper: readonly number[];
  }

  export interface PolygonObstacle {
    readonly kind: "polygon";
    readonly vertices: ReadonlyArray<readonly number[]>;
  }

  export type Obstacle = CircleObstacle | BoxObstacle | PolygonObstacle;

  /** A rewiring: at the moment the tree held `step` vertices, `node` was moved under `parent`. */
  export interface Rewire {
    readonly step: number;
    readonly node: number;
    readonly parent: number;
  }

  /**
   * The structure a planner built.
   *
   * A tree carries one parent per vertex, being the parent the vertex had when it was
   * inserted, plus the rewirings applied afterwards. A roadmap carries an edge list and
   * no parents. The two are distinguished by `kind`.
   */
  export interface Structure {
    readonly kind: "tree" | "roadmap";
    readonly vertices: ReadonlyArray<readonly number[]>;
    readonly parents: readonly number[];
    readonly edges: ReadonlyArray<readonly number[]>;
    readonly rewires: readonly Rewire[];
  }

  export interface Trace {
    readonly planner: string;
    readonly problem: string;
    readonly seed: number;
    readonly success: boolean;
    readonly cost: number | null;
    readonly nodeCount: number;
    readonly iterations: number;
    readonly bounds: Bounds;
    readonly start: readonly number[];
    readonly goal: readonly number[];
    readonly obstacles: readonly Obstacle[];
    readonly structure: Structure;
    readonly path: ReadonlyArray<readonly number[]>;
    readonly costHistory: ReadonlyArray<readonly number[]>;
  }
}
