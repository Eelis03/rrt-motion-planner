/**
 * Mapping between configuration space coordinates and canvas pixels.
 *
 * The mapping keeps the aspect ratio of the configuration space, so a disc obstacle is
 * drawn as a disc whatever the shape of the window, and it flips the vertical axis so
 * that the drawing matches the matplotlib figures produced by the Python package.
 */
namespace Viz {
  export class Viewport {
    private readonly originX: number;
    private readonly originY: number;
    private readonly pixelsPerUnit: number;
    private readonly lowerX: number;
    private readonly lowerY: number;
    private readonly spanY: number;
    private readonly width: number;
    private readonly height: number;

    constructor(bounds: Bounds, width: number, height: number, padding = 18) {
      const spanX = Math.max(bounds.upper[0] - bounds.lower[0], 1e-9);
      const spanY = Math.max(bounds.upper[1] - bounds.lower[1], 1e-9);
      const usableWidth = Math.max(width - 2 * padding, 1);
      const usableHeight = Math.max(height - 2 * padding, 1);

      this.width = width;
      this.height = height;
      this.lowerX = bounds.lower[0];
      this.lowerY = bounds.lower[1];
      this.spanY = spanY;
      this.pixelsPerUnit = Math.min(usableWidth / spanX, usableHeight / spanY);
      this.originX = padding + (usableWidth - spanX * this.pixelsPerUnit) / 2;
      this.originY = padding + (usableHeight - spanY * this.pixelsPerUnit) / 2;
    }

    /** Horizontal pixel coordinate of a configuration coordinate. */
    x(value: number): number {
      return this.originX + (value - this.lowerX) * this.pixelsPerUnit;
    }

    /** Vertical pixel coordinate of a configuration coordinate, with the axis flipped. */
    y(value: number): number {
      return this.originY + (this.spanY - (value - this.lowerY)) * this.pixelsPerUnit;
    }

    /** Length in pixels of a length in configuration units. */
    length(value: number): number {
      return value * this.pixelsPerUnit;
    }

    /** Width of the drawing surface in pixels. */
    get canvasWidth(): number {
      return this.width;
    }

    /** Height of the drawing surface in pixels. */
    get canvasHeight(): number {
      return this.height;
    }
  }
}
