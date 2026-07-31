/**
 * Page wiring: load a trace, run the animation loop, keep the controls in step.
 *
 * There is no framework and no dependency. The page fetches the trace index when it is
 * served over HTTP, and accepts a file chosen from disk otherwise, so it also works
 * when index.html is opened directly.
 */
namespace Viz {
  interface Elements {
    readonly canvas: HTMLCanvasElement;
    readonly select: HTMLSelectElement;
    readonly file: HTMLInputElement;
    readonly play: HTMLButtonElement;
    readonly restart: HTMLButtonElement;
    readonly slider: HTMLInputElement;
    readonly speed: HTMLInputElement;
    readonly status: HTMLElement;
    readonly readout: HTMLElement;
    readonly message: HTMLElement;
  }

  let elements: Elements | null = null;
  let trace: Trace | null = null;
  let revealed = 0;
  let playing = false;
  let lastTimestamp = 0;

  function need<T extends HTMLElement>(id: string): T {
    const found = document.getElementById(id);
    if (found === null) {
      throw new Error(`the page is missing the element #${id}`);
    }
    return found as T;
  }

  function collectElements(): Elements {
    return {
      canvas: need<HTMLCanvasElement>("scene"),
      select: need<HTMLSelectElement>("trace-select"),
      file: need<HTMLInputElement>("trace-file"),
      play: need<HTMLButtonElement>("play"),
      restart: need<HTMLButtonElement>("restart"),
      slider: need<HTMLInputElement>("progress"),
      speed: need<HTMLInputElement>("speed"),
      status: need<HTMLElement>("status"),
      readout: need<HTMLElement>("readout"),
      message: need<HTMLElement>("message"),
    };
  }

  function report(text: string): void {
    if (elements !== null) {
      elements.message.textContent = text;
    }
  }

  function resizeCanvas(canvas: HTMLCanvasElement): { width: number; height: number } {
    const ratio = window.devicePixelRatio || 1;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    const context = canvas.getContext("2d");
    if (context !== null) {
      context.setTransform(ratio, 0, 0, ratio, 0, 0);
    }
    return { width: width, height: height };
  }

  function render(): void {
    if (elements === null || trace === null) {
      return;
    }
    const context = elements.canvas.getContext("2d");
    if (context === null) {
      report("this browser does not provide a 2D canvas context");
      return;
    }
    const size = resizeCanvas(elements.canvas);
    const view = new Viewport(trace.bounds, size.width, size.height);
    drawScene(context, view, trace, revealed);
    updateReadout();
  }

  function updateReadout(): void {
    if (elements === null || trace === null) {
      return;
    }
    const total = trace.structure.vertices.length;
    const shown = clampCount(revealed, total);
    const noun = trace.structure.kind === "tree" ? "tree vertices" : "milestones";
    const outcome = trace.success && trace.cost !== null ? trace.cost.toFixed(3) : "no path";
    elements.status.textContent =
      `${trace.planner} on ${trace.problem}, seed ${trace.seed}, ` +
      `${trace.iterations} iterations, cost ${outcome}`;
    elements.readout.textContent = `${shown} of ${total} ${noun}`;
    elements.slider.max = String(total);
    elements.slider.value = String(shown);
  }

  /**
   * One animation frame.
   *
   * The next frame is requested first, so that the loop keeps running while no trace has
   * been loaded yet and picks the animation up as soon as one arrives. Progress is
   * measured in vertices per second of wall clock time rather than vertices per frame,
   * so the replay runs at the same rate on any refresh rate.
   */
  function step(timestamp: number): void {
    window.requestAnimationFrame(step);
    if (elements === null || trace === null || !playing) {
      lastTimestamp = timestamp;
      return;
    }
    const total = trace.structure.vertices.length;
    const elapsed = lastTimestamp === 0 ? 16 : Math.min(timestamp - lastTimestamp, 100);
    lastTimestamp = timestamp;
    const perSecond = Number(elements.speed.value);
    revealed = Math.min(total, revealed + Math.max(1, (perSecond * elapsed) / 1000));
    if (revealed >= total) {
      revealed = total;
      setPlaying(false);
    }
    render();
  }

  function setPlaying(value: boolean): void {
    playing = value;
    if (elements !== null) {
      elements.play.textContent = value ? "Pause" : "Play";
    }
  }

  function showTrace(loaded: Trace): void {
    trace = loaded;
    revealed = 0;
    lastTimestamp = 0;
    setPlaying(true);
    report("");
    render();
  }

  async function loadFromUrl(url: string): Promise<void> {
    const response = await fetch(url, { cache: "no-cache" });
    if (!response.ok) {
      throw new Error(`${url} returned ${response.status}`);
    }
    showTrace(parseTrace(await response.json()));
  }

  async function loadIndex(): Promise<void> {
    if (elements === null) {
      return;
    }
    const names: unknown = await (await fetch("traces/index.json", { cache: "no-cache" })).json();
    if (!Array.isArray(names)) {
      throw new Error("traces/index.json does not contain a list of file names");
    }
    elements.select.innerHTML = "";
    for (const name of names) {
      const option = document.createElement("option");
      option.value = `traces/${String(name)}`;
      option.textContent = String(name).replace(/\.json$/, "");
      elements.select.appendChild(option);
    }
    if (names.length > 0) {
      await loadFromUrl(elements.select.value);
    }
  }

  function attachHandlers(ready: Elements): void {
    ready.play.addEventListener("click", () => {
      if (trace !== null && revealed >= trace.structure.vertices.length) {
        revealed = 0;
      }
      setPlaying(!playing);
    });
    ready.restart.addEventListener("click", () => {
      revealed = 0;
      setPlaying(true);
      render();
    });
    ready.slider.addEventListener("input", () => {
      revealed = Number(ready.slider.value);
      setPlaying(false);
      render();
    });
    ready.select.addEventListener("change", () => {
      loadFromUrl(ready.select.value).catch((error: unknown) => report(String(error)));
    });
    ready.file.addEventListener("change", () => {
      const chosen = ready.file.files?.[0];
      if (chosen === undefined) {
        return;
      }
      chosen
        .text()
        .then((text) => showTrace(parseTrace(JSON.parse(text))))
        .catch((error: unknown) => report(String(error)));
    });
    window.addEventListener("resize", () => render());
  }

  /** Build the page and start the animation loop. */
  export function start(): void {
    elements = collectElements();
    attachHandlers(elements);
    window.requestAnimationFrame(step);
    loadIndex().catch(() => {
      report(
        "No trace index could be fetched. Serve this directory over HTTP with " +
          "python -m http.server, or choose a trace file from disk with the button above. " +
          "Traces are produced by examples/export_viz_trace.py."
      );
    });
  }
}

document.addEventListener("DOMContentLoaded", () => Viz.start());
