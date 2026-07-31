"use strict";
/**
 * Parsing and validation of a trace document.
 *
 * The parser is deliberately strict. A trace that is missing a field, or that carries a
 * vertex index pointing outside the vertex list, is rejected with a message naming the
 * problem rather than producing a drawing that is quietly wrong.
 */
var Viz;
(function (Viz) {
    Viz.TRACE_FORMAT = "rrt-planner-trace";
    Viz.TRACE_VERSION = 1;
    function fail(message) {
        throw new Error(`malformed trace: ${message}`);
    }
    function asRecord(value, field) {
        if (typeof value !== "object" || value === null || Array.isArray(value)) {
            fail(`${field} must be an object`);
        }
        return value;
    }
    function asNumber(value, field) {
        if (typeof value !== "number" || !Number.isFinite(value)) {
            fail(`${field} must be a finite number`);
        }
        return value;
    }
    function asString(value, field) {
        if (typeof value !== "string") {
            fail(`${field} must be a string`);
        }
        return value;
    }
    function asBoolean(value, field) {
        if (typeof value !== "boolean") {
            fail(`${field} must be a boolean`);
        }
        return value;
    }
    function asArray(value, field) {
        if (!Array.isArray(value)) {
            fail(`${field} must be an array`);
        }
        return value;
    }
    function asPoint(value, field) {
        const entries = asArray(value, field);
        if (entries.length !== 2) {
            fail(`${field} must hold two coordinates, found ${entries.length}`);
        }
        return [asNumber(entries[0], `${field}[0]`), asNumber(entries[1], `${field}[1]`)];
    }
    function asPoints(value, field) {
        return asArray(value, field).map((entry, index) => asPoint(entry, `${field}[${index}]`));
    }
    function parseObstacle(value, field) {
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
    function parseStructure(value, field) {
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
    function parseTrace(value) {
        const record = asRecord(value, "trace");
        const format = asString(record["format"], "trace.format");
        if (format !== Viz.TRACE_FORMAT) {
            fail(`unknown format ${format}, expected ${Viz.TRACE_FORMAT}`);
        }
        const version = asNumber(record["version"], "trace.version");
        if (version !== Viz.TRACE_VERSION) {
            fail(`unsupported version ${version}, expected ${Viz.TRACE_VERSION}`);
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
            obstacles: asArray(record["obstacles"], "trace.obstacles").map((entry, index) => parseObstacle(entry, `trace.obstacles[${index}]`)),
            structure: parseStructure(record["structure"], "trace.structure"),
            path: asPoints(record["path"], "trace.path"),
            costHistory: asArray(record["costHistory"], "trace.costHistory").map((entry, index) => asPoint(entry, `trace.costHistory[${index}]`)),
        };
    }
    Viz.parseTrace = parseTrace;
    /**
     * The parent of every vertex once the first `revealed` vertices exist.
     *
     * Replaying the rewirings recorded up to that point is what makes the animation show
     * RRT star reorganising its tree rather than only extending it. The cost is linear in
     * the number of vertices and rewirings, which is the same order as drawing the frame.
     */
    function parentsAfter(structure, revealed) {
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
    Viz.parentsAfter = parentsAfter;
    /**
     * Reduce a requested reveal position to a whole number of vertices within range.
     *
     * The animation advances by a fractional number of vertices per frame, because it is
     * driven by elapsed time rather than by frame count, and the slider hands back a
     * string. Everything downstream indexes arrays with the result, so it is made a whole
     * number in one place instead of at each use.
     */
    function clampCount(requested, total) {
        if (!Number.isFinite(requested)) {
            return total;
        }
        return Math.max(0, Math.min(Math.floor(requested), total));
    }
    Viz.clampCount = clampCount;
})(Viz || (Viz = {}));
/**
 * Mapping between configuration space coordinates and canvas pixels.
 *
 * The mapping keeps the aspect ratio of the configuration space, so a disc obstacle is
 * drawn as a disc whatever the shape of the window, and it flips the vertical axis so
 * that the drawing matches the matplotlib figures produced by the Python package.
 */
var Viz;
(function (Viz) {
    class Viewport {
        constructor(bounds, width, height, padding = 18) {
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
        x(value) {
            return this.originX + (value - this.lowerX) * this.pixelsPerUnit;
        }
        /** Vertical pixel coordinate of a configuration coordinate, with the axis flipped. */
        y(value) {
            return this.originY + (this.spanY - (value - this.lowerY)) * this.pixelsPerUnit;
        }
        /** Length in pixels of a length in configuration units. */
        length(value) {
            return value * this.pixelsPerUnit;
        }
        /** Width of the drawing surface in pixels. */
        get canvasWidth() {
            return this.width;
        }
        /** Height of the drawing surface in pixels. */
        get canvasHeight() {
            return this.height;
        }
    }
    Viz.Viewport = Viewport;
})(Viz || (Viz = {}));
/**
 * Drawing of one animation frame onto a 2D canvas context.
 *
 * A frame is identified by the number of vertices revealed so far. Nothing is cached
 * between frames: the whole scene is redrawn from the trace each time, which keeps the
 * renderer free of state that could drift out of step with the slider.
 */
var Viz;
(function (Viz) {
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
    function drawObstacles(context, view, obstacles) {
        context.fillStyle = COLOURS.obstacleFill;
        context.strokeStyle = COLOURS.obstacleEdge;
        context.lineWidth = 1;
        for (const obstacle of obstacles) {
            context.beginPath();
            if (obstacle.kind === "circle") {
                context.arc(view.x(obstacle.center[0]), view.y(obstacle.center[1]), view.length(obstacle.radius), 0, Math.PI * 2);
            }
            else if (obstacle.kind === "box") {
                const left = view.x(obstacle.lower[0]);
                const top = view.y(obstacle.upper[1]);
                context.rect(left, top, view.length(obstacle.upper[0] - obstacle.lower[0]), view.length(obstacle.upper[1] - obstacle.lower[1]));
            }
            else {
                obstacle.vertices.forEach((vertex, index) => {
                    const px = view.x(vertex[0]);
                    const py = view.y(vertex[1]);
                    if (index === 0) {
                        context.moveTo(px, py);
                    }
                    else {
                        context.lineTo(px, py);
                    }
                });
                context.closePath();
            }
            context.fill();
            context.stroke();
        }
    }
    function drawTree(context, view, structure, revealed) {
        const parents = Viz.parentsAfter(structure, revealed);
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
    function drawRoadmap(context, view, structure, revealed) {
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
            context.arc(view.x(structure.vertices[index][0]), view.y(structure.vertices[index][1]), 1.6, 0, Math.PI * 2);
            context.fill();
        }
    }
    function drawPath(context, view, path) {
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
    function drawEndpoints(context, view, trace) {
        context.fillStyle = COLOURS.start;
        context.beginPath();
        context.arc(view.x(trace.start[0]), view.y(trace.start[1]), 5, 0, Math.PI * 2);
        context.fill();
        context.fillStyle = COLOURS.goal;
        context.beginPath();
        context.arc(view.x(trace.goal[0]), view.y(trace.goal[1]), 5, 0, Math.PI * 2);
        context.fill();
    }
    function drawFrame(context, view, bounds) {
        context.strokeStyle = COLOURS.frame;
        context.lineWidth = 1;
        context.strokeRect(view.x(bounds.lower[0]), view.y(bounds.upper[1]), view.length(bounds.upper[0] - bounds.lower[0]), view.length(bounds.upper[1] - bounds.lower[1]));
    }
    /** Redraw the whole scene with the first `revealed` vertices of the structure present. */
    function drawScene(context, view, trace, revealed) {
        const total = trace.structure.vertices.length;
        const shown = Viz.clampCount(revealed, total);
        context.fillStyle = COLOURS.background;
        context.fillRect(0, 0, view.canvasWidth, view.canvasHeight);
        drawFrame(context, view, trace.bounds);
        drawObstacles(context, view, trace.obstacles);
        if (trace.structure.kind === "tree") {
            drawTree(context, view, trace.structure, shown);
        }
        else {
            drawRoadmap(context, view, trace.structure, shown);
        }
        if (trace.success && shown >= total) {
            drawPath(context, view, trace.path);
        }
        drawEndpoints(context, view, trace);
    }
    Viz.drawScene = drawScene;
})(Viz || (Viz = {}));
/**
 * Page wiring: load a trace, run the animation loop, keep the controls in step.
 *
 * There is no framework and no dependency. The page fetches the trace index when it is
 * served over HTTP, and accepts a file chosen from disk otherwise, so it also works
 * when index.html is opened directly.
 */
var Viz;
(function (Viz) {
    let elements = null;
    let trace = null;
    let revealed = 0;
    let playing = false;
    let lastTimestamp = 0;
    function need(id) {
        const found = document.getElementById(id);
        if (found === null) {
            throw new Error(`the page is missing the element #${id}`);
        }
        return found;
    }
    function collectElements() {
        return {
            canvas: need("scene"),
            select: need("trace-select"),
            file: need("trace-file"),
            play: need("play"),
            restart: need("restart"),
            slider: need("progress"),
            speed: need("speed"),
            status: need("status"),
            readout: need("readout"),
            message: need("message"),
        };
    }
    function report(text) {
        if (elements !== null) {
            elements.message.textContent = text;
        }
    }
    function resizeCanvas(canvas) {
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
    function render() {
        if (elements === null || trace === null) {
            return;
        }
        const context = elements.canvas.getContext("2d");
        if (context === null) {
            report("this browser does not provide a 2D canvas context");
            return;
        }
        const size = resizeCanvas(elements.canvas);
        const view = new Viz.Viewport(trace.bounds, size.width, size.height);
        Viz.drawScene(context, view, trace, revealed);
        updateReadout();
    }
    function updateReadout() {
        if (elements === null || trace === null) {
            return;
        }
        const total = trace.structure.vertices.length;
        const shown = Viz.clampCount(revealed, total);
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
    function step(timestamp) {
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
    function setPlaying(value) {
        playing = value;
        if (elements !== null) {
            elements.play.textContent = value ? "Pause" : "Play";
        }
    }
    function showTrace(loaded) {
        trace = loaded;
        revealed = 0;
        lastTimestamp = 0;
        setPlaying(true);
        report("");
        render();
    }
    async function loadFromUrl(url) {
        const response = await fetch(url, { cache: "no-cache" });
        if (!response.ok) {
            throw new Error(`${url} returned ${response.status}`);
        }
        showTrace(Viz.parseTrace(await response.json()));
    }
    async function loadIndex() {
        if (elements === null) {
            return;
        }
        const names = await (await fetch("traces/index.json", { cache: "no-cache" })).json();
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
    function attachHandlers(ready) {
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
            loadFromUrl(ready.select.value).catch((error) => report(String(error)));
        });
        ready.file.addEventListener("change", () => {
            const chosen = ready.file.files?.[0];
            if (chosen === undefined) {
                return;
            }
            chosen
                .text()
                .then((text) => showTrace(Viz.parseTrace(JSON.parse(text))))
                .catch((error) => report(String(error)));
        });
        window.addEventListener("resize", () => render());
    }
    /** Build the page and start the animation loop. */
    function start() {
        elements = collectElements();
        attachHandlers(elements);
        window.requestAnimationFrame(step);
        loadIndex().catch(() => {
            report("No trace index could be fetched. Serve this directory over HTTP with " +
                "python -m http.server, or choose a trace file from disk with the button above. " +
                "Traces are produced by examples/export_viz_trace.py.");
        });
    }
    Viz.start = start;
})(Viz || (Viz = {}));
document.addEventListener("DOMContentLoaded", () => Viz.start());
