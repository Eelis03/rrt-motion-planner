"""Rendering of aggregated metrics as Markdown, ready to paste into the README."""

from __future__ import annotations

import math
from collections.abc import Sequence

from rrt_planner.analysis.metrics import Summary

__all__ = ["format_summary_table"]

_HEADERS = (
    "Problem",
    "Planner",
    "Success",
    "Cost mean",
    "Cost sd",
    "Nodes mean",
    "Nodes sd",
    "Time mean (s)",
    "Time sd (s)",
)


def _number(value: float, digits: int) -> str:
    if math.isnan(value):
        return "n/a"
    return f"{value:.{digits}f}"


def format_summary_table(summaries: Sequence[Summary]) -> str:
    """Render summaries as a Markdown table with one row per planner and problem."""
    rows = [
        (
            summary.problem,
            summary.planner,
            f"{summary.successes}/{summary.runs}",
            _number(summary.cost_mean, 2),
            _number(summary.cost_std, 2),
            _number(summary.node_count_mean, 1),
            _number(summary.node_count_std, 1),
            _number(summary.wall_time_mean, 3),
            _number(summary.wall_time_std, 3),
        )
        for summary in summaries
    ]
    widths = [
        max(len(header), *(len(row[column]) for row in rows)) if rows else len(header)
        for column, header in enumerate(_HEADERS)
    ]
    lines = [
        "| " + " | ".join(h.ljust(w) for h, w in zip(_HEADERS, widths, strict=True)) + " |",
        "| " + " | ".join("-" * w for w in widths) + " |",
    ]
    lines.extend(
        "| " + " | ".join(cell.ljust(width) for cell, width in zip(row, widths, strict=True)) + " |"
        for row in rows
    )
    return "\n".join(lines)
