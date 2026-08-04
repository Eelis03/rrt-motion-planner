"""Analysis layer: run traces in, comparison metrics and figures out.

``figures`` is imported lazily by the caller rather than re-exported here, so that
importing :mod:`rrt_planner.analysis` does not pull in matplotlib.
"""

from rrt_planner.analysis.metrics import (
    PairedComparison,
    Summary,
    compare_paired,
    mean_and_deviation,
    summarise,
)
from rrt_planner.analysis.report import format_paired_table, format_summary_table

__all__ = [
    "PairedComparison",
    "Summary",
    "compare_paired",
    "format_paired_table",
    "format_summary_table",
    "mean_and_deviation",
    "summarise",
]
