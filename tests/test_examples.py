"""Tier three: every example script runs to completion under a reduced budget.

The scripts are executed as subprocesses, exactly as a reader would run them, so that a
broken import or a broken command line surfaces here rather than in a demonstration.
Output is redirected into a temporary directory, so a test run never overwrites the
traces committed for the visualisation layer.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from tests.conftest import REPOSITORY_ROOT

EXAMPLES = REPOSITORY_ROOT / "examples"

REDUCED_ARGUMENTS = {
    "plan_single_query.py": ["--problem", "cluttered", "--samples", "400", "--milestones", "120"],
    "run_benchmark.py": ["--repeats", "2", "--samples", "300", "--milestones", "100"],
    "export_viz_trace.py": ["--samples", "300", "--milestones", "100"],
    "make_figures.py": ["--samples", "300", "--milestones", "100"],
}

# The figures committed under docs/figures are snapshots, not build output, so nothing
# regenerates them during a test run and nothing compares them byte for byte: identical
# planning produces PNG bytes that differ between matplotlib builds and platforms. What
# is checked here is that the command in the README still runs and still writes them,
# and that what is committed fits the budget the portfolio applies to every repository.
PUBLISHED_FIGURES = REPOSITORY_ROOT / "docs" / "figures"
FIGURE_BUDGET_BYTES = 250 * 1024


def example_scripts() -> list[Path]:
    """Every runnable example, so a newly added script is covered without editing tests."""
    return sorted(path for path in EXAMPLES.glob("*.py") if path.name != "__init__.py")


def test_every_example_has_a_reduced_configuration() -> None:
    assert {path.name for path in example_scripts()} == set(REDUCED_ARGUMENTS)


@pytest.mark.parametrize("script", example_scripts(), ids=lambda path: path.name)
def test_example_runs_to_completion(script: Path, tmp_path: Path) -> None:
    command = [
        sys.executable,
        str(script),
        *REDUCED_ARGUMENTS[script.name],
        "--output",
        str(tmp_path / script.stem),
    ]
    completed = subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip()
    assert list((tmp_path / script.stem).iterdir())


class TestPublishedFigures:
    """The committed snapshots exist and stay inside the size budget."""

    def test_every_figure_named_by_the_generator_is_committed(self) -> None:
        source = (EXAMPLES / "make_figures.py").read_text(encoding="utf-8")
        expected = {name for name in source.split('"') if name.endswith(".png")}
        assert expected, "the generator names no output files"
        assert {path.name for path in PUBLISHED_FIGURES.glob("*.png")} == expected

    def test_the_committed_figures_fit_the_budget(self) -> None:
        sizes = [path.stat().st_size for path in PUBLISHED_FIGURES.glob("*.png")]
        assert all(size > 0 for size in sizes)
        assert sum(sizes) <= FIGURE_BUDGET_BYTES

    def test_the_readme_embeds_every_committed_figure(self) -> None:
        readme = (REPOSITORY_ROOT / "README.md").read_text(encoding="utf-8")
        for path in PUBLISHED_FIGURES.glob("*.png"):
            assert f"docs/figures/{path.name}" in readme, path.name


def test_help_is_available_for_every_example() -> None:
    for script in example_scripts():
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            cwd=REPOSITORY_ROOT,
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        assert completed.returncode == 0
        assert "--output" in completed.stdout
