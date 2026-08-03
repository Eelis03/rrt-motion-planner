"""Tier one: the package declares what it ships.

A package that passes ``mypy --strict`` still hands no type information to anything
that installs it unless PEP 561 says so, and PEP 561 says so with one empty marker
file inside the package directory. That the file exists is not something any other
test would notice, and losing it would be silent, so it is asserted directly.
"""

from __future__ import annotations

from pathlib import Path

import rrt_planner

PACKAGE_ROOT = Path(rrt_planner.__file__).resolve().parent
MARKER = PACKAGE_ROOT / "py.typed"


class TestTypingMarker:
    """PEP 561 inline type information."""

    def test_the_marker_sits_inside_the_package_directory(self) -> None:
        assert MARKER.is_file()
        assert MARKER.parent == PACKAGE_ROOT
        assert MARKER.name == "py.typed"

    def test_the_marker_is_empty(self) -> None:
        # PEP 561 gives the file no contents. Anything written into it would be a
        # partial-stub declaration, which this package does not make.
        assert MARKER.read_bytes() == b""

    def test_the_package_exposes_a_version(self) -> None:
        assert isinstance(rrt_planner.__version__, str)
        assert rrt_planner.__version__
