"""Run the browser-side tests with the rest of the suite.

The 3D view has its own copy of some geometry — where a wing's span line goes, where the structural origin
sits — and a copy that nobody runs is a copy that drifts. `node --test` over tests/*.mjs is cheap, so it
runs here too, and the wing case compares the browser's arithmetic with this package's, number for number.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

TESTS = Path(__file__).resolve().parent
MJS = sorted(p.name for p in TESTS.glob("test_*.mjs"))


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
@pytest.mark.parametrize("name", MJS)
def test_browser_side(name):
    r = subprocess.run(["node", "--test", name], cwd=TESTS, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stdout + r.stderr
