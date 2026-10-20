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


def test_no_window_confirm_anywhere_in_the_ui():
    """`window.confirm()` returns false at once in this app's web view: a control that asks with it can
    never be switched on. The app asks twice on the control itself (`confirmTwice`) instead, and the
    motor-slider checkbox was the last one still calling confirm() - which is why it could not be ticked.
    """
    import re
    from pathlib import Path
    for f in sorted(Path("ui").glob("*.js")):
        src = f.read_text()
        for n, line in enumerate(src.splitlines(), 1):
            if line.strip().startswith("//"):
                continue
            assert not re.search(r"(?<![A-Za-z])confirm\s*\(", line.replace("confirmTwice(", "").replace("confirmArmed(", "")), \
                f"{f}:{n} asks with window.confirm, which this web view answers 'no' to instantly: {line.strip()[:90]}"
