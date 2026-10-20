"""There is no Save button, so the last edit of a burst has to reach the disk by itself.

Two edits half a second apart used to leave the second one in memory only: the throttle dropped it and
nothing came along to write it. Tick a checkbox, stop touching the app, reload, and the tick was gone.
"""
from __future__ import annotations

import asyncio
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from airframe_designer.geometry.airframe import quad_x
from airframe_designer.server import app as server_app


@pytest.fixture
def routes(tmp_path, monkeypatch):
    monkeypatch.setattr(server_app, "AIRFRAME_DIR", tmp_path)
    af = quad_x()
    af.name = "AUTOSAVE_TEST"
    sim = NS(lock=threading.RLock(), hooks=[], t=0.0, paused=False, airframe=af,
             set_airframe=lambda x: setattr(sim, "airframe", x), reset=Mock())
    state = NS(simulator=sim, conn=NS(mode="sitl", reset_all=lambda: {"steps": []}),
               link=NS(status=lambda: {"armed": False}), log=Mock())
    app = server_app.build_app(state)
    return {r.path: r.endpoint for r in app.routes if hasattr(r, "endpoint")}, tmp_path, af


def test_the_last_edit_of_a_burst_is_written_even_when_it_lands_inside_the_throttle(routes):
    import json
    endpoints, tmp_path, af = routes
    post = endpoints["/api/airframe"]
    for mass in (3.0, 4.0, 5.0):                 # three edits in a row, well inside half a second
        d = af.to_dict()
        d["mass"]["mass"] = mass
        d["mass"]["from_items"] = False
        asyncio.run(post({"airframe": d}))
    saved = tmp_path / "autosave_test.json"
    assert saved.is_file(), "nothing was written at all"
    deadline = time.time() + 2.0
    while time.time() < deadline and json.loads(saved.read_text())["mass"]["mass"] != 5.0:
        time.sleep(0.05)
    assert json.loads(saved.read_text())["mass"]["mass"] == 5.0, "the last edit never reached the disk"
