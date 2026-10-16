"""SITL firmware freshness: custom module tree detection, stale-source detection, relaunch detection."""
import os
import time
from pathlib import Path

from airframe_designer.px4 import firmware as fw


def make_tree(tmp_path, binary=True):
    (tmp_path / "src" / "mod").mkdir(parents=True)
    (tmp_path / "src" / "mod" / "Mod.cpp").write_text("int x;")
    (tmp_path / "msg").mkdir(); (tmp_path / "msg" / "A.msg").write_text("uint64 timestamp")
    if binary:
        b = tmp_path / "build" / "px4_sitl_default" / "bin" / "px4"
        b.parent.mkdir(parents=True); b.write_bytes(b"\x00")
    return tmp_path


def test_custom_tree_vs_stock_checkout(tmp_path):
    t = make_tree(tmp_path / "atlas")
    assert fw.is_custom_tree(str(t))
    stock = tmp_path / "px4"; (stock / "src").mkdir(parents=True); (stock / "Tools").mkdir()
    assert not fw.is_custom_tree(str(stock))
    assert fw.stale_sources(str(stock)) == []


def test_stale_and_relaunch_detection(tmp_path):
    t = make_tree(tmp_path)
    b = t / "build" / "px4_sitl_default" / "bin" / "px4"
    old = time.time() - 100
    for p in (t / "src" / "mod" / "Mod.cpp", t / "msg" / "A.msg"):
        os.utime(p, (old, old))
    os.utime(b, (old + 10, old + 10))
    assert fw.stale_sources(str(t)) == []
    st = fw.status(str(t), launched_binary_mtime=old + 10, max_age=0)
    assert not st["needs_rebuild"] and not st["needs_relaunch"]
    # editing a source makes it stale; a binary newer than the running PX4 asks for a relaunch
    os.utime(t / "src" / "mod" / "Mod.cpp", None)
    assert fw.stale_sources(str(t)) == ["src/mod/Mod.cpp"]
    assert fw.status(str(t), launched_binary_mtime=old + 10, max_age=0)["needs_rebuild"]
    os.utime(b, None)
    st = fw.status(str(t), launched_binary_mtime=old + 10, max_age=0)
    assert st["needs_relaunch"] and not st["needs_rebuild"]


def test_missing_binary_needs_rebuild(tmp_path):
    t = make_tree(tmp_path, binary=False)
    st = fw.status(str(t), None, max_age=0)
    assert st["needs_rebuild"] and len(st["stale"]) == 2


def test_switching_a_module_off_records_its_parameters_with_the_design(tmp_path, monkeypatch):
    """The names come from the module's own params.yaml, and they are kept with the design so a batch run
    or another machine exports exactly the same thing."""
    import asyncio, threading
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    from airframe_designer.geometry.airframe import quad_x
    from airframe_designer.server import app as mod

    monkeypatch.setattr(mod, "AIRFRAME_DIR", tmp_path)
    af = quad_x()
    af.name = "MODULES_TEST"
    sim = NS(lock=threading.RLock(), hooks=[], t=0.0, paused=False, airframe=af,
             set_airframe=lambda x: None, stop_nose_lift=Mock(), reset=Mock())
    state = NS(simulator=sim, conn=NS(mode="sitl", reset_all=lambda: {"steps": []}),
               link=NS(status=lambda: {"armed": False}), log=Mock())
    routes = {r.path: r.endpoint for r in mod.build_app(state).routes if hasattr(r, "endpoint")}
    listed = routes["/api/firmware/modules"]()
    module = next((g for g in listed["groups"] if g["kind"] == "module" and g.get("params")), None)
    if module is None:
        import pytest
        pytest.skip("no firmware module with parameters in this checkout")

    r = asyncio.run(routes["/api/firmware/modules/enable"]({"dir": module["dir"], "on": False}))
    assert r["ok"] and set(af.design["modules_off"][module["dir"]]) == set(module["params"])
    assert not (set(af.px4_params_sitl()) & set(module["params"])), "none of them reach the export"

    assert asyncio.run(routes["/api/firmware/modules/enable"]({"dir": module["dir"], "on": True}))["ok"]
    assert af.design["modules_off"] == {}
    bad = asyncio.run(routes["/api/firmware/modules/enable"]({"dir": "nope", "on": False}))
    assert bad.status_code == 404
