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
