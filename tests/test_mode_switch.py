"""SITL, HITL, FLIGHT: one press, and the aircraft ends up in that state whatever it was in before.

A board left in HITL has its own sensors switched off and cannot fly for real; a board on its own sensors
cannot be driven by the simulator. Both take a parameter, a save and a reboot - the switch does them.
"""
from __future__ import annotations

import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from airframe_designer.px4.connection import ConnectionManager


def manager(hil: bool, armed: bool = False, comes_back: bool = True, mode: str = "hitl"):
    """A connection manager over a fake board that reboots into whatever SYS_HITL was set to."""
    link = NS(ctl_connected=True, heartbeat_time=time.time(), armed=armed, hil_enabled=hil,
              block_hil=False, preflight_storage=Mock(), trim_telemetry=Mock(),
              request_flight_telemetry=Mock(), link_budget_bps=lambda: None)
    written: list = []

    def set_param(name, value, timeout=2.0):
        written.append((name, value))
        return {"ok": True}

    def reboot():
        link.heartbeat_time = time.time()
        if comes_back:
            link.hil_enabled = bool(written and written[-1][1] == 1)
    link.set_param, link.reboot = set_param, reboot

    conn = ConnectionManager.__new__(ConnectionManager)
    conn.link, conn.mode, conn.log = link, mode, lambda *_: None
    conn._mode_job = {"mode": "", "state": "", "step": "", "error": "", "since": 0.0}
    conn._mode_gen = 0
    conn.MODE_SETTLE_S = 2.0
    conn.connect_hitl = lambda *a, **k: {}
    return conn, link, written


def test_flight_turns_a_hitl_board_onto_its_own_sensors():
    conn, link, written = manager(hil=True)
    assert conn.set_mode("real")["ok"]
    assert ("SYS_HITL", 0) in written and link.preflight_storage.called
    assert link.hil_enabled is False and link.block_hil is True
    assert link.request_flight_telemetry.called
    assert conn.mode_state()["state"] == "done"


def test_hitl_turns_a_real_flight_board_back():
    conn, link, written = manager(hil=False)
    assert conn.set_mode("hitl")["ok"]
    assert ("SYS_HITL", 1) in written
    assert link.hil_enabled is True and link.block_hil is False
    assert link.trim_telemetry.called


def test_a_board_already_in_the_right_mode_is_left_alone():
    """Pressing the mode it is already in costs a heartbeat, not a reboot."""
    conn, link, written = manager(hil=False)
    assert conn.set_mode("real")["ok"]
    assert written == [] and link.block_hil is True


def test_a_board_that_does_not_come_back_is_a_sentence_not_a_hang():
    conn, link, written = manager(hil=True, comes_back=False)
    t0 = time.time()
    r = conn.set_mode("real")
    assert not r["ok"] and "did not come back" in r["error"]
    assert time.time() - t0 < conn.MODE_SETTLE_S + 5, "it gave up on time"
    assert conn.mode_state()["state"] == "failed"


def test_an_armed_aircraft_is_refused():
    conn, link, written = manager(hil=True, armed=True)
    r = conn.set_mode("real")
    assert not r["ok"] and "armed" in r["error"] and written == []


def test_a_newer_press_supersedes_the_one_in_flight():
    conn, link, written = manager(hil=True, comes_back=False)
    conn._mode_gen = 5
    r = conn.set_mode("real", gen=4)          # an older attempt, still running
    assert r.get("superseded") or r["ok"], "it stood aside instead of fighting the newer press"
    assert ("SYS_HITL", 0) not in written
