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


# --------------------------------------------------------------------- HITL without a parameter list
def _link(**kw):
    """A link object with just the fields `listening` reads."""
    from airframe_designer.px4.link import PX4Link
    l = PX4Link.__new__(PX4Link)
    l.mode = kw.get("mode", "hitl")
    l.block_hil = kw.get("block_hil", False)
    l.hil_enabled = kw.get("hil_enabled", True)
    l.heartbeat_time = kw.get("heartbeat_time", time.time())
    l.params = kw.get("params", {})
    return l


def test_a_board_in_hitl_is_flown_before_its_parameters_arrive():
    """The heartbeat says HIL_ENABLED long before the parameter list exists; that is the answer.

    Reading SYS_HITL out of an empty parameter cache made a freshly connected HITL board look like a
    real flight, so every HIL_SENSOR was dropped and the board reported no accelerometer, no gyro and
    no attitude - a simulator that appeared to have broken the aircraft.
    """
    assert _link(hil_enabled=True, params={}).listening is False
    assert _link(hil_enabled=False, params={}).listening is True


def test_the_parameter_still_answers_when_no_heartbeat_has_arrived():
    assert _link(hil_enabled=True, heartbeat_time=0.0, params={}).listening is True
    assert _link(hil_enabled=True, heartbeat_time=0.0,
                 params={"SYS_HITL": {"value": 1}}).listening is False


def test_a_stale_heartbeat_falls_back_to_the_parameter():
    old = time.time() - 30
    assert _link(hil_enabled=True, heartbeat_time=old, params={"SYS_HITL": {"value": 0}}).listening is True


def test_blocking_is_a_decision_nothing_overrides():
    assert _link(block_hil=True, hil_enabled=True, params={"SYS_HITL": {"value": 1}}).listening is True


def test_sitl_never_listens():
    assert _link(mode="sitl", hil_enabled=False, params={}).listening is False


def test_a_simulator_is_never_sent_a_nuttx_reboot():
    """`reboot` is a NuttX command. SITL's pxh answers "Invalid command: reboot".

    The MAVLink reboot is denied by a simulated PX4, and the shell fallback then filled the log with a
    command that was never going to work. There is no board to reboot in SITL; Reset relaunches it.
    """
    from airframe_designer.px4.link import PX4Link
    calls = []
    link = PX4Link.__new__(PX4Link)
    link.mode = "sitl"
    link.last_ack = {}
    link.log = lambda m: calls.append(m)
    link.send_command_long = lambda *a, **k: None
    link.shell = lambda *a, **k: calls.append("SHELL " + a[0])
    PX4Link.reboot(link)
    assert not any(c.startswith("SHELL") for c in calls), f"a simulator was sent a board command: {calls}"
    assert any("no board to reboot" in c for c in calls), calls
