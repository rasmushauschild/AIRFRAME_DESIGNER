"""The flight recorder: what it samples, what counts as a flight, and what ends up on disk.

A run that cannot be reproduced is an anecdote, so these check the two things that make it reproducible:
the trace is sampled at the rate it claims, and the airframe and parameters that produced it are saved
with it. The rest is housekeeping — naming, deleting, and refusing an id that is not a plain id.
"""
from __future__ import annotations

import json

import numpy as np
import pytest

from airframe_designer.geometry.airframe import quad_x
from airframe_designer.sim.recorder import COLUMNS, Recorder, RunStore


class FakeBody:
    """Just enough of a body for the recorder: it reads attributes, nothing else."""

    def __init__(self, d=-5.0, on_ground=False):
        self.pos = np.array([1.0, 2.0, d])
        self.vel = np.array([3.0, 0.0, -1.0])
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.euler = np.array([0.0, np.radians(-12.0), 0.0])
        self.rates = np.array([0.0, 0.1, 0.0])
        self.wind_ned = np.array([4.0, 3.0, 0.0])
        self.on_ground = on_ground
        self.breakdown = {"airspeed": 7.0, "thrust": 120.0, "power": 2500.0, "lift": 30.0}


def fill(rec, body, seconds=1.0, dt=0.004, armed=True, start=0.0):
    t = start
    while t < start + seconds:
        rec.sample(t, body, {"armed": armed}, 88.0)
        t += dt
    return rec


def test_it_samples_at_the_rate_it_says_and_nothing_faster():
    rec = Recorder(sample_hz=20.0)
    fill(rec, FakeBody(), seconds=2.0, dt=0.001)
    assert len(rec.rows) == pytest.approx(40, abs=1)
    gaps = np.diff([r[0] for r in rec.rows])
    assert gaps.min() >= 0.049 and gaps.max() <= 0.051
    assert rec.duration == pytest.approx(2.0, abs=0.05)      # the whole span, to within one sample


def test_every_column_is_filled_with_what_it_promises():
    rec = Recorder()
    fill(rec, FakeBody(d=-5.0), seconds=0.2)
    row = dict(zip(COLUMNS, rec.rows[-1]))
    assert len(rec.rows[0]) == len(COLUMNS)
    assert (row["n"], row["e"], row["d"]) == (1.0, 2.0, -5.0)
    assert row["height"] == 5.0                                 # height is up, position is down
    assert row["speed"] == pytest.approx(np.hypot(3.0, 1.0), abs=1e-4)
    assert row["pitch"] == pytest.approx(-12.0, abs=1e-3)       # degrees, not radians
    assert row["q"] == pytest.approx(np.degrees(0.1), abs=1e-3)
    assert row["wind"] == 5.0                                   # 3-4-5
    assert (row["power"], row["lift"], row["battery"]) == (2500.0, 30.0, 88.0)
    assert row["armed"] == 1.0 and row["on_ground"] == 0.0


def test_sitting_on_the_ground_disarmed_is_not_a_flight():
    rec = Recorder()
    fill(rec, FakeBody(d=-0.12, on_ground=True), seconds=3.0, armed=False)
    assert rec.rows and not rec.flew
    fill(rec, FakeBody(d=-0.12, on_ground=True), seconds=1.0, armed=True, start=3.0)
    assert rec.flew                                              # armed: it counts, even if it never left the pad


def test_a_run_is_saved_with_the_design_and_the_parameters_that_produced_it(tmp_path):
    af = quad_x()
    rec = Recorder()
    fill(rec, FakeBody(), seconds=1.0)
    store = RunStore(tmp_path)

    entry = store.save(rec, airframe=af.to_dict(), params={"MC_PITCHRATE_P": 0.15}, name="hover 1")
    assert entry["ok"] and entry["name"] == "hover 1"
    assert not rec.unsaved                                       # the next reset will not keep it twice

    doc = store.get(entry["id"])
    assert doc["params"]["MC_PITCHRATE_P"] == 0.15
    assert doc["airframe"]["name"] == af.name
    assert len(doc["airframe"]["rotors"]) == len(af.rotors)
    assert doc["columns"] == COLUMNS and len(doc["rows"]) == len(rec.rows)
    assert json.loads((tmp_path / f"{entry['id']}.json").read_text())["id"] == entry["id"]

    assert [r["id"] for r in store.list()] == [entry["id"]]
    assert store.rename(entry["id"], "the good one") and store.list()[0]["name"] == "the good one"
    assert store.delete(entry["id"]) and store.list() == []
    assert not store.delete(entry["id"])


def test_an_empty_recorder_saves_nothing(tmp_path):
    assert RunStore(tmp_path).save(Recorder(), airframe={}, params={})["ok"] is False


@pytest.mark.parametrize("bad", ["../../etc/passwd", "a/b", "..", ".hidden", ""])
def test_a_run_id_that_is_not_a_plain_id_is_refused(tmp_path, bad):
    store = RunStore(tmp_path)
    assert store.get(bad) is None
    assert store.delete(bad) is False
    assert store.rename(bad, "x") is False


def test_reset_forgets_the_run_and_the_fact_that_it_was_saved(tmp_path):
    rec = Recorder()
    fill(rec, FakeBody(), seconds=0.5)
    RunStore(tmp_path).save(rec, airframe={}, params={})
    rec.reset()
    assert rec.rows == [] and rec.duration == 0.0 and rec.unsaved
