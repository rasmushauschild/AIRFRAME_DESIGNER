"""Reading the aircraft's own log: over the link, and from a file.

The radio carries ten to twenty samples a second. The board writes two hundred. These tests are about the
second one arriving intact and reading back as a run like any other.
"""
from __future__ import annotations

import queue
import time
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from airframe_designer.px4.logs import CHUNK, Download, LogEntry, download_log, list_logs


class FakeBoard:
    """A flight controller with one log on its card, and a link that loses some of it."""

    def __init__(self, data: bytes, drop_every: int = 0, budget=None):
        self.data = data
        self.drop_every = drop_every
        self.sent = 0
        self.requests = 0
        self.ended = False
        self.log_entries: queue.Queue = queue.Queue()
        self.log_chunks: queue.Queue = queue.Queue()

    def link_budget_bps(self):
        return None

    def send_log_request_list(self, start=0, end=0xFFFF):
        self.log_entries.put({"id": 7, "num_logs": 1, "last_log_num": 7, "time_utc": 1_700_000_000,
                              "size": len(self.data)})

    def send_log_request_data(self, log_id, ofs, count):
        self.requests += 1
        for o in range(ofs, min(ofs + count, len(self.data)), CHUNK):
            self.sent += 1
            if self.drop_every and self.sent % self.drop_every == 0:
                continue                       # the radio ate this one
            self.log_chunks.put({"id": log_id, "ofs": o, "data": self.data[o:o + CHUNK]})

    def send_log_request_end(self):
        self.ended = True


def test_the_list_comes_back_with_size_and_time():
    board = FakeBoard(b"x" * 1000)
    logs = list_logs(board, timeout=2.0)
    assert [l.as_dict() for l in logs] == [{"id": 7, "size": 1000, "time_utc": 1_700_000_000}]


def test_a_log_arrives_byte_for_byte(tmp_path):
    data = bytes(range(256)) * 40                     # 10240 bytes, not a whole number of chunks
    board = FakeBoard(data)
    entry = LogEntry(id=7, size=len(data), time_utc=0)
    job = Download(log_id=7, size=len(data), path=tmp_path / "f.ulg")
    download_log(board, entry, tmp_path / "f.ulg", job, window=CHUNK * 10, patience=0.1)
    assert job.state == "done"
    assert (tmp_path / "f.ulg").read_bytes() == data
    assert job.received == len(data) and board.ended


def test_a_lossy_link_is_re_asked_not_restarted(tmp_path):
    """Over a radio pieces go missing. The transfer asks again for the holes and keeps everything it had -
    which is the difference between a download that finishes and one that never does."""
    data = bytes(range(251)) * 50
    board = FakeBoard(data, drop_every=4)
    entry = LogEntry(id=7, size=len(data), time_utc=0)
    job = Download(log_id=7, size=len(data), path=tmp_path / "f.ulg")
    download_log(board, entry, tmp_path / "f.ulg", job, window=CHUNK * 8, patience=0.1)
    assert job.state == "done"
    assert (tmp_path / "f.ulg").read_bytes() == data
    assert board.requests > len(data) / (CHUNK * 8), "it had to ask again for the dropped pieces"


def test_stopping_a_download_stops_it(tmp_path):
    data = b"y" * 100_000
    board = FakeBoard(data)
    job = Download(log_id=7, size=len(data), path=tmp_path / "f.ulg")
    download_log(board, LogEntry(7, len(data), 0), tmp_path / "f.ulg", job,
                 cancelled=lambda: True, window=CHUNK * 4, patience=0.1)
    assert job.state == "cancelled" and not (tmp_path / "f.ulg").exists()


def test_a_silent_aircraft_says_how_far_it_got(tmp_path):
    class Silent(FakeBoard):
        def send_log_request_data(self, log_id, ofs, count):
            self.requests += 1                 # answers nothing at all
    board = Silent(b"z" * 10_000)
    job = Download(log_id=7, size=10_000, path=tmp_path / "f.ulg")
    t0 = time.time()
    download_log(board, LogEntry(7, 10_000, 0), tmp_path / "f.ulg", job, window=CHUNK * 4, patience=0.05)
    assert job.state == "failed" and "%" in job.error
    assert time.time() - t0 < 5, "it gave up rather than hanging"


ULOG = next((p for p in [
    Path("/Users/rasmushauschild/atlas-simulation/tests/fixtures/log_36_2026-9-9-14-11-36.ulg"),
] if p.is_file()), None)


@pytest.mark.skipif(ULOG is None, reason="no .ulg to read")
def test_a_real_log_reads_back_as_a_run():
    """A log off an ATLAS: the columns are the recorder's, the numbers are the aircraft's."""
    from airframe_designer.sim.recorder import columns_for
    from airframe_designer.sim.ulog_import import ulog_to_run
    run = ulog_to_run(ULOG, rate_hz=50.0)
    assert run["columns"] == columns_for(run["motors"]), "a run from a log is a run like any other"
    assert run["samples"] == len(run["rows"]) and run["duration"] > 1
    assert run["logged_hz"] > 100, "the board logs far faster than any radio carries"
    i = {c: k for k, c in enumerate(run["columns"])}
    col = lambda name: [r[i[name]] for r in run["rows"]]                       # noqa: E731
    assert all(abs(v) < 1e4 for v in col("height")), "nothing absurd came through"
    assert max(col("armed")) == 1.0, "it armed"
    assert 1.0 <= max(col("mode")) <= 10.0, "the mode reads on this app's scale"
    assert -12 < min(col("az")) < -8, "the accelerometer reads specific force, about -9.8 on the ground"
    assert len(set(col("qw"))) > 10, "the attitude actually varies"


def test_the_recorder_keeps_up_with_a_cable_and_not_with_a_radio():
    """"As fast as it can" is set by the link: 50 Hz down a cable, 20 over a radio, because sampling
    faster than the aircraft speaks only writes the same number down five times."""
    import asyncio, threading
    from unittest.mock import Mock
    from airframe_designer.server.app import build_app
    from airframe_designer.sim.recorder import Recorder

    def app_for(baud):
        link = NS(mode="hitl", baud=baud, ctl_connected=True, block_hil=False, listening=True,
                  params={}, status=lambda: {"armed": False}, request_flight_telemetry=Mock(),
                  link_budget_bps=lambda: (baud / 10.0 * 0.28 if baud and baud <= 115200 else None))
        sim = NS(lock=threading.RLock(), hooks=[], t=0.0, paused=True, airframe=None, recorder=Recorder(),
                 motor_override=[1], stop_nose_lift=Mock(), reset=Mock())
        state = NS(simulator=sim, conn=NS(mode="hitl", reset_all=lambda: {"steps": []},
                                          connect_hitl=lambda *a: {}, status=lambda: {}),
                   link=link, log=Mock())
        routes = {r.path: r.endpoint for r in build_app(state).routes if hasattr(r, "endpoint")}
        asyncio.run(routes["/api/connection/flight"]({}))
        return sim

    assert app_for(57600).recorder.sample_hz == 20.0
    assert app_for(921600).recorder.sample_hz == 50.0
    assert app_for(57600).paused is False, "the loop is what follows the telemetry"
