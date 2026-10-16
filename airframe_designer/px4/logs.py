"""The flight controller's own logs, over the same MAVLink link the app is already on.

The board writes a .ulg to its SD card for every flight - everything it knows, at the rate it knows it,
which is ten to fifty times what any radio could carry. This fetches one.

The protocol is three messages: LOG_REQUEST_LIST brings back a LOG_ENTRY per file (id, size, UTC time),
LOG_REQUEST_DATA asks for a byte range and the board answers with LOG_DATA in 90-byte pieces, and
LOG_REQUEST_END lets it go back to flying. Nothing here is clever; what it has to be is honest about a
slow link - a window at a time so progress is real, missing pieces re-asked for rather than the transfer
restarted, and a rate measured from what actually arrives so the time remaining is a measurement and not a
guess.
"""
from __future__ import annotations

import queue
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from .link import mavlink

CHUNK = 90                     # bytes in one LOG_DATA, fixed by the message
LIST_TIMEOUT = 8.0


@dataclass
class LogEntry:
    id: int
    size: int
    time_utc: int

    def as_dict(self) -> dict:
        return {"id": self.id, "size": self.size, "time_utc": self.time_utc}


def list_logs(link, timeout: float = LIST_TIMEOUT) -> list[LogEntry]:
    """Every log on the board, newest id last. Empty if it reports none."""
    link.log_entries = queue.Queue()
    link.send_log_request_list()
    entries: dict[int, LogEntry] = {}
    expected = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            e = link.log_entries.get(timeout=0.5)
        except queue.Empty:
            if entries and expected is not None and len(entries) >= expected:
                break
            continue
        expected = e["num_logs"]
        if expected == 0:
            break
        if e["size"] > 0 or e["time_utc"] > 0:
            entries[e["id"]] = LogEntry(id=e["id"], size=int(e["size"]), time_utc=int(e["time_utc"]))
        if len(entries) >= expected:
            break
        t0 = time.time()                       # it is answering: let it finish
    return [entries[k] for k in sorted(entries)]


@dataclass
class Download:
    """One log coming down the link, and everything the UI needs to show about it."""

    log_id: int
    size: int
    path: Path
    received: int = 0
    state: str = "running"                     # running | done | failed | cancelled
    error: str = ""
    started: float = field(default_factory=time.time)
    bps: float = 0.0
    run_id: str = ""
    _marks: list = field(default_factory=list)

    @property
    def eta_s(self) -> float | None:
        left = max(0, self.size - self.received)
        return round(left / self.bps, 1) if self.bps > 1 else None

    def as_dict(self) -> dict:
        return {"log_id": self.log_id, "size": self.size, "received": self.received, "state": self.state,
                "error": self.error, "bps": round(self.bps), "eta_s": self.eta_s, "run_id": self.run_id,
                "elapsed": round(time.time() - self.started, 1),
                "percent": round(100.0 * self.received / self.size, 1) if self.size else 0.0}

    def note(self, now: float) -> None:
        """Rate over the last few seconds, which is the only rate worth reporting on a radio."""
        self._marks.append((now, self.received))
        while len(self._marks) > 1 and now - self._marks[0][0] > 6.0:
            self._marks.pop(0)
        if len(self._marks) > 1:
            dt = self._marks[-1][0] - self._marks[0][0]
            if dt > 0.5:
                self.bps = (self._marks[-1][1] - self._marks[0][1]) / dt


def download_log(link, entry: LogEntry, path: Path, job: Download,
                 cancelled: Callable[[], bool] = lambda: False, window: int | None = None,
                 patience: float | None = None, tries: int = 6) -> Download:
    """Fetch one log into ``path``, a window at a time, re-asking for what does not arrive.

    ``window`` is how much is in flight at once. A cable can keep a big one full; over a radio a big one
    only means a long wait before anyone notices a hole, so it is sized from the link.
    """
    budget = link.link_budget_bps()
    # How much is asked for before the next request goes out. Every window costs one round trip, and over
    # a radio that is a few tenths of a second, so a small window spends a fifth of the transfer waiting.
    window = window or (CHUNK * 150 if budget else CHUNK * 600)
    patience = patience if patience is not None else (6.0 if budget else 2.0)
    buf = bytearray(entry.size)
    have = bytearray(entry.size // CHUNK + 2)          # one flag per chunk, so a hole is cheap to find
    link.log_chunks = queue.Queue()
    ofs = 0
    try:
        while ofs < entry.size:
            if cancelled():
                job.state = "cancelled"
                return job
            end = min(entry.size, ofs + window)
            for attempt in range(tries):
                missing = [o for o in range(ofs, end, CHUNK) if not have[o // CHUNK]]
                if not missing:
                    break
                # ask for the gap in one piece when it is contiguous, which it is on the first pass
                link.send_log_request_data(entry.id, missing[0], missing[-1] + CHUNK - missing[0])
                left = len(missing)
                quiet_until = time.time() + patience
                while time.time() < quiet_until:
                    if cancelled():
                        job.state = "cancelled"
                        return job
                    try:
                        c = link.log_chunks.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if c["id"] != entry.id:
                        continue
                    o, data = int(c["ofs"]), c["data"]
                    if o % CHUNK or o >= entry.size:
                        continue
                    n = min(len(data), entry.size - o)
                    if not have[o // CHUNK]:
                        buf[o:o + n] = data[:n]
                        have[o // CHUNK] = 1
                        job.received += n
                        job.note(time.time())
                        # only a new piece of *this* window finishes it: a duplicate, or a straggler from
                        # the window before, must not make it look complete while a hole is still open
                        if ofs <= o < end:
                            left -= 1
                    if left <= 0:
                        break                                   # the window is complete; ask for the next
                if left <= 0:
                    break
            else:
                job.state = "failed"
                job.error = (f"the aircraft stopped sending at {100.0 * ofs / max(1, entry.size):.0f}%. "
                             f"Nothing is lost - starting the download again picks up from here.")
                return job
            ofs = end
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(bytes(buf))
        job.state = "done"
        return job
    finally:
        try:
            link.send_log_request_end()
        except Exception:
            pass
