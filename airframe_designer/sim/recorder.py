"""Flight recorder: every simulation run is kept, with the design that produced it.

The point is to be able to come back to a flight. Not to a number someone wrote down about it, but to the
flight: the trace of every quantity the overlay shows, plus the attitude at each instant so the 3D view can
be scrubbed back and forth through it, plus the airframe and the exported parameters exactly as they were.
A run you cannot reproduce is an anecdote.

A run starts when the simulation is reset and ends when it is reset again, or when the app asks for it. It
is written to results/runs/<id>.json with a timestamp, and it can be renamed or deleted from the app. The
columns are fixed and documented here, so a saved run from today still reads in a year.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = ["Recorder", "RunStore", "COLUMNS"]

#: One row per sample. Everything the overlay shows, plus the pose needed to replay the view.
COLUMNS = [
    "t",                                        # s since the run started
    "n", "e", "d",                              # position, world NED, m
    "vn", "ve", "vd",                           # velocity, world NED, m/s
    "qw", "qx", "qy", "qz",                     # attitude, body -> world
    "roll", "pitch", "yaw",                     # degrees, for reading and plotting
    "p", "q", "r",                              # body rates, deg/s
    "height",                                   # m above the ground
    "speed",                                    # ground speed, m/s
    "airspeed",                                 # m/s
    "thrust",                                   # total motor thrust, N
    "power",                                    # motor power, W
    "lift",                                     # wing lift, N
    "battery",                                  # charge left, %
    "wind",                                     # wind speed, m/s
    "armed",                                    # 1 or 0
    "on_ground",                                # 1 or 0
]


@dataclass
class Recorder:
    """Collects one run in memory at a fixed sample rate."""

    sample_hz: float = 20.0
    max_samples: int = 120_000                  # about 100 minutes at 20 Hz; a run longer than that is a mistake
    rows: list[list[float]] = field(default_factory=list)
    started: float = field(default_factory=time.time)
    t0: float | None = None
    saved_at: int = -1                          # rows already written to disk, so a run is never kept twice
    _next_t: float = 0.0

    def reset(self) -> None:
        self.rows = []
        self.started = time.time()
        self.t0 = None
        self.saved_at = -1
        self._next_t = 0.0

    def sample(self, t: float, body, status: dict, battery_percent: float) -> None:
        """Take one sample if enough simulated time has passed. Called from the simulation loop."""
        if self.t0 is None:
            self.t0 = float(t)
        elapsed = float(t) - self.t0
        if elapsed + 1e-9 < self._next_t or len(self.rows) >= self.max_samples:
            return
        self._next_t = elapsed + 1.0 / max(1e-3, self.sample_hz)

        bd = body.breakdown or {}
        roll, pitch, yaw = (float(np.degrees(v)) for v in body.euler)
        rates = [float(np.degrees(v)) for v in body.rates]
        wind = float(np.linalg.norm(body.wind_ned))
        self.rows.append([
            round(elapsed, 4),
            *[round(float(v), 4) for v in body.pos],
            *[round(float(v), 4) for v in body.vel],
            *[round(float(v), 6) for v in body.q],
            round(roll, 3), round(pitch, 3), round(yaw, 3),
            *[round(v, 3) for v in rates],
            round(float(-body.pos[2]), 4),
            round(float(np.linalg.norm(body.vel)), 4),
            round(float(bd.get("airspeed", 0.0)), 4),
            round(float(bd.get("thrust", 0.0)), 3),
            round(float(bd.get("power", 0.0)), 2),
            round(float(bd.get("lift", 0.0)), 3),
            round(float(battery_percent), 2),
            round(wind, 3),
            1.0 if status.get("armed") else 0.0,
            1.0 if body.on_ground else 0.0,
        ])

    @property
    def flew(self) -> bool:
        """True if the aircraft armed or left the ground. A reset after nothing happened is not a flight."""
        ia, ig = COLUMNS.index("armed"), COLUMNS.index("on_ground")
        return any(r[ia] > 0.5 or r[ig] < 0.5 for r in self.rows)

    @property
    def unsaved(self) -> bool:
        return len(self.rows) > self.saved_at

    @property
    def duration(self) -> float:
        return float(self.rows[-1][0]) if self.rows else 0.0

    def as_dict(self) -> dict:
        return {"columns": COLUMNS, "rows": self.rows, "duration": round(self.duration, 3),
                "samples": len(self.rows), "sample_hz": self.sample_hz}


class RunStore:
    """Saved runs on disk: results/runs/<id>.json, newest first."""

    def __init__(self, root: Path):
        self.root = Path(root)

    def _dir(self) -> Path:
        self.root.mkdir(parents=True, exist_ok=True)
        return self.root

    def _path(self, run_id: str) -> Path | None:
        """Resolve an id to its file, refusing anything that is not a plain id."""
        if not run_id or "/" in run_id or "\\" in run_id or run_id.startswith("."):
            return None
        p = self._dir() / f"{run_id}.json"
        return p if p.parent == self._dir() else None

    def save(self, recorder: Recorder, *, airframe: dict, params: dict, name: str = "",
             scenario: str = "", note: str = "") -> dict:
        """Write a run with the design that produced it. Returns its index entry."""
        if not recorder.rows:
            return {"ok": False, "error": "nothing recorded"}
        run_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(recorder.started))
        path = self._path(run_id)
        n = 1
        while path is not None and path.exists():
            n += 1
            run_id = time.strftime("%Y%m%d-%H%M%S", time.localtime(recorder.started)) + f"-{n}"
            path = self._path(run_id)
        if path is None:
            return {"ok": False, "error": "bad run id"}
        doc = {
            "id": run_id,
            "name": name or (scenario or "flight"),
            "started": recorder.started,
            "scenario": scenario,
            "note": note,
            "airframe": airframe,
            "params": params,
            **recorder.as_dict(),
        }
        path.write_text(json.dumps(doc, separators=(",", ":")))
        recorder.saved_at = len(recorder.rows)
        return {"ok": True, **self._entry(doc)}

    @staticmethod
    def _entry(doc: dict) -> dict:
        return {"id": doc["id"], "name": doc.get("name", ""), "started": doc.get("started", 0),
                "duration": doc.get("duration", 0), "samples": doc.get("samples", 0),
                "scenario": doc.get("scenario", ""), "note": doc.get("note", ""),
                "airframe_name": (doc.get("airframe") or {}).get("name", "")}

    def list(self) -> list[dict]:
        out = []
        for f in sorted(self._dir().glob("*.json"), reverse=True):
            try:
                out.append(self._entry(json.loads(f.read_text())))
            except Exception:
                continue
        return out

    def get(self, run_id: str) -> dict | None:
        p = self._path(run_id)
        if p is None or not p.is_file():
            return None
        try:
            return json.loads(p.read_text())
        except Exception:
            return None

    def rename(self, run_id: str, name: str) -> bool:
        doc = self.get(run_id)
        p = self._path(run_id)
        if doc is None or p is None:
            return False
        doc["name"] = str(name)[:80]
        p.write_text(json.dumps(doc, separators=(",", ":")))
        return True

    def delete(self, run_id: str) -> bool:
        p = self._path(run_id)
        if p is None or not p.is_file():
            return False
        p.unlink()
        return True
