"""Flight metrics: a hook that samples the simulation and summarises each scenario phase.

Everything an objective function might want is computed here so studies only pick and combine numbers:
position / altitude / attitude tracking, rates, tilt, motor utilisation and saturation, ideal power and energy,
wing lift share, airspeed, crash and landing detection. Attitude is reported in PX4's hover frame (roll/pitch
relative to the attitude PX4 holds as level), which is what "stable hover" means for a nose-up hovering craft.
"""
from __future__ import annotations

import math

import numpy as np

G = 9.80665


class MetricsRecorder:
    def __init__(self, sample_hz: float = 50.0):
        self.sample_hz = sample_hz
        self.rows: list[list[float]] = []      # decimated time series
        self.phase_of_row: list[str] = []
        self.phases: dict[str, dict] = {}       # name -> {"t0", "t1", ...}
        self.current_phase: str = "init"
        self.events: list[dict] = []
        self._every = None
        self.max_tilt_deg = 0.0
        self.max_alt = 0.0
        self.crashed = False
        self.crash_reason = ""
        self.touchdown_speed: float | None = None
        self._was_airborne = False
        self._t_last = None
        self.energy_j = 0.0
        self.saturation_steps = 0
        self.steps = 0

    # ------------------------------------------------------------ phases
    def begin_phase(self, name: str, t: float) -> None:
        self.end_phase(t)
        self.current_phase = name
        self.phases[name] = {"t0": t, "t1": None}

    def end_phase(self, t: float) -> None:
        ph = self.phases.get(self.current_phase)
        if ph and ph["t1"] is None:
            ph["t1"] = t

    def note(self, t: float, text: str) -> None:
        self.events.append({"t": round(t, 3), "text": text})

    # -------------------------------------------------------------- hook
    def __call__(self, simr) -> None:
        s = simr.sim
        t = simr.t
        if self._every is None:
            self._every = max(1, int(round(simr.sensor_rate / self.sample_hz)))
        dt = (t - self._t_last) if self._t_last is not None else 0.0
        self._t_last = t
        bd = s.breakdown or {}
        power = float(bd.get("power", 0.0))
        self.energy_j += power * dt
        self.steps += 1
        cmd = s.cmd
        if len(cmd) and float(cmd.max()) > 0.95:
            self.saturation_steps += 1
        tilt = s.tilt_deg
        alt = -float(s.pos[2])
        self.max_tilt_deg = max(self.max_tilt_deg, tilt)
        self.max_alt = max(self.max_alt, alt)
        airborne = not s.on_ground
        if airborne:
            self._was_airborne = True
        elif self._was_airborne and self.touchdown_speed is None:
            self.touchdown_speed = float(np.linalg.norm(s.vel))
            self.note(t, f"touchdown at {self.touchdown_speed:.2f} m/s")
        if simr.step_count % self._every == 0:
            r, p, y = s.hover_frame_euler()
            util = float((s.thrust / np.maximum(s.rotors.tmax * s.rotors.scale, 1e-9)).max()) if len(s.thrust) else 0.0
            self.rows.append([t, *s.pos.tolist(), *s.vel.tolist(), r, p, y, *s.rates.tolist(), tilt,
                              float(bd.get("thrust", 0.0)), power, float(bd.get("lift", 0.0)), float(bd.get("airspeed", 0.0)),
                              util, float(cmd.mean()) if len(cmd) else 0.0, 1.0 if airborne else 0.0])
            self.phase_of_row.append(self.current_phase)

    # ------------------------------------------------------------ summary
    COLS = ["t", "n", "e", "d", "vn", "ve", "vd", "roll", "pitch", "yaw", "p", "q", "r", "tilt", "thrust", "power", "lift",
            "airspeed", "util_max", "cmd_mean", "airborne"]

    def array(self) -> np.ndarray:
        return np.array(self.rows, float).reshape(-1, len(self.COLS))

    def summary(self, mass: float, phase_targets: dict[str, dict] | None = None) -> dict:
        """Per-phase and overall statistics. ``phase_targets`` may give per phase {"pos": [n,e,d]} or {"alt": h}
        setpoints for tracking errors."""
        a = self.array()
        out: dict = {"crashed": self.crashed, "crash_reason": self.crash_reason, "max_tilt_deg": round(self.max_tilt_deg, 2),
                     "max_alt_m": round(self.max_alt, 2), "energy_wh": round(self.energy_j / 3600.0, 3),
                     "saturation_fraction": round(self.saturation_steps / max(1, self.steps), 4),
                     "touchdown_speed": None if self.touchdown_speed is None else round(self.touchdown_speed, 3),
                     "events": self.events, "phases": {}}
        if len(a) == 0:
            return out
        col = {c: i for i, c in enumerate(self.COLS)}
        weight = mass * G
        hover_power = None
        for name, ph in self.phases.items():
            m = np.array([p == name for p in self.phase_of_row])
            if not m.any():
                continue
            x = a[m]
            tgt = (phase_targets or {}).get(name, {})
            d = {"t0": round(ph["t0"], 3), "t1": round(ph["t1"] if ph["t1"] is not None else float(x[-1, 0]), 3),
                 "samples": int(m.sum())}
            d["duration"] = round(d["t1"] - d["t0"], 3)
            pos = x[:, col["n"]:col["d"] + 1]
            vel = x[:, col["vn"]:col["vd"] + 1]
            alt = -pos[:, 2]
            d["alt_mean"] = round(float(alt.mean()), 3)
            d["alt_min"] = round(float(alt.min()), 3); d["alt_max"] = round(float(alt.max()), 3)
            d["alt_std"] = round(float(alt.std()), 4)
            d["pos_drift"] = round(float(np.linalg.norm(pos[-1, :2] - pos[0, :2])), 3)
            d["pos_std_xy"] = round(float(np.sqrt(((pos[:, :2] - pos[:, :2].mean(axis=0)) ** 2).sum(axis=1).mean())), 4)
            d["speed_mean"] = round(float(np.linalg.norm(vel[:, :2], axis=1).mean()), 3)
            d["speed_max"] = round(float(np.linalg.norm(vel, axis=1).max()), 3)
            d["vz_std"] = round(float(vel[:, 2].std()), 4)
            for k in ("roll", "pitch"):
                v = np.degrees(x[:, col[k]])
                d[f"{k}_mean_deg"] = round(float(v.mean()), 3)
                d[f"{k}_rms_deg"] = round(float(np.sqrt((v ** 2).mean())), 3)
                d[f"{k}_std_deg"] = round(float(v.std()), 3)
                d[f"{k}_max_deg"] = round(float(np.abs(v).max()), 3)
            yaw = np.degrees(np.unwrap(x[:, col["yaw"]]))
            d["yaw_drift_deg"] = round(float(yaw[-1] - yaw[0]), 3)
            d["yaw_std_deg"] = round(float(yaw.std()), 3)
            rates = np.degrees(x[:, col["p"]:col["r"] + 1])
            d["rates_rms_deg_s"] = round(float(np.sqrt((rates ** 2).sum(axis=1).mean())), 3)
            d["tilt_max_deg"] = round(float(x[:, col["tilt"]].max()), 3)
            d["tilt_mean_deg"] = round(float(x[:, col["tilt"]].mean()), 3)
            d["thrust_mean"] = round(float(x[:, col["thrust"]].mean()), 3)
            d["thrust_to_weight_mean"] = round(float(x[:, col["thrust"]].mean() / weight), 4)
            d["power_mean"] = round(float(x[:, col["power"]].mean()), 2)
            d["power_max"] = round(float(x[:, col["power"]].max()), 2)
            d["lift_share_mean"] = round(float(x[:, col["lift"]].mean() / weight), 4)
            d["airspeed_mean"] = round(float(x[:, col["airspeed"]].mean()), 3)
            d["util_max"] = round(float(x[:, col["util_max"]].max()), 4)
            d["util_mean"] = round(float(x[:, col["util_max"]].mean()), 4)
            d["cmd_mean"] = round(float(x[:, col["cmd_mean"]].mean()), 4)
            d["airborne_fraction"] = round(float(x[:, col["airborne"]].mean()), 3)
            if "alt" in tgt:
                e = alt - float(tgt["alt"])
                d["alt_err_rms"] = round(float(np.sqrt((e ** 2).mean())), 4); d["alt_err_max"] = round(float(np.abs(e).max()), 4)
            if "pos" in tgt:
                e = pos - np.asarray(tgt["pos"], float)
                d["pos_err_rms"] = round(float(np.sqrt((e ** 2).sum(axis=1).mean())), 4)
                d["pos_err_final"] = round(float(np.linalg.norm(e[-1])), 4)
            if "vel" in tgt:
                e = vel - np.asarray(tgt["vel"], float)
                d["vel_err_rms"] = round(float(np.sqrt((e ** 2).sum(axis=1).mean())), 4)
                d["vel_err_final"] = round(float(np.linalg.norm(e[-1])), 4)
            if name.startswith("hover") or name == "hold":
                hover_power = d["power_mean"] if hover_power is None else hover_power
            for k, v in ph.items():                 # extras recorded by the scenario runner (time_to_alt, ...)
                if k not in ("t0", "t1") and k not in d:
                    d[k] = v
            out["phases"][name] = d
        if hover_power:
            for name, d in out["phases"].items():
                d["power_ratio_vs_hover"] = round(d["power_mean"] / hover_power, 4) if hover_power > 0 else None
        out["flight_time"] = round(float(a[:, col["airborne"]].sum() / self.sample_hz), 2)
        out["final_alt"] = round(float(-a[-1, col["d"]]), 3)
        out["final_speed"] = round(float(np.linalg.norm(a[-1, col["vn"]:col["vd"] + 1])), 3)
        return out

    def timeseries(self) -> dict:
        a = self.array()
        return {"columns": self.COLS, "rows": [[round(float(v), 5) for v in r] for r in a.tolist()], "phase": list(self.phase_of_row)}
