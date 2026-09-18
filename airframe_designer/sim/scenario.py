"""Scripted flights: a scenario is a list of phases that drive PX4 through MAVLink while the simulation runs.

A scenario is JSON:
{
  "name": "takeoff_hover_land",
  "max_time": 90,                          # simulation seconds before the run is declared a timeout
  "params": {"MIS_TAKEOFF_ALT": 3},        # PX4 parameters for this scenario (seeded before boot in batch)
  "wind": [0, 0, 0],                       # NED m/s at the start
  "abort": {"max_tilt_deg": 100, "max_alt": 200, "crash_speed": 3.0},
  "phases": [
    {"type": "wait_ready", "timeout": 40},
    {"type": "takeoff", "alt": 3, "settle": 2, "timeout": 30},
    {"type": "hold", "duration": 10},
    {"type": "offboard_velocity", "vel": [12, 0, 0], "duration": 12, "yaw": 0},
    {"type": "offboard_position", "pos": [0, 0, -5], "tolerance": 1.0, "timeout": 30},
    {"type": "manual", "roll": 0, "pitch": 0.6, "throttle": 0.5, "yaw": 0, "duration": 5, "mode": "position"},
    {"type": "wind", "ned": [5, 0, 0]},
    {"type": "motor_failure", "rotor": 2, "scale": 0.0},
    {"type": "param", "name": "MC_PITCH_P", "value": 5.0},
    {"type": "wait", "duration": 2},
    {"type": "land", "timeout": 40}
  ]
}
Every phase may carry "name" (metrics are grouped by it; default = type). All MAVLink traffic is fire-and-forget
and checked on later steps, so the runner never blocks the lockstep loop.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from .metrics import MetricsRecorder

SCENARIO_DIR = Path(__file__).resolve().parents[2] / "scenarios"


@dataclass
class Scenario:
    name: str = "scenario"
    phases: list[dict] = field(default_factory=list)
    max_time: float = 120.0
    params: dict = field(default_factory=dict)
    wind: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    abort: dict = field(default_factory=dict)
    description: str = ""

    @classmethod
    def from_dict(cls, d: dict) -> "Scenario":
        return cls(name=d.get("name", "scenario"), phases=list(d.get("phases", [])), max_time=float(d.get("max_time", 120.0)),
                   params=dict(d.get("params", {}) or {}), wind=list(d.get("wind", [0, 0, 0]) or [0, 0, 0]),
                   abort=dict(d.get("abort", {}) or {}), description=str(d.get("description", "")))

    def to_dict(self) -> dict:
        return {"name": self.name, "description": self.description, "max_time": self.max_time, "params": self.params,
                "wind": self.wind, "abort": self.abort, "phases": self.phases}


def load_scenario(spec: str | Path | dict) -> Scenario:
    if isinstance(spec, dict):
        return Scenario.from_dict(spec)
    p = Path(spec)
    if not p.is_file():
        alt = SCENARIO_DIR / (p.name if p.name.endswith(".json") else p.name + ".json")
        if alt.is_file():
            p = alt
        else:
            raise FileNotFoundError(f"scenario not found: {spec}")
    with open(p) as f:
        return Scenario.from_dict(json.load(f))


class ScenarioRunner:
    """A simulator hook that executes the phases and fills a MetricsRecorder."""

    def __init__(self, scenario: Scenario, link, log=None, metrics: MetricsRecorder | None = None):
        self.sc = scenario
        self.link = link
        self.log = log or (lambda s: None)
        self.metrics = metrics or MetricsRecorder()
        self.index = -1
        self.phase: dict | None = None
        self.phase_t0 = 0.0
        self.done = False
        self.ok = True
        self.status = "running"
        self.failures: list[str] = []
        self.targets: dict[str, dict] = {}
        self._state: dict[str, Any] = {}
        self._rest_alt = None
        self._airborne_once = False
        self._started = False
        self._wind_applied = False
        self.max_time = scenario.max_time

    # ------------------------------------------------------------ helpers
    def _name(self) -> str:
        return self.phase.get("name") or self.phase.get("type", "phase")

    def _elapsed(self, simr) -> float:
        return simr.t - self.phase_t0

    def _next(self, simr) -> None:
        self.index += 1
        if self.index >= len(self.sc.phases):
            self.finish(simr, "done", self.ok)
            return
        self.phase = dict(self.sc.phases[self.index])
        self.phase_t0 = simr.t
        self._state = {}
        self.metrics.begin_phase(self._name(), simr.t)
        self.log(f"[scenario] t={simr.t:.1f}s phase {self.index + 1}/{len(self.sc.phases)}: {self._name()}")

    def fail(self, simr, why: str, fatal: bool = False) -> None:
        self.failures.append(f"{self._name() if self.phase else 'init'}: {why}")
        self.ok = False
        self.log(f"[scenario] t={simr.t:.1f}s FAIL {why}")
        if fatal:
            self.finish(simr, "aborted", False)
        else:
            self._next(simr)

    def finish(self, simr, status: str, ok: bool) -> None:
        if self.done:
            return
        self.metrics.end_phase(simr.t)
        self.done = True
        self.ok = ok and self.ok
        self.status = status
        self.log(f"[scenario] t={simr.t:.1f}s finished: {status} ({'ok' if self.ok else 'failed'})")

    # ---------------------------------------------------------------- hook
    def __call__(self, simr) -> None:
        if self.done:
            return
        s = simr.sim
        if not self._started:
            self._started = True
            self._rest_alt = -float(s.pos[2])
            if any(abs(v) > 0 for v in self.sc.wind):
                simr.sim.wind_ned = np.array(self.sc.wind, float)
            self._next(simr)
            if self.done:
                return
        self.metrics(simr)
        # global abort checks
        ab = self.sc.abort
        airborne = not s.on_ground
        if airborne:
            self._airborne_once = True
        if simr.t > self.max_time:
            self.fail(simr, f"scenario exceeded max_time {self.max_time}s", fatal=True); return
        if self._airborne_once and s.tilt_deg > float(ab.get("max_tilt_deg", 110.0)):
            self.metrics.crashed = True; self.metrics.crash_reason = f"tilt {s.tilt_deg:.0f} deg"
            self.fail(simr, "crashed: tilt " + f"{s.tilt_deg:.0f} deg", fatal=True); return
        if -float(s.pos[2]) > float(ab.get("max_alt", 500.0)):
            self.fail(simr, "flew above max_alt", fatal=True); return
        if self._airborne_once and s.on_ground and float(np.linalg.norm(s.vel)) > float(ab.get("crash_speed", 3.0)):
            self.metrics.crashed = True; self.metrics.crash_reason = f"hit the ground at {np.linalg.norm(s.vel):.1f} m/s"
            self.fail(simr, self.metrics.crash_reason, fatal=True); return
        link = self.link
        if link is not None and link.main_mode == 10 and self._airborne_once:
            self.metrics.crashed = True; self.metrics.crash_reason = "PX4 flight termination"
            self.fail(simr, "PX4 flight termination", fatal=True); return
        handler = getattr(self, "_p_" + self.phase.get("type", ""), None)
        if handler is None:
            self.fail(simr, f"unknown phase type '{self.phase.get('type')}'"); return
        handler(simr)

    # ---------------------------------------------------------------- phases
    def _p_wait(self, simr) -> None:
        if self._elapsed(simr) >= float(self.phase.get("duration", 1.0)):
            self._next(simr)

    def _p_wait_ready(self, simr) -> None:
        link = self.link
        can = link.can_arm_modes() if link is not None else ""
        ready = link is not None and link.ctl_connected and ("takeoff" in can or "loiter" in can or "posctl" in can)
        if ready:
            self.log(f"[scenario] PX4 ready after {simr.t:.1f}s sim time (can arm: {can.replace('|', ', ')})")
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 60.0)):
            self.fail(simr, f"PX4 never became ready to arm (last summary: '{can}')", fatal=True)

    def _p_takeoff(self, simr) -> None:
        st, link = self._state, self.link
        el = self._elapsed(simr)
        alt = -float(simr.sim.pos[2]) - (self._rest_alt or 0.0)
        target = float(self.phase.get("alt", 2.5))
        self.targets[self._name()] = {"alt": target + (self._rest_alt or 0.0)}
        if "sent" not in st:
            if not link.armed:
                link.arm()
            st["sent"] = simr.t
            st["mode_at"] = None
            return
        if st["mode_at"] is None and link.armed and simr.t - st["sent"] > 0.2:
            link.set_mode("takeoff"); st["mode_at"] = simr.t
        elif st["mode_at"] is None and simr.t - st["sent"] > 3.0 and not link.armed:
            link.arm(); st["sent"] = simr.t; st["arm_retries"] = st.get("arm_retries", 0) + 1
            if st["arm_retries"] > 5:
                self.fail(simr, "PX4 refused to arm", fatal=True)
            return
        if st["mode_at"] is not None and not link.mode_is("takeoff") and not link.mode_is("hold") and simr.t - st["mode_at"] > 2.0 and "retry" not in st:
            link.set_mode("takeoff"); st["retry"] = True
        reached = alt >= 0.9 * target and abs(float(simr.sim.vel[2])) < 0.3
        if reached or (link.mode_is("hold") and alt > 0.5 * target):
            if "reached" not in st:
                st["reached"] = simr.t
                self.metrics.note(simr.t, f"takeoff altitude reached after {simr.t - self.phase_t0:.1f}s")
                self.metrics.phases[self._name()]["time_to_alt"] = round(simr.t - self.phase_t0, 2)
            if simr.t - st["reached"] >= float(self.phase.get("settle", 2.0)):
                self._next(simr)
        elif el > float(self.phase.get("timeout", 30.0)):
            self.fail(simr, f"takeoff did not reach {target} m in {el:.0f}s (alt {alt:.2f} m, armed={link.armed}, mode={link.custom_mode >> 16 & 0xFF})", fatal=bool(self.phase.get("fatal", True)))

    def _p_hold(self, simr) -> None:
        st, link = self._state, self.link
        if "sent" not in st:
            link.set_mode("hold"); st["sent"] = simr.t
            st["pos0"] = simr.sim.pos.copy()
            self.targets[self._name()] = {"pos": st["pos0"].tolist()}
        if self._elapsed(simr) >= float(self.phase.get("duration", 10.0)):
            self._next(simr)

    def _p_land(self, simr) -> None:
        st, link = self._state, self.link
        if "sent" not in st:
            link.set_mode("land"); st["sent"] = simr.t
        landed = simr.sim.on_ground and not link.armed
        if landed:
            self.metrics.note(simr.t, "landed and disarmed")
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 60.0)):
            self.fail(simr, "did not land in time")

    def _offboard_stream(self, simr, pos=None, vel=None, yaw=0.0) -> None:
        st = self._state
        every = max(1, int(round(simr.sensor_rate / 20.0)))     # 20 Hz setpoints
        if simr.step_count % every == 0:
            self.link.send_setpoint_local(int(simr.t * 1000), pos=pos, vel=vel, yaw=yaw)
        if "mode_at" not in st and self._elapsed(simr) > 0.5:
            self.link.set_mode("offboard"); st["mode_at"] = simr.t
        elif "mode_at" in st and not self.link.mode_is("offboard") and simr.t - st["mode_at"] > 1.5:
            self.link.set_mode("offboard"); st["mode_at"] = simr.t
            st["mode_retries"] = st.get("mode_retries", 0) + 1
            if st["mode_retries"] > 6:
                self.fail(simr, "PX4 did not accept offboard mode")

    def _p_offboard_velocity(self, simr) -> None:
        vel = [float(v) for v in self.phase.get("vel", [5, 0, 0])]
        yaw = self.phase.get("yaw", 0.0)
        self.targets[self._name()] = {"vel": vel}
        self._offboard_stream(simr, vel=vel, yaw=None if yaw is None else math.radians(float(yaw)))
        if self._elapsed(simr) >= float(self.phase.get("duration", 10.0)):
            self._next(simr)

    def _p_wing_velocity(self, simr) -> None:
        """Experimental SITL body-vector controller; ideal state feedback, explicit trim table."""
        from .wing_controller import WingVelocityController
        if self.link.mode != "sitl":
            self.fail(simr, "Experimental wing controller is SITL-only", fatal=True); return
        if not hasattr(self, "_wing_controller"):
            try:
                self._wing_controller = WingVelocityController(simr, self.phase["trim_table"])
            except (KeyError, ValueError) as exc:
                self.fail(simr, f"Invalid experimental controller configuration: {exc}", fatal=True); return
        target = float(self.phase.get("speed", 0.0))
        self.targets[self._name()] = {"vel": [target, 0., 0.]}
        if simr.step_count % max(1, round(simr.sensor_rate / 20)) == 0:
            self._wing_controller.step(simr, target, float(self.phase.get("ramp", 1.0)))
        st = self._state
        if self._elapsed(simr) > .5 and ("mode_at" not in st or
                (not self.link.mode_is("offboard") and simr.t-st["mode_at"] > 1.5)):
            self.link.set_mode("offboard"); st["mode_at"] = simr.t
        if self._elapsed(simr) >= float(self.phase.get("duration", 30)):
            self._next(simr)

    def _p_offboard_position(self, simr) -> None:
        pos = [float(v) for v in self.phase.get("pos", [0, 0, -5])]
        yaw = self.phase.get("yaw", 0.0)
        self.targets[self._name()] = {"pos": pos}
        self._offboard_stream(simr, pos=pos, yaw=None if yaw is None else math.radians(float(yaw)))
        err = float(np.linalg.norm(simr.sim.pos - np.array(pos)))
        st = self._state
        if err < float(self.phase.get("tolerance", 1.0)) and float(np.linalg.norm(simr.sim.vel)) < float(self.phase.get("speed_tolerance", 0.5)):
            st.setdefault("reached", simr.t)
            if simr.t - st["reached"] >= float(self.phase.get("settle", 1.0)):
                self.metrics.phases[self._name()]["time_to_target"] = round(simr.t - self.phase_t0, 2)
                self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 30.0)):
            self.fail(simr, f"did not reach {pos} (error {err:.1f} m)")

    def _p_manual(self, simr) -> None:
        st, link = self._state, self.link
        every = max(1, int(round(simr.sensor_rate / 50.0)))
        if simr.step_count % every == 0:
            link.send_manual_control(float(self.phase.get("roll", 0)), float(self.phase.get("pitch", 0)),
                                     float(self.phase.get("throttle", 0.5)), float(self.phase.get("yaw", 0)))
        if "mode_at" not in st and self._elapsed(simr) > 0.3:
            link.set_mode(self.phase.get("mode", "position")); st["mode_at"] = simr.t
        if self._elapsed(simr) >= float(self.phase.get("duration", 5.0)):
            self._next(simr)

    def _p_wind(self, simr) -> None:
        simr.sim.wind_ned = np.array(self.phase.get("ned", [0, 0, 0]), float)
        self.metrics.note(simr.t, f"wind set to {simr.sim.wind_ned.tolist()}")
        self._next(simr)

    def _p_motor_failure(self, simr) -> None:
        scales = simr.sim.rotors.scale.copy()
        i = int(self.phase.get("rotor", 0))
        if 0 <= i < len(scales):
            scales[i] = float(self.phase.get("scale", 0.0))
        simr.sim.set_rotor_health(scales)
        self.metrics.note(simr.t, f"rotor {i + 1} thrust scaled to {self.phase.get('scale', 0.0)}")
        self._next(simr)

    def _p_param(self, simr) -> None:
        st, link = self._state, self.link
        name, value = str(self.phase.get("name")), self.phase.get("value")
        if "sent" not in st:
            if not link.param_set_nowait(name, value):
                self.fail(simr, f"parameter {name} unknown to the link (not downloaded)"); return
            st["sent"] = simr.t
            return
        got = link.params.get(name, {}).get("value")
        if got is not None and abs(float(got) - float(value)) <= 1e-4 * max(1.0, abs(float(value))):
            self.metrics.note(simr.t, f"{name} = {value}")
            self._next(simr)
        elif self._elapsed(simr) > 3.0:
            self.fail(simr, f"no echo for parameter {name}")

    def _p_nose_lift(self, simr) -> None:
        """Raise the nose with the given motors (0-based) to target_pitch_deg before arming; the following takeoff
        phase arms PX4 and the sequence hands over by itself."""
        st = self._state
        if "nl" not in st:
            motors = self.phase.get("motors") or []
            kw = {k: self.phase[k] for k in ("rate_deg_s", "kp", "ki", "kd", "max_cmd", "tolerance_deg", "hold_s", "fade_s", "k_ang", "kq", "kqi", "assist_motors", "assist_cmd") if k in self.phase}
            st["nl"] = simr.start_nose_lift(motors, float(self.phase.get("target_pitch_deg", simr.airframe.hover_pitch_deg)), **kw)
            self.targets[self._name()] = {}
            return
        nl = st["nl"]
        if nl.state == "holding":
            self.metrics.note(simr.t, f"nose at {nl.pitch:.1f} deg after {simr.t - self.phase_t0:.1f}s")
            self.metrics.phases[self._name()]["time_to_pitch"] = round(simr.t - self.phase_t0, 2)
            self.metrics.phases[self._name()]["max_cmd"] = round(max((c for _, _, c in nl.history), default=0.0), 3)
            self.metrics.phases[self._name()]["overshoot_deg"] = round(max((p for _, p, _ in nl.history), default=nl.target) - nl.target, 2)
            self.metrics.phases[self._name()]["start_pitch_deg"] = round(nl.history[0][1], 2) if nl.history else None
            self._next(simr)
        elif nl.state == "failed":
            self.fail(simr, f"nose lift failed: {nl.reason}", fatal=bool(self.phase.get("fatal", True)))
        elif self._elapsed(simr) > float(self.phase.get("timeout", 30.0)):
            self.fail(simr, f"nose lift did not reach the target (at {nl.pitch:.1f} deg)", fatal=True)

    def _p_arm(self, simr) -> None:
        st, link = self._state, self.link
        if "sent" not in st:
            link.arm(force=bool(self.phase.get("force", False))); st["sent"] = simr.t
        if link.armed:
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 5.0)):
            self.fail(simr, "did not arm")

    def _p_mode(self, simr) -> None:
        st, link = self._state, self.link
        mode = self.phase.get("mode", "hold")
        if "sent" not in st:
            link.set_mode(mode); st["sent"] = simr.t
        if link.mode_is(mode):
            self._next(simr)
        elif self._elapsed(simr) > float(self.phase.get("timeout", 3.0)):
            self.fail(simr, f"mode {mode} not accepted")

    # ---------------------------------------------------------------- result
    def result(self, mass: float) -> dict:
        m = self.metrics.summary(mass, self.targets)
        return {"ok": self.ok and not self.metrics.crashed, "status": self.status, "failures": self.failures,
                "sim_time": round(self.metrics.rows[-1][0], 2) if self.metrics.rows else 0.0, "metrics": m}
