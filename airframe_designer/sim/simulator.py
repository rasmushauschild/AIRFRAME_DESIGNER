"""Simulation loop: physics + sensors -> PX4 over the HIL link, actuators back.

SITL (lockstep): each HIL_SENSOR advances PX4's clock; the loop waits for the HIL_ACTUATOR_CONTROLS reply
before stepping again, then paces to the wall clock (speed 1 = real time, 0 = as fast as PX4 allows).
HITL (real time): the Pixhawk runs on its own clock; sensors stream at the configured rate.

Hooks: callables invoked once per sensor step with the simulator (after the physics, before the sensors are
sent). Scenario runners and metric recorders are hooks, so scripted flights are deterministic in simulation
time and work identically in the UI, headless and in batch.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

import numpy as np

from ..geometry.airframe import Airframe
from ..dynamics import RigidBody
from ..sensors import SensorSuite, Home


class Simulator:
    def __init__(self, airframe: Airframe, link=None, sensor_rate: float = 250.0, physics_substeps: int = 4,
                 gps_rate: float = 10.0, speed: float = 1.0, lockstep: bool | None = None,
                 home: Home | None = None, log: Callable[[str], None] | None = None, seed: int = 1,
                 physics: str = "python"):
        self.link = link
        self.log = log or (lambda s: print(s, flush=True))
        self.lock = threading.RLock()
        self.airframe = airframe
        self.physics = "python"
        self.home = home or Home()
        self.sim = self._make_body(airframe, physics)
        self.sensors = SensorSuite(home=home, seed=seed)
        self.sensor_rate = float(sensor_rate)
        self.substeps = max(1, int(physics_substeps))
        self.gps_every = max(1, int(round(sensor_rate / gps_rate)))
        self.state_every = max(1, int(round(sensor_rate / 50.0)))
        self.speed = speed
        self.lockstep = (link is not None and link.mode == "sitl") if lockstep is None else bool(lockstep)
        self.time_usec = 0
        self.paused = False
        self.running = False
        self.step_count = 0
        self.lockstep_timeouts = 0
        self._silent_steps = 0          # consecutive lockstep waits without a HIL_ACTUATOR_CONTROLS reply
        self._last_revive = 0.0
        self.real_time_factor = 0.0
        self.diverged = 0
        self._last_diverge_log = 0.0
        self.motor_override: list[float] | None = None
        self.nose_lift = None          # sim.nose_lift.NoseLift while a ground sequence runs
        self.nose_lift_last = None     # status of the last finished sequence (for the UI)
        self.hooks: list[Callable[["Simulator"], None]] = []
        self.link_seq = 0
        self._last_send_err = 0.0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._wall_start = time.perf_counter()
        self._sim_start = 0
        self._last_hb = 0.0
        self._rtf_t0, self._rtf_sim0 = time.perf_counter(), 0

    # ------------------------------------------------------------ backends
    BACKENDS = ("python", "jsbsim")

    def _make_body(self, airframe: Airframe, physics: str):
        physics = (physics or "python").lower()
        if physics == "jsbsim":
            from ..dynamics.jsbsim_backend import JSBSimBody
            body = JSBSimBody(airframe, home_alt_m=self.home.alt, lat_deg=self.home.lat, lon_deg=self.home.lon, log=self.log)
        elif physics == "python":
            body = RigidBody(airframe)
        else:
            raise ValueError(f"unknown physics backend '{physics}' (python, jsbsim)")
        self.physics = physics
        return body

    def set_physics(self, physics: str) -> None:
        """Swap the physics engine under the running loop (the vehicle restarts on the ground)."""
        with self.lock:
            wind = self.sim.wind_ned.copy()
            self.sim = self._make_body(self.airframe, physics)
            self.sim.wind_ned = wind
            self.sim.reset()
            self.nose_lift = None
            if self.link is not None:
                self.link.clear_actuators()
            self.log(f"[sim] physics backend: {self.physics}")

    # ------------------------------------------------------------ control
    @property
    def t(self) -> float:
        return self.time_usec / 1e6

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def reset(self, yaw: float = 0.0) -> None:
        with self.lock:
            self.sim.reset(yaw=yaw)
            self.motor_override = None
            self.nose_lift = None
            if self.link is not None:
                self.link.clear_actuators()

    def set_airframe(self, airframe: Airframe, keep_state: bool = True) -> None:
        with self.lock:
            self.airframe = airframe
            if self.physics == "jsbsim":
                # JSBSim reloads its model; the vehicle restarts on the ground
                self.sim = self._make_body(airframe, "jsbsim"); self.sim.reset()
            else:
                self.sim.set_airframe(airframe)
                if not keep_state:
                    self.sim.reset()

    def set_link(self, link, lockstep: bool) -> None:
        with self.lock:
            self.link = link
            if link is not None:
                link.paused_hint = lambda: self.paused
            self.lockstep = lockstep
            self.link_seq += 1
            self.lockstep_timeouts = 0
            if link is not None:
                self.sim.reset()

    def set_wind(self, north: float, east: float, down: float = 0.0) -> None:
        with self.lock:
            self.sim.wind_ned = np.array([north, east, down], dtype=float)

    def start_nose_lift(self, motors, target_pitch_deg: float, **kw):
        from .nose_lift import NoseLift
        with self.lock:
            self.nose_lift = NoseLift(motors, target_pitch_deg, **kw)
            self.log(f"[sim] nose lift: motors {[m + 1 for m in self.nose_lift.motors]} to {target_pitch_deg:g} deg at {self.nose_lift.rate:g} deg/s")
            return self.nose_lift

    def stop_nose_lift(self) -> None:
        with self.lock:
            self.nose_lift = None

    def set_rotor_health(self, scales) -> None:
        with self.lock:
            self.sim.set_rotor_health(scales)

    # --------------------------------------------------------------- loop
    def _idle(self) -> None:
        """Nothing to do until PX4 has connected (SITL) or while paused."""
        time.sleep(0.02)
        self._wall_start = time.perf_counter()
        self._sim_start = self.time_usec

    def step_once(self) -> bool:
        """One sensor step: actuators -> physics -> hooks -> sensors -> PX4 (-> lockstep wait -> pacing).
        Returns False when the loop had nothing to do (no link / paused / PX4 not connected)."""
        now = time.perf_counter()
        link = self.link
        if link is not None and now - self._last_hb > 1.0:
            try:
                link.send_heartbeat()
            except Exception:
                pass
            self._last_hb = now
        # SITL: wait until PX4 has connected once. After that keep sending even when the link looks silent (e.g.
        # after a pause): PX4's simulator module only answers once sensor data arrives, so waiting for it first
        # would be a standoff. A dead socket surfaces as a send error below and is handled there.
        if link is None or self.paused or (link.mode == "sitl" and not link.connected and link.rx_count == 0):
            self._idle()
            return False
        dt = 1.0 / self.sensor_rate
        sub_dt = dt / self.substeps
        with self.lock:
            cmd = self.motor_override if self.motor_override is not None else link.actuators
            nl = self.nose_lift
            if nl is not None:
                fl = nl.floor(self.sim.rotors.n)
                if fl is None:
                    self.nose_lift_last = nl.status()
                    self.log(f"[sim] nose lift {nl.state}: {nl.reason}")
                    self.nose_lift = None
                else:
                    cmd = np.maximum(np.asarray(cmd, float)[: self.sim.rotors.n], fl)   # the sequence holds a floor under PX4
            self.sim.set_motor_commands(cmd)
            try:
                for k in range(self.substeps):
                    self.sim.step(sub_dt, detail=(k == self.substeps - 1))
                if not self.sim.is_sane():
                    raise FloatingPointError("state is not finite or physically absurd")
            except Exception as e:
                # A diverging integration (typically an inertia far too small for the mass and size, or a leg
                # spring far too stiff for the mass) must never kill the loop: put the vehicle back on the
                # ground, keep streaming sensors so PX4 stays alive, and say what happened.
                self.diverged += 1
                if time.time() - self._last_diverge_log > 2.0:
                    self._last_diverge_log = time.time()
                    self.log(f"[sim] PHYSICS DIVERGED ({e}); vehicle reset. Check mass vs inertia (Estimate on the "
                             f"Geometry tab), leg stiffness and rotor thrust: this airframe is not integrable at "
                             f"{self.sensor_rate * self.substeps:.0f} Hz")
                self.sim.reset()
                link.clear_actuators()
            self.time_usec += int(round(dt * 1e6))
            t_us = self.time_usec
            if self.nose_lift is not None:
                try:
                    self.nose_lift(self)
                except Exception as e:
                    self.log(f"[sim] nose lift failed: {e}"); self.nose_lift = None
            for h in list(self.hooks):
                try:
                    h(self)
                except Exception as e:
                    self.log(f"[sim] hook {getattr(h, '__name__', type(h).__name__)} failed: {e}")
            sensor = self.sensors.hil_sensor(self.sim, t_us)
            gps = self.sensors.hil_gps(self.sim, t_us) if self.step_count % self.gps_every == 0 else None
            send_state = link.mode == "sitl" and self.step_count % self.state_every == 0
            state = self.sensors.hil_state_quaternion(self.sim, t_us) if send_state else None
        self.step_count += 1
        seq_before = link.actuator_seq
        try:
            if gps is not None:
                link.send_hil_gps(gps)
            if state is not None:
                link.send_hil_state_quaternion(state)
            link.send_hil_sensor(sensor)
        except Exception as e:
            if time.time() - self._last_send_err > 3.0:
                self.log(f"[sim] send failed: {e}")
                self._last_send_err = time.time()
            time.sleep(0.1)
            return False
        if self.lockstep:
            if not link.wait_for_actuators(seq_before, timeout=0.1):
                self.lockstep_timeouts += 1
                self._silent_steps += 1
                # PX4 left in Offboard direct-actuator mode after a disarm has no motor-command publisher, so its output
                # driver goes quiet and it never answers our sensors again ("simulator link lost" after 5 s, PX4 alive).
                # A mode change to Hold restarts the allocator and the stream; the firmware does this itself, this is
                # the simulator-side backstop.
                if (self._silent_steps * dt >= 1.0 and getattr(link, "mode", "") == "sitl" and link.ctl_connected
                        and not link.armed and link.main_mode == 6 and time.time() - self._last_revive > 3.0):
                    self._last_revive = time.time()
                    self.log("[sim] PX4 stopped answering while disarmed in Offboard (no motor-command publisher); "
                             "requesting Hold to keep the lockstep link alive")
                    try:
                        link.set_mode("hold")
                    except Exception as e:
                        self.log(f"[sim] could not request Hold: {e}")
            else:
                self._silent_steps = 0
        if self.speed > 0:
            target = self._wall_start + (self.time_usec - self._sim_start) / 1e6 / self.speed
            remaining = target - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            elif remaining < -0.5:
                self._wall_start = time.perf_counter()
                self._sim_start = self.time_usec
        if time.perf_counter() - self._rtf_t0 > 1.0:
            self.real_time_factor = ((self.time_usec - self._rtf_sim0) / 1e6) / (time.perf_counter() - self._rtf_t0)
            self._rtf_t0, self._rtf_sim0 = time.perf_counter(), self.time_usec
        return True

    def _run(self) -> None:
        self.running = True
        self._wall_start = time.perf_counter()
        self._sim_start = self.time_usec
        self.log(f"[sim] loop started: {self.sensor_rate:.0f} Hz sensors, physics {self.sensor_rate * self.substeps:.0f} Hz")
        while not self._stop.is_set():
            try:
                self.step_once()
            except Exception as e:
                self.log(f"[sim] loop error: {type(e).__name__}: {e}")
                time.sleep(0.05)
        self.running = False
        self.log("[sim] loop stopped")

    def run_sync(self, until: Callable[["Simulator"], bool] | None = None, max_sim_time: float | None = None,
                 max_wall_time: float | None = None) -> str:
        """Drive the loop in the calling thread (headless runs). Returns why it stopped."""
        self.running = True
        self._wall_start = time.perf_counter()
        self._sim_start = self.time_usec
        t0 = time.perf_counter()
        t_sim0 = self.t
        try:
            while not self._stop.is_set():
                if until is not None and until(self):
                    return "done"
                if max_sim_time is not None and self.t - t_sim0 >= max_sim_time:
                    return "max_sim_time"
                if max_wall_time is not None and time.perf_counter() - t0 >= max_wall_time:
                    return "max_wall_time"
                self.step_once()
            return "stopped"
        finally:
            self.running = False

    # --------------------------------------------------------- UI snapshot
    def snapshot(self) -> dict:
        with self.lock:
            s = self.sim
            r, p, y = s.euler
            bd = s.breakdown or {}
            return {
                "t": self.time_usec / 1e6, "pos": s.pos.tolist(), "vel": s.vel.tolist(), "q": s.q.tolist(),
                "euler": [r, p, y], "rates": s.rates.tolist(), "accel_body": s.accel_body.tolist(),
                "on_ground": bool(s.on_ground), "feet_down": int(s.feet_down),
                "rotors": [{"cmd": float(c), "omega": float(o), "thrust": float(t)} for c, o, t in zip(s.cmd, s.omega, s.thrust)],
                "forces": {k: (float(v) if not isinstance(v, (list, bool)) else v) for k, v in bd.items()},
                "rtf": self.real_time_factor, "paused": self.paused, "lockstep_timeouts": self.lockstep_timeouts,
                "diverged": self.diverged,
                "motor_override": self.motor_override, "wind": s.wind_ned.tolist(),
                "rotor_health": s.rotors.scale.tolist(),
                "physics": self.physics,
                "nose_lift": self.nose_lift.status() if self.nose_lift is not None else self.nose_lift_last,
            }
