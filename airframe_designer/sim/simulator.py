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

from ..dynamics.quaternion import q_from_euler, q_normalize
from ..geometry.airframe import Airframe
from ..sensors import SensorSuite, Home
from .recorder import Recorder
from .wind import WindModel


class Simulator:
    def __init__(self, airframe: Airframe, link=None, sensor_rate: float = 250.0, physics_substeps: int = 4,
                 gps_rate: float = 10.0, speed: float = 1.0, lockstep: bool | None = None,
                 home: Home | None = None, log: Callable[[str], None] | None = None, seed: int = 1):
        self.link = link
        self.log = log or (lambda s: print(s, flush=True))
        self.lock = threading.RLock()
        self.airframe = airframe
        self.home = home or Home()
        self.sim = self._make_body(airframe)
        self.sensors = SensorSuite(home=home, seed=seed)
        self.sensor_rate = float(sensor_rate)
        self.substeps = max(1, int(physics_substeps))
        self.gps_every = max(1, int(round(sensor_rate / gps_rate)))
        self.state_every = max(1, int(round(sensor_rate / 50.0)))
        self.flow_every = max(1, int(round(sensor_rate / 20.0)))   # the H-flow sensor's frame rate
        self.wind = WindModel()
        self._last_flow: dict = {}       # the last H-flow reading, for the recorder
        self._tele_t: float | None = None    # wall clock of the last telemetry sample (real flight)
        self._flow_quiet: float | None = None   # when a real flight started without any optical flow arriving
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
        self.energy_wh = 0.0            # ideal electrical energy drawn since the last reset (battery gauge)
        self.power_avg_w = 0.0          # ~5 s moving average of the power, for the minutes-left estimate
        self._last_diverge_log = 0.0
        self.motor_override: list[float] | None = None
        self.nose_lift = None          # sim.nose_lift.NoseLift while a ground sequence runs
        self.nose_lift_last = None     # status of the last finished sequence (for the UI)
        self.hooks: list[Callable[["Simulator"], None]] = []
        # The flight recorder. Always on: a run you did not think to record is the one you want back.
        self.recorder = Recorder()
        self.link_seq = 0
        self._last_send_err = 0.0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._wall_start = time.perf_counter()
        self._sim_start = 0
        self._last_hb = 0.0
        self._rtf_t0, self._rtf_sim0 = time.perf_counter(), 0

    # ------------------------------------------------------------ the physics engine
    # There is one, JSBSim, and it is not selectable. An engine you can switch is an engine you have to ask
    # which one produced a result, and the answer has to be right in every log, every saved run and every
    # comparison between two designs. One engine removes that whole question.
    physics = "jsbsim"

    def _make_body(self, airframe: Airframe):
        from ..dynamics.jsbsim_backend import JSBSimBody
        return JSBSimBody(airframe, home_alt_m=self.home.alt, lat_deg=self.home.lat,
                          lon_deg=self.home.lon, log=self.log)

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

    def _account_energy(self, dt: float) -> None:
        """Integrate the ideal motor power of the last physics step (battery gauge; see energy_wh in the snapshot)."""
        pw = float((getattr(self.sim, "breakdown", None) or {}).get("power", 0.0) or 0.0)
        self.energy_wh += pw * dt / 3600.0
        self.power_avg_w += (pw - self.power_avg_w) * min(1.0, dt / 5.0)

    def reset(self, yaw: float = 0.0) -> None:
        """Put the aircraft back on the ground. The recording is not touched: a reset is something that
        happens *during* a session, and the graph is the session. Only saving a run clears it."""
        with self.lock:
            self.sim.reset(yaw=yaw)
            self.energy_wh = 0.0
            self.power_avg_w = 0.0
            self.motor_override = None
            self.nose_lift = None
            if self.link is not None:
                self.link.clear_actuators()

    @staticmethod
    def _reported(st: dict, key: str) -> np.ndarray | None:
        """A vector the aircraft actually reported, or None.

        PX4 publishes its local position whether or not the estimate is valid, and an invalid one is NaN.
        Copying that into the vehicle's state loses the aircraft from the 3D view, poisons the recording,
        and - because NaN is not JSON - stops every websocket frame and every status reply dead. An
        aircraft that has not said where it is simply has not said."""
        v = st.get(key)
        if v is None:
            return None
        a = np.asarray(v, float)
        return a if a.size and bool(np.all(np.isfinite(a))) else None

    def _follow_telemetry(self, link, dt: float) -> None:
        """Put the aircraft where the vehicle says it is, and record that. No physics, no sensors."""
        now = time.perf_counter()
        if self._tele_t is None:
            self._flow_quiet = time.time()        # if nothing flows in ten seconds, say so once
        elapsed = min(0.5, max(0.0, now - (self._tele_t or now)))
        self._tele_t = now
        st = dict(getattr(link, "board_state", {}) or {})
        with self.lock:
            s = self.sim
            pos, vel, q, euler, rates = (self._reported(st, k) for k in ("pos", "vel", "q", "euler", "rates"))
            if pos is not None:
                s.pos = pos
            if vel is not None:
                s.vel = vel
            if q is not None:
                s.q = q_normalize(q)
            elif euler is not None:
                s.q = q_from_euler(*[float(v) for v in euler])
            if rates is not None:
                s.rates = rates
            s.on_ground = bool(-float(s.pos[2]) < 0.4)
            s.breakdown = {}
            self.time_usec += int(round(elapsed * 1e6))
            try:
                self.recorder.sample(self.time_usec / 1e6, s, self._link_status(link),
                                     self.battery_percent(), motors=list(getattr(link, "board_motors", []) or []),
                                     extra=self._flight_extra(link, s))
            except Exception as e:
                self.log(f"[sim] recorder failed: {e}")
        self._idle()

    def _flight_extra(self, link, s) -> dict:
        """What the aircraft's *sensors* report, for the columns the simulated run fills the same way.

        Without this the H-flow columns of a real flight are flat zero, which reads as a sensor that sees
        nothing rather than one nobody asked about. The rate conversion is the one in sensors/models.py,
        so flow_x in a recording means the same thing whether the flight happened here or outside.
        """
        now = time.time()
        fresh = lambda d, age=2.0: bool(d) and now - d.get("t", 0.0) < age      # noqa: E731
        flow, rng, alt = getattr(link, "board_flow", None), getattr(link, "board_range", None), getattr(link, "board_alt", None)
        extra: dict = {}
        if fresh(flow):
            extra["flow_rate"] = flow["rate"]
            extra["flow_quality"] = flow["quality"]
            if flow.get("range", -1.0) >= 0:
                extra["flow_range"] = flow["range"]
        if "flow_range" not in extra and fresh(rng):
            extra["flow_range"] = rng["m"]
        extra["alt_baro"] = alt["relative"] if fresh(alt, 5.0) else float(-s.pos[2])
        if not fresh(flow) and self._flow_quiet is not None and now - self._flow_quiet > 10.0:
            self._flow_quiet = None
            self.log("[link] the aircraft is not sending optical flow. Either no flow sensor is running on it "
                     "(nothing publishes vehicle_optical_flow), or this firmware has no OPTICAL_FLOW_RAD stream.")
        elif fresh(flow) and self._flow_quiet is not None:
            self._flow_quiet = None
        return extra

    @staticmethod
    def _link_status(link) -> dict:
        """What the recorder wants to know about the flight controller: armed, and which mode it is in.

        PX4 packs the mode into its heartbeat; the number kept here is the main mode, with the Auto sub-mode
        behind the decimal point (4.03 is Auto/Hold), so one column plots as a readable staircase."""
        custom = int(getattr(link, "custom_mode", 0) or 0)
        main, sub = (custom >> 16) & 0xFF, (custom >> 24) & 0xFF
        return {"armed": bool(getattr(link, "armed", False)),
                "mode": main + (sub / 100.0 if main == 4 else 0.0)}

    def battery_percent(self) -> float:
        """Charge left, as the gauge shows it. 100 when no capacity is configured."""
        cap = float((getattr(self.airframe, "design", None) or {}).get("battery_wh", 0) or 0)
        if cap <= 0:
            return 100.0
        return max(0.0, min(100.0, 100.0 * (cap - self.energy_wh) / cap))

    def set_airframe(self, airframe: Airframe) -> None:
        """Load an edited airframe. JSBSim rebuilds its model, so the vehicle restarts on the ground."""
        with self.lock:
            self.airframe = airframe
            self.sim = self._make_body(airframe)
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
        """A steady wind from a vector. Any gusts stop: the wind has one owner, and this is it."""
        self.set_wind_model(WindModel.from_vector(north, east, down))

    def set_wind_model(self, model: WindModel) -> WindModel:
        with self.lock:
            self.wind = model
            self.sim.wind_ned = model.at(self.time_usec / 1e6)
            return self.wind

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
        if getattr(link, "listening", False):
            # A real flight: there is nothing to simulate. The aircraft is the truth, so the 3D view and the
            # recording follow its telemetry and no sensor is ever sent to it.
            self._follow_telemetry(link, dt)
            return True
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
            self.sim.wind_ned = self.wind.at(self.time_usec / 1e6)
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
            self._account_energy(dt)
            t_us = self.time_usec
            if self.nose_lift is not None:
                try:
                    self.nose_lift(self)
                except Exception as e:
                    self.log(f"[sim] nose lift failed: {e}"); self.nose_lift = None
            try:
                self.recorder.sample(self.time_usec / 1e6, self.sim, self._link_status(link),
                                     self.battery_percent(), extra=self._last_flow)
            except Exception as e:
                self.log(f"[sim] recorder failed: {e}")
            for h in list(self.hooks):
                try:
                    h(self)
                except Exception as e:
                    self.log(f"[sim] hook {getattr(h, '__name__', type(h).__name__)} failed: {e}")
            sensor = self.sensors.hil_sensor(self.sim, t_us)
            gps = self.sensors.hil_gps(self.sim, t_us) if self.step_count % self.gps_every == 0 else None
            send_state = link.mode == "sitl" and self.step_count % self.state_every == 0
            state = self.sensors.hil_state_quaternion(self.sim, t_us) if send_state else None
            flow = dist = None
            if self.step_count % self.flow_every == 0:
                flow = self.sensors.hil_optical_flow(self.sim, t_us, self.flow_every * dt)
                dist = self.sensors.distance_sensor(self.sim, t_us)
                if flow is not None:
                    # what the sensor read, for the graph: the rate it saw and how far the ground was
                    span = max(1e-6, flow["integration_time_us"] * 1e-6)
                    self._last_flow = {"flow_rate": (-flow["integrated_x"] / span, -flow["integrated_y"] / span),
                                       "flow_range": flow["distance"], "flow_quality": flow["quality"],
                                       "alt_baro": sensor["pressure_alt"] - self.sensors.home.alt}
                else:
                    self._last_flow = {"alt_baro": sensor["pressure_alt"] - self.sensors.home.alt}
        self.step_count += 1
        seq_before = link.actuator_seq
        try:
            if gps is not None:
                link.send_hil_gps(gps)
            if state is not None:
                link.send_hil_state_quaternion(state)
            if flow is not None:
                link.send_hil_optical_flow(flow)
                link.send_distance_sensor(dist)
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
                "energy_wh": round(self.energy_wh, 4), "power_avg_w": round(self.power_avg_w, 1),
                "motor_override": self.motor_override, "wind": s.wind_ned.tolist(),
                "rotor_health": s.rotors.scale.tolist(),
                "physics": self.physics,
                "nose_lift": self.nose_lift.status() if self.nose_lift is not None else self.nose_lift_last,
            }
