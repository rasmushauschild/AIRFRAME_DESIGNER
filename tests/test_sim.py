"""Metrics recorder and scenario runner, driven by hand without PX4."""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.dynamics import q_from_euler
from airframe_designer.dynamics.jsbsim_backend import JSBSimBody
from airframe_designer.geometry.airframe import quad_x
from airframe_designer.sim import MetricsRecorder, Scenario, ScenarioRunner, Simulator, load_scenario
from airframe_designer.sim.scenario import SCENARIO_DIR

from .conftest import SCENARIOS_DIR

G = 9.80665


# --------------------------------------------------------------- fakes
class FakeSim:
    """The attributes MetricsRecorder reads from a Simulator."""

    def __init__(self, sim: JSBSimBody, sensor_rate: float = 250.0):
        self.sim = sim
        self.t = 0.0
        self.step_count = 0
        self.sensor_rate = sensor_rate


class FakeLink:
    """A PX4Link stand-in: remembers what the runner asked for and answers like a cooperative vehicle."""
    MODES = {"manual": (1, 0), "altitude": (2, 0), "position": (3, 0), "acro": (5, 0), "offboard": (6, 0),
             "stabilized": (7, 0), "takeoff": (4, 2), "hold": (4, 3), "mission": (4, 4), "rtl": (4, 5), "land": (4, 6)}

    def __init__(self, ready: bool = True, arms: bool = True):
        self.armed = False
        self.custom_mode = 0
        self.ctl_connected = True
        self.connected = True
        self.ready = ready
        self.arms = arms
        self.params: dict[str, dict] = {}
        self.recent_events: list = []
        self.calls: list[tuple] = []
        self.setpoints: list[dict] = []
        self.manual: list[tuple] = []

    @property
    def main_mode(self) -> int:
        return (self.custom_mode >> 16) & 0xFF

    def can_arm_modes(self) -> str:
        return "takeoff|loiter|posctl" if self.ready else ""

    def arm(self, force: bool = False) -> None:
        self.calls.append(("arm", force))
        self.armed = self.arms

    def set_mode(self, name: str) -> None:
        main, sub = self.MODES[name]
        self.custom_mode = (main << 16) | (sub << 24)
        self.calls.append(("mode", name))

    def mode_is(self, name: str) -> bool:
        main, sub = self.MODES[name]
        return ((self.custom_mode >> 16) & 0xFF) == main and (sub == 0 or ((self.custom_mode >> 24) & 0xFF) == sub)

    def send_setpoint_local(self, time_boot_ms, pos=None, vel=None, yaw=0.0, yaw_rate=None) -> None:
        self.setpoints.append({"t": time_boot_ms, "pos": pos, "vel": vel, "yaw": yaw})

    def send_manual_control(self, roll, pitch, throttle, yaw, buttons: int = 0) -> None:
        self.manual.append((roll, pitch, throttle, yaw))

    def param_set_nowait(self, name, value) -> bool:
        if name not in self.params:
            return False
        self.params[name]["value"] = value
        return True


def make_sim(sensor_rate: float = 250.0) -> Simulator:
    """The batch defaults (250 Hz sensors, 2 physics sub-steps): the leg spring-damper needs a physics step of a
    few milliseconds to stay stable."""
    return Simulator(quad_x(), link=None, sensor_rate=sensor_rate, physics_substeps=2, speed=0.0, lockstep=False,
                     log=lambda s: None)


def advance(simr: Simulator, runner: ScenarioRunner, seconds: float, physics: bool = True, max_steps: int | None = None) -> None:
    """Drive the simulator by hand: physics, clock, hook - like Simulator.step_once without a link."""
    dt = 1.0 / simr.sensor_rate
    n = int(round(seconds / dt))
    for _ in range(n):
        if runner.done:
            return
        if physics:
            for _ in range(simr.substeps):
                simr.sim.step(dt / simr.substeps)
        simr.time_usec += int(round(dt * 1e6))
        runner(simr)
        simr.step_count += 1


# ------------------------------------------------------------- metrics
def test_metrics_recorder_rows_and_summary(quad):
    rb = JSBSimBody(quad)
    fs = FakeSim(rb, sensor_rate=250.0)
    m = MetricsRecorder(sample_hz=50.0)
    m.begin_phase("init", 0.0)
    dt = 1.0 / fs.sensor_rate
    for i in range(500):                                   # 2 s at rest on the legs
        rb.step(dt / 2); rb.step(dt / 2)
        fs.t += dt
        fs.step_count += 1
        if i == 250:
            m.begin_phase("hover", fs.t)
        m(fs)
    m.end_phase(fs.t)
    assert m._every == 5
    assert len(m.rows) == 100 and len(m.phase_of_row) == 100
    assert m.array().shape == (100, len(MetricsRecorder.COLS))
    assert m.steps == 500 and m.saturation_steps == 0
    assert m.max_tilt_deg == pytest.approx(0.0, abs=1e-3)   # JSBSim settles on its springs
    assert m.max_alt == pytest.approx(0.12, abs=0.01)
    s = m.summary(quad.mass.mass, {"hover": {"alt": 0.12}})
    assert set(s) >= {"crashed", "crash_reason", "max_tilt_deg", "max_alt_m", "energy_wh", "saturation_fraction",
                      "touchdown_speed", "events", "phases", "flight_time", "final_alt", "final_speed"}
    assert s["crashed"] is False and s["flight_time"] == 0.0 and s["touchdown_speed"] is None
    assert set(s["phases"]) == {"init", "hover"}
    ph = s["phases"]["hover"]
    for key in ("t0", "t1", "duration", "samples", "alt_mean", "alt_min", "alt_max", "alt_std", "pos_drift", "pos_std_xy",
                "speed_mean", "speed_max", "vz_std", "roll_mean_deg", "roll_rms_deg", "pitch_rms_deg", "pitch_max_deg",
                "yaw_drift_deg", "rates_rms_deg_s", "tilt_max_deg", "thrust_mean", "thrust_to_weight_mean", "power_mean",
                "lift_share_mean", "airspeed_mean", "util_max", "cmd_mean", "airborne_fraction", "alt_err_rms", "alt_err_max"):
        assert key in ph, key
    assert "power_ratio_vs_hover" not in ph                     # motors idle: no hover power to compare against
    assert "alt_err_rms" not in s["phases"]["init"]
    assert ph["alt_mean"] == pytest.approx(0.12, abs=0.01)
    assert ph["alt_err_rms"] < 0.01 and ph["pos_std_xy"] == 0.0 and ph["airborne_fraction"] == 0.0
    assert ph["thrust_mean"] == 0.0 and ph["duration"] == pytest.approx(1.0, abs=0.01)
    assert ph["samples"] == 50
    ts = m.timeseries()
    assert ts["columns"] == MetricsRecorder.COLS and len(ts["rows"]) == 100 and len(ts["phase"]) == 100


def test_metrics_recorder_tracks_tilt_saturation_and_touchdown(quad):
    rb = JSBSimBody(quad)
    fs = FakeSim(rb, sensor_rate=100.0)
    m = MetricsRecorder(sample_hz=50.0)
    m.begin_phase("air", 0.0)
    rb.reset(pos_ned=[0, 0, -0.5])
    rb.q = q_from_euler(math.radians(20.0), 0.0, 0.0)
    rb.set_motor_commands([1.0] * 4)                     # saturated
    rb.breakdown = {"power": 100.0}
    dt = 1.0 / fs.sensor_rate
    fs.t += dt; fs.step_count += 1; rb.on_ground = False
    m(fs)
    assert m.max_tilt_deg == pytest.approx(20.0, abs=1e-6)
    assert m.saturation_steps == 1
    rb.on_ground = True
    rb.vel = np.array([0.0, 0.0, 1.5])
    rb.set_motor_commands([0.0] * 4)
    fs.t += dt; fs.step_count += 1
    m(fs)
    assert m.touchdown_speed == pytest.approx(1.5)
    assert any("touchdown" in e["text"] for e in m.events)
    assert m.energy_j == pytest.approx(100.0 * dt)
    s = m.summary(quad.mass.mass)
    assert s["touchdown_speed"] == 1.5 and s["saturation_fraction"] == 0.5
    assert s["phases"]["air"]["tilt_max_deg"] == pytest.approx(20.0, abs=1e-3)


def test_metrics_summary_empty():
    m = MetricsRecorder()
    s = m.summary(1.0)
    assert s["phases"] == {} and "flight_time" not in s


def test_no_spurious_touchdown_with_single_physics_step(quad):
    rb = JSBSimBody(quad)
    fs = FakeSim(rb, sensor_rate=250.0)
    m = MetricsRecorder()
    dt = 1.0 / fs.sensor_rate
    for _ in range(50):
        rb.step(dt)
        fs.t += dt; fs.step_count += 1
        m(fs)
    assert m.touchdown_speed is None and m.events == []


# ------------------------------------------------------------ scenarios
def test_load_bundled_scenarios():
    assert SCENARIO_DIR == SCENARIOS_DIR
    for p in sorted(SCENARIOS_DIR.glob("*.json")):
        sc = load_scenario(p)
        assert sc.phases and sc.max_time > 0
        assert all("type" in ph for ph in sc.phases)
    hover = load_scenario("hover")
    assert hover.name == "hover" and hover.params == {"MIS_TAKEOFF_ALT": 3}
    assert load_scenario("hover.json").to_dict() == hover.to_dict()
    assert Scenario.from_dict(hover.to_dict()) == hover
    with pytest.raises(FileNotFoundError):
        load_scenario("no_such_scenario")


def test_wait_ready_proceeds_when_px4_can_take_off():
    simr = make_sim()
    link = FakeLink(ready=True)
    sc = Scenario(name="t", phases=[{"type": "wait_ready", "timeout": 5}, {"type": "wait", "name": "idle", "duration": 0.5}], max_time=30)
    runner = ScenarioRunner(sc, link)
    simr.hooks.append(runner)
    advance(simr, runner, 0.1, physics=False)
    assert runner.index == 1 and runner.phase["type"] == "wait"    # ready on the first step
    advance(simr, runner, 1.0, physics=False)
    assert runner.done and runner.ok and runner.status == "done"
    assert set(runner.metrics.phases) == {"wait_ready", "idle"}
    r = runner.result(simr.airframe.mass.mass)
    assert r["ok"] is True and r["failures"] == [] and r["metrics"]["crashed"] is False
    assert r["metrics"]["phases"]["idle"]["duration"] == pytest.approx(0.5, abs=0.05)


def test_wait_ready_times_out():
    simr = make_sim()
    link = FakeLink(ready=False)
    sc = Scenario(phases=[{"type": "wait_ready", "timeout": 2}, {"type": "wait", "duration": 1}], max_time=30)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 3.0, physics=False)
    assert runner.done and not runner.ok and runner.status == "aborted"
    assert runner.failures and "never became ready" in runner.failures[0]


def test_takeoff_times_out_when_altitude_never_rises():
    simr = make_sim()
    link = FakeLink()
    sc = Scenario(phases=[{"type": "takeoff", "alt": 3, "settle": 1, "timeout": 5}, {"type": "hold", "duration": 2}], max_time=60)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 7.0)                              # motors idle: the vehicle stays on its legs
    assert runner.done and runner.ok is False and runner.status == "aborted"
    assert len(runner.failures) == 1 and "takeoff did not reach 3.0 m" in runner.failures[0]
    assert ("arm", False) in link.calls and ("mode", "takeoff") in link.calls
    assert link.armed and link.mode_is("takeoff")
    assert runner.targets["takeoff"]["alt"] == pytest.approx(3.0 + 0.12, abs=0.01)
    r = runner.result(simr.airframe.mass.mass)
    assert r["ok"] is False and r["status"] == "aborted" and r["sim_time"] >= 5.0
    assert "alt_err_rms" in r["metrics"]["phases"]["takeoff"]
    assert simr.sim.on_ground


def test_takeoff_retries_arming_then_gives_up():
    simr = make_sim()
    link = FakeLink(arms=False)
    sc = Scenario(phases=[{"type": "takeoff", "alt": 3, "timeout": 60}], max_time=120)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 30.0, physics=False)
    assert runner.done and not runner.ok
    assert "refused to arm" in runner.failures[0]
    assert sum(1 for c in link.calls if c[0] == "arm") == 7


def _fly_takeoff(alt: float = 1.0) -> tuple[Simulator, ScenarioRunner]:
    """The physics does the flying: a crude altitude controller stands in for PX4's takeoff; the link is faked."""
    simr = make_sim()
    sc = Scenario(phases=[{"type": "takeoff", "alt": alt, "settle": 0.5, "timeout": 20}, {"type": "wait", "name": "after", "duration": 0.2}], max_time=60)
    runner = ScenarioRunner(sc, FakeLink())
    rb = simr.sim
    w_hover = math.sqrt(rb.mass * G / (4 * 8.0))
    dt = 1.0 / simr.sensor_rate
    for _ in range(int(20.0 / dt)):
        if runner.done:
            break
        h = -rb.pos[2] - 0.12
        w = w_hover * math.sqrt(max(0.2, 1.0 + 0.3 * (alt - h) - 0.4 * (-rb.vel[2])))
        rb.set_motor_commands([w] * 4)
        for _ in range(simr.substeps):
            rb.step(dt / simr.substeps)
        simr.time_usec += int(round(dt * 1e6))
        runner(simr)
        simr.step_count += 1
    return simr, runner


def test_takeoff_succeeds_when_the_vehicle_rises():
    simr, runner = _fly_takeoff()
    assert runner.done and runner.ok, runner.failures
    assert "time_to_alt" in runner.metrics.phases["takeoff"]
    assert any("takeoff altitude reached" in e["text"] for e in runner.metrics.events)
    r = runner.result(simr.sim.mass)
    assert r["metrics"]["phases"]["takeoff"]["airborne_fraction"] > 0.5
    assert r["metrics"]["phases"]["takeoff"]["alt_max"] == pytest.approx(1.12, abs=0.3)
    assert r["metrics"]["flight_time"] > 1.0


def test_time_to_alt_reaches_the_result_summary():
    simr, runner = _fly_takeoff()
    assert runner.done and runner.ok
    r = runner.result(simr.sim.mass)
    assert "time_to_alt" in r["metrics"]["phases"]["takeoff"]
    from airframe_designer.batch.objective import evaluate_expression
    assert evaluate_expression("phases.takeoff.time_to_alt < 12", r) is True


def test_wait_wind_and_motor_failure_phases():
    simr = make_sim()
    link = FakeLink()
    sc = Scenario(phases=[
        {"type": "wait", "name": "w1", "duration": 0.5},
        {"type": "wind", "ned": [-5, 1, 0]},
        {"type": "motor_failure", "rotor": 0, "scale": 0.0},
        {"type": "wait", "name": "w2", "duration": 0.3},
        {"type": "motor_failure", "name": "half", "rotor": 2, "scale": 0.5},
        {"type": "wind", "name": "calm", "ned": [0, 0, 0]},
    ], max_time=60)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 0.3)
    assert runner.phase["name"] == "w1" and simr.sim.wind_ned.tolist() == [0.0, 0.0, 0.0]
    advance(simr, runner, 0.3)
    # the wind is kept as a speed and a direction now (sim/wind.py), so a vector round-trips through trig
    assert simr.sim.wind_ned.tolist() == pytest.approx([-5.0, 1.0, 0.0])
    assert simr.sim.rotors.scale.tolist() == [0.0, 1.0, 1.0, 1.0]
    assert runner.phase["name"] == "w2"
    advance(simr, runner, 1.0)
    assert runner.done and runner.ok and runner.status == "done"
    assert simr.sim.rotors.scale.tolist() == [0.0, 1.0, 0.5, 1.0]
    assert simr.sim.wind_ned.tolist() == [0.0, 0.0, 0.0]
    texts = [e["text"] for e in runner.metrics.events]
    assert "wind set to [-5.0, 1.0, 0.0]" in texts
    assert "rotor 1 thrust scaled to 0.0" in texts and "rotor 3 thrust scaled to 0.5" in texts
    assert list(runner.metrics.phases) == ["w1", "wind", "motor_failure", "w2", "half", "calm"]
    assert link.calls == []                                   # none of these phases talks to PX4
    assert simr.sim.on_ground and simr.sim.tilt_deg < 1.0    # still on its legs in the wind


def test_scenario_initial_wind_applied():
    simr = make_sim()
    sc = Scenario(phases=[{"type": "wait", "duration": 0.1}], wind=[3, 0, 0])
    runner = ScenarioRunner(sc, FakeLink())
    advance(simr, runner, 0.05, physics=False)
    assert simr.sim.wind_ned.tolist() == pytest.approx([3.0, 0.0, 0.0])


def test_hold_phase_sets_mode_and_target():
    simr = make_sim()
    link = FakeLink()
    sc = Scenario(phases=[{"type": "hold", "name": "hover", "duration": 0.5}], max_time=10)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 1.0, physics=False)
    assert runner.done and runner.ok
    assert ("mode", "hold") in link.calls and link.mode_is("hold")
    assert runner.targets["hover"]["pos"] == pytest.approx([0.0, 0.0, -0.12], abs=5e-3)
    r = runner.result(simr.airframe.mass.mass)
    assert r["metrics"]["phases"]["hover"]["pos_err_rms"] == pytest.approx(0.0, abs=1e-6)


def test_abort_on_tilt_marks_crash():
    simr = make_sim()
    link = FakeLink()
    sc = Scenario(phases=[{"type": "wait", "name": "w", "duration": 5}], max_time=60, abort={"max_tilt_deg": 60})
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 0.2)                              # on the ground: tilt checks are off
    assert not runner.done
    rb = simr.sim
    rb.reset(pos_ned=[0.0, 0.0, -3.0], roll_deg=30.0)       # in the air, banked, no thrust
    advance(simr, runner, 0.1)
    assert not runner.done and runner.metrics.crashed is False   # 30 degrees is within the limit
    rb.reset(pos_ned=[0.0, 0.0, -3.0], roll_deg=100.0)      # past the 60 degree abort limit
    advance(simr, runner, 0.1)
    assert runner.done and runner.status == "aborted" and runner.ok is False
    assert runner.metrics.crashed is True and runner.metrics.crash_reason.startswith("tilt")
    assert runner.failures[-1].startswith("w: crashed: tilt")
    r = runner.result(rb.mass)
    assert r["ok"] is False and r["metrics"]["crashed"] is True


def test_abort_on_ground_impact_and_max_time():
    simr = make_sim()
    sc = Scenario(phases=[{"type": "wait", "duration": 100}], max_time=1.0, abort={"crash_speed": 3.0})
    runner = ScenarioRunner(sc, FakeLink())
    advance(simr, runner, 2.0, physics=False)
    assert runner.done and runner.status == "aborted" and "max_time" in runner.failures[0]
    # a hard landing
    simr = make_sim()
    runner = ScenarioRunner(Scenario(phases=[{"type": "wait", "duration": 100}], max_time=100, abort={"crash_speed": 3.0}), FakeLink())
    rb = simr.sim
    rb.reset(pos_ned=[0.0, 0.0, -3.0])
    rb.on_ground = False
    advance(simr, runner, 0.02, physics=False)
    rb.on_ground = True
    rb.vel = np.array([0.0, 0.0, 5.0])
    advance(simr, runner, 0.02, physics=False)
    assert runner.done and runner.metrics.crashed and "hit the ground" in runner.metrics.crash_reason


def test_flight_termination_from_px4_counts_as_crash():
    simr = make_sim()
    link = FakeLink()
    runner = ScenarioRunner(Scenario(phases=[{"type": "wait", "duration": 100}], max_time=100), link)
    simr.sim.reset(pos_ned=[0.0, 0.0, -3.0]); simr.sim.on_ground = False
    advance(simr, runner, 0.02, physics=False)
    link.custom_mode = 10 << 16
    advance(simr, runner, 0.02, physics=False)
    assert runner.done and runner.metrics.crash_reason == "PX4 flight termination"


def test_unknown_phase_type_is_a_failure_not_a_crash():
    simr = make_sim()
    runner = ScenarioRunner(Scenario(phases=[{"type": "teleport"}, {"type": "wait", "duration": 0.1}], max_time=10), FakeLink())
    advance(simr, runner, 0.5, physics=False)
    assert runner.done and runner.status == "done" and runner.ok is False
    assert "unknown phase type 'teleport'" in runner.failures[0]


def test_param_mode_and_arm_phases():
    simr = make_sim()
    link = FakeLink()
    link.params["MC_PITCH_P"] = {"value": 6.0, "type": 9, "index": 0}
    sc = Scenario(phases=[{"type": "param", "name": "MC_PITCH_P", "value": 5.0}, {"type": "arm"}, {"type": "mode", "mode": "position"},
                          {"type": "param", "name": "NOPE", "value": 1}], max_time=10)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 1.0, physics=False)
    assert runner.done
    assert link.params["MC_PITCH_P"]["value"] == 5.0 and link.armed and link.mode_is("position")
    assert any(e["text"] == "MC_PITCH_P = 5.0" for e in runner.metrics.events)
    assert runner.ok is False and "NOPE" in runner.failures[0]


def test_offboard_and_manual_phases_stream_to_link():
    simr = make_sim(sensor_rate=100.0)
    link = FakeLink()
    sc = Scenario(phases=[{"type": "offboard_velocity", "name": "cruise", "vel": [3, 0, 0], "duration": 1.0},
                          {"type": "manual", "pitch": 0.5, "duration": 0.5, "mode": "position"}], max_time=10)
    runner = ScenarioRunner(sc, link)
    advance(simr, runner, 2.0, physics=False)
    assert runner.done and runner.ok
    assert len(link.setpoints) >= 15 and link.setpoints[0]["vel"] == [3.0, 0.0, 0.0]
    assert ("mode", "offboard") in link.calls and ("mode", "position") in link.calls
    assert len(link.manual) >= 20 and link.manual[0][1] == 0.5
    assert runner.targets["cruise"]["vel"] == [3.0, 0.0, 0.0]


def test_simulator_without_link_idles():
    simr = make_sim()
    assert simr.step_once() is False
    assert simr.t == 0.0 and simr.step_count == 0
    snap = simr.snapshot()
    assert snap["on_ground"] is True and len(snap["rotors"]) == 4 and snap["rotor_health"] == [1.0] * 4
    simr.set_wind(1.0, 2.0)
    assert simr.sim.wind_ned.tolist() == pytest.approx([1.0, 2.0, 0.0])
    simr.set_rotor_health([0.0])
    assert simr.snapshot()["rotor_health"] == [0.0, 1.0, 1.0, 1.0]
