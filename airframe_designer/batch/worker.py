"""One headless simulation: boot a private PX4 SITL instance with every parameter pre-seeded, run a scenario in
lockstep as fast as PX4 allows, return the metrics. Everything a study needs is in the returned dict; nothing is
kept on disk unless asked for."""
from __future__ import annotations

import json
import os
import time
import traceback
from pathlib import Path

from ..geometry.airframe import Airframe
from ..geometry.paths import apply_variables
from ..px4.events import EventDecoder
from ..px4.link import PX4Link
from ..px4 import param_meta
from ..px4.sitl import launch_px4, stop_px4, free_px4_instance, PX4Instance, find_px4_dir, param_types_from_meta, instance_is_free
from ..sensors import Home
from ..sim.metrics import MetricsRecorder
from ..sim.scenario import Scenario, ScenarioRunner, load_scenario
from ..sim.simulator import Simulator

# PX4 parameters every batch run gets unless the airframe or scenario says otherwise
BATCH_PX4_DEFAULTS: dict[str, float | int] = {
    "SDLOG_MODE": 3,          # "log while AUX1 > 30%": with no RC input this never logs (no ulog files, no disk I/O)
    "SDLOG_PROFILE": 0,
    "COM_RC_IN_MODE": 1,      # manual control from MAVLink only (scripted sticks), no RC receiver expected
    "NAV_DLL_ACT": 0,         # no GCS-loss failsafe
    "COM_OBL_RC_ACT": 5,      # offboard loss -> hold
    "MAV_0_RATE": 0,          # the GCS link is unused; keep PX4 from spending time on telemetry
}

_TYPES: dict[str, str] | None = None


def px4_param_types(px4_dir: str | None) -> dict[str, str]:
    """{name: 'Int32'|'Float'} from the PX4 build's parameters.json (cached). Without it, integer-valued floats
    would be written as INT32 and PX4 would reject them at import."""
    global _TYPES
    if _TYPES is None:
        meta, _ = param_meta.load_local(find_px4_dir(px4_dir), None)
        _TYPES = param_types_from_meta(meta)
        if not _TYPES:
            print("[batch] warning: PX4 parameters.json not found; parameter types guessed from values", flush=True)
    return _TYPES


def _load_airframe(spec) -> Airframe:
    if isinstance(spec, Airframe):
        return spec.copy()
    if isinstance(spec, dict):
        return Airframe.from_dict(spec)
    return Airframe.load(spec)


def run_once(airframe, scenario, *, variables: dict | None = None, px4_dir: str | None = None, instance: int | None = None,
             speed: float = 0.0, rate: float = 250.0, substeps: int = 2, home: Home | None = None, noise: bool = True,
             seed: int = 1, timeout_wall: float = 600.0, connect_timeout: float = 40.0, log=None, quiet: bool = True,
             px4_model: str = "none_iris", extra_params: dict | None = None, timeseries_path: str | None = None,
             workdir: str | None = None, task_id: str | None = None, physics: str = "python") -> dict:
    """Run ``scenario`` on ``airframe`` (path, dict or Airframe), optionally with parameter-path ``variables``
    applied first. Returns {"ok", "status", "failures", "metrics", "timing", "airframe", ...}."""
    t_wall0 = time.perf_counter()
    lines: list[str] = []

    def _log(s: str) -> None:
        lines.append(s)
        if log:
            log(s)

    px4_dir = find_px4_dir(px4_dir)
    af = _load_airframe(airframe)
    if variables:
        af = apply_variables(af, variables)
    af.resolve_mass()
    from ..aero.airfoils import ensure_polars
    ensure_polars(af)
    sc = scenario if isinstance(scenario, Scenario) else load_scenario(scenario)
    result: dict = {"id": task_id, "ok": False, "status": "error", "airframe_name": af.name, "scenario": sc.name,
                    "variables": variables or {}, "physics": physics, "failures": [], "metrics": {}, "timing": {}}
    if instance is None:
        instance = free_px4_instance(start=1)
    elif not instance_is_free(instance):
        result["failures"] = [f"PX4 instance {instance} is busy"]
        return result
    inst = PX4Instance(instance, workdir, fresh=True)
    params: dict[str, float | int] = dict(BATCH_PX4_DEFAULTS)
    params.update(af.px4_params_sitl())
    params.update(sc.params or {})
    params.update(extra_params or {})
    proc = None
    link = None
    try:
        proc = launch_px4(px4_dir, px4_model, _log, instance=instance, rootfs=str(inst.workdir), params=params,
                          param_types=px4_param_types(px4_dir), fresh=False, quiet=quiet)
        link = PX4Link("sitl", inst.tcp_address, ctl_address=inst.ctl_address, log=_log)
        dec = EventDecoder(); dec.load_local(px4_dir)
        link.event_decoder = dec
        link.param_types = {k: (6 if v == "Int32" else 9) for k, v in px4_param_types(px4_dir).items()}
        link.open()
        simr = Simulator(af, link, sensor_rate=rate, physics_substeps=substeps, speed=speed, lockstep=True, home=home,
                         log=_log, seed=seed, physics=physics)
        simr.sensors.noise.enabled = bool(noise)
        metrics = MetricsRecorder()
        runner = ScenarioRunner(sc, link, log=_log, metrics=metrics)
        simr.hooks.append(runner)
        # wait for PX4's simulator link (it connects to us once its startup script reaches the simulator module)
        t0 = time.perf_counter()
        while not link.connected and time.perf_counter() - t0 < connect_timeout:
            if proc.poll() is not None:
                raise RuntimeError(f"PX4 exited during startup (code {proc.poll()})")
            simr.step_once()      # idles (and sends heartbeats) until connected
        if not link.connected:
            raise RuntimeError("PX4 did not connect to the simulator link")
        t_conn = time.perf_counter()
        # read back a few seeded parameters once PX4's control link is up (its sysid is only known then);
        # the replies arrive while the loop runs and are checked after the run
        sentinels = [k for k in ("SYS_AUTOSTART", "CA_ROTOR_COUNT", "SENS_BOARD_Y_OFF", "MIS_TAKEOFF_ALT", "SDLOG_MODE", "CA_ROTOR0_PX")
                     if k in params]
        asked = {"next_t": 1.5, "tries": 0}

        def ask_sentinels(s):
            # PX4 does not answer parameter requests during its first moments of boot: ask from 1.5 s of
            # simulation time on, and again every 2 s until every sentinel has been echoed (5 tries at most)
            if asked["tries"] >= 5 or not link.ctl_connected or s.t < asked["next_t"]:
                return
            missing = [k for k in sentinels if k not in link.params]
            if not missing:
                asked["tries"] = 5
                return
            asked["tries"] += 1
            asked["next_t"] = s.t + 2.0
            for k in missing:
                try:
                    link.param_request_read(k)
                except Exception:
                    pass
        simr.hooks.insert(0, ask_sentinels)
        why = simr.run_sync(until=lambda s: runner.done, max_wall_time=timeout_wall)
        verified = {}
        for k in sentinels:
            got = link.params.get(k, {}).get("value")
            verified[k] = {"seeded": params[k], "vehicle": got,
                           "ok": got is not None and abs(float(got) - float(params[k])) <= 1e-4 * max(1.0, abs(float(params[k])))}
        result["px4_params_verified"] = verified
        bad = [k for k, v in verified.items() if v["vehicle"] is not None and not v["ok"]]
        if bad:
            runner.failures.append(f"seeded PX4 parameters not in effect: {bad}")
            runner.ok = False
        if not runner.done:
            runner.finish(simr, why, False)
            runner.failures.append(f"loop stopped: {why}")
        r = runner.result(af.mass.mass)
        result.update(r)
        result["timing"] = {"wall_s": round(time.perf_counter() - t_wall0, 2), "px4_boot_s": round(t_conn - t_wall0, 2),
                            "sim_s": round(simr.t, 2), "rtf": round(simr.t / max(1e-6, time.perf_counter() - t_conn), 2),
                            "lockstep_timeouts": simr.lockstep_timeouts, "steps": simr.step_count, "instance": instance}
        result["px4_params_seeded"] = len(params)
        if timeseries_path:
            Path(timeseries_path).parent.mkdir(parents=True, exist_ok=True)
            with open(timeseries_path, "w") as f:
                json.dump({"result": {k: v for k, v in result.items() if k != "log"}, "timeseries": metrics.timeseries()}, f)
            result["timeseries_path"] = str(timeseries_path)
    except Exception as e:
        result["status"] = "error"
        result["failures"].append(f"{type(e).__name__}: {e}")
        result["traceback"] = traceback.format_exc()
        _log(f"[batch] error: {e}")
    finally:
        try:
            if link is not None:
                link.close()
        finally:
            stop_px4(proc)
    result["log"] = lines[-60:] if not quiet else [l for l in lines if "[scenario]" in l or "FAIL" in l or "error" in l.lower()][-40:]
    result["airframe"] = af.to_dict()
    return result
