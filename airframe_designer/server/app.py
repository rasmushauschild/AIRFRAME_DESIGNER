"""FastAPI web server: serves the 3D UI, streams sim state over a websocket, exposes control REST endpoints."""
from __future__ import annotations

import asyncio
import json
import time
from collections import deque
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from starlette.concurrency import run_in_threadpool

from ..geometry.airframe import Airframe, PRESETS
from ..geometry.gear import generate_legs
from ..geometry import cad as cadmod
from ..geometry.paths import list_paths, apply_variables
from ..analysis import static as design
from ..analysis.geometric_optimiser import optimise as geometric_optimise, default_groups
from ..px4.link import mavlink
from ..px4 import param_meta
from ..px4.sitl import instance_is_free
from ..aero import airfoils

PROJECT_DIR = Path(__file__).resolve().parents[2]
UI_DIR = PROJECT_DIR / "ui"
AIRFRAME_DIR = PROJECT_DIR / "airframes"
SCENARIO_DIR = PROJECT_DIR / "scenarios"
STUDY_DIR = PROJECT_DIR / "studies"
RESULTS_DIR = PROJECT_DIR / "results"

# PX4 custom mode encoding: main_mode << 16 | sub_mode << 24
PX4_MODES = {
    "manual": (1, 0), "altitude": (2, 0), "position": (3, 0), "acro": (5, 0), "stabilized": (7, 0),
    "takeoff": (4, 2), "hold": (4, 3), "mission": (4, 4), "rtl": (4, 5), "land": (4, 6),
}
PX4_MAIN_MODE_NAMES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized",
                       8: "Rattitude", 9: "Simple", 10: "Termination"}
PX4_SUB_MODE_NAMES = {1: "Ready", 2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land", 8: "Follow", 9: "Precland"}


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    name = PX4_MAIN_MODE_NAMES.get(main, f"mode{main}")
    if main == 4:
        name = PX4_SUB_MODE_NAMES.get(sub, f"Auto{sub}")
    return name


def json_safe(x):
    """Replace non-finite floats (which the JSON encoder refuses) with None, recursively."""
    import math
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: json_safe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [json_safe(v) for v in x]
    return x


class _NoLink:
    """Stand-in while no PX4 link exists so the endpoints degrade gracefully."""
    mode = "none"
    connected = False
    ctl_connected = False
    params: dict = {}
    param_count = 0

    def status(self):
        return {"mode": "none", "address": "", "connected": False, "ctl_connected": False, "armed": False,
                "hil_enabled": False, "custom_mode": 0, "rx_count": 0, "param_count": 0, "params_loaded": 0,
                "actuator_seq": 0, "qgc_proxy": None, "target_system": 0, "ctl_address": ""}

    def __getattr__(self, name):
        def noop(*a, **k):
            return {"ok": False, "error": "PX4 not connected"}
        return noop


class AppState:
    def __init__(self, simulator, conn, args, log_buffer: deque, log):
        self.simulator = simulator
        self.conn = conn                     # ConnectionManager
        self.args = args
        self.log_buffer = log_buffer
        self.log = log
        self.meta: dict[str, dict] = {}
        self.meta_source = ""
        self.export_log: deque = deque(maxlen=500)
        self.opt_job: dict = {"running": False, "progress": 0.0, "message": "", "result": None, "error": None}
        self.batch_jobs: dict[str, dict] = {}
        self.study_job: dict = {"running": False}

    @property
    def link(self):
        return self.conn.link if self.conn.link is not None else _NoLink()


def build_app(state: AppState) -> FastAPI:
    app = FastAPI(title="AIRFRAME_DESIGNER")
    app.mount("/static", StaticFiles(directory=str(UI_DIR)), name="static")

    @app.middleware("http")
    async def no_cache(request, call_next):
        # the UI is edited live; browsers otherwise keep stale copies of app.js/scene.js across reloads
        response = await call_next(request)
        if request.url.path.startswith("/static/") or request.url.path == "/":
            response.headers["Cache-Control"] = "no-store, must-revalidate"
            response.headers["Pragma"] = "no-cache"
        return response
    sim = state.simulator

    class _LinkProxy:
        def __getattr__(self, name):
            return getattr(state.link, name)

    link = _LinkProxy()   # always resolves to the current link

    # -------------------------------------------------------------- pages
    @app.get("/")
    async def index():
        return FileResponse(str(UI_DIR / "index.html"))

    # ------------------------------------------------------------- status
    def status_dict() -> dict:
        s = link.status()
        s["mode_name"] = mode_name(s["custom_mode"])
        s["px4_running"] = state.conn.px4_running()
        s["params_session"] = getattr(state.conn, "_params_session", 0)   # bumps on every (re)connect: the page re-downloads
        s["conn_mode"] = state.conn.mode
        s["conn_error"] = state.conn.error
        s["flashing"] = state.conn.firmware_job.running() and state.conn.firmware_job.action == "upload"
        # does the connected firmware carry the ATLAS nose-lift module, and is it enabled? (the UI decides the Takeoff
        # path from this instead of from its lazily loaded parameter list)
        lp = getattr(state.link, "params", None) or {}
        s["native_module"] = "NLF_ENABLE" in lp
        s["nlf_enable"] = (lp.get("NLF_ENABLE") or {}).get("value") if "NLF_ENABLE" in lp else None
        try:
            if state.conn.mode == "sitl":          # SITL module tree: sources newer than the binary?
                s["firmware"] = state.conn.firmware_status()
            elif state.conn.mode == "hitl":        # board image older than the sources, or newer than what was flashed?
                b = state.conn.board_firmware_status()
                s["firmware"] = {"board": True, "custom": True, "needs_rebuild": b["needs_rebuild"], "needs_flash": b["needs_flash"], "stale": b["stale"]}
            else:
                s["firmware"] = None
        except Exception:
            s["firmware"] = None
        # arm gating: PX4's last arming-check summary must report no system errors and a usable position
        ready, why = False, "waiting for PX4's arming check report"
        ev = getattr(state.link, "recent_events", None)
        for x in reversed(list(ev if isinstance(ev, (list, tuple, deque)) else [])):   # no link (e.g. while flashing): nothing
            if x.get("name") == "commander_arming_check_summary":
                d = dict(zip(x.get("arg_names", []), x.get("args", [])))
                # PX4 lists the modes it would arm in; the error mask also carries the always-failing
                # offboard/mission checks ("system"), so it is not usable as a gate on its own.
                can = str(d.get("can_arm", ""))
                mode_now = s.get("mode_name", "").lower()
                aliases = {"hold": "loiter", "stabilized": "stab", "position": "posctl", "altitude": "altctl"}
                ready = ("takeoff" in can) or ("loiter" in can) or (aliases.get(mode_now, mode_now) in can.split("|"))
                why = "" if ready else "PX4 will not arm yet (estimator or health checks); see the Flight tab"
                break
        s["resetting"] = max(0.0, state.conn._reset_busy_until - time.time())
        s["arm_ready"] = bool(s["ctl_connected"]) and (ready or bool(s["armed"])) and s["resetting"] <= 0
        s["arm_block_reason"] = why
        s["px4_ports"] = [p["device"] for p in state.conn.list_ports_cached() if p["likely_px4"]]
        # a RadioMaster radio that shows up as a *serial* port was powered on in VCP/config mode (M + Power);
        # in that mode it is not a joystick
        s["radio_vcp_ports"] = [p["device"] for p in state.conn.list_ports_cached() if "radiomaster" in (p["device"] + p["description"]).lower()]
        s["meta_loaded"] = len(state.meta)
        s["meta_source"] = state.meta_source
        s["home"] = {"lat": sim.sensors.home.lat, "lon": sim.sensors.home.lon, "alt": sim.sensors.home.alt}
        s["speed"] = sim.speed
        s["sensor_rate"] = sim.sensor_rate
        s["lockstep"] = sim.lockstep
        s["noise"] = sim.sensors.noise.enabled
        s["paused"] = sim.paused
        s["physics"] = sim.physics
        return s

    @app.get("/api/status")
    async def get_status():
        return status_dict()

    @app.get("/api/log")
    async def get_log(since: float = 0.0):
        return [e for e in state.log_buffer if e[0] > since]

    # ----------------------------------------------------------- airframe
    AUTOSAVE = AIRFRAME_DIR / "_autosave.json"
    _autosave_t = [0.0]

    def autosave(af: Airframe) -> None:
        """Every accepted edit is written to airframes/_autosave.json (at most twice a second), so a design
        survives a closed terminal or a crash even if it was never saved by name."""
        now = time.time()
        if now - _autosave_t[0] < 0.5:
            return
        _autosave_t[0] = now
        try:
            AIRFRAME_DIR.mkdir(exist_ok=True)
            af.save(AUTOSAVE)
        except Exception as e:
            state.log(f"[ui] autosave failed: {e}")

    @app.get("/api/airframe")
    async def get_airframe():
        return sim.airframe.to_dict()

    @app.post("/api/airframe")
    async def set_airframe(body: dict):
        try:
            af = Airframe.from_dict(body.get("airframe", body))
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"invalid airframe ({type(e).__name__}: {e}); an empty number field?"}, status_code=400)
        keep = bool(body.get("keep_state", True))
        af.resolve_mass()
        try:
            await run_in_threadpool(airfoils.ensure_polars, af, state.log)   # polar wings: section tables ready before the physics
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"airfoil polar: {e}"}, status_code=400)
        sim.set_airframe(af, keep_state=keep)
        autosave(af)
        hc = af.hover_check()
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    @app.get("/api/airframe/hover_check")
    async def hover_check():
        return json_safe(sim.airframe.hover_check())

    @app.get("/api/airframes")
    async def list_airframes():
        files = sorted(p.name for p in AIRFRAME_DIR.glob("*.json"))
        return {"presets": list(PRESETS), "files": files}

    @app.post("/api/airframe/load")
    async def load_airframe(body: dict):
        name = body.get("name", "")
        if name in PRESETS:
            af = PRESETS[name]()
        else:
            p = AIRFRAME_DIR / name
            if not p.is_file():
                return JSONResponse({"ok": False, "error": f"not found: {name}"}, status_code=404)
            af = Airframe.load(p)
        try:
            await run_in_threadpool(airfoils.ensure_polars, af, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"airfoil polar: {e}"}, status_code=400)
        sim.set_airframe(af, keep_state=False)
        hc = af.hover_check()
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    @app.post("/api/airframe/save")
    async def save_airframe(body: dict):
        name = body.get("name", "").strip()
        if not name:
            return JSONResponse({"ok": False, "error": "name required"}, status_code=400)
        if not name.endswith(".json"):
            name += ".json"
        name = Path(name).name
        AIRFRAME_DIR.mkdir(exist_ok=True)
        sim.airframe.save(AIRFRAME_DIR / name)
        return {"ok": True, "path": str(AIRFRAME_DIR / name)}

    # ------------------------------------------------------------------ CAD (STEP bodies -> masses, CG)
    CAD_DIR = AIRFRAME_DIR / "cad"

    def cad_file_path(model) -> Path:
        p = Path(model.file)
        return p if p.is_absolute() else PROJECT_DIR / p

    @app.post("/api/cad/import")
    async def cad_import(request: Request, filename: str = "model.step"):
        """Upload a STEP file (raw bytes): it is copied to airframes/cad/, every solid measured and meshed, and the
        bodies attached to the live airframe (masses/offsets of bodies with the same id are kept on re-import)."""
        data = await request.body()
        if not data:
            return JSONResponse({"ok": False, "error": "empty upload"}, status_code=400)
        CAD_DIR.mkdir(parents=True, exist_ok=True)
        stem = cadmod._safe_stem(filename)
        suffix = ".stp" if filename.lower().endswith(".stp") else ".step"
        dst = CAD_DIR / (stem + suffix)
        dst.write_bytes(data)
        try:
            imported = await run_in_threadpool(cadmod.import_step, dst, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"STEP import failed: {e}"}, status_code=400)
        af = sim.airframe.copy()
        af.cad = cadmod.model_from_import(imported, str(dst.relative_to(PROJECT_DIR)), af.cad)
        af.resolve_mass()
        sim.set_airframe(af, keep_state=True)
        autosave(af)
        return {"ok": True, "airframe": json_safe(af.to_dict()), "bodies": len(af.cad.bodies),
                "mesh": cadmod.mesh_payload(af.cad, imported)}

    @app.get("/api/cad/mesh")
    async def cad_mesh():
        """Meshes of the live airframe's CAD bodies in the structural frame (axes/origin/scale applied, offsets not)."""
        m = sim.airframe.cad
        if not m:
            return {"ok": True, "file": None, "bodies": []}
        path = cad_file_path(m)
        if not path.exists():
            return JSONResponse({"ok": False, "error": f"CAD file missing: {m.file}"}, status_code=404)
        try:
            imported = await run_in_threadpool(cadmod.import_step, path, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"STEP import failed: {e}"}, status_code=400)
        return cadmod.mesh_payload(m, imported)

    @app.get("/api/cad/totals")
    async def cad_totals():
        m = sim.airframe.cad
        return {"ok": True, "totals": m.totals() if m else None}

    @app.post("/api/airframe/estimate_inertia")
    async def estimate_inertia():
        af = sim.airframe
        af.mass.inertia = af.estimate_inertia()
        sim.set_airframe(af)
        return {"ok": True, "inertia": af.mass.inertia}

    @app.post("/api/airframe/legs/generate")
    async def legs_generate(body: dict):
        """Four legs from a height below the CG, spreads and the landed pitch (replaces the airframe's legs)."""
        af = sim.airframe
        kw = {}
        if body.get("stiffness") is not None:
            kw["stiffness"] = float(body["stiffness"]); kw["damping"] = float(body.get("damping", 150.0))
        legs = generate_legs(float(body.get("height", 0.2)), float(body.get("spread_x", 0.2)), float(body.get("spread_y", body.get("spread_x", 0.2))),
                             float(body.get("landed_pitch_deg", af.landed_pitch_deg)), float(body.get("attach_z", 0.0)), cg=af.mass.cg,
                             mass=af.mass.mass, **kw)
        return {"ok": True, "legs": [l.to_dict() for l in legs]}

    @app.post("/api/airframe/legs/auto")
    async def legs_auto(body: dict | None = None):
        """Size every leg's spring/damper from the mass (2 cm static sink, damping ratio 0.8 by default)."""
        body = body or {}
        af = sim.airframe
        af.auto_leg_constants(float(body.get("compression_m", 0.02)), float(body.get("zeta", 0.8)))
        sim.set_airframe(af, keep_state=True)
        autosave(af)
        return {"ok": True, "legs": [l.to_dict() for l in af.legs], "static": af.leg_static()}

    @app.get("/api/airfoils")
    async def list_airfoils():
        return airfoils.list_airfoils()

    @app.get("/api/airfoil/coords")
    async def airfoil_coords(name: str):
        """Section coordinates (chord-normalised, Selig order: upper TE -> LE -> lower TE) for drawing."""
        try:
            pts = await run_in_threadpool(airfoils.load_coordinates, name)
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=404)
        return {"ok": True, "name": airfoils.normalise_name(name), "coords": [[round(float(x), 5), round(float(y), 5)] for x, y in pts]}

    @app.post("/api/airfoil/polar")
    async def airfoil_polar(body: dict):
        """Build (or load) the polar table of an airfoil by name (NACA 4/5-digit, a local airfoils/<name>.dat, or a
        UIUC database name) and return its headline numbers. {"name": "naca23006", "force": false, "re": 5e5}"""
        name = str(body.get("name", "")).strip()
        if not name:
            return JSONResponse({"ok": False, "error": "name required"}, status_code=400)
        try:
            table = await run_in_threadpool(airfoils.build_polar, name, None, float(body.get("ncrit", 9.0)), str(body.get("source", "auto")), bool(body.get("force", False)))
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)
        Re = float(body.get("re", 5e5))
        return {"ok": True, "name": table["name"], "source": table["source"], "built_s": table.get("built_s"), "note": table.get("note"),
                "re_list": table["re"], "summary": airfoils.polar_summary(table, Re), "valid": table["valid"],
                "curve": {"alpha": table["alpha"][::4], "cl": table["cl"][min(3, len(table["re"]) - 1)][::4], "cd": table["cd"][min(3, len(table["re"]) - 1)][::4]}}

    @app.get("/api/airframe/paths")
    async def airframe_paths():
        return {"paths": list_paths(sim.airframe)}

    # ---------------------------------------------------------------- design
    def design_speed(body: dict | None) -> float:
        kmh = (body or {}).get("speed_kmh")
        if kmh is None:
            kmh = sim.airframe.design.get("cruise_speed_kmh", 50.0)
        return float(kmh) / 3.6

    def design_notes(speed: float) -> list[str]:
        """Things PX4 must allow for the cruise case: velocity limits and tilt."""
        notes = []
        v = link.params.get("MPC_XY_VEL_MAX", {}).get("value")
        if v is not None and speed > float(v) + 1e-6:
            notes.append(f"MPC_XY_VEL_MAX is {float(v):g} m/s, cruise needs {speed:.1f} m/s")
        c = link.params.get("MPC_XY_CRUISE", {}).get("value")
        if c is not None and speed > float(c) + 1e-6:
            notes.append(f"MPC_XY_CRUISE is {float(c):g} m/s, missions fly at that speed")
        return notes

    @app.post("/api/design/analysis")
    async def design_analysis(body: dict | None = None):
        body = body or {}
        if "airframe" in body:
            try:
                af = Airframe.from_dict(body["airframe"])
            except Exception as e:
                return JSONResponse({"ok": False, "error": f"invalid airframe: {e}"}, status_code=400)
        else:
            af = sim.airframe
        speed = design_speed(body)
        tilt = link.params.get("MPC_TILTMAX_AIR", {}).get("value")
        tilt = float(tilt) if tilt is not None else 45.0
        r = await run_in_threadpool(design.analyse, af, speed, tilt)
        r["notes"] = design_notes(speed)
        r["tilt_limit_deg"] = tilt
        r["groups"] = default_groups(af)
        return json_safe(r)

    @app.post("/api/design/optimize")
    async def design_optimize(body: dict):
        job = state.opt_job
        if job["running"]:
            return JSONResponse({"ok": False, "error": "an optimisation is already running"}, status_code=409)
        try:
            af = Airframe.from_dict(body["airframe"]) if "airframe" in body else sim.airframe
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"invalid airframe: {e}"}, status_code=400)
        spec = dict(body.get("spec") or {})
        spec.setdefault("speed_kmh", af.design.get("cruise_speed_kmh", 50.0))
        tilt = link.params.get("MPC_TILTMAX_AIR", {}).get("value")
        spec.setdefault("tilt_limit_deg", float(tilt) if tilt is not None else 45.0)
        job.update(running=True, progress=0.0, message="starting", result=None, error=None)

        def progress(frac, msg):
            job["progress"], job["message"] = float(frac), str(msg)

        def run():
            try:
                job["result"] = geometric_optimise(af, spec, progress)
            except Exception as e:
                job["error"] = f"{type(e).__name__}: {e}"
                state.log(f"[design] optimisation failed: {e}")
            finally:
                job["running"] = False

        import threading
        threading.Thread(target=run, name="design-optimise", daemon=True).start()
        return {"ok": True}

    @app.get("/api/design/optimize")
    async def design_optimize_status():
        j = state.opt_job
        return {"running": j["running"], "progress": j["progress"], "message": j["message"],
                "result": j["result"], "error": j["error"]}

    # ------------------------------------------------------------ connection
    @app.get("/api/connection")
    async def get_connection():
        st = state.conn.status()
        st["checklist"] = state.conn.checklist(export_params() if state.conn.mode == "hitl" else None)
        return st

    @app.post("/api/connection/connect")
    async def connect(body: dict):
        mode = body.get("mode", "sitl")
        sim.paused = False          # a paused loop sends PX4 nothing; connecting while paused can only fail
        if mode == "hitl":
            r = await run_in_threadpool(state.conn.connect_hitl, body.get("serial"), body.get("baud"))
        else:
            r = await run_in_threadpool(state.conn.connect_sitl, body.get("launch"))
        return r

    @app.post("/api/connection/disconnect")
    async def disconnect():
        return await run_in_threadpool(state.conn.disconnect)

    @app.post("/api/px4/reset_all")
    async def px4_reset_all():
        sim.paused = False
        r = await run_in_threadpool(state.conn.reset_all)
        state.log("[px4] reset: " + ", ".join(r.get("steps", [])))
        return r

    @app.post("/api/px4/recover")
    async def px4_recover():
        r = await run_in_threadpool(state.conn.recover)
        state.log("[px4] recover: " + ", ".join(r.get("steps", [])))
        return r

    @app.post("/api/connection/restart_estimator")
    async def restart_estimator():
        return await run_in_threadpool(state.conn.restart_estimator)

    @app.post("/api/connection/enable_hitl")
    async def enable_hitl():
        return await run_in_threadpool(state.conn.enable_hitl)

    @app.post("/api/firmware/build")
    async def firmware_build(body: dict | None = None):
        """Build board firmware; atlas=true builds it with the ATLAS modules the SITL uses (from the SITL's PX4 tree)."""
        body = body or {}
        return await run_in_threadpool(state.conn.build_firmware, body.get("target"), bool(body.get("atlas", False)))

    @app.post("/api/firmware/upload")
    async def firmware_upload(body: dict | None = None):
        body = body or {}
        return await run_in_threadpool(state.conn.upload_firmware, body.get("target"), bool(body.get("atlas", False)), None, str(body.get("note", "")))

    # ---------------------------------------------------------- firmware archive (GitHub)
    state.conn.export_params_fn = lambda: export_params()   # resolved at call time (defined further down)

    @app.get("/api/archive")
    async def archive_list():
        a = state.conn.archive
        await run_in_threadpool(a.ensure)
        return {"status": a.status(), "versions": a.versions(), "restore": state.conn.restore_job,
                "board_connected": state.conn.mode == "hitl" and bool(link.ctl_connected)}

    @app.post("/api/archive/pull")
    async def archive_pull():
        return await run_in_threadpool(state.conn.archive.pull)

    @app.post("/api/archive/snapshot")
    async def archive_snapshot(body: dict | None = None):
        body = body or {}
        return await run_in_threadpool(state.conn.archive_snapshot, "manual", None, None, str(body.get("note", "")))

    @app.post("/api/archive/restore")
    async def archive_restore(body: dict):
        return await run_in_threadpool(state.conn.archive_restore, str(body.get("id", "")), bool(body.get("flash", True)),
                                       bool(body.get("params", True)))

    @app.post("/api/archive/load_airframe")
    async def archive_load_airframe(body: dict):
        v = state.conn.archive.get(str(body.get("id", "")))
        if not v or not v.get("airframe"):
            return JSONResponse({"ok": False, "error": "no airframe in this version"}, status_code=404)
        af = Airframe.from_dict(v["airframe"])
        sim.set_airframe(af, keep_state=True)
        autosave(af)
        return {"ok": True, "airframe": json_safe(af.to_dict())}

    @app.post("/api/px4/shell")
    async def px4_shell(body: dict):
        cmd = str(body.get("command", "")).strip()
        if not cmd:
            return JSONResponse({"ok": False, "error": "command required"}, status_code=400)
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 not connected"}, status_code=409)
        out = await run_in_threadpool(link.shell, cmd, float(body.get("timeout", 3.0)))
        return {"ok": True, "output": out}

    @app.get("/api/rc")
    async def get_rc():
        rc = dict(getattr(state.link, "rc", {}) or {})
        rc = rc if rc and time.time() - rc.get("t", 0) < 3.0 else {}
        # PX4 channel mapping (1-based channel numbers, 0 = unassigned)
        names = {"RC_MAP_ROLL": "Roll", "RC_MAP_PITCH": "Pitch", "RC_MAP_THROTTLE": "Throttle", "RC_MAP_YAW": "Yaw",
                 "RC_MAP_FLTMODE": "Flight mode", "RC_MAP_ARM_SW": "Arm", "RC_MAP_KILL_SW": "Kill", "RC_MAP_RETURN_SW": "Return",
                 "RC_MAP_LOITER_SW": "Loiter", "RC_MAP_OFFB_SW": "Offboard", "RC_MAP_GEAR_SW": "Gear", "RC_MAP_FLAPS": "Flaps",
                 "RC_MAP_AUX1": "Aux 1", "RC_MAP_AUX2": "Aux 2", "RC_MAP_AUX3": "Aux 3", "RC_MAP_AUX4": "Aux 4",
                 "RC_MAP_AUX5": "Aux 5", "RC_MAP_AUX6": "Aux 6", "RC_MAP_PARAM1": "Param 1", "RC_MAP_PARAM2": "Param 2",
                 "RC_MAP_PARAM3": "Param 3", "RC_MAP_TRANS_SW": "Transition", "RC_MAP_ENG_MOT": "Engine/motor",
                 "RC_MAP_PAY_SW": "Payload", "RC_MAP_FAILSAFE": "Failsafe"}
        mapping: dict[int, list[str]] = {}
        params = getattr(state.link, "params", {}) or {}
        for k, label in names.items():
            v = params.get(k, {}).get("value")
            if isinstance(v, (int, float)) and int(v) > 0:
                mapping.setdefault(int(v), []).append(label)
        rc["mapping"] = {str(k): v for k, v in mapping.items()}
        rc["rc_in_mode"] = params.get("COM_RC_IN_MODE", {}).get("value")
        return rc

    @app.get("/api/events")
    async def get_events():
        return {"source": state.conn.event_decoder.source if state.conn.event_decoder else "",
                "events": list(link.recent_events) if hasattr(state.link, "recent_events") else []}

    @app.post("/api/events/meta/fetch")
    async def fetch_events_meta():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        local, msg = await run_in_threadpool(param_meta.fetch_extra, link, "all_events.json.xz", state.log)
        if local is None:
            return JSONResponse({"ok": False, "error": msg}, status_code=500)
        from .events import _read_json
        n = state.conn.event_decoder.load(_read_json(local), str(local))
        return {"ok": True, "count": n}

    @app.get("/api/firmware")
    async def firmware_status():
        b = state.conn.detected_board()
        return {"job": state.conn.firmware_job.status(), "board": b, "toolchain": state.conn.toolchain_present(),
                "built": state.conn.firmware_file(b["target"]), "built_atlas": state.conn.firmware_is_atlas(b["target"])}

    # ------------------------------------------------------------ PX4 export
    def export_params() -> dict[str, float | int]:
        hitl = link.mode == "hitl"
        return sim.airframe.px4_params(hitl=True) if hitl else sim.airframe.px4_params_sitl()

    @app.get("/api/px4/export")
    async def get_export():
        params = export_params()
        current = {k: link.params.get(k, {}).get("value") for k in params}
        ov = sim.airframe.px4_overrides or {}
        return json_safe({"params": params, "current": current, "problems": sim.airframe.validate() + sim.airframe.hover_check()["problems"],
                "file": sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                "overrides": ov, "geometry_keys": [k for k in params if k not in ov]})

    @app.get("/api/px4/export.params")
    async def get_export_file():
        return PlainTextResponse(sim.airframe.px4_params_file(hitl=link.mode == "hitl"),
                                 headers={"Content-Disposition": f'attachment; filename="{sim.airframe.name}.params"'})

    @app.get("/api/px4/firmware")
    async def px4_firmware_status():
        return state.conn.firmware_status(max_age=0.0)   # fresh: scripts ask right after editing a source

    @app.post("/api/px4/firmware/update")
    async def px4_firmware_update(body: dict | None = None):
        """Rebuild the SITL firmware if its sources changed and relaunch PX4 on it (what Update PX4 does first)."""
        r = await run_in_threadpool(state.conn.update_firmware, (body or {}).get("relaunch", True))
        return r if r.get("ok") else JSONResponse(r, status_code=500)

    @app.post("/api/px4/push")
    async def push_params(body: dict | None = None):
        body = body or {}
        fw_res = {"ok": True, "rebuilt": False, "relaunched": False, "flashed": False}
        if body.get("firmware", True) and state.conn.mode == "sitl" and getattr(state.args, "launch_px4", False):
            if link.armed:
                return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before updating PX4"}, status_code=409)
            fw_res = await run_in_threadpool(state.conn.update_firmware)
            if not fw_res.get("ok"):
                return JSONResponse({"ok": False, "error": fw_res.get("error"), "firmware": fw_res}, status_code=500)
        elif body.get("firmware", True) and state.conn.mode == "hitl" and (export_params().get("NLF_ENABLE") == 1):
            b = state.conn.board_firmware_status()
            if b["needs_rebuild"] or b["needs_flash"]:
                if link.armed:
                    return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before flashing"}, status_code=409)
                fw_res = await run_in_threadpool(state.conn.update_board_firmware)
                if not fw_res.get("ok"):
                    return JSONResponse({"ok": False, "error": fw_res.get("error"), "firmware": fw_res}, status_code=500)
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before updating PX4"}, status_code=409)
        params = export_params()
        only = body.get("only")
        if only:
            params = {k: v for k, v in params.items() if k in only}
        # skip params this firmware does not have (HIL_ACT on SITL, the NLF_* nose-lift module on a stock board).
        # While the parameter list is still downloading, keep the NLF_* ones: they belong to the ATLAS SITL build and
        # are simply not listed yet.
        complete = bool(link.params) and link.param_count and link.status().get("params_loaded", 0) >= link.param_count
        # PX4 lists a module's parameters only once something has used them; the nose-lift model parameters
        # (NLF_MASS, NLF_MOM, ...) are read by param_find at takeoff, so they are settable but not yet listed.
        has_module = "NLF_ENABLE" in (link.params or {})
        if link.params:
            keep = lambda k: k in link.params or (k.startswith("NLF_") and (has_module or not complete))
            missing = [k for k in params if not keep(k)]
            params = {k: v for k, v in params.items() if keep(k)}
            if complete and any(k.startswith("NLF_") for k in missing):
                state.log("[export] this firmware has no atlas_nose_lift module (NLF_* parameters unknown): the native "
                          "ground sequence is unavailable on it; the simulator's own nose-lift hook is used instead")
        else:
            missing = []
        state.export_log.clear()
        rot_before = link.params.get("SENS_BOARD_Y_OFF", {}).get("value")

        def progress(name, res):
            state.export_log.append({"t": time.time(), **res})

        results = await run_in_threadpool(link.set_params, params, progress)
        ok = all(r["ok"] for r in results)
        if body.get("save", True) and ok:
            link.preflight_storage(True)
        rot_after = params.get("SENS_BOARD_Y_OFF")
        if ok and rot_after is not None and rot_before is not None and abs(float(rot_after) - float(rot_before)) > 1e-3:
            # the IMU frame just changed under the running estimator: rest the sim at the new hover attitude and
            # restart EKF2 so it aligns from clean data
            state.log(f"[export] board rotation changed ({rot_before} -> {rot_after} deg): resetting sim, restarting estimator")
            sim.reset()
            await run_in_threadpool(state.conn.restart_estimator)     # HITL: reboots the board
        failed = [r for r in results if not r["ok"]]
        state.log(f"[export] pushed {len(results) - len(failed)}/{len(results)} params to PX4"
                  + (f", failed: {[r['name'] for r in failed]}" if failed else ""))
        return {"ok": ok, "results": results, "missing": missing, "firmware": fw_res}

    # ---------------------------------------------------------- parameters
    @app.get("/api/params")
    async def get_params():
        return {"count": link.param_count, "params": link.params}

    @app.post("/api/params/refresh")
    async def refresh_params():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        params = await run_in_threadpool(link.fetch_all_params)
        return {"ok": True, "count": link.param_count, "params": params}

    @app.post("/api/params/set")
    async def set_param(body: dict):
        name = body.get("name")
        value = body.get("value")
        if name is None or value is None:
            return JSONResponse({"ok": False, "error": "name and value required"}, status_code=400)
        res = await run_in_threadpool(link.set_param, name, value)
        if res.get("ok"):
            # remember it with the airframe so Save keeps it and Update PX4 re-applies it
            sim.airframe.px4_overrides[name] = res["value"]
        return res

    @app.post("/api/airframe/override")
    async def set_override(body: dict):
        """Record an edited parameter without touching the vehicle (used when not connected)."""
        name, value = body.get("name"), body.get("value")
        if not name or value is None:
            return JSONResponse({"ok": False, "error": "name and value required"}, status_code=400)
        sim.airframe.px4_overrides[name] = value
        return {"ok": True, "overrides": sim.airframe.px4_overrides}

    @app.post("/api/airframe/override_remove")
    async def remove_override(body: dict):
        sim.airframe.px4_overrides.pop(body.get("name", ""), None)
        return {"ok": True, "overrides": sim.airframe.px4_overrides}

    @app.post("/api/params/set_many")
    async def set_params_many(body: dict):
        """Write several parameters at once ({"params": {name: value}}), e.g. an RC calibration; saves them."""
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "vehicle is armed"}, status_code=409)
        wanted = {str(k): v for k, v in (body.get("params") or {}).items()}
        results = await run_in_threadpool(link.set_params, wanted, lambda name, res: None)
        ok = all(r["ok"] for r in results)
        if ok and body.get("save", True):
            link.preflight_storage(True)
        return {"ok": ok, "results": results}

    @app.post("/api/params/save")
    async def save_params():
        link.preflight_storage(True)
        return {"ok": True}

    @app.get("/api/params/meta")
    async def get_meta():
        return {"source": state.meta_source, "meta": state.meta}

    @app.post("/api/params/meta/fetch")
    async def fetch_meta():
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        meta, src = await run_in_threadpool(param_meta.fetch_from_vehicle, link, state.log)
        if meta:
            state.meta, state.meta_source = meta, src
            return {"ok": True, "count": len(meta), "source": src}
        return JSONResponse({"ok": False, "error": src}, status_code=500)

    # ------------------------------------------------------------ vehicle
    @app.post("/api/px4/command")
    async def px4_command(body: dict):
        cmd = body.get("command", "")
        if cmd in ("arm", "takeoff") and sim.paused:
            return JSONResponse({"ok": False, "error": "the simulation is paused; resume it first (Pause button)"}, status_code=409)
        if cmd in ("nose_lift_takeoff", "nose_lift_land"):
            # the ATLAS module's trigger, identical on SITL, HITL and the real aircraft (any GCS can send it)
            link.send_command_long(mavlink.MAV_CMD_USER_1, 1.0 if cmd == "nose_lift_takeoff" else 2.0)
            state.log(f"[px4] {'takeoff' if cmd == 'nose_lift_takeoff' else 'landing'} requested from the nose-lift module (MAV_CMD_USER_1)")
        elif cmd == "arm":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0 if body.get("force") else 0.0)
        elif cmd == "disarm":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0 if body.get("force") else 0.0)
        elif cmd == "kill":
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        elif cmd == "mode":
            main, sub = PX4_MODES.get(body.get("mode", ""), (None, None))
            if main is None:
                return JSONResponse({"ok": False, "error": "unknown mode"}, status_code=400)
            link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   float(main), float(sub))
        elif cmd == "takeoff":
            main, sub = PX4_MODES["takeoff"]
            link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                                   float(main), float(sub))
        elif cmd == "reboot":
            link.reboot()
        elif cmd == "save_params":
            link.preflight_storage(True)
        else:
            return JSONResponse({"ok": False, "error": "unknown command"}, status_code=400)
        return {"ok": True}

    # ---------------------------------------------------------------- sim
    @app.post("/api/sim/reset")
    async def sim_reset(body: dict | None = None):
        body = body or {}
        sim.reset(yaw=float(body.get("yaw", 0.0)))
        return {"ok": True}

    @app.post("/api/sim/pause")
    async def sim_pause(body: dict):
        sim.paused = bool(body.get("paused", not sim.paused))
        return {"ok": True, "paused": sim.paused}

    @app.post("/api/sim/speed")
    async def sim_speed(body: dict):
        sim.speed = max(0.0, float(body.get("speed", 1.0)))
        return {"ok": True, "speed": sim.speed}

    HITL_PLANT = "with a flight controller connected (HITL) the simulator is only the plant: nothing on this side may drive the motors or the sequence"

    @app.post("/api/sim/motor_override")
    async def motor_override(body: dict):
        v = body.get("values")
        if v is not None and state.conn.mode == "hitl":
            return JSONResponse({"ok": False, "error": HITL_PLANT}, status_code=409)
        sim.motor_override = None if v is None else [float(x) for x in v]
        return {"ok": True}

    @app.post("/api/sim/physics")
    async def sim_physics(body: dict):
        """Switch the live physics engine: {"physics": "python" | "jsbsim"}. The vehicle restarts on the ground."""
        name = str(body.get("physics", "python")).lower()
        if name not in sim.BACKENDS:
            return JSONResponse({"ok": False, "error": f"unknown physics '{name}'"}, status_code=400)
        if link.armed:
            return JSONResponse({"ok": False, "error": "disarm first"}, status_code=409)
        try:
            await run_in_threadpool(sim.set_physics, name)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"{type(e).__name__}: {e}"}, status_code=500)
        return {"ok": True, "physics": sim.physics}

    @app.post("/api/sim/nose_lift")
    async def sim_nose_lift(body: dict):
        """Start ({"motors": [8, 9], "target_pitch_deg": 25, "rate_deg_s": 8}) or stop ({"stop": true}) the nose-lift
        ground sequence. The snapshot's "nose_lift" field reports its state; arm PX4 once it is "holding"."""
        if body.get("stop"):
            sim.stop_nose_lift()
            return {"ok": True}
        if state.conn.mode == "hitl":
            return JSONResponse({"ok": False, "error": HITL_PLANT}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "disarm first: the nose lift runs before arming"}, status_code=409)
        if not sim.sim.on_ground:
            return JSONResponse({"ok": False, "error": "the vehicle is not on the ground"}, status_code=409)
        motors = [int(m) for m in body.get("motors") or []]
        if not motors:
            return JSONResponse({"ok": False, "error": "choose the motors that lift the nose"}, status_code=400)
        kw = {k: float(body[k]) for k in ("rate_deg_s", "kp", "ki", "kd", "max_cmd", "tolerance_deg", "hold_s", "fade_s", "k_ang", "kq", "kqi", "assist_cmd") if k in body}
        if body.get("assist_motors"):
            kw["assist_motors"] = [int(m) for m in body["assist_motors"]]
        nl = sim.start_nose_lift(motors, float(body.get("target_pitch_deg", sim.airframe.hover_pitch_deg)), **kw)
        return {"ok": True, "status": nl.status()}

    @app.post("/api/sim/wind")
    async def sim_wind(body: dict):
        sim.set_wind(float(body.get("north", 0)), float(body.get("east", 0)), float(body.get("down", 0)))
        return {"ok": True}

    @app.post("/api/sim/noise")
    async def sim_noise(body: dict):
        sim.sensors.noise.enabled = bool(body.get("enabled", True))
        return {"ok": True}

    @app.post("/api/sim/home")
    async def sim_home(body: dict):
        sim.sensors.set_home(float(body["lat"]), float(body["lon"]), float(body.get("alt", 0.0)))
        return {"ok": True}

    # Visible scenario runs share the interactive simulator and its websocket.
    live = {"runner": None, "hook": None}

    @app.post("/api/sim/scenario/start")
    async def live_scenario_start(body: dict):
        from ..sim.scenario import Scenario, ScenarioRunner, load_scenario
        if state.conn.mode != "sitl":
            return JSONResponse({"error": "Visible scripted tests require SITL"}, status_code=409)
        if link.status().get("armed") or (live["runner"] and not live["runner"].done):
            return JSONResponse({"error": "Disarm and finish the current test first"}, status_code=409)
        try:
            sc = load_scenario(SCENARIO_DIR / Path(str(body["scenario"])).name) if "scenario" in body else Scenario.from_dict(body)
        except (ValueError, FileNotFoundError) as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        if not sc.phases:
            return JSONResponse({"error": "A test needs phases"}, status_code=400)
        # Params must already have been pushed and verified; never block the sim hook on replies.
        if sc.params:
            return JSONResponse({"error": "Push scenario parameters before starting; omit params here"}, status_code=400)
        with sim.lock:
            if live["hook"] in sim.hooks:
                sim.hooks.remove(live["hook"])
            runner = ScenarioRunner(sc, state.link, log=state.log)
            runner.max_time += sim.t
            def step_live(simr):
                if state.conn.mode != "sitl":
                    runner.finish(simr, "connection changed", False)
                    simr.hooks.remove(step_live)
                    return
                runner(simr)
                if runner.done and step_live in simr.hooks:
                    simr.hooks.remove(step_live)
                    if not runner.ok:
                        simr.paused = True
            live.update(runner=runner, hook=step_live)
            sim.hooks.append(step_live)
            sim.speed = 1.0
            sim.paused = False
        return {"ok": True, "name": sc.name}

    @app.get("/api/sim/scenario")
    async def live_scenario_status():
        # Keep metric rows and their phase labels aligned while the sim records.
        with sim.lock:
            r = live["runner"]
            if r is None:
                return {"running": False}
            return json_safe({"running": not r.done, "phase": r.phase, **r.result(sim.airframe.mass.mass)})

    @app.get("/api/sim/scenario/timeseries")
    async def live_scenario_timeseries():
        with sim.lock:
            r = live["runner"]
            return r.metrics.timeseries() if r else {"columns": [], "rows": [], "phase": []}

    @app.post("/api/sim/scenario/stop")
    async def live_scenario_stop():
        if state.conn.mode != "sitl":
            return JSONResponse({"error": "Visible scripted tests require SITL"}, status_code=409)
        with sim.lock:
            r = live["runner"]
            if r and not r.done:
                r.finish(sim, "stopped", False)
            if live["hook"] in sim.hooks:
                sim.hooks.remove(live["hook"])
            sim.paused = True
        return {"ok": True, "paused": True}

    # ------------------------------------------------------------ scenarios / batch / studies
    @app.get("/api/scenarios")
    async def list_scenarios():
        out = []
        for p in sorted(SCENARIO_DIR.glob("*.json")):
            try:
                d = json.loads(p.read_text())
                out.append({"file": p.name, "name": d.get("name", p.stem), "description": d.get("description", ""),
                            "phases": [x.get("name") or x.get("type") for x in d.get("phases", [])]})
            except Exception:
                pass
        return {"scenarios": out}

    @app.get("/api/studies")
    async def list_studies():
        out = []
        for p in sorted(STUDY_DIR.glob("*.json")):
            try:
                d = json.loads(p.read_text())
                out.append({"file": p.name, "name": d.get("name", p.stem), "description": d.get("description", ""),
                            "variables": d.get("variables", []), "objective": d.get("objective")})
            except Exception:
                pass
        return {"studies": out}

    @app.post("/api/batch/run")
    async def batch_run(body: dict):
        """Headless run of a scenario on the current (or given) airframe in a private PX4 instance. Returns a job id;
        poll /api/batch/jobs. Never touches the interactive simulation."""
        from ..batch.worker import run_once
        scenario = body.get("scenario", "hover")
        af = Airframe.from_dict(body["airframe"]) if body.get("airframe") else sim.airframe.copy()
        variables = body.get("variables") or {}
        opts = dict(body.get("options") or {})
        job_id = f"job{int(time.time() * 1000) % 100000000}"
        job = {"id": job_id, "scenario": scenario, "variables": variables, "physics": str(opts.get("physics", "python")), "running": True,
               "t0": time.time(), "result": None, "log": []}
        state.batch_jobs[job_id] = job
        if len(state.batch_jobs) > 50:
            for k in list(state.batch_jobs)[:-50]:
                state.batch_jobs.pop(k, None)

        def run():
            try:
                r = run_once(af, scenario, variables=variables, px4_dir=state.args.px4_dir, log=lambda s: job["log"].append(s),
                             quiet=True, **{k: v for k, v in opts.items() if k in ("speed", "rate", "substeps", "noise", "seed", "timeout_wall", "physics")})
                r.pop("airframe", None)
                job["result"] = r
            except Exception as e:
                job["result"] = {"ok": False, "status": "error", "failures": [str(e)]}
            finally:
                job["running"] = False
                job["t1"] = time.time()

        import threading
        threading.Thread(target=run, name=f"batch-{job_id}", daemon=True).start()
        return {"ok": True, "id": job_id}

    @app.get("/api/batch/jobs")
    async def batch_jobs():
        jobs = []
        for j in state.batch_jobs.values():
            jobs.append({k: v for k, v in j.items() if k != "log"} | {"log": j["log"][-12:]})
        return {"jobs": jobs[::-1], "free_instances": [i for i in range(1, 10) if instance_is_free(i)]}

    @app.post("/api/study/run")
    async def study_run(body: dict):
        from ..batch.study import run_study, load_study
        if state.study_job.get("running"):
            return JSONResponse({"ok": False, "error": "a study is already running"}, status_code=409)
        spec = load_study(body["spec"]) if isinstance(body.get("spec"), str) else dict(body.get("spec") or {})
        if body.get("use_current_airframe", False) or "airframe" not in spec:
            spec["airframe"] = sim.airframe.to_dict()
        job = state.study_job
        job.update(running=True, name=spec.get("name", "study"), trials=[], summary=None, error=None, log=[], t0=time.time())

        def run():
            try:
                job["summary"] = run_study(spec, workers=body.get("workers"), log=lambda s: job["log"].append(s),
                                           progress=lambda t: job["trials"].append({k: v for k, v in t.items() if k not in ("metrics", "timing")}))
            except Exception as e:
                job["error"] = f"{type(e).__name__}: {e}"
            finally:
                job["running"] = False

        import threading
        threading.Thread(target=run, name="study", daemon=True).start()
        return {"ok": True}

    @app.get("/api/study/status")
    async def study_status():
        j = state.study_job
        return {"running": j.get("running", False), "name": j.get("name"), "trials": j.get("trials", [])[-200:],
                "summary": j.get("summary"), "error": j.get("error"), "log": j.get("log", [])[-20:]}

    @app.post("/api/airframe/apply_variables")
    async def airframe_apply_variables(body: dict):
        """Apply {path: value} to the live airframe (what a study's 'Apply best' does)."""
        try:
            af = apply_variables(sim.airframe, body.get("variables") or {})
        except Exception as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)
        sim.set_airframe(af, keep_state=True)
        return {"ok": True, "airframe": af.to_dict()}

    # ------------------------------------------------------------ websocket
    @app.websocket("/ws")
    async def ws(websocket: WebSocket):
        await websocket.accept()
        last_log = 0.0
        last_status = 0.0

        async def receive_loop():
            """Client -> server: joystick frames (and nothing else for now)."""
            while True:
                raw = await websocket.receive_text()
                try:
                    m = json.loads(raw)
                except Exception:
                    continue
                # the app's radio drives PX4 like QGC's joystick: on SITL always, on a real board only while it is in HIL
                # (simulated outputs); a receiver bound to the board needs no forwarding at all
                if m.get("type") == "manual" and link.ctl_connected and (state.conn.mode == "sitl" or bool(getattr(link, "hil_enabled", False))):
                    try:
                        await run_in_threadpool(link.send_manual_control, float(m.get("roll", 0)), float(m.get("pitch", 0)),
                                                float(m.get("throttle", 0)), float(m.get("yaw", 0)), int(m.get("buttons", 0)),
                                                [float(v) for v in (m.get("aux") or [])])
                    except Exception as e:
                        state.log(f"[joystick] send failed: {e}")

        rx_task = asyncio.create_task(receive_loop())
        try:
            await websocket.send_text(json.dumps({"type": "airframe", "airframe": sim.airframe.to_dict()}))
            while True:
                now = time.time()
                msg: dict[str, Any] = {"type": "state", "state": sim.snapshot()}
                ls = link.status()
                msg["rc"] = ls.get("rc") or {}                  # receiver on the flight controller (RC_CHANNELS)
                msg["manual"] = ls.get("manual") or {}          # last MANUAL_CONTROL the app sent (USB/serial radio)
                if now - last_status > 0.5:
                    msg["status"] = status_dict()
                    last_status = now
                new_logs = [e for e in state.log_buffer if e[0] > last_log]
                if new_logs:
                    msg["log"] = new_logs
                    last_log = new_logs[-1][0]
                await websocket.send_text(json.dumps(msg))
                await asyncio.sleep(1 / 30)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            rx_task.cancel()

    return app
