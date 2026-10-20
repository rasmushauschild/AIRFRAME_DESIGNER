"""FastAPI web server: serves the 3D UI, streams sim state over a websocket, exposes control REST endpoints."""
from __future__ import annotations

import asyncio
import json
import re
import shutil
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse, PlainTextResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.concurrency import run_in_threadpool

from ..geometry.airframe import Airframe, PRESETS, blank
from ..sim.wind import WindModel
from ..geometry.gear import generate_legs
from ..geometry import cad as cadmod
from ..geometry.paths import list_paths, apply_variables
from ..analysis import static as design
from ..analysis.geometric_optimiser import optimise as geometric_optimise, default_groups
from ..px4.link import mavlink
from ..px4 import param_meta
from ..sim.recorder import RunStore
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
    "manual": (1, 0), "altitude": (2, 0), "position": (3, 0), "acro": (5, 0), "offboard": (6, 0), "stabilized": (7, 0),
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


class SafeJSON(JSONResponse):
    """Every response goes out as JSON a browser can actually parse.

    A single non-finite float - a rate before the first sample, a telemetry field the aircraft has not
    filled in - makes the encoder refuse the whole response, and what reaches the user is "Internal Server
    Error" with no clue which number it was. There is a `json_safe` call at most of the places that can
    produce one; this is the place that cannot be forgotten.
    """

    def render(self, content) -> bytes:
        return json.dumps(json_safe(content), ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")


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
    app = FastAPI(title="AIRFRAME_DESIGNER", default_response_class=SafeJSON)
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
        """Always the current link, for reading *and* writing.

        Without the setter, `link.share = x` quietly lands on the proxy and the real link never hears it -
        the attribute is then shadowed for the rest of the session, because __getattr__ is only consulted
        when normal lookup fails.
        """

        def __getattr__(self, name):
            return getattr(state.link, name)

        def __setattr__(self, name, value):
            setattr(state.link, name, value)

    link = _LinkProxy()   # always resolves to the current link

    # -------------------------------------------------------------- pages
    def ui_token() -> str:
        """The newest change time anywhere in the UI, in milliseconds."""
        files = (f for f in UI_DIR.rglob("*") if f.is_file() and "vendor" not in f.parts)
        return str(max((f.stat().st_mtime_ns // 1_000_000 for f in files), default=0))

    @app.get("/")
    async def index():
        # A browser holds an ES module by URL, and no-store alone does not always dislodge it: an edited
        # app.js or scene.js could still be served from cache after a reload. The token in the script URL
        # changes whenever any UI file changes, which does.
        html = (UI_DIR / "index.html").read_text().replace("__UIV__", ui_token())
        return HTMLResponse(html, headers={"Cache-Control": "no-store"})

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
                s["firmware"] = {"board": True, "custom": True, "needs_rebuild": b["needs_rebuild"],
                                 "needs_flash": b["needs_flash"], "stale": b["stale"], "bootloader": b["bootloader"]}
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
        return s

    @app.get("/api/status")
    async def get_status():
        return status_dict()

    @app.get("/api/log")
    async def get_log(since: float = 0.0):
        return [e for e in state.log_buffer if e[0] > since]

    # ----------------------------------------------------------- airframe
    AUTOSAVE = AIRFRAME_DIR / "_autosave.json"
    _autosave = {"t": 0.0, "timer": None}

    def preset_path_for(name: str) -> Path:
        slug = re.sub(r"[^a-z0-9]+", "_", (name or "").strip().lower()).strip("_")
        return AIRFRAME_DIR / f"{slug}.json" if slug else AUTOSAVE

    def preset_path(af: Airframe) -> Path:
        """Where a preset lives: airframes/<its name, slugged>.json, or the scratch file when it has no name."""
        slug = re.sub(r"[^a-z0-9]+", "_", (af.name or "").strip().lower()).strip("_")
        return AIRFRAME_DIR / f"{slug}.json" if slug else AUTOSAVE

    def autosave(af: Airframe) -> None:
        """Every accepted edit, from any tab, is written straight back to the preset (at most twice a second).

        There is no Save button and no unsaved state: the file on disk is always what the app is flying. A
        version is a different thing entirely, taken on purpose and pushed to GitHub, and never overwritten
        by this.

        An edit that lands inside that half second is not dropped, it is held: the last change in a burst -
        the checkbox you ticked and then stopped touching - is exactly the one that must reach the disk.
        """
        def write() -> None:
            _autosave["t"] = time.time()
            _autosave["timer"] = None
            try:
                AIRFRAME_DIR.mkdir(exist_ok=True)
                af.save(preset_path(af))
            except Exception as e:
                state.log(f"[ui] autosave failed: {e}")

        if _autosave["timer"] is not None:
            _autosave["timer"].cancel()                  # a newer edit replaces the one waiting
            _autosave["timer"] = None
        wait = 0.5 - (time.time() - _autosave["t"])
        if wait <= 0:
            write()
            return
        _autosave["timer"] = threading.Timer(wait, write)
        _autosave["timer"].daemon = True
        _autosave["timer"].start()

    @app.get("/api/airframe")
    async def get_airframe():
        return sim.airframe.to_dict()

    @app.post("/api/airframe")
    async def set_airframe(body: dict):
        try:
            af = Airframe.from_dict(body.get("airframe", body))
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"invalid airframe ({type(e).__name__}: {e}); an empty number field?"}, status_code=400)
        af.resolve_mass()
        try:
            await run_in_threadpool(airfoils.ensure_polars, af, state.log)   # polar wings: section tables ready before the physics
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"airfoil polar: {e}"}, status_code=400)
        sim.set_airframe(af)
        autosave(af)
        hc = af.hover_check()
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    @app.get("/api/airframe/hover_check")
    async def hover_check():
        return json_safe(sim.airframe.hover_check())

    @app.get("/api/airframe/mount_margin")
    async def mount_margin():
        """How far each rotor could be mounted out of true, and what the aircraft as built can do now.

        Tens of milliseconds of arithmetic on a copy of the rotors, so it is asked for when the section
        that shows it is open, not on every poll.
        """
        return json_safe(await run_in_threadpool(sim.airframe.mount_margin))

    @app.get("/api/airframes")
    async def list_airframes():
        # files starting with _ are the app's own (the scratch file an unnamed design is written to)
        files = sorted(p.name for p in AIRFRAME_DIR.glob("*.json") if not p.name.startswith("_"))
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
        cancel_live("airframe changed")
        sim.set_airframe(af)
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

    @app.post("/api/airframe/new")
    async def new_airframe(body: dict | None = None):
        """Start a design from scratch, under a name that is not taken yet."""
        base = str((body or {}).get("name") or "New design").strip() or "New design"
        name, n = base, 1
        while preset_path(Airframe(name=name)).exists():
            n += 1
            name = f"{base} {n}"
        af = blank(name)
        af.resolve_mass()
        cancel_live("airframe changed")
        sim.set_airframe(af)
        autosave(af)
        hc = af.hover_check()
        state.log(f"[preset] new design {name}")
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    @app.post("/api/airframe/delete")
    async def delete_airframe(body: dict):
        """Delete a saved design. Never the one being flown: autosave would write it straight back, so the
        app opens another design first and then asks for this."""
        name = Path(str(body.get("name", ""))).name
        if not name.endswith(".json"):
            return JSONResponse({"ok": False, "error": "a design file is required"}, status_code=400)
        p = AIRFRAME_DIR / name
        if not p.is_file():
            return JSONResponse({"ok": False, "error": f"not found: {name}"}, status_code=404)
        if p.resolve() == preset_path(sim.airframe).resolve():
            return JSONResponse({"ok": False, "error": "this design is the one being flown"}, status_code=400)
        p.unlink()
        state.log(f"[preset] deleted {name}")
        return {"ok": True}

    @app.post("/api/airframe/rename")
    async def rename_airframe(body: dict):
        """Rename the design being flown: the name and its file, not a copy of it.

        The file a design lives in is its name slugged, so changing the name used to leave the old file
        behind and start a new one. A rename moves it."""
        name = str(body.get("name", "")).strip()
        if not name:
            return JSONResponse({"ok": False, "error": "a name is required"}, status_code=400)
        old_path = preset_path(sim.airframe)
        sim.airframe.name = name
        new_path = preset_path(sim.airframe)
        if new_path != old_path and new_path.exists():
            sim.airframe.name = Airframe.load(old_path).name if old_path.is_file() else sim.airframe.name
            return JSONResponse({"ok": False, "error": f"{new_path.name} already exists"}, status_code=409)
        if old_path.is_file() and new_path != old_path:
            old_path.rename(new_path)
            state.log(f"[preset] renamed {old_path.name} -> {new_path.name}")
        autosave(sim.airframe)
        return {"ok": True, "file": new_path.name, "name": name}

    @app.post("/api/airframe/duplicate")
    async def duplicate_airframe(body: dict | None = None):
        """Copy this design under a new name and open the copy. The original is left as it was."""
        body = body or {}
        base = str(body.get("name", "")).strip() or f"{sim.airframe.name} copy"
        name, n = base, 2
        while preset_path_for(name).exists():
            name, n = f"{base} {n}", n + 1
        af = sim.airframe.copy()
        af.name = name
        af.resolve_mass()
        sim.set_airframe(af)
        autosave(af)
        hc = af.hover_check()
        state.log(f"[preset] duplicated as {preset_path(af).name}")
        return json_safe({"ok": True, "airframe": af.to_dict(), "problems": af.validate() + hc["problems"], "hover": hc})

    # ------------------------------------------------------------------ CAD (STEP bodies -> masses, CG)
    CAD_DIR = AIRFRAME_DIR / "cad"

    def cad_file_path(model) -> Path:
        p = Path(model.file)
        return p if p.is_absolute() else PROJECT_DIR / p

    @app.post("/api/cad/import")
    async def cad_import(request: Request, filename: str = "model.step"):
        """Upload a model (raw bytes): it is copied to airframes/cad/ and attached to the live airframe.

        A STEP file gives solids, each measured and meshed, which can be given masses (masses/offsets of
        bodies with the same id survive a re-import). An OBJ or STL gives triangles only: a shape to look
        at, with no mass of its own."""
        data = await request.body()
        if not data:
            return JSONResponse({"ok": False, "error": "empty upload"}, status_code=400)
        CAD_DIR.mkdir(parents=True, exist_ok=True)
        stem = cadmod._safe_stem(filename)
        low = filename.lower()
        suffix = next((x for x in (".stp", ".obj", ".stl") if low.endswith(x)), ".step")
        dst = CAD_DIR / (stem + suffix)
        dst.write_bytes(data)
        try:
            imported = await run_in_threadpool(cadmod.import_any, dst, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"import failed: {e}"}, status_code=400)
        af = sim.airframe.copy()
        af.cad = cadmod.model_from_import(imported, str(dst.relative_to(PROJECT_DIR)), af.cad)
        af.resolve_mass()
        sim.set_airframe(af)
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
            imported = await run_in_threadpool(cadmod.import_any, path, state.log)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"import failed: {e}"}, status_code=400)
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
        sim.set_airframe(af)
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
        try:
            st["checklist"] = state.conn.checklist(export_params() if state.conn.mode == "hitl" else None)
        except ValueError as e:
            # The browser polls this one. A design whose real-flight set cannot be built must not take the
            # whole status with it - the switch, the link, the telemetry and the ports all live here.
            st["checklist"] = None
            st["checklist_error"] = str(e)
        return st

    @app.post("/api/connection/connect")
    async def connect(body: dict):
        mode = body.get("mode", "sitl")
        sim.paused = False          # a paused loop sends PX4 nothing; connecting while paused can only fail
        sim.recorder.sample_hz = 20.0      # the simulation's own rate; only a real flight on a cable beats it
        if mode == "hitl":
            r = await run_in_threadpool(state.conn.connect_hitl, body.get("serial"), body.get("baud"))
        else:
            r = await run_in_threadpool(state.conn.connect_sitl, body.get("launch"))
        return r

    @app.post("/api/connection/flight")
    async def connect_flight(body: dict):
        """Listen to a real flight: attach to the board (USB or a radio), inject nothing, record everything.

        The simulator stops being the aircraft's world the moment this is on - `PX4Link.listening` - and the
        loop follows the telemetry instead of its own physics."""
        dev = body.get("serial") or None
        baud = body.get("baud")          # left out = let the port decide (ConnectionManager.port_baud)
        if state.conn.mode != "hitl" or not link.ctl_connected or dev:
            r = await run_in_threadpool(state.conn.connect_hitl, dev, baud)
        else:
            r = state.conn.status()
        state.link.block_hil = True
        sim.paused = False                 # the loop is what follows the telemetry: a paused app watches nothing
        # A cable can feed the recorder as fast as it keeps samples; a radio cannot, and sampling faster
        # than the aircraft speaks only writes the same number down five times. The cost of the fast one
        # is the session buffer: 120k samples is 40 minutes at 50 Hz instead of 100 at 20.
        sim.recorder.sample_hz = 20.0 if link.link_budget_bps() else 50.0
        sim.motor_override = None
        await run_in_threadpool(state.link.request_flight_telemetry)
        state.log("[link] real flight: the simulator is only listening; no sensors are sent to the aircraft")
        return r

    # ------------------------------------------------- the switch: one press, whatever it takes
    mode_job: dict = {"thread": None}

    @app.post("/api/connection/mode")
    async def set_mode(body: dict):
        """SITL, HITL or FLIGHT, arranged in the background so the click never waits on a reboot.

        The simulator's own housekeeping belongs here rather than in the connection manager: what the loop
        does with the link is this app's business, not the board's.
        """
        mode = str(body.get("mode", ""))
        if mode not in ("sitl", "hitl", "real"):
            return JSONResponse({"ok": False, "error": "mode must be sitl, hitl or real"}, status_code=400)
        if mode == "real" and not body.get("confirmed", True):
            return JSONResponse({"ok": False, "error": "not confirmed"}, status_code=409)
        state.conn._mode_gen += 1
        gen = state.conn._mode_gen
        dev = body.get("serial") or None
        baud = body.get("baud")
        sim.paused = False                 # a paused loop neither flies the simulation nor follows a flight
        sim.motor_override = None
        cancel_live("mode change")

        def work():
            r = state.conn.set_mode(mode, dev, baud, gen=gen)
            if gen != state.conn._mode_gen:
                return                     # someone pressed something else; their job owns the state now
            if r.get("ok") and mode == "real":
                state.link.block_hil = True
                sim.recorder.sample_hz = 20.0 if state.link.link_budget_bps() else 50.0
                state.log("[link] real flight: the simulator is only listening; no sensors are sent to the aircraft")
            elif r.get("ok"):
                sim.recorder.sample_hz = 20.0

        t = threading.Thread(target=work, name=f"mode-{mode}", daemon=True)
        mode_job["thread"] = t
        t.start()
        return {"ok": True, "job": state.conn.mode_state()}

    @app.get("/api/connection/mode")
    async def get_mode():
        return {"ok": True, "job": state.conn.mode_state()}

    @app.post("/api/connection/telemetry")  # noqa: E301
    async def set_telemetry(body: dict):
        """How much of a narrow link the telemetry may have: light, balanced or full.

        Over a radio this is the resolution dial. The rates are worked out from the link's own speed, so
        the same choice means more on a faster radio.
        """
        share = str(body.get("share", "balanced"))
        if share not in link.SHARES:
            return JSONResponse({"ok": False, "error": f"share must be one of {', '.join(link.SHARES)}"}, status_code=400)
        link.share = share
        if link.listening and link.ctl_connected:      # HITL has its own streams; this is the real-flight one
            await run_in_threadpool(link.request_flight_telemetry)
        return {"ok": True, "telemetry": link.status().get("telemetry")}

    @app.post("/api/connection/disconnect")
    async def disconnect():
        return await run_in_threadpool(state.conn.disconnect)

    @app.post("/api/px4/reset_all")
    async def px4_reset_all():
        cancel_live("reset")
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

    real_flight_lock = asyncio.Lock()

    # ------------------------------------------------------------------ custom firmware modules
    FW_ROOT = (PROJECT_DIR / "firmware" / "atlas").resolve()
    FW_EDITABLE = (".cpp", ".hpp", ".h", ".c", ".yaml", ".msg", ".txt", ".md", ".cmake")

    def fw_files() -> list[str]:
        """Every source file of the custom modules, as paths relative to firmware/atlas."""
        if not FW_ROOT.is_dir():
            return []
        out = []
        for f in sorted(FW_ROOT.rglob("*")):
            if not f.is_file() or f.suffix.lower() not in FW_EDITABLE:
                continue
            rel = f.relative_to(FW_ROOT)
            if rel.parts and rel.parts[0] in ("build", "results"):
                continue
            out.append(str(rel))
        return out

    def fw_resolve(rel: str) -> Path | None:
        """Resolve a path inside firmware/atlas, or None. Anything that escapes the tree is None."""
        try:
            p = (FW_ROOT / rel).resolve()
        except (OSError, ValueError):
            return None
        if FW_ROOT not in p.parents or p.suffix.lower() not in FW_EDITABLE:
            return None
        return p

    @app.post("/api/firmware/modules/enable")
    async def firmware_module_enable(body: dict):
        """Switch a firmware module on or off for this design.

        Off means two things and the second is the point: the build stops being the only question, and the
        module's parameters leave the export - so the Parameters table stops showing settings for a module
        this aircraft does not use. The names come from the module's own params.yaml and are kept with the
        design, so the export is the same wherever it runs.
        """
        d = str(body.get("dir", ""))
        on = bool(body.get("on", True))
        groups = firmware_modules()["groups"]
        g = next((x for x in groups if x.get("dir") == d and x.get("kind") == "module"), None)
        if g is None:
            return JSONResponse({"ok": False, "error": f"no module at {d}"}, status_code=404)
        off = dict(sim.airframe.design.get("modules_off") or {})
        if on:
            off.pop(d, None)
        else:
            off[d] = list(g.get("params") or [])
        sim.airframe.design["modules_off"] = off
        autosave(sim.airframe)
        state.log(f"[firmware] {g['name']} is {'in' if on else 'not in'} this design"
                  + ("" if on else f" ({len(off[d])} parameters leave the export)"))
        return {"ok": True, "modules_off": off}

    @app.get("/api/firmware/modules")
    def firmware_modules():
        """The custom modules in firmware/atlas, with their sources.

        A module is a directory whose CMakeLists.txt calls px4_add_module; that is PX4's own definition and
        it is the one the build uses. Anything else under firmware/atlas that holds editable sources — the
        allocator overlay, the uORB messages — is listed beside them as a group, because they are yours to
        edit too even though the build treats them differently. `user_command` says whether the module
        answers MAV_CMD_USER_1, which is what the Mission tab hands a takeoff or a landing to."""
        groups, claimed = [], set()
        for cmake in sorted(FW_ROOT.rglob("CMakeLists.txt")) if FW_ROOT.is_dir() else []:
            text = cmake.read_text(errors="replace")
            if "px4_add_module" not in text:
                continue
            d = cmake.parent
            name = (re.search(r"MODULE\s+(\S+)", text) or [None, d.name])[1]
            files = [str(f.relative_to(FW_ROOT)) for f in sorted(d.rglob("*"))
                     if f.is_file() and f.suffix.lower() in FW_EDITABLE]
            claimed.update(files)
            src = "\n".join((FW_ROOT / f).read_text(errors="replace") for f in files if f.endswith((".cpp", ".hpp", ".h")))
            groups.append({"kind": "module", "name": d.name, "module": str(name),
                           "dir": str(d.relative_to(FW_ROOT)), "files": files,
                           "enabled": str(d.relative_to(FW_ROOT)) not in (sim.airframe.design.get("modules_off") or {}),
                           "user_command": "MAV_CMD_USER_1" in src,
                           "params": sorted({m for m in re.findall(r"^\s{0,4}([A-Z][A-Z0-9_]{2,15}):", 
                                                                   (d / "params.yaml").read_text(errors="replace"), re.M)})
                                     if (d / "params.yaml").is_file() else []})
        # Everything else under firmware/atlas, named for what it is rather than for its directory: a
        # group called "." or "src" tells you nothing about what is in it.
        WHAT = {"allocator_overlay": "PX4 allocator, patched",
                "msg": "uORB messages of your own",
                "src": "which modules the build includes",
                ".": "notes and board files"}
        other: dict[str, list[str]] = {}
        for f in fw_files():
            if f in claimed:
                continue
            other.setdefault(f.split("/")[0] if "/" in f else ".", []).append(f)
        groups += [{"kind": "sources", "name": WHAT.get(k, k), "module": "", "dir": k, "files": v,
                    "user_command": False, "params": []}
                   for k, v in sorted(other.items())]
        return {"groups": groups}

    # PX4 v1.18 ModuleBase is not a template: a module owns a static Descriptor and hands it to
    # ModuleBase::main. The scaffold follows the one module in this tree so a new one builds unchanged.
    MODULE_MAIN = """// {name}: a PX4 module of your own. It runs on SITL and, once you are sure of it, on the board.
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <string.h>

using namespace time_literals;      // 100_ms and friends

class {cls} : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{{
public:
	static Descriptor desc;

	{cls}() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {{}}

	static int task_spawn(int, char **)
	{{
		auto *p = new {cls}();
		if (!p) {{ return PX4_ERROR; }}
		desc.object.store(p);
		desc.task_id = task_id_is_work_queue;
		p->ScheduleOnInterval(100_ms);          // ten times a second; change it to suit
		return PX4_OK;
	}}

	static int custom_command(int, char **) {{ return print_usage("unknown command"); }}

	static int print_usage(const char *reason = nullptr)
	{{
		if (reason) {{ PX4_WARN("%s", reason); }}
		PRINT_MODULE_USAGE_NAME("{name}", "module");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return PX4_OK;
	}}

private:
	void Run() override
	{{
		if (should_exit()) {{ ScheduleClear(); exit_and_cleanup(desc); return; }}

		vehicle_local_position_s pos{{}};
		if (_local_position.update(&pos)) {{
			// Your code here. pos.x, pos.y, pos.z are the estimate in metres, NED (z is down).
		}}
	}}

	uORB::Subscription _local_position{{ORB_ID(vehicle_local_position)}};
}};

ModuleBase::Descriptor {cls}::desc{{{cls}::task_spawn, {cls}::custom_command, {cls}::print_usage}};

extern "C" __EXPORT int {name}_main(int argc, char *argv[]) {{ return ModuleBase::main({cls}::desc, argc, argv); }}
"""

    MODULE_CMAKE = """px4_add_module(
	MODULE external__{name}
	MAIN {name}
	SRCS {cls}.cpp
	EXTERNAL
)

target_link_libraries(external__{name} PRIVATE px4_work_queue)
"""

    @app.post("/api/firmware/module/new")
    def firmware_module_new(body: dict):
        """Start a module: a directory, a CMakeLists.txt that adds it, and a source file that builds.

        It is deliberately the smallest thing that compiles and runs — a work item on a timer with the
        local position subscribed — because an empty file that does not build teaches nothing, and a
        module full of someone else's logic is harder to read than to write.
        """
        name = re.sub(r"[^a-z0-9_]+", "_", str(body.get("name", "")).strip().lower()).strip("_")
        if not name or not name[0].isalpha():
            return JSONResponse({"ok": False, "error": "a module name starts with a letter: letters, digits and _"}, status_code=400)
        d = FW_ROOT / "src" / name
        if d.exists():
            return JSONResponse({"ok": False, "error": f"{name} already exists"}, status_code=409)
        cls = "".join(p.title() for p in name.split("_"))
        d.mkdir(parents=True)
        (d / "CMakeLists.txt").write_text(MODULE_CMAKE.format(name=name, cls=cls))
        (d / f"{cls}.cpp").write_text(MODULE_MAIN.format(name=name, cls=cls))
        # the build only looks at the modules named in src/CMakeLists.txt
        lists = FW_ROOT / "src" / "CMakeLists.txt"
        txt = lists.read_text()
        m = re.search(r"set\(config_module_list_external([^)]*)PARENT_SCOPE\)", txt)
        if m and name not in m.group(1).split():
            txt = txt[:m.start(1)] + m.group(1).rstrip() + f" {name} " + txt[m.end(1):]
            lists.write_text(txt)
        elif not m:
            lists.write_text(txt + f"\nset(config_module_list_external {name} PARENT_SCOPE)\n")
        state.log(f"[firmware] new module {name} in src/{name}")
        return {"ok": True, "name": name, "dir": f"src/{name}", "files": [f"src/{name}/CMakeLists.txt", f"src/{name}/{cls}.cpp"]}

    @app.get("/api/firmware/files")
    async def firmware_files():
        return {"root": str(FW_ROOT), "files": fw_files()}

    @app.get("/api/firmware/file")
    async def firmware_file(path: str):
        p = fw_resolve(path)
        if p is None or not p.is_file():
            return JSONResponse({"ok": False, "error": "no such module file"}, status_code=404)
        return {"ok": True, "path": path, "text": p.read_text(errors="replace")}

    @app.post("/api/firmware/file")
    async def firmware_file_write(body: dict):
        p = fw_resolve(str(body.get("path", "")))
        if p is None or not p.is_file():
            return JSONResponse({"ok": False, "error": "no such module file"}, status_code=404)
        text = body.get("text")
        if not isinstance(text, str):
            return JSONResponse({"ok": False, "error": "text required"}, status_code=400)
        p.write_text(text)
        state.log(f"[firmware] wrote {p.relative_to(FW_ROOT)} ({len(text)} bytes). Build to compile it.")
        return {"ok": True, "path": str(p.relative_to(FW_ROOT))}

    @app.post("/api/firmware/open")
    async def firmware_open(body: dict):
        """Open a module file in VS Code, where editing C++ is actually pleasant."""
        p = fw_resolve(str(body.get("path", "")))
        if p is None or not p.is_file():
            return JSONResponse({"ok": False, "error": "no such module file"}, status_code=404)
        code = shutil.which("code") or next((c for c in (
            "/usr/local/bin/code",
            "/Applications/Visual Studio Code.app/Contents/Resources/app/bin/code") if Path(c).exists()), None)
        if code is None:
            return JSONResponse({"ok": False, "error": "VS Code's 'code' command is not on the PATH "
                                                      "(VS Code: Shell Command: Install 'code' command in PATH)"},
                                status_code=404)
        try:
            subprocess.Popen([code, str(FW_ROOT), "--goto", str(p)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as e:
            return JSONResponse({"ok": False, "error": f"could not start VS Code: {e}"}, status_code=500)
        return {"ok": True, "path": str(p.relative_to(FW_ROOT))}

    @app.post("/api/firmware/real_flight")
    async def prepare_real_flight(body: dict):
        if body.get("confirmed") is not True:
            return JSONResponse({"ok": False, "error": "Confirm the real-flight conversion in Connect first"}, status_code=409)
        if real_flight_lock.locked():
            return JSONResponse({"ok": False, "error": "Real-flight setup is already running"}, status_code=409)
        async with real_flight_lock:
            return await run_in_threadpool(state.conn.prepare_real_flight)

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

    @app.get("/api/preset/versions")
    async def preset_versions():
        """Versions of the preset currently loaded, newest first."""
        a = state.conn.archive
        await run_in_threadpool(a.ensure)
        name = (sim.airframe.name or "").strip()
        rows = [v for v in a.versions() if (v.get("airframe") or "") == name]
        return {"name": name, "versions": rows, "archive": a.status()}

    @app.post("/api/preset/push")
    async def preset_push(body: dict | None = None):
        """Push this preset to GitHub as a new version: geometry, every parameter, and the firmware sources."""
        body = body or {}
        af = sim.airframe
        af.resolve_mass()
        autosave(af)                                    # the working file first, so the version matches disk
        return await run_in_threadpool(state.conn.preset_snapshot, str(body.get("note", "")))

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
        sim.set_airframe(af)
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
        if hitl and link.params.get("SYS_HITL", {}).get("value") == 0:
            from ..px4.real_flight import flight_params
            return flight_params(sim.airframe)
        return sim.airframe.px4_params(hitl=True) if hitl else sim.airframe.px4_params_sitl()

    @app.get("/api/px4/export")
    async def get_export():
        try:
            params = export_params()
        except ValueError as e:
            # A design the real-flight set cannot be built for (motors not wired to ports yet, more motors
            # than the board has outputs). That is a sentence to read, not a 500 to stare at.
            return JSONResponse({"ok": False, "error": str(e), "params": {}, "current": {}, "overrides": {},
                                 "geometry_keys": [], "file": "",
                                 "problems": [str(e)] + sim.airframe.validate()}, status_code=409)
        current = {k: link.params.get(k, {}).get("value") for k in params}
        ov = sim.airframe.px4_overrides or {}
        geometry = sim.airframe.px4_params(hitl=link.mode == "hitl", overrides=False)
        return json_safe({"params": params, "current": current, "problems": sim.airframe.validate() + sim.airframe.hover_check()["problems"],
                "file": "\n".join(["# AIRFRAME_DESIGNER parameters", *[f"1\t1\t{k}\t{v}\t{6 if isinstance(v, int) else 9}" for k, v in export_params().items()]]) + "\n",
                # "geometry dependent" is not a curated list: it is exactly the set the design works out
                # and keeps working out, so a row sits in that section whether or not you have taken it
                # over, and the value beside it is what the geometry would say if you had not.
                "overrides": ov, "geometry": geometry, "geometry_keys": [k for k in geometry if k not in ov],
                # parameters of a module this design has switched off: not exported, and not shown either -
                # the value is kept, so switching the module back on brings it back as it was
                "modules_off": sorted(sim.airframe.modules_off_params()),
                # and what the same design is when it is a real aircraft rather than a simulation, so a
                # row can say "this one is not what you will be flying" before you find out in the air
                **flight_view()})

    def flight_view() -> dict:
        """The real-flight set beside the simulator's, and why each parameter is or is not in both."""
        from ..px4.real_flight import flight_note, flight_params
        sitl = sim.airframe.px4_params_sitl()
        try:
            flight = flight_params(sim.airframe)
            error = ""
        except ValueError as e:
            flight, error = {}, str(e)
        return {"flight": flight, "flight_error": error, "sitl": sitl,
                "flight_notes": {k: flight_note(k) for k in sitl if k not in flight}}

    @app.get("/api/px4/export.params")
    async def get_export_file():
        return PlainTextResponse("\n".join(["# AIRFRAME_DESIGNER parameters", *[f"1\t1\t{k}\t{v}\t{6 if isinstance(v, int) else 9}" for k, v in export_params().items()]]) + "\n",
                                 headers={"Content-Disposition": f'attachment; filename="{sim.airframe.name}.params"'})

    @app.get("/api/px4/firmware")
    async def px4_firmware_status():
        return state.conn.firmware_status(max_age=0.0)   # fresh: scripts ask right after editing a source

    @app.post("/api/px4/firmware/update")
    async def px4_firmware_update(body: dict | None = None):
        """Rebuild the SITL firmware if its sources changed and relaunch PX4 on it (what Update PX4 does first)."""
        r = await run_in_threadpool(state.conn.update_firmware, (body or {}).get("relaunch", True))
        return r if r.get("ok") else JSONResponse(r, status_code=500)

    push_job: dict = {"total": 0, "done": 0, "failed": 0, "running": False, "step": ""}

    @app.post("/api/px4/push")
    async def push_params(body: dict | None = None):
        body = body or {}
        push_job.update(total=0, done=0, failed=0, running=True, step="getting ready")
        fw_res = {"ok": True, "rebuilt": False, "relaunched": False, "flashed": False}
        # Listening to a real flight: the parameters go out over whatever link there is (USB or a radio) and
        # nothing is built or flashed - that needs the board on a cable, and the aircraft is not on one.
        firmware_wanted = bool(body.get("firmware", True)) and not link.listening
        if firmware_wanted and state.conn.mode == "sitl" and getattr(state.args, "launch_px4", False):
            if link.armed:
                push_job.update(running=False, step='')
                return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before updating PX4"}, status_code=409)
            fw_res = await run_in_threadpool(state.conn.update_firmware)
            if not fw_res.get("ok"):
                push_job.update(running=False, step='')
                return JSONResponse({"ok": False, "error": fw_res.get("error"), "firmware": fw_res}, status_code=500)
        elif firmware_wanted and state.conn.mode == "hitl":
            # The firmware is the project's, not the design's. This used to flash only when the design
            # exported NLF_ENABLE = 1, so a design with the nose-lift module switched off - or any design
            # that does not use it - left the board on whatever it had while the button still read
            # "Flash firmware" and the banner still said the firmware was out of date. The button and the
            # action have to mean the same thing: a board whose image is stale gets the image.
            b = state.conn.board_firmware_status()
            if b["needs_rebuild"] or b["needs_flash"]:
                if link.armed:
                    push_job.update(running=False, step='')
                    return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before flashing"}, status_code=409)
                push_job.update(step="flashing the board" if not b["needs_rebuild"] else "building the firmware")
                fw_res = await run_in_threadpool(state.conn.update_board_firmware)
                if not fw_res.get("ok"):
                    push_job.update(running=False, step='')
                    return JSONResponse({"ok": False, "error": fw_res.get("error"), "firmware": fw_res}, status_code=500)
                if b["bootloader"]:
                    # It had no firmware a moment ago, so it has no parameters either and the link that
                    # follows is a fresh boot. Saying so and stopping beats a hundred failed writes.
                    push_job.update(running=False, step='')
                    return {"ok": True, "firmware": fw_res, "results": [], "flashed_only": True,
                            "note": "the board was in its bootloader and has been flashed; press Update PX4 "
                                    "again once it has booted to write the parameters"}
        if not link.ctl_connected:
            push_job.update(running=False, step='')
            return JSONResponse({"ok": False, "error": "PX4 control link not connected"}, status_code=409)
        if state.conn.mode == "hitl" and state.conn.detected_board().get("bootloader"):
            # The link reports "up" because the loader's USB enumerates and heartbeats arrive as sysid 0;
            # it is not a vehicle, and every parameter written at it fails.
            push_job.update(running=False, step='')
            return JSONResponse({"ok": False, "error": "the board is in its bootloader, not running firmware: "
                                                       "flash it (Flash firmware) before writing parameters"},
                                status_code=409)
        if link.armed:
            push_job.update(running=False, step='')
            return JSONResponse({"ok": False, "error": "vehicle is armed; disarm before updating PX4"}, status_code=409)
        try:
            params = export_params()
        except ValueError as e:
            # A design the real-flight set cannot be built for - motors not on ports, two motors on one -
            # is a sentence to read, not a 500 to stare at. The export endpoint has always answered this
            # way; the push forgot to, and "Internal Server Error" is all the button could say.
            push_job.update(running=False, step='')
            return JSONResponse({"ok": False, "error": str(e), "problems": [str(e)] + sim.airframe.validate()},
                                status_code=409)
        only = body.get("only")
        if only:
            params = {k: v for k, v in params.items() if k in only}
        # A wiring is the whole wiring. Setting the ports this design uses and leaving the rest as they
        # were means every re-map leaves a ghost behind: the board ends up with one motor on three
        # outputs, and testing it spins three fans. So any other output still pointing at one of this
        # design's motors is cleared - the same thing `real_flight.apply_verified` has always done on the
        # way to a real flight, which is where the asymmetry came from. A servo or any other function is
        # left alone: only motor numbers this design owns are taken back.
        if not only and any(k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) for k in params):
            mine = {int(v) for k, v in params.items() if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) and int(v)}
            stale = {k: 0 for k, e in (link.params or {}).items()
                     if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) and k not in params and int(e["value"]) in mine}
            if stale:
                state.log(f"[export] clearing {len(stale)} output(s) still assigned to this design's motors: "
                          + ", ".join(sorted(stale)))
                params.update(stale)
        complete = bool(link.params) and link.param_count and link.status().get("params_loaded", 0) >= link.param_count
        if link.mode == "hitl" and link.params.get("SYS_HITL", {}).get("value") == 0 and complete:
            # Only a complete list proves an output is absent. Over a radio the list is not downloaded
            # unless you ask for it, and "I have not looked" is not "it is not there".
            missing_outputs = [k for k in params if k.startswith("PWM_") and k not in link.params]
            if missing_outputs:
                push_job.update(running=False, step='')
                return JSONResponse({"ok": False, "error": "Real-flight setup incomplete; missing outputs: " + ", ".join(missing_outputs)}, status_code=409)
        # skip params this firmware does not have (HIL_ACT on SITL, the NLF_* nose-lift module on a stock board).
        # While the parameter list is still downloading, keep the NLF_* ones: they belong to the ATLAS SITL build and
        # are simply not listed yet.
        # PX4 lists a module's parameters only once something has used them; the nose-lift model parameters
        # (NLF_MASS, NLF_MOM, ...) are read by param_find at takeoff, so they are settable but not yet listed.
        has_module = "NLF_ENABLE" in (link.params or {})
        if link.params and complete:
            keep = lambda k: k in link.params or (k.startswith("NLF_") and has_module)
            # Only a complete list is evidence that a parameter is absent. With a partial one - a radio
            # link, where the download is not automatic - every parameter is tried and the ones this
            # firmware has not got come back as individual failures, which is the honest answer anyway.
            missing = [k for k in params if not keep(k)]
            params = {k: v for k, v in params.items() if keep(k)}
            if complete and any(k.startswith("NLF_") for k in missing):
                state.log("[export] this firmware has no atlas_nose_lift module (NLF_* parameters unknown): the native "
                          "ground sequence is unavailable on it; the simulator's own nose-lift hook is used instead")
        else:
            missing = []
        state.export_log.clear()
        push_job.update(total=len(params), done=0, failed=0, running=True, step="writing parameters")
        rot_before = link.params.get("SENS_BOARD_Y_OFF", {}).get("value")

        def progress(name, res):
            state.export_log.append({"t": time.time(), **res})
            push_job["done"] += 1
            push_job["failed"] += 0 if res.get("ok") else 1

        # Over a radio every write waits for its echo behind whatever telemetry is already queued, and the
        # answer can take longer than the wait. So the streams stop for the duration and start again after:
        # a hundred parameters that all arrive beats a live graph during the ten seconds it takes.
        narrow = link.listening and link.link_budget_bps() is not None
        if narrow:
            await run_in_threadpool(link.quiet_streams)
        try:
            results = await run_in_threadpool(link.set_params, params, progress,
                                              *( (6.0,) if narrow else () ))
        finally:
            if narrow and link.ctl_connected and link.listening:
                await run_in_threadpool(link.request_flight_telemetry)
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
        # Then everything else: a board is meant to be the simulator you tuned in, and a parameter this
        # design never mentions is still a parameter that has to match. What is never copied from a
        # simulator to an aircraft is the aircraft's own truth - calibration, RC, serial links, battery,
        # safety checks, failsafes and the mode slots (ConnectionManager.SYNC_EXCLUDE); writing those
        # would hand a flying machine a laptop's idea of where its sensors point.
        rest = {"written": 0, "skipped": 0, "why": ""}
        if body.get("rest", True) and link.mode == "hitl" and not link.armed:
            # To know what differs the board's list has to be read. On a cable that is seconds, so it is
            # simply done; over a radio it is ten minutes and is left to the Refresh button to ask for.
            if not complete and link.link_budget_bps() is None:
                await run_in_threadpool(link.fetch_all_params)
            diff = state.conn.sitl_diff()
            if not diff.get("ok"):
                rest["why"] = diff.get("error", "")
            elif diff["differ"]:
                r2 = await run_in_threadpool(state.conn.sync_to_sitl)
                rest = {"written": r2.get("written", 0), "skipped": r2.get("skipped", 0),
                        "why": "" if r2.get("ok") else f"{len(r2.get('failed', []))} would not write",
                        "failed": r2.get("failed", [])}
            else:
                rest["skipped"] = diff.get("skipped", 0)
        push_job.update(running=False, step="done")
        return {"ok": ok, "results": results, "missing": missing, "firmware": fw_res, "rest": rest}

    # ---------------------------------------------------------- parameters
    @app.get("/api/px4/push/progress")
    async def push_progress():
        """How far Update PX4 has got. Polled while the push itself is still in flight, so a hundred
        parameters over a radio is a number going up rather than a button that looks stuck."""
        return {"ok": True, **push_job}

    @app.get("/api/params")
    async def get_params():
        return {"count": link.param_count, "params": link.params}

    @app.get("/api/params/one")
    async def get_one_param(name: str):
        """Read one parameter straight from the vehicle. One round trip, which over a radio is the
        difference between an answer and a ten-minute download of all of them."""
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "no link to the aircraft"}, status_code=409)
        info = await run_in_threadpool(link.get_param, name)
        if info is None:
            return JSONResponse({"ok": False, "error": f"the vehicle has no parameter {name}"}, status_code=404)
        return {"ok": True, "name": name, "value": info["value"], "type": info.get("type")}

    @app.get("/api/px4/shell")
    async def px4_shell(c: str, timeout: float = 8.0):
        """Run one PX4/NuttX shell command on whatever is on the other end of the link.

        `PX4Link.shell` has always been here - `mavlink status` is how `board_send_cap` finds out which
        instance can hear us - but only the app could reach it. A bench test is mostly `listener`, and
        `listener` lives in the flight controller, so it needs a door: one command in, its output back as
        plain text, over the same radio the app is already holding (nothing else can open that port).
        It works on a listening link: SERIAL_CONTROL asks the aircraft a question, it does not fly it.
        """
        if not link.ctl_connected:
            return PlainTextResponse("no link to the aircraft\n", status_code=409)
        out = await run_in_threadpool(link.shell, c, timeout)
        return PlainTextResponse(out if out else "(no output: the command printed nothing, or the link "
                                                "dropped the reply - try a longer timeout)\n")

    @app.post("/api/px4/bench")
    async def px4_bench(body: dict | None = None):
        """Stop (or start) the aircraft's land detector, so a bench test can move it by hand.

        Nothing about a hand-carried aircraft is in the air, and PX4 knows it: `vehicle_land_detected`
        says `at_rest` while it is landed and turning slower than 3 degrees a second (hardcoded in
        LandDetector.cpp, not a parameter), and `ZeroVelocityUpdate` then fuses a zero velocity every
        200 ms. That observation is far stronger than the flow, so the position is pinned and the H-flow
        looks broken when it is working. Without the module publishing, EKF2's own default for the flag
        is false (`systemFlagUpdate.at_rest`), so the flow is the only thing left saying how fast the
        aircraft is going - which is the measurement we came to look at.

        It is `land_detector stop` over the link: nothing is written to flash and a reboot restores it.
        """
        on = bool((body or {}).get("on", True))
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "no link to the aircraft"}, status_code=409)
        if link.armed:
            return JSONResponse({"ok": False, "error": "the aircraft is armed: this is a bench test"}, status_code=409)
        out = await run_in_threadpool(link.shell, f"land_detector {'stop' if on else 'start'}", 8.0)
        status = await run_in_threadpool(link.shell, "land_detector status", 8.0)
        running = "running" in status and "not running" not in status
        state.log(f"[bench] land detector {'stopped' if not running else 'running'}"
                  + (" - the estimator will no longer hold the aircraft still, and cannot detect a landing" if not running else ""))
        return {"ok": True, "land_detector": "running" if running else "stopped", "wanted_off": on,
                "out": (out + status).strip()}

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

    @app.get("/api/px4/sitl_diff")
    async def px4_sitl_diff():
        return state.conn.sitl_diff()

    @app.post("/api/px4/sync_sitl")
    async def px4_sync_sitl():
        """Make the board's parameters identical to the last SITL run (hardware/calibration/safety ones excepted)."""
        r = await run_in_threadpool(state.conn.sync_to_sitl)
        return r if r.get("ok") else JSONResponse(r, status_code=409)

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
        if body.get("expected_mode") and state.conn.mode != body["expected_mode"]:
            return JSONResponse({"ok": False, "error": "Connection mode changed; command cancelled"}, status_code=409)
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
        cancel_live("reset")
        await run_in_threadpool(sim.reset, float(body.get("yaw", 0.0)))
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

    from ..px4.motor_test import MotorTest
    hardware_motor_test = MotorTest()

    @app.post("/api/px4/motor_test")
    async def motor_test(body: dict):
        try:
            if body.get("confirmed") is not True:
                raise ValueError("Enable motor testing and acknowledge the bench-test precautions first")
            return hardware_motor_test.send(link, body.get("motor"), float(body.get("value", 0)), len(sim.airframe.active_rotors()))
        except (ValueError, TypeError, AttributeError) as exc:
            return JSONResponse({"ok": False, "error": str(exc)}, status_code=409)

    @app.post("/api/sim/motor_override")
    async def motor_override(body: dict):
        v = body.get("values")
        if v is not None and state.conn.mode == "hitl":
            return JSONResponse({"ok": False, "error": HITL_PLANT}, status_code=409)
        sim.motor_override = None if v is None else [float(x) for x in v]
        return {"ok": True}


    @app.get("/api/sim/wind")
    async def sim_wind_get():
        return {"ok": True, "wind": sim.wind.to_dict(), "now": sim.sim.wind_ned.tolist()}

    @app.post("/api/sim/wind")
    async def sim_wind(body: dict):
        """Set the wind: either a plain {north, east, down} vector, or the whole model with its gusts."""
        model = sim.set_wind_model(WindModel.from_dict(body))
        return {"ok": True, "wind": model.to_dict(), "now": sim.sim.wind_ned.tolist()}

    @app.post("/api/sim/wind/preview")
    async def sim_wind_preview(body: dict):
        """What that model would blow over the next minute. Drawn in the Wind section; changes nothing."""
        return {"ok": True, **WindModel.from_dict(body).preview(float(body.get("seconds", 60.0)))}

    @app.post("/api/sim/noise")
    async def sim_noise(body: dict):
        sim.sensors.noise.enabled = bool(body.get("enabled", True))
        return {"ok": True}

    @app.post("/api/sim/home")
    async def sim_home(body: dict):
        sim.sensors.set_home(float(body["lat"]), float(body["lon"]), float(body.get("alt", 0.0)))
        return {"ok": True}

    # Visible scenario runs share the interactive simulator and its websocket.
    live = {"runner": None, "hook": None, "preparing": False, "generation": 0}

    def cancel_live(reason):
        with sim.lock:
            live["generation"] += 1
            runner = live["runner"]
            if runner and not runner.done:
                runner.finish(sim, reason, False)
            if live["hook"] in sim.hooks:
                sim.hooks.remove(live["hook"])


    @app.post("/api/sim/scenario/start")
    async def live_scenario_start(body: dict):
        from ..sim.scenario import Scenario, ScenarioRunner, load_scenario
        if state.conn.mode != "sitl":
            return JSONResponse({"error": "Visible scripted tests require SITL"}, status_code=409)
        if live["preparing"] or link.status().get("armed") or (live["runner"] and not live["runner"].done):
            return JSONResponse({"error": "Disarm and finish the current test first"}, status_code=409)
        try:
            sc = load_scenario(SCENARIO_DIR / Path(str(body["scenario"])).name) if "scenario" in body else Scenario.from_dict(body)
        except (ValueError, FileNotFoundError) as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        if not sc.phases:
            return JSONResponse({"error": "A test needs phases"}, status_code=400)
        if sc.airframe and sc.airframe != sim.airframe.name:
            return JSONResponse({"error": f"This scenario requires {sc.airframe}; current model is {sim.airframe.name}. Choose the matching full-flight scenario."}, status_code=400)
        generation = live["generation"]
        model = sim.airframe
        # Apply on a worker thread before attaching the flight hook: lockstep sensors must keep flowing.
        if sc.params:
            from .scenario_params import prepare_scenario_params
            live["preparing"] = True
            try:
                await prepare_scenario_params(state, sc.params)
            except ValueError as exc:
                return JSONResponse({"error": str(exc)}, status_code=409)
            finally:
                live["preparing"] = False
        if generation != live["generation"] or sim.airframe is not model:
            return JSONResponse({"error": "Scenario cancelled by reset or model change"}, status_code=409)
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
            live.update(runner=runner, hook=step_live, animation_cg=list(sim.airframe.mass.cg), animation_name=sim.airframe.name)
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
            return json_safe({"running": not r.done, "phase": r.phase, "index": r.index,
                              "phase_count": len(r.sc.phases), **r.result(sim.airframe.mass.mass)})

    @app.get("/api/sim/scenario/timeseries")
    async def live_scenario_timeseries():
        with sim.lock:
            r = live["runner"]
            return r.metrics.timeseries() if r else {"columns": [], "rows": [], "phase": []}

    @app.get("/api/sim/scenario/animation")
    async def live_animation():
        from ..sim.animation import animation_zip
        with sim.lock:
            r = live["runner"]
            if not r or not r.metrics.motion:
                return JSONResponse({"error": "Run a scenario first to record motion"}, status_code=404)
            rows = [row[:] for row in r.metrics.motion]
            cg = list(live["animation_cg"])
            name = live["animation_name"]
        data = await run_in_threadpool(animation_zip, rows, cg, name)
        return Response(data, media_type="application/zip", headers={"Content-Disposition": 'attachment; filename="aircraft-animation.zip"'})

    @app.get("/api/batch/jobs/{job_id}/animation")
    async def batch_animation(job_id: str):
        job = state.batch_jobs.get(job_id)
        path = (job.get("result") or {}).get("animation_path") if job else None
        if not path or not Path(path).is_file():
            return JSONResponse({"error": "No recorded animation available for this job"}, status_code=404)
        return FileResponse(path, media_type="application/zip", filename=job_id + "-animation.zip")

    @app.post("/api/sim/scenario/stop")
    async def live_scenario_stop():
        if state.conn.mode != "sitl":
            return JSONResponse({"error": "Visible scripted tests require SITL"}, status_code=409)
        cancel_live("stopped")
        # Stop stops the sequence, not the simulation: the aircraft keeps flying under PX4 (Pause, Land or
        # Kill are the things that stop it), and the graph keeps recording.
        return {"ok": True}

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

    # ------------------------------------------------------------------ flight recorder: saved runs
    RUNS = RunStore(PROJECT_DIR / "results" / "runs")

    def save_current_run(name: str = "", note: str = "") -> dict:
        """Keep the run that just happened, with the design and the parameters that produced it."""
        try:
            params = state.conn.export_params_fn() if state.conn.export_params_fn else {}
        except Exception:
            params = {}
        # the flight controller's own words from the span this run covers, kept with it
        t0 = sim.recorder.started - 1.0
        log = [[t, text] for t, text in list(state.log_buffer) if t >= t0 and str(text).startswith("[px4]")]
        return RUNS.save(sim.recorder, airframe=sim.airframe.to_dict(), params=params,
                         name=name or sim.airframe.name, note=note, log=log)

    # ------------------------------------------------- the board's own flight logs (.ulg)
    # The aircraft logs everything it knows at the rate it knows it - two hundred times a second, against
    # the ten or twenty a radio carries. This brings one over the same link and turns it into a run, so it
    # is plotted, scrubbed and compared exactly like a flight watched live.
    LOG_DIR = PROJECT_DIR / "results" / "logs"
    log_job: dict = {"job": None, "cancel": False, "thread": None, "entries": [], "listed": 0.0,
                     "measured_bps": 0.0}      # what the last finished download actually managed

    def log_entry_name(e: dict) -> str:
        t = e.get("time_utc") or 0
        when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) if t else f"log {e['id']}"
        return f"{when} · {e['size'] / 1e6:.1f} MB"

    @app.get("/api/logs")
    async def logs_list(refresh: int = 0):
        """What is on the board's card. Cached, because asking costs a round trip per log."""
        from ..px4.logs import list_logs
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "no link to the aircraft"}, status_code=409)
        # Listing 384 logs is 17 kB down the radio: asked for, not done on a timer behind your back.
        if refresh or not log_job["entries"]:
            entries = await run_in_threadpool(list_logs, state.link)
            log_job["entries"] = [e.as_dict() for e in entries]
            log_job["listed"] = time.time()
            state.log(f"[logs] the aircraft has {len(entries)} log(s) on its card")
        rate = (link.status().get("telemetry") or {}).get("bytes_s") or 0
        # The only honest basis for "how long will this take" is how fast the last one actually came down
        # this link. Until there has been one: whatever the telemetry is managing right now is a floor the
        # link has demonstrated, and a paused telemetry hands most of the rest over - but not the round
        # trips, so a quarter more is as optimistic as this is allowed to be.
        budget = link.link_budget_bps()
        free = log_job["measured_bps"] or (max(rate, (budget or 0) * 0.9) * 1.25 if budget else 60_000.0)
        free = max(300.0, free)
        return {"ok": True, "logs": [{**e, "label": log_entry_name(e), "eta_s": round(e["size"] / free)}
                                     for e in log_job["entries"]],
                "rate_now": rate, "assumed_bps": round(free), "measured": bool(log_job["measured_bps"]),
                "job": (log_job["job"] or {}) and log_job["job"].as_dict()}

    @app.get("/api/logs/job")
    async def logs_job():
        job = log_job["job"]
        return {"ok": True, "job": job.as_dict() if job else None}

    @app.post("/api/logs/cancel")
    async def logs_cancel():
        log_job["cancel"] = True
        return {"ok": True}

    def run_from_ulog(path: Path, note: str) -> dict:
        """Parse a .ulg and file it as a run, with the design currently loaded for reference."""
        from ..sim.ulog_import import ulog_to_run
        doc = ulog_to_run(path)
        rid = time.strftime("%Y%m%d-%H%M%S", time.localtime(path.stat().st_mtime)) + "-log"
        entry = RUNS.save_doc({
            "name": f"{sim.airframe.name} · {path.stem}", "started": path.stat().st_mtime,
            "scenario": "", "note": note or f"flight log {path.name}",
            "airframe": sim.airframe.to_dict(), "params": {}, "log": [],
            **{k: v for k, v in doc.items() if k not in ("source", "motors", "topics", "logged_hz")},
        }, rid)
        state.log(f"[logs] {path.name}: {doc['duration']:.0f} s, {doc['samples']} samples at {doc['sample_hz']:.0f} Hz "
                  f"(the board logged up to {doc['logged_hz']:.0f} Hz), {doc['motors']} motors")
        return entry

    @app.post("/api/logs/download")
    async def logs_download(body: dict):
        """Fetch one log over the link, in the background, and turn it into a run when it lands."""
        from ..px4.logs import LogEntry, Download, download_log
        if not link.ctl_connected:
            return JSONResponse({"ok": False, "error": "no link to the aircraft"}, status_code=409)
        if log_job["thread"] is not None and log_job["thread"].is_alive():
            return JSONResponse({"ok": False, "error": "a log is already coming down"}, status_code=409)
        want = int(body.get("id", -1))
        e = next((x for x in log_job["entries"] if x["id"] == want), None)
        if e is None:
            return JSONResponse({"ok": False, "error": "unknown log; refresh the list"}, status_code=404)
        entry = LogEntry(id=e["id"], size=e["size"], time_utc=e["time_utc"])
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(entry.time_utc or time.time()))
        path = LOG_DIR / f"log_{entry.id}_{stamp}.ulg"
        job = Download(log_id=entry.id, size=entry.size, path=path)
        log_job.update(job=job, cancel=False)
        narrow = link.link_budget_bps() is not None

        def work():
            try:
                if narrow and link.listening:
                    state.link.quiet_streams()          # the log gets the link to itself
                download_log(state.link, entry, path, job, cancelled=lambda: log_job["cancel"])
                if job.state == "done" and job.bps > 1:
                    log_job["measured_bps"] = job.bps
                if job.state == "done":
                    job.run_id = run_from_ulog(path, f"downloaded from the aircraft ({entry.size / 1e6:.1f} MB)")["id"]
            except Exception as exc:
                job.state, job.error = "failed", str(exc)
                state.log(f"[logs] download failed: {exc}")
            finally:
                if narrow and link.ctl_connected and link.listening:
                    try:
                        state.link.request_flight_telemetry()
                    except Exception:
                        pass

        t = threading.Thread(target=work, name="log-download", daemon=True)
        log_job["thread"] = t
        t.start()
        state.log(f"[logs] downloading log {entry.id} ({entry.size / 1e6:.1f} MB)")
        return {"ok": True, "job": job.as_dict()}

    @app.post("/api/logs/import")
    async def logs_import(request: Request):
        """A .ulg from the SD card, dropped in. Same result as a download, without the wait."""
        form = await request.form()
        f = form.get("file")
        if f is None:
            return JSONResponse({"ok": False, "error": "no file"}, status_code=400)
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        path = LOG_DIR / Path(str(f.filename or "flight.ulg")).name
        path.write_bytes(await f.read())
        try:
            return {"ok": True, "run": await run_in_threadpool(run_from_ulog, path, f"imported from {path.name}")}
        except Exception as exc:
            return JSONResponse({"ok": False, "error": f"could not read {path.name}: {exc}"}, status_code=400)

    @app.get("/api/runs")
    async def runs_list():
        return {"runs": RUNS.list(), "recording": {"samples": len(sim.recorder.rows),
                                                   "duration": round(sim.recorder.duration, 2)}}

    @app.get("/api/runs/current")
    async def runs_current(since: int = 0):
        """The run in progress, straight from the recorder, so the graph shows the whole flight and not
        only the part that happened while this page was open. Incremental: ask for the rows after `since`."""
        rows = sim.recorder.rows
        since = max(0, min(int(since), len(rows)))
        return {"columns": list(sim.recorder.columns), "rows": rows[since:], "total": len(rows), "since": since,
                "duration": round(sim.recorder.duration, 2), "flew": sim.recorder.flew}

    @app.get("/api/runs/{run_id}")
    async def runs_get(run_id: str):
        doc = RUNS.get(run_id)
        if doc is None:
            return JSONResponse({"ok": False, "error": "no such run"}, status_code=404)
        return doc

    @app.post("/api/runs/clear")
    async def runs_clear():
        """Throw the recording away and start the graph again. The simulation is not touched."""
        sim.recorder.reset()
        return {"ok": True}

    @app.post("/api/runs/save")
    async def runs_save(body: dict | None = None):
        body = body or {}
        r = await run_in_threadpool(save_current_run, str(body.get("name", "")), str(body.get("note", "")))
        if r.get("ok"):
            state.log(f"[runs] saved {r['id']} ({r['samples']} samples, {r['duration']:.1f} s)")
            if body.get("clear"):
                # Saving a run ends it: the graph starts again from zero while the simulation keeps flying.
                sim.recorder.reset()
        return r

    @app.post("/api/runs/{run_id}/rename")
    async def runs_rename(run_id: str, body: dict):
        ok = RUNS.rename(run_id, str(body.get("name", "")))
        return {"ok": ok} if ok else JSONResponse({"ok": False, "error": "no such run"}, status_code=404)

    @app.post("/api/runs/{run_id}/delete")
    async def runs_delete(run_id: str):
        ok = RUNS.delete(run_id)
        if ok:
            state.log(f"[runs] deleted {run_id}")
        return {"ok": ok} if ok else JSONResponse({"ok": False, "error": "no such run"}, status_code=404)

    @app.post("/api/runs/{run_id}/load")
    async def runs_load(run_id: str):
        """Load the airframe a run was flown with, so the design can be put back as it was."""
        doc = RUNS.get(run_id)
        if doc is None or not doc.get("airframe"):
            return JSONResponse({"ok": False, "error": "no such run"}, status_code=404)
        af = Airframe.from_dict(doc["airframe"])
        af.resolve_mass()
        sim.set_airframe(af)
        autosave(af)
        state.log(f"[runs] loaded the airframe from {run_id}")
        return {"ok": True, "airframe": af.to_dict()}

    @app.post("/api/airframe/path")
    async def airframe_set_path(body: dict):
        """Set one geometry value by its parameter path, e.g. rotors[*].tilt_deg. Used by the batch list."""
        from ..geometry.paths import set_path
        path, value = str(body.get("path", "")), body.get("value")
        if not path:
            return JSONResponse({"ok": False, "error": "path required"}, status_code=400)
        try:
            set_path(sim.airframe, path, value)
            sim.airframe.resolve_mass()
            sim.set_airframe(sim.airframe)
            autosave(sim.airframe)
        except Exception as e:
            return JSONResponse({"ok": False, "error": f"{type(e).__name__}: {e}"}, status_code=400)
        return {"ok": True, "path": path, "value": value}

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
        job = {"id": job_id, "scenario": scenario, "variables": variables,  "running": True,
               "t0": time.time(), "result": None, "log": []}
        state.batch_jobs[job_id] = job
        if len(state.batch_jobs) > 50:
            for k in list(state.batch_jobs)[:-50]:
                state.batch_jobs.pop(k, None)

        def run():
            try:
                r = run_once(af, scenario, variables=variables, px4_dir=state.args.px4_dir, log=lambda s: job["log"].append(s),
                             animation_path=str(RESULTS_DIR / "animations" / (job_id + ".zip")),
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
        sim.set_airframe(af)
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
            # json.dumps writes a bare NaN by default and the browser's JSON.parse refuses the whole
            # frame, so the view simply stops updating with nothing to show for it. Everything goes out
            # through json_safe, as the REST responses do.
            send = lambda m: websocket.send_text(json.dumps(json_safe(m), allow_nan=False))   # noqa: E731
            await send({"type": "airframe", "airframe": sim.airframe.to_dict()})
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
                await send(msg)
                await asyncio.sleep(1 / 30)
        except WebSocketDisconnect:
            pass
        except Exception:
            pass
        finally:
            rx_task.cancel()

    return app
