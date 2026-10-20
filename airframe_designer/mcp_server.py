"""MCP server: lets any AI assistant that speaks the Model Context Protocol (ChatGPT connectors, Claude Desktop,
Cursor, Codex, ...) read and edit the airframe, push parameters to PX4, run headless simulations and studies,
exactly as Claude Code does through the REST API and CLI.

  .venv/bin/python -m airframe_designer.mcp_server            # stdio (Claude Desktop, Cursor, Codex, local agents)
  .venv/bin/python -m airframe_designer.mcp_server --http 8765 # streamable HTTP on http://127.0.0.1:8765/mcp
                                                              # (ChatGPT needs a public HTTPS URL: put a tunnel such
                                                              #  as ngrok or cloudflared in front of that port)

Tools that touch the *live* aircraft talk to the running app (AFD_APP_URL, default http://127.0.0.1:8081); the
headless tools run their own PX4 instance and need no app.
"""
from __future__ import annotations

import json
import os
import sys
import urllib.request
from pathlib import Path
from typing import Any

from mcp.server.mcpserver import MCPServer

PROJECT_DIR = Path(__file__).resolve().parents[1]
APP_URL = os.environ.get("AFD_APP_URL", "http://127.0.0.1:8081").rstrip("/")

server = MCPServer(
    name="airframe-designer",
    instructions=(
        "AIRFRAME_DESIGNER: a PX4-in-the-loop aircraft design simulator. Use get_status/get_airframe to see the live "
        "design, set_parameters to edit it (parameter paths like rotors[0:8].tilt_deg, wings[0].pitch_deg, mass.cg[0], "
        "legs[*].length, px4.MC_PITCHRATE_P; see the 'schema' resource), run_scenario for a headless PX4 flight "
        "with metrics, run_study to optimise, analyse for instant static hover/cruise numbers. Read the 'guide' "
        "resource first."
    ),
)


def _api(path: str, body: dict | None = None, method: str | None = None) -> Any:
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(APP_URL + path, data=data, method=method or ("POST" if body is not None else "GET"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read().decode())


# ----------------------------------------------------------------- resources
@server.resource("afd://guide")
def guide() -> str:
    """How to drive the simulator from a script or an AI agent (CLI, API, scenarios, studies, metrics)."""
    return (PROJECT_DIR / "docs" / "AI_GUIDE.md").read_text()


@server.resource("afd://schema")
def schema() -> str:
    """The airframe JSON schema and the parameter-path syntax."""
    return (PROJECT_DIR / "docs" / "SCHEMA.md").read_text()


# ----------------------------------------------------------------- live app
@server.tool()
def get_status() -> dict:
    """Live app: PX4 connection, armed state, flight mode, paused flag."""
    s = _api("/api/status")
    return {k: s.get(k) for k in ("connected", "ctl_connected", "armed", "mode_name", "paused", "arm_ready",
                                  "arm_block_reason", "px4_running", "speed", "home")}


@server.tool()
def get_airframe(name: str | None = None) -> dict:
    """The airframe JSON: the live design in the app (name=None), or a file from airframes/ by name."""
    if name:
        from .geometry.airframe import Airframe
        p = PROJECT_DIR / "airframes" / (name if name.endswith(".json") else name + ".json")
        return Airframe.load(p).to_dict()
    return _api("/api/airframe")


@server.tool()
def list_airframes() -> list[str]:
    """Airframe files in airframes/."""
    return sorted(p.stem for p in (PROJECT_DIR / "airframes").glob("*.json") if not p.name.startswith("_"))


@server.tool()
def list_paths() -> list[str]:
    """Every editable parameter path of the live airframe (e.g. rotors[0].tilt_deg, wings[0].pitch_deg, px4.MC_PITCH_P)."""
    return _api("/api/airframe/paths")["paths"]


@server.tool()
def set_parameters(variables: dict, push_to_px4: bool = False, save_as: str | None = None) -> dict:
    """Edit the live airframe: {path: value}, e.g. {"rotors[0:8].tilt_deg": 30, "wings[0].incidence_deg": 6,
    "mass.cg[0]": 0.03, "px4.MC_PITCHRATE_P": 0.25}. push_to_px4 writes the PX4 parameters to the running
    flight controller (vehicle must be disarmed); save_as stores the airframe under that name in airframes/."""
    r = _api("/api/airframe/apply_variables", {"variables": variables})
    out = {"ok": r.get("ok"), "applied": variables}
    if push_to_px4:
        out["push"] = _api("/api/px4/push", {"save": True})
    if save_as:
        out["saved"] = _api("/api/airframe/save", {"name": save_as})
    hc = _api("/api/airframe/hover_check")
    out["hover_check"] = {"ok": hc.get("ok"), "problems": hc.get("problems")}
    return out


@server.tool()
def load_airframe(name: str) -> dict:
    """Load an airframe file from airframes/ into the app (replaces the live design)."""
    r = _api("/api/airframe/load", {"name": name if name.endswith(".json") else name + ".json"})
    return {"ok": r.get("ok"), "name": r.get("airframe", {}).get("name"), "problems": r.get("problems")}


@server.tool()
def px4_command(command: str, mode: str | None = None) -> dict:
    """Send a flight command to the live PX4: arm, disarm, kill, takeoff, land, or mode (mode=position/hold/altitude/...)."""
    body = {"command": command}
    if mode:
        body["mode"] = mode
    return _api("/api/px4/command", body)


@server.tool()
def sim_control(reset: bool = False, paused: bool | None = None, speed: float | None = None,
                wind_ned: list[float] | None = None) -> dict:
    """Control the live simulation: reset the vehicle to the ground, pause/resume, real-time factor (0 = as fast
    as possible), wind vector [north, east, down] m/s."""
    out = {}
    if reset:
        out["reset"] = _api("/api/sim/reset", {})
    if paused is not None:
        out["paused"] = _api("/api/sim/pause", {"paused": paused})
    if speed is not None:
        out["speed"] = _api("/api/sim/speed", {"speed": speed})
    if wind_ned is not None:
        out["wind"] = _api("/api/sim/wind", {"north": wind_ned[0], "east": wind_ned[1], "down": wind_ned[2] if len(wind_ned) > 2 else 0})
    return out


@server.tool()
def list_scenarios() -> list[dict]:
    """Bundled flight scripts (hover, takeoff_hover_land, cruise, box, gust, motor_out, manual_push)."""
    out = []
    for p in sorted((PROJECT_DIR / "scenarios").glob("*.json")):
        d = json.loads(p.read_text())
        out.append({"name": p.stem, "description": d.get("description", ""), "phases": [x.get("name") or x.get("type") for x in d.get("phases", [])]})
    return out


@server.tool()
def analyse(airframe: str | dict | None = None, speed_kmh: float | None = None) -> dict:
    """Instant static analysis (no PX4): PX4 hover allocation (busiest motor, authority, problems) and a steady
    cruise trim (pitch, power vs hover, wing lift share, angle of attack). airframe: file name, dict, or None for
    the live design."""
    from .geometry.airframe import Airframe
    from .analysis.static import analyse as _an
    af = _load(airframe)
    r = _an(af, (speed_kmh / 3.6) if speed_kmh else None)
    r["validate"] = af.validate(); r["hover_check_problems"] = af.hover_check()["problems"]
    return json.loads(json.dumps(r, default=float))


def _load(airframe):
    from .geometry.airframe import Airframe
    if airframe is None:
        return Airframe.from_dict(_api("/api/airframe"))
    if isinstance(airframe, dict):
        return Airframe.from_dict(airframe)
    p = Path(airframe)
    if not p.is_file():
        p = PROJECT_DIR / "airframes" / (str(airframe) if str(airframe).endswith(".json") else str(airframe) + ".json")
    return Airframe.load(p)


@server.tool()
def run_scenario(scenario: str = "hover", airframe: str | dict | None = None, variables: dict | None = None,
                 timeout_s: float = 600.0) -> dict:
    """Fly a scenario headless with PX4 in the loop (own PX4 instance, unthrottled) and return the metrics.
    airframe: file name, dict, or None for the live design. variables: {path: value} applied first."""
    from .batch.worker import run_once
    af = _load(airframe)
    r = run_once(af, scenario, variables=variables or {}, timeout_wall=timeout_s, quiet=True)
    return {k: v for k, v in r.items() if k not in ("airframe", "log", "traceback")}


def run_study(spec: dict | str, workers: int = 4) -> dict:
    """Optimisation study: spec dict (or a file name in studies/) with airframe, scenario, variables (paths +
    ranges), objective expression, constraints, algorithm (random|grid|cmaes|nelder_mead, budget). Returns the
    best trial and where results were written."""
    from .batch.study import run_study as _rs
    if isinstance(spec, dict) and "airframe" not in spec:
        spec = dict(spec); spec["airframe"] = _api("/api/airframe")
    s = _rs(spec, workers=workers, log=lambda m: print(m, file=sys.stderr, flush=True))
    return s


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] == "--http":
        port = int(argv[1]) if len(argv) > 1 else 8765
        import uvicorn
        app = server.streamable_http_app()
        print(f"[mcp] streamable HTTP on http://127.0.0.1:{port}/mcp  (expose with a tunnel for ChatGPT)", file=sys.stderr, flush=True)
        uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")
    else:
        server.run("stdio")
    return 0


if __name__ == "__main__":
    sys.exit(main())
