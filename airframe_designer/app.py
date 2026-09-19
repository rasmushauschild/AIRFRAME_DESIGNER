"""Interactive app: 3D editor + PX4 SITL/HITL + remote.

  airframe-designer ui                              # PX4 SITL (started for you) + sim + UI
  airframe-designer ui --mode auto                  # Pixhawk if one is plugged in, otherwise SITL
  airframe-designer ui --mode hitl                  # Pixhawk over USB (first PX4-looking port)
  airframe-designer ui --mode hitl --serial /dev/cu.usbmodem01

The UI's Connect tab can switch between SITL and a Pixhawk at any time without restarting.
"""
from __future__ import annotations

import argparse
import atexit
import faulthandler
import os
import signal
import sys
import threading
import time
import webbrowser
from collections import deque
from pathlib import Path

import uvicorn

from .geometry.airframe import Airframe, quad_x
from .px4.connection import list_serial_ports
from .native import NativeConnectionManager as ConnectionManager, NativeSimulator as Simulator
from .px4.events import EventDecoder
from .px4 import param_meta
from .px4.sitl import find_px4_dir
from .sensors import Home
from .server.app import AppState, build_app

PROJECT_DIR = Path(__file__).resolve().parent.parent
DEFAULT_AIRFRAME = "atlas_09.json"


def free_http_port(host: str, port: int, log, tries: int = 20) -> int:
    """The requested port, or the next free one when another app (e.g. an older simulator) still holds it."""
    import socket
    for p in range(port, port + tries):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, p))
            except OSError:
                continue
        if p != port:
            log(f"[ui] port {port} is in use (another simulator running?); using port {p}")
        return p
    raise RuntimeError(f"no free port between {port} and {port + tries - 1}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="airframe-designer ui", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["sitl", "hitl", "auto"], default="sitl",
                    help="sitl: PX4 SITL; hitl: Pixhawk over USB; auto: hitl if a Pixhawk is plugged in, else sitl")
    ap.add_argument("--tcp", default="0.0.0.0:4560", help="SITL: address to listen on for PX4 (default 0.0.0.0:4560)")
    ap.add_argument("--ctl", default=None, help="SITL: pymavlink address of PX4's onboard MAVLink link for params/commands "
                                               "(default udpin:127.0.0.1:14540+instance)")
    ap.add_argument("--serial", default=None, help="HITL: Pixhawk serial port (default: first PX4-looking USB port)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--qgc", default="127.0.0.1:14550", help="HITL: forward vehicle MAVLink to QGC at this UDP address ('' to disable)")
    ap.add_argument("--airframe", default=None, help=f"airframe JSON to load (default: airframes/{DEFAULT_AIRFRAME})")
    ap.add_argument("--px4-dir", default=str(PROJECT_DIR / "firmware" / "atlas"))
    ap.add_argument("--launch-px4", action="store_true", default=True, help="SITL: start PX4 SITL from --px4-dir (default)")
    ap.add_argument("--no-launch-px4", dest="launch_px4", action="store_false", help="SITL: do not start PX4, wait for one")
    ap.add_argument("--px4-model", default="none_iris", help="PX4_SIM_MODEL for --launch-px4 (default none_iris)")
    ap.add_argument("--px4-rootfs", default=None, help="PX4 SITL working dir (params, logs); default ~/.airframe_designer/px4_instance_<n>")
    ap.add_argument("--px4-instance", type=int, default=None,
                    help="SITL instance number (default: first free); the simulator port is 4560 + instance")
    ap.add_argument("--param-meta", default=None, help="parameters.json(.xz) for parameter descriptions")
    ap.add_argument("--rate", type=float, default=250.0, help="sensor rate Hz (default 250)")
    ap.add_argument("--speed", type=float, default=1.0, help="SITL real-time factor, 0 = unthrottled")
    ap.add_argument("--no-lockstep", action="store_true")
    ap.add_argument("--home", default="55.6761,12.5683,10", help="lat,lon,alt of the home point")
    ap.add_argument("--http", default="127.0.0.1:8080", help="UI address")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)
    # Never silently substitute the old stock PX4 build.
    args.px4_dir = str(Path(args.px4_dir).expanduser())
    if args.mode != "hitl" and args.launch_px4 and not (Path(args.px4_dir) / "build/px4_sitl_default/bin/px4").is_file():
        ap.error("Native PX4 build missing; see firmware/atlas/README.md for build instructions")

    log_buffer: deque = deque(maxlen=1000)

    def log(s: str) -> None:
        print(s, flush=True)
        log_buffer.append((time.time(), s))

    # airframe
    if args.airframe:
        airframe = Airframe.load(args.airframe)
    elif (PROJECT_DIR / "airframes" / DEFAULT_AIRFRAME).is_file():
        airframe = Airframe.load(PROJECT_DIR / "airframes" / DEFAULT_AIRFRAME)
    else:
        airframe = quad_x()
    log(f"[sim] airframe: {airframe.name} ({len(airframe.rotors)} rotors, {len(airframe.wings)} wings, {len(airframe.legs)} legs, {airframe.mass.mass:.2f} kg)")

    lat, lon, alt = (float(x) for x in args.home.split(","))
    simulator = Simulator(airframe, None, sensor_rate=args.rate, speed=args.speed, home=Home(lat, lon, alt), log=log)
    simulator.start()

    conn = ConnectionManager(simulator, args, log)
    conn.event_decoder = EventDecoder()
    n_ev = conn.event_decoder.load_local(args.px4_dir)
    log(f"[events] metadata: {n_ev} event definitions from {conn.event_decoder.source or 'nowhere'}")
    state = AppState(simulator, conn, args, log_buffer, log)
    state.meta, state.meta_source = param_meta.load_local(args.px4_dir, args.param_meta)
    log(f"[params] metadata: {len(state.meta)} entries from {state.meta_source or 'nowhere (use Fetch descriptions)'}")

    # initial connection
    mode = args.mode
    if mode == "auto":
        px4_ports = [p for p in list_serial_ports() if p["likely_px4"]]
        mode = "hitl" if px4_ports else "sitl"
        log(f"[link] auto: {'Pixhawk found on ' + px4_ports[0]['device'] if px4_ports else 'no Pixhawk on USB'} -> {mode.upper()}")
    if mode == "hitl":
        conn.connect_hitl(args.serial, args.baud)
        if conn.link is None:
            log("[link] HITL connect failed; use the Connect tab in the UI to retry or switch to SITL")
    else:
        conn.connect_sitl(args.launch_px4)

    app = build_app(state)
    host, port = args.http.rsplit(":", 1)
    port = free_http_port(host, int(port), log)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(f"http://{host}:{port}/")).start()
    log(f"[ui] http://{host}:{port}/")

    def shutdown(*_):
        simulator.stop()
        conn.disconnect()

    atexit.register(conn.stop_px4)

    # uvicorn re-raises SIGINT/SIGTERM after its loop exits when it owns the main thread, which would kill us
    # before cleanup. Run it in a worker thread and keep signal handling here instead.
    server = uvicorn.Server(uvicorn.Config(app, host=host, port=int(port), log_level="warning"))
    server_thread = threading.Thread(target=server.run, name="uvicorn", daemon=True)
    server_thread.start()
    stop_event = threading.Event()

    def on_signal(signum, _frame):
        log(f"[sim] received signal {signum}, shutting down")
        stop_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    # `kill -USR1 <pid>` appends every thread's Python stack to ~/.airframe_sim/stacks.log (hang diagnosis)
    os.makedirs(os.path.expanduser("~/.airframe_designer"), exist_ok=True)
    _dump = open(os.path.expanduser("~/.airframe_designer/stacks.log"), "a")
    faulthandler.register(signal.SIGUSR1, file=_dump, all_threads=True)
    try:
        while not stop_event.is_set() and server_thread.is_alive():
            stop_event.wait(0.5)
    finally:
        server.should_exit = True
        shutdown()
        server_thread.join(3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
