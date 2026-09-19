"""Runtime connection management: switch between PX4 SITL and a physical Pixhawk (HITL) without restarting.

Also owns the PX4 SITL child process and the HITL readiness checklist the UI shows.
"""
from __future__ import annotations

import glob
import json
import os
import re
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable

from .link import PX4Link
from .sitl import launch_px4, free_px4_instance, stop_px4 as _stop_px4, find_px4_dir, px4_binary
from . import firmware as fw
from .archive import FirmwareArchive

PROJECT_DIR = Path(__file__).resolve().parents[2]

# USB vendor ids commonly seen on PX4 flight controllers
PX4_VENDORS = {0x26AC: "3D Robotics", 0x1209: "PX4/pid.codes", 0x3162: "Holybro", 0x2DAE: "CubePilot",
               0x0483: "STMicro (bootloader)", 0x35A7: "Auterion", 0x1FC9: "NXP", 0x27AC: "PX4"}
PX4_HINTS = re.compile(r"px4|pixhawk|fmu|cube|holybro|ardupilot|auterion|autopilot", re.I)


def list_serial_ports() -> list[dict]:
    """Serial ports that could be a flight controller, PX4-looking ones first."""
    ports: list[dict] = []
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            dev = p.device
            if os.name == "posix" and dev.startswith("/dev/tty.") and os.path.exists(dev.replace("/dev/tty.", "/dev/cu.")):
                dev = dev.replace("/dev/tty.", "/dev/cu.")   # macOS: use the call-out device
            desc = " ".join(x for x in [p.manufacturer or "", p.product or p.description or ""] if x).strip()
            likely = bool(PX4_HINTS.search(desc)) or (p.vid in PX4_VENDORS)
            if p.vid in PX4_VENDORS and not PX4_HINTS.search(desc):
                desc = f"{PX4_VENDORS[p.vid]} {desc}".strip()
            if "bluetooth" in dev.lower() or "debug-console" in dev.lower() or p.vid is None:
                continue   # Bluetooth/virtual serial ports have no USB vendor id
            ports.append({"device": dev, "description": desc or "serial device", "vid": p.vid, "pid": p.pid,
                          "serial_number": p.serial_number, "likely_px4": likely})
    except Exception:
        for dev in sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/ttyACM*")):
            ports.append({"device": dev, "description": "USB modem", "vid": None, "pid": None, "serial_number": None,
                          "likely_px4": True})
    seen = set()
    out = []
    for p in ports:
        if p["device"] in seen:
            continue
        seen.add(p["device"])
        out.append(p)
    out.sort(key=lambda p: (not p["likely_px4"], p["device"]))
    return out


def board_target_from_description(desc: str | None) -> str | None:
    """'Auterion PX4 FMU v6X.x' -> 'px4_fmu-v6x'; 'PX4 FMU v5' -> 'px4_fmu-v5'."""
    if not desc:
        return None
    m = re.search(r"FMU\s*v(\d)([A-Za-z]?)", desc, re.I)
    if not m:
        return None
    return f"px4_fmu-v{m.group(1)}{m.group(2).lower()}"


class FirmwareJob:
    """Runs scripts/build_hitl_firmware.sh in the background, streaming output into the app log."""

    def __init__(self, log):
        self.log = log
        self.proc: subprocess.Popen | None = None
        self.action = None
        self.board = None
        self.result: str | None = None
        self.exit_code: int | None = None
        self.atlas = False
        self.firmware_file: str | None = None

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, board: str, action: str, px4_dir: str, venv_bin: str, ref: str | None = None, atlas: bool = False,
              firmware_file: str | None = None) -> dict:
        if self.running():
            return {"ok": False, "error": f"{self.action} already running"}
        script = PROJECT_DIR / "scripts" / "build_hitl_firmware.sh"
        env = dict(os.environ)
        env["PX4_DIR"] = px4_dir
        if firmware_file:
            env["FIRMWARE_FILE"] = firmware_file
        self.firmware_file = firmware_file
        if ref and not atlas:
            env["PX4_REF"] = ref
        if atlas:
            env["ATLAS_MODULES"] = str(PROJECT_DIR / "firmware" / "atlas")
        self.atlas = atlas
        env["PATH"] = venv_bin + ":" + env.get("PATH", "")
        self.action, self.board, self.result, self.exit_code = action, board, None, None
        self.log(f"[firmware] {action} {board} (this takes a few minutes; watch the log)")
        self.proc = subprocess.Popen(["/bin/bash", str(script), board, action], env=env, cwd=str(PROJECT_DIR),
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                     start_new_session=True)
        ansi = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

        def pump():
            last = ""
            repeats = 0
            for line in self.proc.stdout:
                line = ansi.sub("", line).rstrip()
                if not line:
                    continue
                if line == last:
                    repeats += 1
                    if repeats == 3:
                        self.log("[firmware] (repeating…)")
                    continue
                repeats = 0
                last = line
                # ninja progress lines are very chatty; keep every 25th plus anything that is not a build step
                if line.startswith("[") and "/" in line[:12] and "]" in line[:14]:
                    try:
                        n = int(line[1:line.index("/")])
                        if n % 25:
                            continue
                    except ValueError:
                        pass
                self.log(f"[firmware] {line}")
            code = self.proc.wait()
            self.exit_code = code
            self.result = "ok" if code == 0 else f"failed ({code}): {last}"
            self.log(f"[firmware] {self.action} {'finished' if code == 0 else 'FAILED'} (exit {code})")

        threading.Thread(target=pump, daemon=True).start()
        if action == "upload":
            def watchdog():
                deadline = time.time() + 240
                while self.running() and time.time() < deadline:
                    time.sleep(1.0)
                if self.running():
                    self.log("[firmware] upload took too long, giving up (is the port free? unplug/replug the board and retry)")
                    try:
                        os.killpg(self.proc.pid, signal.SIGTERM)
                    except Exception:
                        pass
            threading.Thread(target=watchdog, daemon=True).start()
        return {"ok": True}

    def status(self) -> dict:
        return {"running": self.running(), "action": self.action, "board": self.board, "result": self.result,
                "exit_code": self.exit_code, "atlas": self.atlas}


class ConnectionManager:
    def __init__(self, simulator, args, log: Callable[[str], None]):
        self.sim = simulator
        self.args = args
        self.log = log
        self.link: PX4Link | None = None
        self.mode: str | None = None          # "sitl" | "hitl" | None
        self.serial: str | None = None
        self.baud = args.baud
        self.px4_process: subprocess.Popen | None = None
        self.px4_instance: int | None = None
        self.error: str | None = None
        self.busy = False
        self.on_params: Callable[[], None] | None = None   # called after a fresh parameter download
        self.event_decoder = None                           # events.EventDecoder shared by all links
        self._lock = threading.RLock()
        self.firmware_job = FirmwareJob(log)
        self.launched_binary_mtime: float | None = None    # mtime of the SITL binary the running PX4 was started from
        self.archive = FirmwareArchive(log)                # every flash to a board is versioned and pushed to GitHub
        self.export_params_fn = None                        # set by the server: () -> exported PX4 parameters
        self.restore_job: dict = {"running": False}
        self._params_session = 0
        self._stop = threading.Event()
        threading.Thread(target=self._watch, name="link-watch", daemon=True).start()

    def _launch_sitl(self, instance):
        return launch_px4(self.args.px4_dir, self.args.px4_model, self.log,
                          instance=instance, rootfs=self.args.px4_rootfs)

    # ------------------------------------------------------------ connect
    def connect_sitl(self, launch: bool | None = None) -> dict:
        with self._lock:
            self.busy = True
            try:
                self._close_link()
                launch = self.args.launch_px4 if launch is None else launch
                instance = self.args.px4_instance
                if instance is None and self.px4_running() and self.px4_instance is not None:
                    # reconnecting to SITL: the PX4 we launched still holds its instance. Keep the instance and start
                    # it fresh, because PX4's simulator link does not recover once the simulator side has closed.
                    instance = self.px4_instance
                    self.log(f"[px4] reconnecting SITL: restarting our PX4 instance {instance}")
                    self.stop_px4()
                    time.sleep(0.5)
                if instance is None:
                    instance = free_px4_instance() if launch else 0
                    if instance:
                        self.log(f"[px4] SITL instance 0 is busy (another PX4 is running); using instance {instance}")
                tcp = self.args.tcp if (self.args.tcp != "0.0.0.0:4560" or not instance) else f"0.0.0.0:{4560 + instance}"
                ctl = self.args.ctl or f"udpin:127.0.0.1:{14540 + instance}"
                link = PX4Link("sitl", tcp, ctl_address=ctl, log=self.log)
                link.open()
                self._install(link, "sitl")
                self.px4_instance = instance
                if launch and (self.px4_process is None or self.px4_process.poll() is not None):
                    try:
                        self.px4_process = self._launch_sitl(instance)
                        self._note_launched_binary()
                    except RuntimeError as e:
                        self.error = str(e)
                        self.log(f"[px4] {e}")
                return self.status()
            finally:
                self.busy = False

    # ------------------------------------------------------------ SITL firmware (custom module tree)
    def _note_launched_binary(self) -> None:
        try:
            self.launched_binary_mtime = px4_binary(find_px4_dir(self.args.px4_dir)).stat().st_mtime
        except Exception:
            self.launched_binary_mtime = None

    def firmware_status(self, max_age: float = 3.0) -> dict:
        return fw.status(self.args.px4_dir, self.launched_binary_mtime, max_age=max_age)

    def update_firmware(self, relaunch: bool = True) -> dict:
        """Rebuild the SITL firmware when its sources are newer than the binary, then relaunch PX4 on the fresh
        binary (also when a binary was built elsewhere since PX4 started). Blocking; streams to the log."""
        st = fw.status(self.args.px4_dir, self.launched_binary_mtime, max_age=0)
        out = {"ok": True, "rebuilt": False, "relaunched": False, "stale": st["stale"]}
        if not st["custom"]:
            return out
        armed = self.link is not None and self.link.armed
        if st["needs_rebuild"]:
            if armed:
                return {**out, "ok": False, "error": "vehicle is armed; disarm before rebuilding the firmware"}
            names = ", ".join(st["stale"][:4]) + ("…" if len(st["stale"]) > 4 else "")
            self.log(f"[firmware] {len(st['stale'])} source file(s) newer than the PX4 binary ({names}): rebuilding")
            r = fw.build(self.args.px4_dir, self.log)
            if not r["ok"]:
                return {**out, "ok": False, "error": r["error"]}
            out["rebuilt"] = True
            out["build_seconds"] = r.get("seconds")
        st = fw.status(self.args.px4_dir, self.launched_binary_mtime, max_age=0)
        if relaunch and (out["rebuilt"] or st["needs_relaunch"]) and self.mode == "sitl" and self.args.launch_px4:
            if armed:
                return {**out, "ok": False, "error": "vehicle is armed; disarm before relaunching PX4"}
            self.log("[firmware] relaunching PX4 SITL on the new binary")
            self.connect_sitl(True)
            deadline = time.time() + 90
            while time.time() < deadline:
                l = self.link
                if l is not None and l.ctl_connected and l.param_count and l.status().get("params_loaded", 0) >= l.param_count:
                    break
                time.sleep(0.5)
            else:
                return {**out, "ok": False, "error": "PX4 did not come back within 90 s after the relaunch"}
            out["relaunched"] = True
        return out

    def connect_hitl(self, serial: str | None = None, baud: int | None = None) -> dict:
        if self.firmware_job.running() and self.firmware_job.action == "upload":
            self.error = "firmware upload in progress; the link reconnects by itself when it is done"
            return self.status()
        with self._lock:
            self.busy = True
            try:
                ports = list_serial_ports()
                if not serial:
                    cands = [p for p in ports if p["likely_px4"]] or ports
                    if not cands:
                        self.error = "No serial port found. Plug the Pixhawk in over USB and rescan."
                        return self.status()
                    serial = cands[0]["device"]
                self.baud = baud or self.baud
                self.stop_px4()
                self._close_link()
                link = PX4Link("hitl", serial, baud=self.baud, qgc_proxy=self.args.qgc or None, log=self.log)
                try:
                    link.open()
                except Exception as e:
                    msg = str(e)
                    if "busy" in msg.lower() or "permission" in msg.lower() or "resource" in msg.lower():
                        msg += " — another program holds the port. Close QGroundControl (or disable its serial " \
                               "auto-connect) and try again."
                    self.error = f"Could not open {serial}: {msg}"
                    self.log(f"[link] {self.error}")
                    self.link = None
                    self.mode = None
                    return self.status()
                self._install(link, "hitl")
                self.serial = serial
                self.log(f"[link] HITL on {serial}. QGroundControl can connect on udp://{self.args.qgc}")
                return self.status()
            finally:
                self.busy = False

    def disconnect(self) -> dict:
        with self._lock:
            self.stop_px4()
            self._close_link()
            return self.status()

    def _install(self, link: PX4Link, mode: str) -> None:
        link.event_decoder = self.event_decoder
        self.link = link
        self.mode = mode
        self.native_module_seen = False
        if mode == "hitl":
            # a real flight controller flies the aircraft exactly as it would on the real drone: the simulator is only the
            # plant, so drop every simulator-side assist (motor overrides, the Python nose-lift hook)
            self.sim.motor_override = None
            try:
                self.sim.stop_nose_lift()
            except Exception:
                pass
        self.serial = link.address if mode == "hitl" else None
        self.error = None
        self._params_session += 1
        self.sim.set_link(link, lockstep=(mode == "sitl" and not self.args.no_lockstep))

    def _close_link(self) -> None:
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
        self.link = None
        self.mode = None
        self.serial = None
        self.sim.set_link(None, lockstep=False)

    # ------------------------------------------------------------ PX4 SITL
    def stop_px4(self) -> None:
        proc = self.px4_process
        if proc and proc.poll() is None:
            self.log("[px4] stopping PX4 SITL")
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(3)
            except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        self.px4_process = None

    def px4_running(self) -> bool:
        return self.px4_process is not None and self.px4_process.poll() is None

    # ------------------------------------------------------------ firmware
    def detected_board(self) -> dict:
        """Which PX4 make target the plugged-in board needs, from its USB descriptor."""
        for p in self.list_ports_cached():
            if p["likely_px4"]:
                t = board_target_from_description(p["description"])
                return {"device": p["device"], "description": p["description"], "target": t}
        return {"device": None, "description": None, "target": None}

    TOOLCHAIN_DIRS = ("/opt/homebrew/opt/arm-gcc-bin@13/bin", "/usr/local/opt/arm-gcc-bin@13/bin")

    def toolchain_present(self) -> bool:
        from shutil import which
        return which("arm-none-eabi-gcc") is not None or any(Path(d, "arm-none-eabi-gcc").is_file() for d in self.TOOLCHAIN_DIRS)

    def firmware_variant(self, target: str | None) -> str:
        """Same choice as scripts/build_hitl_firmware.sh: 'multicopter' when the board offers it, else 'default'."""
        if not target:
            return "default"
        board_dir = Path(self.board_source_dir()) / "boards" / target.replace("_", "/", 1)
        return "multicopter" if (board_dir / "multicopter.px4board").is_file() else "default"

    def firmware_file(self, target: str | None) -> str | None:
        if not target:
            return None
        v = self.firmware_variant(target)
        f = Path(self.board_source_dir()) / "build" / f"{target}_{v}" / f"{target}_{v}.px4"
        return str(f) if f.is_file() else None

    def firmware_is_atlas(self, target: str | None) -> bool:
        """Was the built firmware compiled with the ATLAS modules (stamp written by the build script)?"""
        if not target:
            return False
        v = self.firmware_variant(target)
        return (Path(self.board_source_dir()) / "build" / f"{target}_{v}" / f"{target}_{v}.atlas").is_file()

    def board_source_dir(self) -> str:
        """The PX4 checkout board firmware is built from: the SITL's source tree (the ATLAS overlay is pinned to it)."""
        return os.environ.get("PX4_SOURCE_DIR", os.path.expanduser("~/PX4-Autopilot"))

    def build_firmware(self, target: str | None = None, atlas: bool = False) -> dict:
        target = target or self.detected_board()["target"]
        if not target:
            return {"ok": False, "error": "could not tell the board type from USB; pass the target, e.g. px4_fmu-v6x"}
        if not self.toolchain_present():
            return {"ok": False, "error": "ARM toolchain missing. Run:  brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13   then try again."}
        venv_bin = str(PROJECT_DIR / ".venv" / "bin")
        return self.firmware_job.start(target, "build", self.board_source_dir(), venv_bin, ref=self.board_release_tag(), atlas=atlas)

    def board_release_tag(self) -> str | None:
        """'1.17.0 release' on the board -> 'v1.17.0', so the HITL build matches what is flashed."""
        fw = self.link.firmware if self.link else {}
        ver = (fw or {}).get("version", "")
        m = re.match(r"(\d+\.\d+\.\d+) release", ver)
        return f"v{m.group(1)}" if m else None

    def upload_firmware(self, target: str | None = None, atlas: bool = False, firmware_file: str | None = None, note: str = "") -> dict:
        """Flash the built firmware (or an archived image). We must release the serial port first; the link reconnects
        after, and the result is archived to GitHub."""
        target = target or self.detected_board()["target"]
        if not target or not (firmware_file or self.firmware_file(target)):
            return {"ok": False, "error": "no built firmware for this board yet; build it first"}
        if atlas and not firmware_file and not self.firmware_is_atlas(target):
            return {"ok": False, "error": "the built firmware has no ATLAS modules; build the ATLAS firmware first"}
        with self._lock:
            was_hitl = self.mode == "hitl"
            serial = self.serial
            ref = self.board_release_tag()
            if was_hitl:
                self._close_link()
                time.sleep(1.0)   # let the OS release the device
        venv_bin = str(PROJECT_DIR / ".venv" / "bin")
        image = firmware_file or self.firmware_file(target)
        r = self.firmware_job.start(target, "upload", self.board_source_dir(), venv_bin, ref=ref, atlas=atlas, firmware_file=firmware_file)
        if r.get("ok") and was_hitl:
            def reconnect():
                while self.firmware_job.running():
                    time.sleep(1.0)
                time.sleep(4.0)   # let the board reboot into the new firmware and re-enumerate
                self.log("[firmware] reconnecting to the board")
                self.connect_hitl(serial, self.baud)
                if self.firmware_job.exit_code == 0:
                    self.record_flash(target, image)
                    self.archive_after_flash(image, atlas=(atlas or (firmware_file is None and self.firmware_is_atlas(target))), note=note)
            threading.Thread(target=reconnect, daemon=True).start()
        return r

    native_module_seen = False

    def _module_present(self) -> bool:
        """Does the connected firmware have the nose-lift module (answers its status command)?"""
        link = self.link
        if link is None or not link.ctl_connected:
            return False
        try:
            out = link.shell("atlas_nose_lift status", timeout=1.5)
        except Exception:
            return False
        return "phase=" in out or "not running" in out

    def ensure_native_module(self) -> bool:
        """On a board flashed with the ATLAS firmware, start the nose-lift module (idle until a takeoff is requested,
        and refusing to run on hardware unless NLF_HW_OK = 1) so PX4 lists its parameters. True when it was started."""
        link = self.link
        if link is None or not link.ctl_connected:
            return False
        try:
            out = link.shell("atlas_nose_lift status", timeout=1.5)
        except Exception:
            return False
        if "not running" not in out or "not found" in out:
            return False
        try:
            link.shell("atlas_nose_lift start", timeout=1.5)
            self.log("[px4] ATLAS firmware on the board: nose-lift module started (idle; takeoff needs NLF_HW_OK = 1)")
            time.sleep(0.5)
            return True
        except Exception as e:
            self.log(f"[px4] could not start the nose-lift module: {e}")
            return False

    # ------------------------------------------------------------ firmware archive (GitHub)
    def _wait_params(self, timeout: float = 90.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            l = self.link
            if l is not None and l.ctl_connected and l.param_count and l.status().get("params_loaded", 0) >= l.param_count:
                return True
            time.sleep(0.5)
        return False

    def archive_after_flash(self, image: str | None, atlas: bool, note: str = "") -> dict:
        """Called once the board is back after a flash: store image + full parameter set + airframe + module sources."""
        if not self._wait_params():
            self.log("[archive] board parameters did not come back in time; snapshot skipped (use Snapshot now later)")
            return {"ok": False, "error": "parameters not downloaded"}
        return self.archive_snapshot(kind="flash", image=image, atlas=atlas, note=note)

    def archive_snapshot(self, kind: str = "manual", image: str | None = None, atlas: bool | None = None, note: str = "") -> dict:
        link = self.link
        if link is None or link.mode != "hitl" or not link.ctl_connected:
            return {"ok": False, "error": "snapshot needs a connected board (HITL)"}
        if not (link.param_count and link.status().get("params_loaded", 0) >= link.param_count):
            return {"ok": False, "error": "board parameters still downloading"}
        board = self.detected_board()
        if atlas is None:
            atlas = "NLF_ENABLE" in link.params
        export = {}
        try:
            export = self.export_params_fn() if self.export_params_fn else {}
        except Exception as e:
            self.log(f"[archive] export failed: {e}")
        af = self.sim.airframe.to_dict() if getattr(self.sim, "airframe", None) else None
        return self.archive.snapshot(kind=kind, board=board, firmware_file=image, board_firmware=dict(link.firmware or {}),
                                     params_board=dict(link.params), params_export=export, airframe=af, atlas=atlas, note=note)

    def archive_restore(self, version_id: str, flash: bool = True, params: bool = True, note: str = "") -> dict:
        """Put the board back to an archived version: flash its image, then write its full parameter set."""
        v = self.archive.get(version_id)
        if not v:
            return {"ok": False, "error": f"unknown version {version_id}"}
        if self.restore_job.get("running"):
            return {"ok": False, "error": "a restore is already running"}
        if self.link is None or self.link.mode != "hitl":
            return {"ok": False, "error": "restore needs the board connected (HITL)"}
        if self.link.armed:
            return {"ok": False, "error": "vehicle is armed"}
        if flash and not v.get("firmware_file"):
            return {"ok": False, "error": "this version has no firmware image (parameters-only snapshot)"}
        self.restore_job = {"running": True, "id": version_id, "steps": [], "error": None, "t0": time.time()}

        def run():
            job = self.restore_job
            try:
                if flash:
                    self.log(f"[archive] restoring {version_id}: flashing its firmware image")
                    r = self.upload_firmware(self.detected_board()["target"], atlas=False, firmware_file=v["firmware_file"], note=f"restore of {version_id}")
                    if not r.get("ok"):
                        job["error"] = r.get("error"); return
                    while self.firmware_job.running():
                        time.sleep(1.0)
                    if self.firmware_job.exit_code != 0:
                        job["error"] = f"flash failed: {self.firmware_job.result}"; return
                    job["steps"].append("firmware flashed")
                    time.sleep(6.0)
                if params:
                    if not self._wait_params():
                        job["error"] = "board parameters did not come back"; return
                    link = self.link
                    wanted = v.get("params_board") or {}
                    todo = {k: p["value"] for k, p in wanted.items()
                            if k in link.params and abs(float(link.params[k]["value"]) - float(p["value"])) > 1e-9
                            and not k.startswith(("SYS_AUTOSTART",)) }
                    self.log(f"[archive] restoring {len(todo)} parameters that differ from {version_id}")
                    results = link.set_params(todo, lambda name, res: None) if todo else []
                    failed = [r["name"] for r in results if not r["ok"]]
                    link.preflight_storage(True)
                    job["steps"].append(f"{len(todo) - len(failed)}/{len(todo)} parameters restored" + (f", failed: {failed[:6]}" if failed else ""))
                self.log(f"[archive] restore of {version_id} done: " + "; ".join(job["steps"]))
            except Exception as e:
                job["error"] = f"{type(e).__name__}: {e}"
                self.log(f"[archive] restore failed: {job['error']}")
            finally:
                job["running"] = False
                job["t1"] = time.time()

        threading.Thread(target=run, name="archive-restore", daemon=True).start()
        return {"ok": True}

    # ------------------------------------------------------------ HITL helpers
    def recover(self) -> dict:
        """Bring PX4 back to an armable state after a crash: force-disarm, rest the sim, then either restart the
        estimator or, if the commander is in termination/failsafe, reboot the board."""
        link = self.link
        steps = []
        if link is None or not link.ctl_connected:
            self.sim.reset()
            return {"ok": True, "steps": ["sim reset (no PX4 link)"]}
        from .link import mavlink
        if link.armed:
            link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)   # force disarm
            steps.append("force-disarmed")
            time.sleep(1.0)
        self.sim.reset()
        steps.append("sim reset")
        main_mode = (link.custom_mode >> 16) & 0xFF
        recent = " ".join(x.get("text", "") for x in list(link.recent_events)[-20:]).lower()
        terminated = main_mode == 10 or "termination" in recent or "failsafe" in recent
        if terminated:
            self.log("[px4] recover: commander is in termination/failsafe, rebooting the board")
            link.reboot()
            self._after_reboot(link)
            steps.append("rebooted (termination/failsafe)")
        elif self.mode == "hitl":
            link.reboot()
            self._after_reboot(link)
            steps.append("rebooted (HITL never restarts EKF2 in place)")
        else:
            time.sleep(1.5)
            link.restart_estimator()
            steps.append("estimator restarted")
        return {"ok": True, "steps": steps}

    _reset_busy_until = 0.0

    def reset_all(self) -> dict:
        """Put the whole simulation back to zero, fast: force-disarm, forget motor commands, vehicle back on the
        ground at its hover attitude, PX4 messages cleared, and PX4 brought back to an armable state. Only when the
        commander is in termination/failsafe (which nothing but a reboot clears) is the board rebooted."""
        if time.time() < self._reset_busy_until:
            return {"ok": True, "steps": [f"reset already in progress ({self._reset_busy_until - time.time():.0f} s left)"]}
        link = self.link
        steps = []
        from .link import mavlink
        if link is not None and link.ctl_connected:
            if link.armed:
                link.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                steps.append("force-disarmed")
                time.sleep(0.6)
            link.recent_events.clear()
        self.sim.reset()
        steps.append("vehicle reset")
        if link is None or not link.ctl_connected:
            return {"ok": True, "steps": steps}
        main_mode = (link.custom_mode >> 16) & 0xFF
        if self.mode == "hitl":
            # HITL: always reboot. Termination needs it, and it is the only estimator reset that works on the
            # board (see _post_boot_ekf_restart); the sim is already at rest so EKF2 aligns on good data.
            self._reset_busy_until = time.time() + 20
            link.reboot()
            self._after_reboot(link)
            steps.append("board rebooted (was in flight termination)" if main_mode == 10 else "board rebooted")
        else:
            with self._lock:
                if main_mode == 10 and self.px4_running() and self.args.launch_px4:
                    self._reset_busy_until = time.time() + 20
                    self.stop_px4(); time.sleep(1.0)
                    self.px4_process = self._launch_sitl(self.px4_instance or 0)
                    steps.append("PX4 SITL relaunched (was in flight termination)")
                else:
                    self._reset_busy_until = time.time() + 8
                    threading.Thread(target=self._settle_after_reset, args=(link,), daemon=True).start()
                    steps.append("estimator restarting")
        return {"ok": True, "steps": steps}

    def _settle_after_reset(self, link) -> None:
        """Let the fresh sensor data flow for a moment, then restart EKF2 so it re-aligns on the reset vehicle."""
        time.sleep(2.0)
        if self.link is link:
            ok = False
            try:
                ok = link.restart_estimator()
            except Exception as e:
                self.log(f"[px4] estimator restart failed: {e}")
        self._select_default_mode(link)

    def _select_default_mode(self, link, timeout: float = 40.0) -> None:
        """Default mode is Takeoff: once PX4 reports it could arm for takeoff, request it (retry until accepted)."""
        from .link import mavlink
        t0 = time.time()
        while time.time() - t0 < timeout and self.link is link and link.ctl_connected:
            summary = None
            for x in reversed(list(link.recent_events)):
                if x.get("name") == "commander_arming_check_summary":
                    summary = dict(zip(x.get("arg_names", []), x.get("args", []))); break
            if summary and "takeoff" in str(summary.get("can_arm", "")):
                if ((link.custom_mode >> 16) & 0xFF) == 4 and ((link.custom_mode >> 24) & 0xFF) == 2:
                    return   # already Takeoff
                link.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 4.0, 2.0)
                time.sleep(2.0)
                if ((link.custom_mode >> 16) & 0xFF) == 4 and ((link.custom_mode >> 24) & 0xFF) == 2:
                    self.log("[px4] Takeoff selected")
                    return
            time.sleep(1.0)

    def restart_estimator(self) -> dict:
        link = self.link
        if link is None or not link.ctl_connected:
            return {"ok": False, "error": "not connected"}
        if self.mode == "hitl":
            self._reset_busy_until = time.time() + 30
            link.reboot()
            self._after_reboot(link)
            return {"ok": True, "error": None}
        ok = link.restart_estimator()
        return {"ok": ok, "error": None if ok else "no reply from the board's shell; use Reset to reboot the board"}

    def enable_hitl(self) -> dict:
        """Set SYS_HITL=1 on the board, save, reboot. The serial link reconnects by itself."""
        link = self.link
        if link is None or link.mode != "hitl" or not link.ctl_connected:
            return {"ok": False, "error": "not connected to a Pixhawk"}
        r = link.set_param("SYS_HITL", 1)
        if not r["ok"]:
            return {"ok": False, "error": f"could not set SYS_HITL: {r.get('error')}"}
        link.preflight_storage(True)
        time.sleep(0.5)
        link.reboot()
        self.log("[hitl] SYS_HITL=1 saved, rebooting the flight controller; waiting for it to come back…")
        return {"ok": True}

    def checklist(self, export_params: dict | None = None) -> list[dict]:
        """The few things HITL really needs: a link, the board in HIL mode streaming outputs, and (when the airframe
        uses it) the ATLAS firmware with the sequence allowed. Everything else is informational elsewhere."""
        link = self.link
        steps = []
        hitl = link is not None and link.mode == "hitl"
        up = hitl and link.ctl_connected and (time.time() - link.ctl_rx_time < 3.0)
        ports = [p["device"] for p in self.list_ports_cached() if p["likely_px4"]]
        steps.append({"id": "link", "label": "Pixhawk link", "ok": up,
                      "detail": (f"{link.address} · PX4 {link.firmware.get('version', '')}".strip(" ·") if up else
                                 (self.error or (f"found on {ports[0]}: click Connect" if ports else "plug the Pixhawk in over USB")))})
        job = self.firmware_job.status()
        params_ok = hitl and link.param_count > 0 and len(link.params) >= link.param_count
        streaming = hitl and up and link.hil_enabled and link.actuator_seq > 0
        sys_hitl = link.params.get("SYS_HITL", {}).get("value") if hitl else None
        has_hil_driver = params_ok and any(k.startswith("HIL_ACT_FUNC") for k in link.params)
        board = self.detected_board()
        hil_detail, hil_action = "", None
        if streaming:
            hil_detail = f"{link.actuator_seq} actuator messages"
        elif not up:
            hil_detail = ""
        elif sys_hitl is None:
            hil_detail = "reading the board…"
        elif sys_hitl != 1:
            hil_detail = "SYS_HITL is 0 on the board"; hil_action = "enable_hitl"
        elif params_ok and not has_hil_driver:
            hil_detail = "this firmware has no HIL output driver: flash the ATLAS firmware"
        else:
            hil_detail = "waiting for the board to enter HIL mode (reboot it if this stays)"; hil_action = "reboot"
        steps.append({"id": "hil", "label": "Board in HIL mode, streaming outputs", "ok": streaming, "detail": hil_detail, "action": hil_action})
        want_module = bool(export_params) and export_params.get("NLF_ENABLE") == 1
        if hitl and want_module:
            has_module = self.native_module_seen or "NLF_ENABLE" in link.params
            fw = self.board_firmware_status()
            a_detail, a_action = "", None
            if job["running"]:
                a_detail = f"{job['action']}ing {job['board']}… see the log"
            elif has_module and not (fw["needs_rebuild"] or fw["needs_flash"]):
                a_detail = "same modules as the SITL"
            elif has_module:
                a_detail = "newer module sources than the board: use Flash firmware below"
            elif not up:
                a_detail = ""
            elif not self.toolchain_present():
                a_detail = "ARM toolchain missing (see the README)"
            elif board["target"]:
                a_detail = "not on the board yet: use Flash firmware below (builds first, about 10 minutes the first time)"
            if job["result"] and job["result"] != "ok" and job.get("atlas"):
                a_detail += f" · last {job['action']} {job['result']}"
            steps.append({"id": "atlas", "label": "ATLAS firmware on the board", "ok": has_module and not (fw["needs_rebuild"] or fw["needs_flash"]),
                          "detail": a_detail, "action": a_action, "busy": job["running"]})
            hw_ok = link.params.get("NLF_HW_OK", {}).get("value") if has_module else None
            steps.append({"id": "hw_ok", "label": "Ground sequence allowed on hardware (NLF_HW_OK)", "ok": hw_ok == 1,
                          "detail": "" if hw_ok == 1 else ("your explicit consent to run the experimental sequence on the real aircraft" if has_module else "needs the ATLAS firmware"),
                          "action": "enable_hw" if (has_module and up and hw_ok != 1) else None})
        return steps

    # ------------------------------------------------------------ board == SITL: full parameter sync
    SITL_REFERENCE = Path(os.path.expanduser("~/.airframe_designer")) / "sitl_reference.json"
    # never written to a real flight controller: sensor/RC calibration, links, hardware drivers, safety checks and
    # failsafes, the user's mode slots and consent, and the SITL-only conveniences the app seeds
    SYNC_EXCLUDE = re.compile(r"^(CAL_|RC\d|RC_|COM_RC_IN_MODE|COM_FLTMODE|MAV_|SER_|UAVCAN|SENS_EN_|SENS_IMU|SENS_BOARD_ROT|IMU_|BAT\d?_|PWM_|HIL_ACT|SDLOG|SYS_HITL|SYS_AUTOSTART|SYS_AUTOCONFIG|SYS_HAS_|SYS_PARAM_VER|SYS_BL|SYS_USE_IO|MNT_|GPS_|TEL_|NLF_HW_OK|NLF_RC_|MAN_ARM_GESTURE|MAN_KILL_GEST|CBRK_|COM_ARM_|COM_PREARM|COM_DISARM_|NAV_DLL_ACT|NAV_RCL_ACT|COM_OBL_|COM_RCL_|COM_LOW_BAT|COM_POWER_|COM_CPU_|BAT_|LND_|_HASH)")

    def save_sitl_reference(self) -> None:
        link = self.link
        if link is None or link.mode != "sitl":
            return
        try:
            self.SITL_REFERENCE.parent.mkdir(parents=True, exist_ok=True)
            self.SITL_REFERENCE.write_text(json.dumps({"time": time.time(), "airframe": getattr(self.sim.airframe, "name", None),
                                                       "count": link.param_count,
                                                       "params": {k: v["value"] for k, v in link.params.items() if k != "_HASH_CHECK"}}))
            self.log(f"[params] SITL reference saved ({len(link.params)} parameters) for board syncing")
        except Exception as e:
            self.log(f"[params] could not save the SITL reference: {e}")

    def sitl_reference(self) -> dict | None:
        try:
            return json.loads(self.SITL_REFERENCE.read_text())
        except Exception:
            return None

    def sitl_diff(self) -> dict:
        """Parameters on the connected board that differ from the last SITL run (same names, excluded ones skipped)."""
        ref = self.sitl_reference()
        link = self.link
        if not ref:
            return {"ok": False, "error": "no SITL reference yet: connect the app to PX4 SITL once (Connect tab), it is captured automatically"}
        if link is None or link.mode != "hitl" or not link.param_count or len(link.params) < link.param_count:
            return {"ok": False, "error": "board parameters not fully downloaded yet"}
        differ, skipped = [], 0
        for name, sv in ref["params"].items():
            bp = link.params.get(name)
            if bp is None:
                continue
            if self.SYNC_EXCLUDE.match(name):
                if abs(float(bp["value"]) - float(sv)) > 1e-6:
                    skipped += 1
                continue
            if abs(float(bp["value"]) - float(sv)) > 1e-6:
                differ.append({"name": name, "board": bp["value"], "sitl": sv})
        return {"ok": True, "reference_time": ref["time"], "reference_airframe": ref.get("airframe"), "differ": differ, "skipped": skipped,
                "compared": sum(1 for n in ref["params"] if n in link.params)}

    def sync_to_sitl(self) -> dict:
        """Write every differing (non-excluded) parameter of the SITL reference to the board. Blocking."""
        d = self.sitl_diff()
        if not d.get("ok"):
            return d
        link = self.link
        if link.armed:
            return {"ok": False, "error": "vehicle is armed"}
        todo = {x["name"]: x["sitl"] for x in d["differ"]}
        t0 = time.time()
        results = link.set_params(todo, lambda name, res: None) if todo else []
        failed = [r["name"] for r in results if not r["ok"]]
        if todo:
            link.preflight_storage(True)
        self.log(f"[params] board synced to the SITL reference: {len(todo) - len(failed)}/{len(todo)} written in {time.time() - t0:.1f} s"
                 + (f", failed: {failed[:8]}" if failed else ""))
        return {"ok": not failed, "written": len(todo) - len(failed), "failed": failed, "seconds": round(time.time() - t0, 1), "skipped": d["skipped"]}

    # ------------------------------------------------------------ board firmware freshness (HITL)
    def _last_flash_file(self, target: str) -> Path:
        return Path(os.path.expanduser("~/.airframe_designer")) / f"last_flash_{target}.json"

    def record_flash(self, target: str, image: str | None) -> None:
        try:
            p = Path(image) if image else None
            d = {"target": target, "image": image, "mtime": p.stat().st_mtime if p and p.exists() else None, "time": time.time()}
            self._last_flash_file(target).parent.mkdir(parents=True, exist_ok=True)
            self._last_flash_file(target).write_text(json.dumps(d))
        except Exception as e:
            self.log(f"[firmware] could not record the flash: {e}")

    def board_firmware_status(self) -> dict:
        """Is the board image older than the module sources (needs a build) or newer than what was last flashed?"""
        target = self.detected_board().get("target")
        image = self.firmware_file(target) if target else None
        atlas_dir = str(PROJECT_DIR / "firmware" / "atlas")
        out = {"target": target, "image": image, "needs_rebuild": False, "needs_flash": False, "stale": []}
        if not target:
            return out
        if not image:
            out["needs_rebuild"] = True
            out["stale"] = ["no board image built yet"]
            return out
        t_img = Path(image).stat().st_mtime
        stale = [str(p.relative_to(atlas_dir)) for p in fw.source_files(atlas_dir) if p.stat().st_mtime > t_img + 1e-3]
        out["stale"] = stale
        out["needs_rebuild"] = bool(stale) or not self.firmware_is_atlas(target)
        try:
            last = json.loads(self._last_flash_file(target).read_text())
            out["needs_flash"] = (last.get("mtime") is None) or t_img > float(last["mtime"]) + 1e-3
        except Exception:
            out["needs_flash"] = True      # never flashed by this app
        return out

    def update_board_firmware(self) -> dict:
        """Build (if sources changed) and flash the ATLAS firmware to the connected board, then wait for it to come
        back with its parameters. Blocking; what 'Flash firmware' does before pushing the parameters."""
        st = self.board_firmware_status()
        out = {"ok": True, "rebuilt": False, "flashed": False, "stale": st["stale"]}
        if self.link is not None and self.link.armed:
            return {**out, "ok": False, "error": "vehicle is armed"}
        target = st["target"]
        if not target:
            return {**out, "ok": False, "error": "no board detected on USB"}
        if st["needs_rebuild"]:
            self.log(f"[firmware] building the ATLAS firmware for {target} ({len(st['stale'])} changed source file(s))")
            r = self.build_firmware(target, atlas=True)
            if not r.get("ok"):
                return {**out, "ok": False, "error": r.get("error")}
            while self.firmware_job.running():
                time.sleep(1.0)
            if self.firmware_job.exit_code != 0:
                return {**out, "ok": False, "error": f"build {self.firmware_job.result}"}
            out["rebuilt"] = True
            st = self.board_firmware_status()
        if out["rebuilt"] or st["needs_flash"]:
            r = self.upload_firmware(target, atlas=True, note="flashed from Update PX4")
            if not r.get("ok"):
                return {**out, "ok": False, "error": r.get("error")}
            while self.firmware_job.running():
                time.sleep(1.0)
            if self.firmware_job.exit_code != 0:
                return {**out, "ok": False, "error": f"flash {self.firmware_job.result}"}
            out["flashed"] = True
            if not self._wait_params(150.0):
                return {**out, "ok": False, "error": "the board did not come back with its parameters after the flash"}
        return out

    _ports_cache: tuple[float, list] = (0.0, [])

    def list_ports_cached(self, max_age: float = 2.0) -> list[dict]:
        t, ports = self._ports_cache
        if time.time() - t > max_age:
            ports = list_serial_ports()
            self._ports_cache = (time.time(), ports)
        return ports

    def _post_boot_ekf_restart(self, link, session: int | None = None) -> None:
        """After a (re)boot: wait for HIL data to flow again, give the EKF a few seconds, then select the default
        mode. EKF2 is deliberately NOT restarted at runtime: on the FMU v6X (PX4 v1.17) an "ekf2 stop / start" over
        the shell leaves an estimator that passes the arming checks but stops publishing the moment the vehicle
        arms ("Waiting for estimator to initialize", then flight termination). A fresh boot with the simulator
        already streaming initialises cleanly, so a reboot is the only estimator reset used in HITL."""
        seq0 = link.actuator_seq
        t0 = time.time()
        while time.time() - t0 < 60 and self.link is link and (session is None or self._params_session == session):
            if link.hil_enabled and link.actuator_seq - seq0 > 300 and time.time() - link.ctl_rx_time < 2.0:
                try:
                    link.trim_telemetry()      # a reboot restores the board's default stream rates
                except Exception as e:
                    self.log(f"[link] telemetry throttle failed: {e}")
                time.sleep(2.0)
                if self.link is link:
                    self._select_default_mode(link)
                return
            time.sleep(0.5)

    def _after_reboot(self, link) -> None:
        """Explicitly schedule the post-boot estimator restart for a reboot we triggered ourselves
        (the link watcher only catches it if the control link visibly drops)."""
        self._own_reboot_until = time.time() + 40.0
        def run():
            t0 = time.time()
            while time.time() - t0 < 20 and time.time() - link.ctl_rx_time < 1.5:
                time.sleep(0.5)                       # wait for the board to actually go away
            self._post_boot_ekf_restart(link, None)
        threading.Thread(target=run, daemon=True).start()

    # ------------------------------------------------------------ status
    def status(self) -> dict:
        link = self.link
        return {
            "mode": self.mode,
            "serial": self.serial,
            "baud": self.baud,
            "error": self.error,
            "busy": self.busy,
            "px4_running": self.px4_running(),
            "flashing": self.firmware_job.running() and self.firmware_job.action == "upload",
            "resetting": max(0.0, self._reset_busy_until - time.time()),
            "px4_instance": self.px4_instance,
            "ports": self.list_ports_cached(0.5),
            "qgc": self.args.qgc,
            "link": link.status() if link else None,
        }

    # ------------------------------------------------------------ watcher
    def _watch(self) -> None:
        """Re-download parameters whenever the control link (re)connects, e.g. after a reboot."""
        fetched_for = -1
        was_up = False
        while not self._stop.is_set():
            time.sleep(0.5)
            link = self.link
            if link is None:
                was_up = False
                continue
            up = link.ctl_connected and (time.time() - link.ctl_rx_time < 3.0)
            if up and not was_up:
                self._params_session += 1
            if up and fetched_for != self._params_session:
                fetched_for = self._params_session
                time.sleep(1.0)
                keep = (time.time() < getattr(self, "_own_reboot_until", 0.0) and link.param_count
                        and len(link.params) >= link.param_count)
                try:
                    link.request_autopilot_version()
                    if link.mode == "hitl":
                        # what the checklist needs, in a few hundred milliseconds; the full list follows in the background
                        self.native_module_seen = self.ensure_native_module() or self.native_module_seen or self._module_present()
                        for name in ("SYS_HITL", "NLF_ENABLE", "NLF_HW_OK"):
                            try:
                                link.get_param(name, timeout=1.0)
                            except Exception:
                                pass
                    if keep:
                        self.log(f"[params] board rebooted by us: keeping the {len(link.params)} cached parameters")
                    else:
                        link.fetch_all_params()
                    if link.mode != "hitl" and self.ensure_native_module():
                        link.fetch_all_params()          # the module's parameters are listed only once it runs
                    if self.on_params:
                        self.on_params()
                except Exception as e:
                    self.log(f"[params] fetch failed: {e}")
                if link.mode == "sitl" and link.param_count and len(link.params) >= link.param_count:
                    self.save_sitl_reference()      # what "identical to the SITL" means for a board
                if link.mode == "hitl" and link.param_count:
                    try:
                        link.trim_telemetry()
                    except Exception as e:
                        self.log(f"[link] telemetry throttle failed: {e}")
                if not link.param_count or len(link.params) < link.param_count:
                    # the board was still booting (or the link hiccupped): try again on the next pass
                    self.log("[params] incomplete download, retrying")
                    fetched_for = -1
                    time.sleep(3.0)
                    continue
                # no automatic estimator restart: with a board connected the app never nudges the flight controller
            was_up = up
