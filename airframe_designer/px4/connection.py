"""Runtime connection management: switch between PX4 SITL and a physical Pixhawk (HITL) without restarting.

Also owns the PX4 SITL child process and the HITL readiness checklist the UI shows.
"""
from __future__ import annotations

import glob
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

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self, board: str, action: str, px4_dir: str, venv_bin: str, ref: str | None = None) -> dict:
        if self.running():
            return {"ok": False, "error": f"{self.action} already running"}
        script = PROJECT_DIR / "scripts" / "build_hitl_firmware.sh"
        env = dict(os.environ)
        env["PX4_DIR"] = px4_dir
        if ref:
            env["PX4_REF"] = ref
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
                "exit_code": self.exit_code}


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
        board_dir = Path(self.args.px4_dir) / "boards" / target.replace("_", "/", 1)
        return "multicopter" if (board_dir / "multicopter.px4board").is_file() else "default"

    def firmware_file(self, target: str | None) -> str | None:
        if not target:
            return None
        v = self.firmware_variant(target)
        f = Path(self.args.px4_dir) / "build" / f"{target}_{v}" / f"{target}_{v}.px4"
        return str(f) if f.is_file() else None

    def build_firmware(self, target: str | None = None) -> dict:
        target = target or self.detected_board()["target"]
        if not target:
            return {"ok": False, "error": "could not tell the board type from USB; pass the target, e.g. px4_fmu-v6x"}
        if not self.toolchain_present():
            return {"ok": False, "error": "ARM toolchain missing. Run:  brew tap osx-cross/arm; brew trust osx-cross/arm && brew install osx-cross/arm/arm-gcc-bin@13 && brew link --overwrite --force arm-gcc-bin@13   then try again."}
        venv_bin = str(PROJECT_DIR / ".venv" / "bin")
        return self.firmware_job.start(target, "build", self.args.px4_dir, venv_bin, ref=self.board_release_tag())

    def board_release_tag(self) -> str | None:
        """'1.17.0 release' on the board -> 'v1.17.0', so the HITL build matches what is flashed."""
        fw = self.link.firmware if self.link else {}
        ver = (fw or {}).get("version", "")
        m = re.match(r"(\d+\.\d+\.\d+) release", ver)
        return f"v{m.group(1)}" if m else None

    def upload_firmware(self, target: str | None = None) -> dict:
        """Flash the built firmware. We must release the serial port first; the link reconnects after."""
        target = target or self.detected_board()["target"]
        if not target or not self.firmware_file(target):
            return {"ok": False, "error": "no built firmware for this board yet; build it first"}
        with self._lock:
            was_hitl = self.mode == "hitl"
            serial = self.serial
            ref = self.board_release_tag()
            if was_hitl:
                self._close_link()
                time.sleep(1.0)   # let the OS release the device
        venv_bin = str(PROJECT_DIR / ".venv" / "bin")
        r = self.firmware_job.start(target, "upload", self.args.px4_dir, venv_bin, ref=ref)
        if r.get("ok") and was_hitl:
            def reconnect():
                while self.firmware_job.running():
                    time.sleep(1.0)
                time.sleep(4.0)   # let the board reboot into the new firmware and re-enumerate
                self.log("[firmware] reconnecting to the board")
                self.connect_hitl(serial, self.baud)
            threading.Thread(target=reconnect, daemon=True).start()
        return r

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
        link = self.link
        steps = []
        ports = self.list_ports_cached()
        px4_ports = [p for p in ports if p["likely_px4"]]
        steps.append({"id": "port", "label": "Pixhawk detected on USB",
                      "ok": bool(px4_ports) or (link is not None and link.mode == "hitl" and link.connected),
                      "detail": ", ".join(p["device"] for p in px4_ports) if px4_ports else "no flight controller found on USB"})
        hitl = link is not None and link.mode == "hitl"
        up = hitl and link.ctl_connected and (time.time() - link.ctl_rx_time < 3.0)
        steps.append({"id": "link", "label": "Serial link up", "ok": up,
                      "detail": (f"{link.address} · sysid {link.target_system}" if up else
                                 (self.error or ("connecting…" if hitl else "not connected to the board")))})
        params_ok = hitl and link.param_count > 0 and len(link.params) >= link.param_count
        steps.append({"id": "params", "label": "Parameters downloaded", "ok": params_ok,
                      "detail": f"{len(link.params)}/{link.param_count}" if hitl else ""})
        has_hil_driver = hitl and params_ok and any(k.startswith("HIL_ACT_FUNC") for k in link.params)
        fw = link.firmware if hitl else {}
        board = self.detected_board()
        built = self.firmware_file(board["target"])
        job = self.firmware_job.status()
        fw_action = None
        fw_detail = f"PX4 v{fw.get('version')}" if fw else ""
        if hitl and params_ok and not has_hil_driver:
            fw_detail += " · built without the HIL output driver (pwm_out_sim), so HITL cannot run on it. "
            if job["running"]:
                fw_detail += f"{job['action'].capitalize()}ing {job['board']}… see the log."
            elif built:
                fw_detail += f"A HITL-capable build for {board['target']} is ready: flash it (about a minute, the board reboots)."
                fw_action = "upload_firmware"
            elif not self.toolchain_present():
                fw_detail += "Install the ARM toolchain once (see the README: brew trust osx-cross/arm, then brew install osx-cross/arm/arm-gcc-bin@13), then build here."
                fw_action = "build_firmware"
            elif board["target"]:
                fw_detail += f"Build a HITL-capable firmware for {board['target']} here (a few minutes), then flash it."
                fw_action = "build_firmware"
            else:
                fw_detail += "Run scripts/build_hitl_firmware.sh <board> and flash the result with QGroundControl."
            if job["result"] and job["result"] != "ok":
                fw_detail += f" Last {job['action']} {job['result']}"
        steps.append({"id": "firmware", "label": "Firmware supports HITL (has the HIL output driver)",
                      "ok": has_hil_driver, "detail": fw_detail, "action": fw_action, "busy": job["running"]})
        sys_hitl = link.params.get("SYS_HITL", {}).get("value") if hitl else None
        steps.append({"id": "sys_hitl", "label": "HITL enabled on the board (SYS_HITL = 1)",
                      "ok": sys_hitl == 1, "detail": f"SYS_HITL = {sys_hitl}" if sys_hitl is not None else "",
                      "action": "enable_hitl" if (hitl and up and has_hil_driver and sys_hitl not in (None, 1)) else None})
        streaming = hitl and up and link.hil_enabled and link.actuator_seq > 0
        steps.append({"id": "hil", "label": "Board is in HIL mode and streaming actuator outputs", "ok": streaming,
                      "detail": (f"{link.actuator_seq} actuator messages" if streaming else
                                 ("heartbeat has no HIL flag — reboot after enabling HITL" if hitl and up else ""))})
        summary = None
        if hitl:
            for x in reversed(list(link.recent_events)):
                if x.get("name") == "commander_arming_check_summary":
                    summary = dict(zip(x.get("arg_names", []), x.get("args", [])))
                    break
        can_arm = str(summary.get("can_arm", "")) if summary else ""
        ready = bool(summary) and ("takeoff" in can_arm or "loiter" in can_arm)
        blockers = [x["text"] for x in list(link.recent_events)[-12:] if x.get("level", 9) <= 3 and x.get("group") in ("health", "arming_check")
                    and "offboard" not in x["text"].lower() and "mission" not in x["text"].lower()] if hitl else []
        steps.append({"id": "arming", "label": "Estimator settled, board can arm (Takeoff / Hold)",
                      "ok": streaming and ready,
                      "detail": ("can arm in: " + can_arm.replace("|", ", ") if ready else
                                 (("; ".join(dict.fromkeys(blockers)) or "waiting for the arming check report…") if streaming else "")) +
                                ("" if not streaming or ready else " · if this never clears after a sim reset, reboot the board so the estimator starts clean"),
                      "action": ("ekf" if any(k in " ".join(blockers).lower() for k in ("attitude", "accel", "height", "velocity"))
                                 else "reboot") if (streaming and not ready) else None})
        if export_params is not None and hitl and params_ok:
            diff = [k for k, v in export_params.items() if k in link.params and abs(float(link.params[k]["value"]) - float(v)) > 1e-4]
            missing = [k for k in export_params if k not in link.params]
            steps.append({"id": "geometry", "label": "Airframe geometry pushed to the board", "ok": not diff,
                          "detail": (f"{len(diff)} parameters differ" if diff else "matches") +
                                    (f" · not in firmware: {len(missing)}" if missing else ""),
                          "action": "push" if diff else None})
        else:
            steps.append({"id": "geometry", "label": "Airframe geometry pushed to the board", "ok": False, "detail": ""})
        return steps

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
                    if keep:
                        self.log(f"[params] board rebooted by us: keeping the {len(link.params)} cached parameters")
                    else:
                        link.fetch_all_params()
                    if self.on_params:
                        self.on_params()
                except Exception as e:
                    self.log(f"[params] fetch failed: {e}")
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
                # HITL: the board booted while our sensor stream was (re)starting and the EKF often initialises on
                # the first bad samples. Once HIL data has flowed for a few seconds, restart the estimator once.
                if link.mode == "hitl":
                    threading.Thread(target=self._post_boot_ekf_restart, args=(link, self._params_session), daemon=True).start()
            was_up = up
