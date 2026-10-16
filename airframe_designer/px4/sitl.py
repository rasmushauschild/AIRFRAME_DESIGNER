"""PX4 SITL process control.

* ``launch_px4``: start the PX4 SITL binary for an instance with its own working directory, watched by a shell
  wrapper that sends it SIGINT when this process dies, so a crashed simulator never leaves a PX4 behind.
* ``write_param_file``: PX4 SITL imports ``fs/parameters.bson`` from its working directory at boot, so a batch
  run pre-seeds every parameter (airframe geometry, hand edits, scenario overrides) before PX4 even starts,
  instead of pushing them one by one over MAVLink afterwards. The file is PX4's own BSON subset: int32 (0x10)
  for INT32 parameters and double (0x01) for FLOAT ones.
* ``PX4Instance``: the pair (instance number, ports) plus a fresh or reused working directory.
"""
from __future__ import annotations

import fcntl
import os
import re
import shutil
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

DEFAULT_PX4_DIR = os.path.expanduser("~/PX4-Autopilot")
DEFAULT_WORK_ROOT = os.path.expanduser("~/.airframe_designer")


def find_px4_dir(explicit: str | None = None) -> str:
    for cand in ([explicit] if explicit else []) + [os.environ.get("PX4_DIR"), DEFAULT_PX4_DIR]:
        if cand and (Path(cand).expanduser() / "build" / "px4_sitl_default" / "bin" / "px4").is_file():
            return str(Path(cand).expanduser())
    return str(Path(explicit or DEFAULT_PX4_DIR).expanduser())


def px4_binary(px4_dir: str) -> Path:
    return Path(px4_dir) / "build" / "px4_sitl_default" / "bin" / "px4"


def free_px4_instance(start: int = 0, limit: int = 10) -> int:
    """PX4 SITL holds an flock on /tmp/px4_lock-<instance>; find the first instance nobody holds.
    Instances above 9 share MAVLink ports in PX4's startup scripts, so the search stops there."""
    for i in range(start, min(start + 16, limit)):
        if instance_is_free(i):
            return i
    raise RuntimeError(f"no free PX4 SITL instance between {start} and {limit - 1}")


def instance_is_free(i: int) -> bool:
    path = f"/tmp/px4_lock-{i}"
    try:
        fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return True
    except OSError:
        return False
    finally:
        os.close(fd)


# ------------------------------------------------------------------ BSON parameter file
def _bson_int32(name: str, v: int) -> bytes:
    return b"\x10" + name.encode() + b"\x00" + struct.pack("<i", int(v))


def _bson_double(name: str, v: float) -> bytes:
    return b"\x01" + name.encode() + b"\x00" + struct.pack("<d", float(v))


def encode_param_bson(params: dict[str, float | int], int_names: set[str] | None = None,
                      types: dict[str, str] | None = None) -> bytes:
    """PX4 parameter file. Values whose Python type is int (or whose metadata type is Int32) are INT32, the rest
    FLOAT. ``types`` maps name -> "Int32"/"Float" from parameters.json when available (the safest source)."""
    body = b""
    for name, v in sorted(params.items()):
        if isinstance(v, bool):
            v = int(v)
        t = (types or {}).get(name)
        is_int = (t == "Int32") if t else (name in (int_names or set()) or isinstance(v, int))
        body += _bson_int32(name, int(round(float(v)))) if is_int else _bson_double(name, float(v))
    body += b"\x00"
    return struct.pack("<i", len(body) + 4) + body


def write_param_file(workdir: str | Path, params: dict[str, float | int], types: dict[str, str] | None = None) -> Path:
    fs = Path(workdir) / "fs"
    fs.mkdir(parents=True, exist_ok=True)
    data = encode_param_bson(params, types=types)
    p = fs / "parameters.bson"
    p.write_bytes(data)
    (fs / "parameters_backup.bson").write_bytes(data)
    return p


def param_types_from_meta(meta: dict[str, dict]) -> dict[str, str]:
    """{name: 'Int32'|'Float'} from flattened parameters.json metadata."""
    return {k: ("Int32" if str(v.get("type", "")).lower().startswith("int") else "Float") for k, v in meta.items()}


# ------------------------------------------------------------------ launching
class PX4Instance:
    """A SITL instance number with its ports and working directory."""

    def __init__(self, instance: int, workdir: str | Path | None = None, fresh: bool = False):
        self.instance = int(instance)
        self.sim_port = 4560 + self.instance                # simulator_mavlink connects here (TCP)
        self.ctl_port = 14540 + self.instance               # PX4 sends its onboard MAVLink here (UDP)
        self.workdir = Path(workdir) if workdir else Path(DEFAULT_WORK_ROOT) / f"px4_instance_{self.instance}"
        if " " in str(self.workdir):
            raise RuntimeError(f"PX4 working directory must not contain spaces: {self.workdir}")
        if fresh and self.workdir.exists():
            shutil.rmtree(self.workdir, ignore_errors=True)
        self.workdir.mkdir(parents=True, exist_ok=True)

    @property
    def tcp_address(self) -> str:
        return f"0.0.0.0:{self.sim_port}"

    @property
    def ctl_address(self) -> str:
        return f"udpin:127.0.0.1:{self.ctl_port}"


def autostart_id(px4_dir: str, model: str) -> int | None:
    """SYS_AUTOSTART of a SITL model (the numeric prefix of its airframe file), e.g. none_iris -> 10016."""
    d = Path(find_px4_dir(px4_dir)) / "build" / "px4_sitl_default" / "etc" / "init.d-posix" / "airframes"
    for p in d.glob(f"*_{model}"):
        m = re.match(r"(\d+)_", p.name)
        if m:
            return int(m.group(1))
    return None


def clear_stale_socket(instance: int, path: str | None = None, wait_s: float = 3.0) -> bool:
    """Remove /tmp/px4-sock-<i> when nobody is listening on it.

    PX4 refuses to start ("PX4 server already running for instance N", exit 2) while that socket exists, and
    a PX4 killed outright leaves it behind: one crashed run then blocks every later one on that instance.
    Connecting is the test - a live daemon accepts, a dead one's socket refuses - so a running PX4 is never
    removed. The previous one is often still shutting down when the next app starts, which is why this
    waits a few seconds for it to let go before giving up: deciding too early leaves the file behind and
    the launch fails anyway."""
    path = path or f"/tmp/px4-sock-{int(instance)}"
    deadline = time.time() + max(0.0, wait_s)
    while True:
        if not os.path.exists(path):
            return False
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(0.3)
        try:
            sock.connect(path)
            alive = True                  # somebody is home
        except OSError:
            alive = False
        finally:
            sock.close()
        if not alive:
            break
        if time.time() >= deadline:
            return False                  # a PX4 really is running on this instance
        time.sleep(0.25)
    try:
        os.unlink(path)
        return True
    except OSError:
        return False


def launch_px4(px4_dir: str, model: str, log, instance: int = 0, rootfs: str | None = None,
               params: dict[str, float | int] | None = None, param_types: dict[str, str] | None = None,
               fresh: bool = False, quiet: bool = False) -> subprocess.Popen:
    """Start PX4 SITL. ``params`` are written to the instance's parameter file first (see write_param_file)."""
    px4_dir = find_px4_dir(px4_dir)
    build = Path(px4_dir) / "build" / "px4_sitl_default"
    binary = build / "bin" / "px4"
    if not binary.is_file():
        raise RuntimeError(f"PX4 SITL binary not found at {binary}. Build it with: cd {px4_dir} && make px4_sitl_default")
    inst = PX4Instance(instance, rootfs, fresh=fresh)
    if clear_stale_socket(instance):
        log(f"[px4] removed the socket a dead PX4 left on instance {instance}")
    if params:
        params = dict(params)
        # PX4's rcS resets every parameter when SYS_AUTOSTART in the file differs from the model's id (a fresh
        # "autoconfig"), which would throw the seeded values away. Pin it, and mark the config as done.
        sa = autostart_id(px4_dir, model)
        if sa is not None:
            params.setdefault("SYS_AUTOSTART", sa)
        params.setdefault("SYS_AUTOCONFIG", 0)
        params.setdefault("MAV_SYS_ID", instance + 1)
        params.setdefault("UXRCE_DDS_KEY", instance + 1)
        write_param_file(inst.workdir, params, param_types)
    env = dict(os.environ)
    env["PX4_SIM_MODEL"] = model
    env.setdefault("PX4_SIMULATOR", "mavlink")
    px4_cmd = [str(binary), "-d", "-i", str(instance), "-w", str(inst.workdir), str(build / "etc")]
    log(f"[px4] launching: PX4_SIM_MODEL={model} {' '.join(px4_cmd)}")
    # If the app goes away the PX4 it started must go too, and go for certain: one that ignores the INT
    # keeps its /tmp/px4-sock-<i> and blocks the next launch on that instance, so it is given three seconds
    # and then killed outright.
    watchdog = (
        'child=""; trap \'[ -n "$child" ] && kill -INT $child 2>/dev/null\' TERM INT; '
        '"$@" & child=$!; '
        'while kill -0 $AIRFRAME_SIM_PID 2>/dev/null && kill -0 $child 2>/dev/null; do sleep 0.5; done; '
        'kill -INT $child 2>/dev/null; '
        'i=0; while kill -0 $child 2>/dev/null && [ $i -lt 6 ]; do sleep 0.5; i=$((i+1)); done; '
        'kill -9 $child 2>/dev/null; wait $child'
    )
    env["AIRFRAME_SIM_PID"] = str(os.getpid())
    cmd = ["/bin/sh", "-c", watchdog, "px4-watchdog"] + px4_cmd
    proc = subprocess.Popen(cmd, cwd=str(inst.workdir), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, start_new_session=True)
    ansi = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

    def pump():
        for line in proc.stdout:
            line = ansi.sub("", line).rstrip()
            if quiet and not re.search(r"ERROR|WARN|error|failed", line):
                continue
            log(f"[px4] {line}")
        log(f"[px4] exited with code {proc.poll()}")

    threading.Thread(target=pump, daemon=True).start()
    return proc


def stop_px4(proc: subprocess.Popen | None, timeout: float = 3.0) -> None:
    import signal
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout)
    except (subprocess.TimeoutExpired, ProcessLookupError, PermissionError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(2)
        except Exception:
            pass


def wait_for_exit(proc: subprocess.Popen, timeout: float) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        if proc.poll() is not None:
            return True
        time.sleep(0.05)
    return False
