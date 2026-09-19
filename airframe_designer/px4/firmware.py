"""SITL firmware freshness: is the PX4 binary older than the custom module sources, and rebuild it if so.

Applies to a *custom firmware tree* such as firmware/atlas (external modules + allocator overlay built against the
PX4 source in ~/PX4-Autopilot). A stock PX4 checkout (no ``src/<module>`` next to ``build/``) is never rebuilt here.
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path

from .sitl import px4_binary

PROJECT_DIR = Path(__file__).resolve().parents[2]
SOURCE_PATTERNS = ("src/**/*.cpp", "src/**/*.hpp", "src/**/*.h", "src/**/*.c", "src/**/*.yaml", "src/**/CMakeLists.txt",
                   "src/**/Kconfig", "msg/*.msg", "msg/CMakeLists.txt", "allocator_overlay/*")
DEFAULT_PX4_SOURCE = os.path.expanduser("~/PX4-Autopilot")
_cache: dict[str, tuple[float, dict]] = {}


def is_custom_tree(px4_dir: str) -> bool:
    d = Path(os.path.expanduser(px4_dir))
    return (d / "src").is_dir() and not (d / "Tools").is_dir()   # a module tree, not a PX4 checkout


def source_files(px4_dir: str) -> list[Path]:
    d = Path(os.path.expanduser(px4_dir))
    out: list[Path] = []
    for pat in SOURCE_PATTERNS:
        out.extend(p for p in d.glob(pat) if p.is_file())
    return sorted(set(out))


def stale_sources(px4_dir: str) -> list[str]:
    """Source files newer than the built binary (all of them when there is no binary)."""
    if not is_custom_tree(px4_dir):
        return []
    binary = px4_binary(os.path.expanduser(px4_dir))
    t = binary.stat().st_mtime if binary.exists() else -1.0
    return [str(p.relative_to(os.path.expanduser(px4_dir))) for p in source_files(px4_dir) if p.stat().st_mtime > t + 1e-3]


def status(px4_dir: str, launched_binary_mtime: float | None = None, max_age: float = 3.0) -> dict:
    """Cached (max_age s) freshness report: needs_rebuild, needs_relaunch, stale file list."""
    key = str(px4_dir)
    now = time.time()
    hit = _cache.get(key)
    if hit and now - hit[0] < max_age and hit[1].get("launched") == launched_binary_mtime:
        return hit[1]
    custom = is_custom_tree(px4_dir)
    binary = px4_binary(os.path.expanduser(px4_dir))
    bm = binary.stat().st_mtime if binary.exists() else None
    stale = stale_sources(px4_dir) if custom else []
    s = {"custom": custom, "binary": str(binary) if custom else None, "binary_mtime": bm, "launched": launched_binary_mtime,
         "stale": stale, "needs_rebuild": custom and (bm is None or bool(stale)),
         "needs_relaunch": custom and bm is not None and launched_binary_mtime is not None and bm > launched_binary_mtime + 1e-3}
    _cache[key] = (now, s)
    return s


def invalidate() -> None:
    _cache.clear()


def build(px4_dir: str, log, jobs: int = 6, timeout: float = 1800.0) -> dict:
    """Configure (first time) and build the SITL binary; streams a condensed log. Blocking."""
    d = Path(os.path.expanduser(px4_dir))
    build_dir = d / "build" / "px4_sitl_default"
    src = os.environ.get("PX4_SOURCE_DIR", DEFAULT_PX4_SOURCE)
    env = dict(os.environ)
    env.setdefault("CCACHE_DIR", "/tmp/atlas-nl-ccache")
    python = sys.executable if sys.executable else str(PROJECT_DIR / ".venv" / "bin" / "python")
    t0 = time.time()
    steps = []
    if not (build_dir / "CMakeCache.txt").exists():
        if not (Path(src) / "CMakeLists.txt").exists():
            return {"ok": False, "error": f"PX4 source not found at {src} (set PX4_SOURCE_DIR)"}
        cmd = ["cmake", "-S", src, "-B", str(build_dir), "-G", "Ninja", "-DCONFIG=px4_sitl_default",
               f"-DEXTERNAL_MODULES_LOCATION={d}", f"-DPYTHON_EXECUTABLE={python}"]
        log(f"[firmware] configuring {build_dir.relative_to(PROJECT_DIR) if build_dir.is_relative_to(PROJECT_DIR) else build_dir}")
        steps.append("configure")
        r = _run(cmd, d, env, log, timeout)
        if r != 0:
            return {"ok": False, "error": f"cmake configure failed ({r})", "steps": steps}
    log("[firmware] building PX4 SITL with the ATLAS modules (rebuilds only what changed)")
    steps.append("build")
    r = _run(["cmake", "--build", str(build_dir), "-j", str(jobs)], d, env, log, timeout)
    invalidate()
    if r != 0:
        return {"ok": False, "error": f"firmware build failed ({r}); see the log", "steps": steps}
    log(f"[firmware] build finished in {time.time() - t0:.0f} s")
    return {"ok": True, "steps": steps, "seconds": round(time.time() - t0, 1)}


def _run(cmd: list[str], cwd: Path, env: dict, log, timeout: float) -> int:
    ansi = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
    proc = subprocess.Popen(cmd, cwd=str(cwd), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    deadline = time.time() + timeout
    last = ""
    for line in proc.stdout:
        line = ansi.sub("", line).rstrip()
        if not line:
            continue
        last = line
        if line.startswith("[") and "/" in line[:12] and "]" in line[:14]:      # ninja progress: keep every 50th
            try:
                if int(line[1:line.index("/")]) % 50:
                    continue
            except ValueError:
                pass
        if "error" in line.lower() or "warning: unused" in line.lower() or line.startswith("[") or "FAILED" in line:
            log(f"[firmware] {line[:160]}")
        if time.time() > deadline:
            proc.kill()
            log("[firmware] build timed out")
            return -1
    return proc.wait()
