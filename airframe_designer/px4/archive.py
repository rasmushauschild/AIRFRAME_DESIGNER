"""Firmware archive: every flash to a physical board (and manual snapshots) becomes a version in a git repository
that is pushed to GitHub, so the firmware image, the board's complete parameter set, the module sources and the
airframe can always be restored.

Layout of the archive repository (AFD_FIRMWARE_ARCHIVE, default https://github.com/rasmushauschild/PX4_FIRMWARE.git,
cloned to ~/.airframe_designer/px4_firmware_archive):

    versions/<YYYYMMDD-HHMMSS>_<target>/
        manifest.json          board, PX4 version on the board, source commits, airframe name, note, sha256, ...
        firmware.px4           the flashed image (absent for a parameters-only snapshot)
        params_board.params    every parameter the board reported, QGroundControl format
        params_export.json     what the app exported (geometry, allocation, module settings)
        airframe.json          the airframe as it was in the app
        atlas/                 the module sources the firmware was built from (src, msg, allocator_overlay)
    versions/index.json        newest first
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[2]
DEFAULT_URL = "https://github.com/rasmushauschild/PX4_FIRMWARE.git"
DEFAULT_DIR = Path(os.path.expanduser("~/.airframe_designer/px4_firmware_archive"))
ATLAS_PARTS = ("src", "msg", "allocator_overlay", "README.md", "LANDING.md", "atlas_07d.native.json", "atlas_07d-nose-lift.sitl.params")


def _git(args: list[str], cwd: Path, timeout: float = 300.0) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.setdefault("GIT_TERMINAL_PROMPT", "0")
    return subprocess.run(["git", *args], cwd=str(cwd), env=env, capture_output=True, text=True, timeout=timeout)


def _commit_of(path: Path) -> str | None:
    try:
        r = _git(["rev-parse", "HEAD"], path, timeout=10)
        return r.stdout.strip() or None if r.returncode == 0 else None
    except Exception:
        return None


def params_to_qgc(params: dict, sysid: int = 1, compid: int = 1) -> str:
    """PX4/QGroundControl .params text: '# Onboard parameters' then 'sysid\\tcompid\\tNAME\\tVALUE\\tTYPE'."""
    lines = ["# Onboard parameters for Vehicle 1", "#", "# Vehicle-Id Component-Id Name Value Type"]
    for name in sorted(params):
        p = params[name]
        val, typ = (p.get("value"), p.get("type", 9)) if isinstance(p, dict) else (p, 9)
        if val is None:
            continue
        if typ in (6,):
            txt = str(int(val))
        else:
            txt = repr(float(val)) if not float(val).is_integer() else f"{float(val):.1f}"
        lines.append(f"{sysid}\t{compid}\t{name}\t{txt}\t{typ}")
    return "\n".join(lines) + "\n"


def qgc_to_params(text: str) -> dict:
    """Inverse of params_to_qgc: name -> {'value', 'type'}."""
    out = {}
    for line in text.splitlines():
        if not line or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) < 5:
            continue
        name, val, typ = parts[2], parts[3], int(parts[4])
        out[name] = {"value": int(val) if typ == 6 else float(val), "type": typ}
    return out


class FirmwareArchive:
    def __init__(self, log, url: str | None = None, path: Path | None = None):
        self.log = log
        self.url = url or os.environ.get("AFD_FIRMWARE_ARCHIVE", DEFAULT_URL)
        self.path = Path(path or os.environ.get("AFD_FIRMWARE_ARCHIVE_DIR", DEFAULT_DIR))
        self.error: str | None = None
        self.last_push: float | None = None
        self.last_push_ok: bool | None = None
        self.busy = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ repo
    def ensure(self) -> bool:
        """Clone the archive (or initialise it when the GitHub repository is still empty)."""
        with self._lock:
            if (self.path / ".git").is_dir():
                return True
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.log(f"[archive] cloning {self.url}")
            r = _git(["clone", "--quiet", self.url, str(self.path)], self.path.parent, timeout=600)
            if r.returncode != 0:
                self.error = f"clone failed: {r.stderr.strip()[:200]}"
                self.log(f"[archive] {self.error}")
                return False
            if not (self.path / "versions").is_dir():
                (self.path / "versions").mkdir()
                (self.path / "README.md").write_text(
                    "# PX4_FIRMWARE\n\nFirmware archive written by AIRFRAME_DESIGNER: one folder per flash under `versions/` "
                    "(image, full parameter set, exported parameters, airframe, module sources). Restore any of them from the "
                    "app's Versions tab.\n")
                (self.path / "versions" / "index.json").write_text("[]\n")
                _git(["add", "-A"], self.path)
                _git(["-c", "user.name=AIRFRAME_DESIGNER", "-c", "user.email=airframe-designer@local", "commit", "-q", "-m", "initialise firmware archive"], self.path)
                self._push()
            self.error = None
            return True

    def pull(self) -> dict:
        if not self.ensure():
            return {"ok": False, "error": self.error}
        with self._lock:
            r = _git(["pull", "--quiet", "--rebase", "--autostash"], self.path, timeout=600)
            if r.returncode != 0:
                self.error = f"pull failed: {r.stderr.strip()[:200]}"
                return {"ok": False, "error": self.error}
            self.error = None
            return {"ok": True}

    def _push(self) -> bool:
        r = _git(["push", "--quiet", "-u", "origin", "HEAD"], self.path, timeout=600)
        self.last_push = time.time()
        self.last_push_ok = r.returncode == 0
        if r.returncode != 0:
            self.error = f"push failed: {r.stderr.strip()[:200]}"
            self.log(f"[archive] {self.error}")
        else:
            self.error = None
            self.log("[archive] pushed to GitHub")
        return self.last_push_ok

    def _commit_and_push(self, message: str) -> None:
        with self._lock:
            self.busy = True
            try:
                _git(["add", "-A"], self.path)
                r = _git(["-c", "user.name=AIRFRAME_DESIGNER", "-c", "user.email=airframe-designer@local", "commit", "-q", "-m", message], self.path)
                if r.returncode != 0 and "nothing to commit" not in (r.stdout + r.stderr):
                    self.error = f"commit failed: {r.stderr.strip()[:200]}"
                    self.log(f"[archive] {self.error}")
                    return
                self._push()
            finally:
                self.busy = False

    # --------------------------------------------------------------- versions
    def versions(self) -> list[dict]:
        idx = self.path / "versions" / "index.json"
        if not idx.exists():
            return []
        try:
            return json.loads(idx.read_text())
        except Exception:
            return []

    def _write_index(self) -> None:
        entries = []
        for d in sorted((self.path / "versions").iterdir(), reverse=True):
            m = d / "manifest.json"
            if m.is_file():
                try:
                    entries.append(json.loads(m.read_text()))
                except Exception:
                    pass
        (self.path / "versions" / "index.json").write_text(json.dumps(entries, indent=1) + "\n")

    def version_dir(self, version_id: str) -> Path | None:
        if not version_id or "/" in version_id or version_id.startswith("."):
            return None
        d = self.path / "versions" / version_id
        return d if d.is_dir() else None

    def get(self, version_id: str) -> dict | None:
        d = self.version_dir(version_id)
        if not d:
            return None
        m = json.loads((d / "manifest.json").read_text())
        m["files"] = sorted(p.name for p in d.iterdir())
        m["firmware_file"] = str(d / "firmware.px4") if (d / "firmware.px4").is_file() else None
        m["params_board"] = qgc_to_params((d / "params_board.params").read_text()) if (d / "params_board.params").is_file() else {}
        m["params_export"] = json.loads((d / "params_export.json").read_text()) if (d / "params_export.json").is_file() else {}
        m["airframe"] = json.loads((d / "airframe.json").read_text()) if (d / "airframe.json").is_file() else None
        return m

    def snapshot(self, *, kind: str, board: dict, firmware_file: str | None, board_firmware: dict | None,
                 params_board: dict, params_export: dict, airframe: dict | None, atlas: bool,
                 note: str = "", extra: dict | None = None) -> dict:
        """Write a version folder and commit+push it in the background. Returns the manifest."""
        if not self.ensure():
            return {"ok": False, "error": self.error}
        target = (board or {}).get("target") or "board"
        vid = datetime.now().strftime("%Y%m%d-%H%M%S") + "_" + target
        d = self.path / "versions" / vid
        d.mkdir(parents=True, exist_ok=True)
        sha = None
        size = None
        if firmware_file and Path(firmware_file).is_file():
            shutil.copy2(firmware_file, d / "firmware.px4")
            sha = hashlib.sha256(Path(firmware_file).read_bytes()).hexdigest()
            size = Path(firmware_file).stat().st_size
        (d / "params_board.params").write_text(params_to_qgc(params_board or {}))
        (d / "params_export.json").write_text(json.dumps(params_export or {}, indent=1, sort_keys=True) + "\n")
        if airframe is not None:
            (d / "airframe.json").write_text(json.dumps(airframe, indent=1) + "\n")
        if atlas:
            src = PROJECT_DIR / "firmware" / "atlas"
            for part in ATLAS_PARTS:
                p = src / part
                if p.is_dir():
                    shutil.copytree(p, d / "atlas" / part, dirs_exist_ok=True)
                elif p.is_file():
                    (d / "atlas").mkdir(exist_ok=True)
                    shutil.copy2(p, d / "atlas" / part)
        manifest = {
            "id": vid, "kind": kind, "time": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "board": board, "board_firmware": board_firmware, "atlas": bool(atlas),
            "firmware_sha256": sha, "firmware_bytes": size,
            "airframe": (airframe or {}).get("name") if airframe else None,
            "params_count": len(params_board or {}), "export_count": len(params_export or {}),
            "px4_source_commit": _commit_of(Path(os.path.expanduser(os.environ.get("PX4_SOURCE_DIR", "~/PX4-Autopilot")))),
            "designer_commit": _commit_of(PROJECT_DIR), "note": note or "", **(extra or {}),
        }
        (d / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
        self._write_index()
        msg = f"{kind}: {target} · {(board_firmware or {}).get('version', '?')} · {manifest['airframe'] or '-'}" + (f" · {note}" if note else "")
        self.log(f"[archive] saved version {vid} ({'with' if sha else 'without'} firmware image, {manifest['params_count']} board parameters)")
        threading.Thread(target=self._commit_and_push, args=(msg,), name="archive-push", daemon=True).start()
        return {"ok": True, **manifest}

    def status(self) -> dict:
        web = self.url[:-4] if self.url.endswith(".git") else self.url
        return {"url": self.url, "web": web, "path": str(self.path), "cloned": (self.path / ".git").is_dir(),
                "error": self.error, "last_push": self.last_push, "last_push_ok": self.last_push_ok, "busy": self.busy,
                "count": len(self.versions())}
