"""Command line entry points that need no PX4, run as subprocesses with the venv python."""
from __future__ import annotations

import json
import subprocess
import sys

import pytest

from airframe_designer.cli import _parse_set, main

from .conftest import AIRFRAMES_DIR, PROJECT_DIR

QUAD = str(AIRFRAMES_DIR / "quad_x.json")


def run_cli(*args: str, timeout: float = 60.0) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-m", "airframe_designer", *args], cwd=str(PROJECT_DIR), capture_output=True,
                          text=True, timeout=timeout)


def test_scenarios_lists_bundled_files():
    r = run_cli("scenarios")
    assert r.returncode == 0, r.stderr
    assert "hover.json" in r.stdout and "motor_out.json" in r.stdout
    assert len(r.stdout.strip().splitlines()) == len(list((PROJECT_DIR / "scenarios").glob("*.json")))


def test_analyse_quad():
    r = run_cli("analyse", "--airframe", QUAD)
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout)
    assert out["hover"]["ok"] is True and out["hover_check"]["ok"] is True and out["validate"] == []
    # The default cruise speed is 50 km/h. Every rotor is a ducted jet now, and a duct of this size swallows
    # enough air at that speed that the momentum drag exceeds the weight, so the trim correctly fails. The
    # same aircraft trims fine at a walking pace; see test_analysis.
    assert out["cruise"]["converged"] is False
    assert out["cruise"]["problems"]


def test_analyse_with_set_and_speed():
    r = run_cli("analyse", "--airframe", QUAD, "--speed-kmh", "36", "--set", "mass.mass=1.2", "--set", "rotors[*].tilt_deg=5")
    assert r.returncode == 0, r.stderr
    out = json.loads(r.stdout)
    assert out["airspeed"] == pytest.approx(10.0)
    assert out["hover"]["max_util"] == pytest.approx(1.2 * 9.80665 / 4 / 8.0, rel=0.05)


def test_paths_quad():
    r = run_cli("paths", "--airframe", QUAD)
    assert r.returncode == 0, r.stderr
    lines = r.stdout.splitlines()
    assert "rotors[0].tilt_deg = 0.0" in lines
    assert "mass.cg[0] = 0.0" in lines
    assert any(l.startswith("legs[3].length = ") for l in lines)


def test_export_quad(tmp_path):
    r = run_cli("export", "--airframe", QUAD)
    assert r.returncode == 0, r.stderr
    assert "# Airframe: Quad X" in r.stdout
    assert "1\t1\tCA_ROTOR_COUNT\t4\t6" in r.stdout
    assert "SYS_HITL" not in r.stdout
    out = tmp_path / "quad.params"
    r = run_cli("export", "--airframe", QUAD, "--hitl", "--out", str(out))
    assert r.returncode == 0 and r.stdout.strip() == str(out)
    text = out.read_text()
    assert "1\t1\tSYS_HITL\t1\t6" in text and "HIL_ACT_FUNC1\t101" in text


def test_export_without_hitl_maps_pwm_outputs():
    r = run_cli("export", "--airframe", QUAD)
    assert r.returncode == 0
    assert "1\t1\tPWM_MAIN_FUNC1\t101\t6" in r.stdout
    assert "HIL_ACT_FUNC1" not in r.stdout


def test_migrate_schema1(tmp_path):
    from .conftest import SCHEMA1_FIXTURE
    dst = tmp_path / "atlas.json"
    r = run_cli("migrate", str(SCHEMA1_FIXTURE), str(dst))
    assert r.returncode == 0, r.stderr
    assert "10 rotors, 1 wings, 4 legs" in r.stdout
    assert json.loads(dst.read_text())["schema"] == 2


def test_main_in_process(capsys):
    assert main(["scenarios"]) == 0
    assert "hover.json" in capsys.readouterr().out
    assert main(["paths", "--airframe", QUAD]) == 0
    assert "hover_pitch_deg = 0.0" in capsys.readouterr().out


def test_parse_set():
    assert _parse_set(["a.b=1", "c=2.5", "d=[1,2]", "e=text", "f=true"]) == {"a.b": 1, "c": 2.5, "d": [1, 2], "e": "text", "f": True}
    assert _parse_set(None) == {}
    with pytest.raises(SystemExit):
        _parse_set(["novalue"])


def test_unknown_command_fails():
    r = run_cli("frobnicate")
    assert r.returncode != 0
