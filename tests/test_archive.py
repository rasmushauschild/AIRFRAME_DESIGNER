"""Firmware archive: QGC parameter format round trip, versions written to a git repo and pushed to its origin."""
import json
import subprocess
import time
from pathlib import Path

from airframe_designer.px4 import archive as arch


def test_params_qgc_round_trip():
    params = {"MC_PITCH_P": {"value": 3.0, "type": 9}, "NLF_ENABLE": {"value": 1, "type": 6}, "SENS_BOARD_Y_OFF": {"value": 23.5, "type": 9}}
    text = arch.params_to_qgc(params)
    assert text.splitlines()[0].startswith("# Onboard parameters")
    back = arch.qgc_to_params(text)
    assert back["NLF_ENABLE"] == {"value": 1, "type": 6}
    assert back["MC_PITCH_P"]["value"] == 3.0 and back["SENS_BOARD_Y_OFF"]["value"] == 23.5


def test_snapshot_commits_and_pushes(tmp_path):
    origin = tmp_path / "origin.git"
    subprocess.run(["git", "init", "--bare", "-q", str(origin)], check=True)
    logs = []
    a = arch.FirmwareArchive(logs.append, url=str(origin), path=tmp_path / "clone")
    assert a.ensure(), a.error
    image = tmp_path / "fw.px4"; image.write_bytes(b"\x00\x01\x02")
    r = a.snapshot(kind="flash", board={"target": "px4_fmu-v6x"}, firmware_file=str(image),
                   board_firmware={"version": "1.18.0 beta"}, params_board={"NLF_ENABLE": {"value": 1, "type": 6}},
                   params_export={"CA_ROTOR_COUNT": 10}, airframe={"name": "ATLAS_09"}, atlas=True, note="first")
    assert r["ok"] and r["firmware_sha256"] and r["params_count"] == 1 and r["airframe"] == "ATLAS_09"
    v = a.get(r["id"])
    assert v["firmware_file"] and Path(v["firmware_file"]).read_bytes() == b"\x00\x01\x02"
    assert v["params_board"]["NLF_ENABLE"]["value"] == 1 and v["params_export"]["CA_ROTOR_COUNT"] == 10
    assert (Path(v["firmware_file"]).parent / "atlas" / "src").is_dir()          # module sources travel with the version
    assert a.versions()[0]["id"] == r["id"]
    for _ in range(100):                                                            # the push runs in the background
        if a.last_push is not None and not a.busy:
            break
        time.sleep(0.1)
    assert a.last_push_ok, a.error
    pushed = subprocess.run(["git", "--git-dir", str(origin), "log", "--oneline", "-1"], capture_output=True, text=True).stdout
    assert "flash: px4_fmu-v6x" in pushed and "first" in pushed
    assert a.get("../etc") is None
