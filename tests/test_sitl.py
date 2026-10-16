"""PX4 SITL helpers that need no running PX4: the BSON parameter file, instances and the model lookup."""
from __future__ import annotations

import os
import struct
from pathlib import Path

import pytest

from airframe_designer.px4.sitl import (PX4Instance, autostart_id, encode_param_bson, find_px4_dir, free_px4_instance,
                                        instance_is_free, param_types_from_meta, write_param_file)

from .conftest import PX4_BINARY


def decode_bson(data: bytes) -> dict:
    """A tiny decoder for PX4's BSON subset: int32 (0x10) and double (0x01) elements."""
    total = struct.unpack_from("<i", data, 0)[0]
    assert total == len(data), "length prefix must cover the whole document"
    assert data[-1] == 0, "document must end with a 0 byte"
    out = {}
    i = 4
    while i < len(data) - 1:
        t = data[i]; i += 1
        end = data.index(b"\x00", i)
        name = data[i:end].decode(); i = end + 1
        if t == 0x10:
            out[name] = ("int32", struct.unpack_from("<i", data, i)[0]); i += 4
        elif t == 0x01:
            out[name] = ("double", struct.unpack_from("<d", data, i)[0]); i += 8
        else:
            raise AssertionError(f"unexpected element type {t:#x}")
    assert i == len(data) - 1
    return out


def test_encode_param_bson_round_trip():
    params = {"CA_ROTOR_COUNT": 4, "CA_ROTOR0_PX": 0.1768, "MC_AIRMODE": True, "SYS_AUTOSTART": 10016, "MPC_THR_HOVER": 0.5,
              "FLOAT_AS_INT_META": 3.0, "INT_AS_FLOAT_META": 7}
    data = encode_param_bson(params, types={"FLOAT_AS_INT_META": "Int32", "INT_AS_FLOAT_META": "Float"})
    d = decode_bson(data)
    assert list(d) == sorted(params)                         # written in sorted order
    assert d["CA_ROTOR_COUNT"] == ("int32", 4)
    assert d["SYS_AUTOSTART"] == ("int32", 10016)
    assert d["MC_AIRMODE"] == ("int32", 1)
    assert d["CA_ROTOR0_PX"] == ("double", 0.1768)
    assert d["MPC_THR_HOVER"] == ("double", 0.5)
    assert d["FLOAT_AS_INT_META"] == ("int32", 3)            # metadata type wins over the Python type
    assert d["INT_AS_FLOAT_META"] == ("double", 7.0)
    # int_names as a fallback when no metadata is given
    d2 = decode_bson(encode_param_bson({"X": 2.6}, int_names={"X"}))
    assert d2["X"] == ("int32", 3)
    assert decode_bson(encode_param_bson({})) == {}


def test_write_param_file(tmp_path):
    p = write_param_file(tmp_path / "inst", {"A_INT": 1, "B_FLT": 2.5})
    assert p == tmp_path / "inst" / "fs" / "parameters.bson"
    backup = tmp_path / "inst" / "fs" / "parameters_backup.bson"
    assert p.is_file() and backup.is_file()
    assert p.read_bytes() == backup.read_bytes()
    assert decode_bson(p.read_bytes()) == {"A_INT": ("int32", 1), "B_FLT": ("double", 2.5)}


def test_param_types_from_meta():
    assert param_types_from_meta({"A": {"type": "Int32"}, "B": {"type": "Float"}, "C": {}}) == {"A": "Int32", "B": "Float", "C": "Float"}


def test_px4_instance_ports_and_workdir(tmp_path):
    inst = PX4Instance(8, tmp_path / "wd")
    assert inst.sim_port == 4568 and inst.ctl_port == 14548
    assert inst.tcp_address == "0.0.0.0:4568" and inst.ctl_address == "udpin:127.0.0.1:14548"
    assert inst.workdir.is_dir()
    (inst.workdir / "stale").write_text("x")
    PX4Instance(8, tmp_path / "wd", fresh=True)
    assert not (tmp_path / "wd" / "stale").exists()
    with pytest.raises(RuntimeError, match="spaces"):
        PX4Instance(8, tmp_path / "with space")


def test_free_px4_instance():
    i = free_px4_instance(start=1)
    assert isinstance(i, int) and 1 <= i <= 9
    assert instance_is_free(i)
    with pytest.raises(RuntimeError):
        free_px4_instance(start=10, limit=10)


def test_find_px4_dir_falls_back_to_default(monkeypatch, tmp_path):
    monkeypatch.delenv("PX4_DIR", raising=False)
    assert find_px4_dir(str(tmp_path)) in (str(tmp_path), os.path.expanduser("~/PX4-Autopilot"))
    assert find_px4_dir(None).endswith("PX4-Autopilot")


@pytest.mark.skipif(not PX4_BINARY.is_file(), reason="PX4 SITL build not present")
def test_autostart_id_none_iris():
    px4_dir = str(PX4_BINARY.parents[3])
    assert autostart_id(px4_dir, "none_iris") == 10016
    assert autostart_id(px4_dir, "no_such_model_xyz") is None


def test_autostart_id_without_build(tmp_path):
    assert autostart_id(str(tmp_path), "none_iris") in (None, 10016)   # falls through to the default dir if it exists
    d = tmp_path / "build" / "px4_sitl_default" / "etc" / "init.d-posix" / "airframes"
    d.mkdir(parents=True)
    (d / "4242_test_model").write_text("")
    (tmp_path / "build" / "px4_sitl_default" / "bin").mkdir()
    (tmp_path / "build" / "px4_sitl_default" / "bin" / "px4").write_text("")
    assert autostart_id(str(tmp_path), "test_model") == 4242


def test_a_dead_px4_socket_is_cleared_and_a_live_one_is_not():
    """PX4 refuses to start while /tmp/px4-sock-<i> exists, and a killed PX4 leaves one behind: one crashed
    run would block every later run on that instance. Only a socket nobody answers on is removed."""
    import shutil
    import socket as pysocket
    import tempfile
    from pathlib import Path
    from airframe_designer.px4.sitl import clear_stale_socket

    # a short directory: a unix socket path is limited to about 100 characters, and pytest's tmp_path is longer
    tmp = Path(tempfile.mkdtemp(dir="/tmp"))
    live = tmp / "live.sock"
    server = pysocket.socket(pysocket.AF_UNIX, pysocket.SOCK_STREAM)
    server.bind(str(live))
    server.listen(16)                              # it answers every probe, as a running PX4 does
    dead = tmp / "dead.sock"
    bound = pysocket.socket(pysocket.AF_UNIX, pysocket.SOCK_STREAM)
    bound.bind(str(dead))
    bound.close()                                  # the file stays, nobody is listening: what a crash leaves
    try:
        assert clear_stale_socket(1, str(live), wait_s=0.2) is False and live.exists()   # a live one is left alone
        assert clear_stale_socket(2, str(dead)) is True and not dead.exists()
        assert clear_stale_socket(3, str(tmp / "never-existed.sock")) is False
    finally:
        server.close()
        shutil.rmtree(tmp, ignore_errors=True)
