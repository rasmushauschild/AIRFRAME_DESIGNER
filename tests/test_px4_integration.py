"""Closed-loop run with a real PX4 SITL (instance 8 only; instances 0-4 belong to other processes).

Skipped automatically when ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 is missing. Deselect with -m "not px4"."""
from __future__ import annotations

import pytest

from .conftest import DATA_DIR

pytestmark = [pytest.mark.px4, pytest.mark.slow]

PX4_INSTANCE = 8


def test_hover_scenario_on_quad_x():
    from airframe_designer.batch.worker import run_once
    from airframe_designer.px4.sitl import instance_is_free

    if not instance_is_free(PX4_INSTANCE):
        pytest.skip(f"PX4 SITL instance {PX4_INSTANCE} is in use")
    result = run_once(str(DATA_DIR / "quad_x.json"), "hover", instance=PX4_INSTANCE, timeout_wall=240, quiet=True)
    assert result["ok"], {"status": result.get("status"), "failures": result.get("failures"), "log": result.get("log")}
    assert result["status"] == "done" and result["failures"] == []
    m = result["metrics"]
    assert m["crashed"] is False
    hover = m["phases"]["hover"]
    assert 2.0 < hover["alt_mean"] < 4.0                     # 3 m takeoff altitude above the ~0.12 m leg rest offset
    assert hover["airborne_fraction"] == 1.0
    assert hover["tilt_max_deg"] < 20.0
    assert hover["util_max"] < 1.0
    takeoff = m["phases"]["takeoff"]
    assert takeoff["alt_max"] > 2.5 and takeoff["airborne_fraction"] > 0.5
    assert any("takeoff altitude reached" in e["text"] for e in m["events"])
    # NB: takeoff["time_to_alt"] is not asserted: summary() drops it (see test_sim.test_time_to_alt_reaches_the_result_summary)
    verified = result["px4_params_verified"]
    assert verified, "no seeded parameters were read back"
    assert all(v["ok"] for v in verified.values()), verified
    assert verified["CA_ROTOR_COUNT"]["vehicle"] == 4
    assert result["timing"]["instance"] == PX4_INSTANCE and result["timing"]["sim_s"] > 10.0
    assert result["airframe"]["name"] == "Quad X"
