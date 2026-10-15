"""Static analysis (hover allocation, cruise trim) and the geometric optimiser."""
from __future__ import annotations

import time

import pytest

from airframe_designer.analysis import VehicleModel, analyse, default_groups, optimise
from airframe_designer.geometry.airframe import G, quad_x

CRUISE_KEYS = {"ok", "converged", "problems", "airspeed", "pitch_deg", "px4_pitch_deg", "collective", "pitch_torque", "thrust",
               "util", "max_util", "total_thrust", "power", "lift", "lift_share", "wing_drag", "ram_drag", "body_drag",
               "alpha_deg", "saturated", "negative", "residual", "power_ratio"}
HOVER_KEYS = {"ok", "problems", "thrust", "util", "max_util", "negative", "power", "total_thrust", "waste", "authority",
              "residual_force", "residual_yaw"}


def test_analyse_quad_x(quad):
    r = analyse(quad)
    assert set(r) == {"hover", "cruise", "airspeed"}
    assert r["airspeed"] == pytest.approx(50.0 / 3.6)          # default cruise speed 50 km/h
    h = r["hover"]
    assert set(h) == HOVER_KEYS
    assert h["ok"] is True and h["problems"] == [] and h["negative"] == []
    assert h["max_util"] < 1.0 and h["max_util"] == pytest.approx(quad.mass.mass * G / 4 / 8.0)
    assert h["total_thrust"] == pytest.approx(quad.mass.mass * G) and h["waste"] == pytest.approx(0.0, abs=1e-9)
    assert h["power"] > 0 and set(h["authority"]) == {"roll", "pitch", "yaw"} and all(v > 0 for v in h["authority"].values())
    # 50 km/h is past what this aircraft can do: every rotor is a ducted jet, and the momentum drag of four
    # 250 mm ducts at that speed is larger than the whole weight. The trim says so rather than inventing one.
    c = r["cruise"]
    assert set(c) == CRUISE_KEYS
    assert c["converged"] is False and c["ok"] is False and c["problems"]
    assert c["ram_drag"] > 0

    # at a speed it can actually hold, everything trims and the readouts are consistent
    slow = analyse(quad, airspeed=5.0)["cruise"]
    assert slow["converged"] is True and slow["ok"] is True
    assert slow["airspeed"] == 5.0
    assert slow["pitch_deg"] < 0 and slow["px4_pitch_deg"] == slow["pitch_deg"]   # a plain quad noses down to cruise
    assert slow["lift"] == 0.0 and slow["lift_share"] == 0.0 and slow["alpha_deg"] is None
    assert slow["body_drag"] > 0 and slow["ram_drag"] > 0
    assert slow["power_ratio"] == pytest.approx(slow["power"] / h["power"])
    assert slow["power"] > h["power"]                            # tilting to push against drag costs power
    assert len(slow["thrust"]) == 4 and slow["saturated"] == [] and slow["negative"] == []


def test_analyse_explicit_speed_and_tilt_limit(quad):
    r = analyse(quad, airspeed=30.0, tilt_limit_deg=10.0)
    assert r["airspeed"] == 30.0
    assert r["cruise"]["airspeed"] == 30.0
    assert any("exceeds MPC_TILTMAX_AIR" in p for p in r["cruise"]["problems"]) and r["cruise"]["ok"] is False


def test_analyse_plane_quad_reports_wing_and_infeasible_cruise(plane):
    """The wing quad has vertical rotors only, so at 18 m/s the drag has to be met by tilting the whole
    aircraft, and with ducted rotors the momentum drag rises with the tilt faster than the thrust can answer.
    No trim exists. The analysis reports that instead of pretending, which is the point of the check."""
    r = analyse(plane, airspeed=18.0)
    c = r["cruise"]
    assert c["alpha_deg"] is not None and c["wing_drag"] > 0 and c["lift"] != 0.0
    assert c["lift_share"] == pytest.approx(c["lift"] / (plane.mass.mass * G))
    assert c["ok"] is False and c["problems"]
    # at a gentle speed the same vehicle trims cleanly
    slow = analyse(plane, airspeed=5.0)["cruise"]
    assert slow["converged"] and slow["saturated"] == []


def test_vehicle_model_hover_without_rotors():
    af = quad_x(); af.rotors = []
    h = VehicleModel(af).hover()
    assert h["ok"] is False and h["problems"] == ["no rotors"]
    assert VehicleModel(af).cruise(10.0)["ok"] is False


def test_vehicle_model_hover_flags_bad_geometry():
    af = quad_x(); af.mass.cg = [0.3, 0.0, 0.0]
    h = VehicleModel(af).hover()
    assert h["ok"] is False and h["negative"] == [2, 4]
    af = quad_x()
    for r in af.rotors:
        r.km = 0.05
    assert "no yaw authority" in VehicleModel(af).hover()["problems"]


def test_analyse_atlas08(atlas08):
    r = analyse(atlas08)
    assert r["hover"]["ok"] and r["hover"]["max_util"] < 1.0
    c = r["cruise"]
    assert c["converged"] and c["ok"]
    assert c["ram_drag"] > 0                                      # ducted fans
    assert c["alpha_deg"] is not None and c["wing_drag"] > 0
    assert c["total_thrust"] > atlas08.mass.mass * G * 0.9
    assert c["power_ratio"] > 1.0
    assert abs(c["px4_pitch_deg"]) <= 45.0


def test_default_groups(atlas08, quad):
    g = default_groups(quad)
    assert list(g) == ["A"] and sorted(g["A"]["rotors"]) == [0, 1, 2, 3]
    g = default_groups(atlas08)
    assert len(g) >= 2
    assert sorted(i for grp in g.values() for i in grp["rotors"]) == list(range(len(atlas08.rotors)))
    # groups are named front to back
    fronts = [max(atlas08.rotors[i].pos[0] for i in grp["rotors"]) for grp in g.values()]
    assert fronts == sorted(fronts, reverse=True)


def test_geometric_optimiser_one_variable(atlas08):
    t0 = time.perf_counter()
    msgs = []
    r = optimise(atlas08, {"variables": [{"path": "hover_pitch_deg", "range": [0, 30]}], "samples": 20, "refine": 1, "seed": 1},
                 progress=lambda f, m: msgs.append((f, m)))
    dt = time.perf_counter() - t0
    assert dt < 5.0, f"optimiser took {dt:.1f}s"
    assert r["ok"] is True
    assert r["variables"] == [{"path": "hover_pitch_deg", "kind": "", "group": "", "lo": 0.0, "hi": 30.0}]
    assert r["evaluated"] >= 34                                   # current + 20 random + 13 grid points
    assert r["results"] and len(r["results"]) <= 8
    assert msgs[-1] == (1.0, "done")
    best = r["results"][0]
    assert set(best) >= {"x", "values", "score", "pareto", "hover", "cruise", "hover_pitch_deg", "axes", "airframe"}
    assert 0.0 <= best["x"][0] <= 30.0
    assert best["values"] == {"hover_pitch_deg": best["x"][0]}
    assert best["hover_pitch_deg"] == best["x"][0] and best["airframe"]["hover_pitch_deg"] == best["x"][0]
    scores = [m["score"] for m in r["results"]]
    assert scores[:6] == sorted(scores[:6])
    assert r["current"]["values"] == {"hover_pitch_deg": atlas08.hover_pitch_deg}
    assert r["feasible"] >= 1
    assert atlas08.hover_pitch_deg == 15.0                        # the input airframe is untouched


def test_geometric_optimiser_legacy_groups_and_no_variables(quad):
    assert optimise(quad, {})["ok"] is False
    r = optimise(quad, {"groups": {"A": {"rotors": [0, 1, 2, 3], "tilt": [0, 20]}}, "hover_pitch": [0, 20], "samples": 5, "refine": 0})
    assert r["ok"]
    assert [v["path"] for v in r["variables"]] == ["rotors[0,1,2,3].tilt_deg", "hover_pitch_deg"]
    assert [v["kind"] for v in r["variables"]] == ["tilt", "hover_pitch"]
