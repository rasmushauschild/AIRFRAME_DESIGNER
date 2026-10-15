"""Geometry segment: frames, rotors, wings, legs, mass, the airframe container, migration and PX4 export."""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.geometry import (Airframe, Leg, MassItem, MassProperties, Rotor, Wing, axis_to_tilt_cant,
                                        generate_legs, migrate, tilt_cant_to_axis, unit, wing_panels)
from airframe_designer.geometry.airframe import G, quad_x

from .conftest import AIRFRAMES_DIR


def _approx_equal(a, b, tol=1e-9) -> bool:
    """Recursive comparison of JSON-like structures with a float tolerance."""
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(_approx_equal(a[k], b[k], tol) for k in a)
    if isinstance(a, (list, tuple)):
        return isinstance(b, (list, tuple)) and len(a) == len(b) and all(_approx_equal(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= tol
    return a == b


# ----------------------------------------------------------------- frames
@pytest.mark.parametrize("side", [1.0, -1.0])
@pytest.mark.parametrize("tilt,cant", [(0.0, 0.0), (25.0, 0.0), (0.0, 15.0), (30.0, 20.0), (-20.0, -10.0), (60.0, 5.0)])
def test_tilt_cant_axis_round_trip(tilt, cant, side):
    axis = tilt_cant_to_axis(tilt, cant, side)
    assert abs(np.linalg.norm(axis) - 1.0) < 1e-5
    t, c = axis_to_tilt_cant(axis, side)
    assert t == pytest.approx(tilt, abs=1e-3)
    assert c == pytest.approx(cant, abs=1e-3)


def test_cant_is_measured_outward():
    """Positive cant leans the axis away from the centreline: +y on the right side, -y on the left."""
    right = tilt_cant_to_axis(0.0, 20.0, side=1.0)
    left = tilt_cant_to_axis(0.0, 20.0, side=-1.0)
    assert right[1] > 0 and left[1] < 0
    assert right[1] == pytest.approx(-left[1])
    assert tilt_cant_to_axis(0.0, 0.0) == pytest.approx([0.0, 0.0, -1.0])
    assert tilt_cant_to_axis(90.0, 0.0) == pytest.approx([1.0, 0.0, 0.0], abs=1e-6)


def test_unit_fallback():
    assert unit([0.0, 0.0, 0.0]) == [0.0, 0.0, -1.0]
    assert unit([0.0, 3.0, 4.0]) == pytest.approx([0.0, 0.6, 0.8])


# ----------------------------------------------------------------- rotors
@pytest.mark.parametrize("y", [0.3, -0.3])
def test_rotor_virtual_tilt_cant_setters(y):
    r = Rotor(pos=[0.1, y, 0.0])
    assert r.tilt_deg == pytest.approx(0.0) and r.cant_deg == pytest.approx(0.0)
    r.tilt_deg = 30.0
    assert r.tilt_deg == pytest.approx(30.0, abs=1e-3)
    assert r.cant_deg == pytest.approx(0.0, abs=1e-3)
    assert r.axis[0] == pytest.approx(math.sin(math.radians(30.0)), abs=1e-5)
    r.cant_deg = 10.0
    assert r.cant_deg == pytest.approx(10.0, abs=1e-3)
    assert r.tilt_deg == pytest.approx(30.0, abs=1e-3)      # setting the cant keeps the tilt
    # outward: the y component has the sign of the rotor's side
    assert math.copysign(1.0, r.axis[1]) == math.copysign(1.0, y)
    assert abs(np.linalg.norm(r.axis) - 1.0) < 1e-5


def test_a_jetfoil_rotor_from_an_older_file_keeps_exactly_the_thrust_it_had():
    """Jetfoils are gone. A file that had one is migrated by folding the bend's thrust penalty into
    max_thrust, so the aircraft flies identically; the direction was already the jet's and does not move."""
    old = {"name": "M1", "pos": [0.1, 0.2, 0.0], "axis": [0.0, 0.0, -1.0], "km": 0.01,
           "max_thrust": 36.0, "kind": "ducted", "duct_axis": [1.0, 0.0, 0.0], "turn_loss": 0.1,
           "ram_drag": True, "intake_pos": [0.1, 0.2, 0.3], "diameter": 0.16}
    r = Rotor.from_dict(old)
    assert r.max_thrust == pytest.approx(36.0 * 0.9)      # a 90 degree bend cost ten per cent
    assert r.effective_max_thrust() == pytest.approx(36.0 * 0.9)
    assert r.axis == pytest.approx([0.0, 0.0, -1.0])      # the thrust still points where it did
    assert not hasattr(r, "duct_axis") and not hasattr(r, "kind")

    half = dict(old, axis=list(tilt_cant_to_axis(45.0, 0.0)))
    assert Rotor.from_dict(half).max_thrust == pytest.approx(36.0 * 0.95, rel=1e-3)

    straight = dict(old, duct_axis=None)
    assert Rotor.from_dict(straight).max_thrust == pytest.approx(36.0)


def test_a_disabled_rotor_produces_nothing():
    r = Rotor(max_thrust=36.0)
    assert r.effective_max_thrust() == 36.0
    r.enabled = False
    assert r.effective_max_thrust() == 0.0


def test_rotor_from_dict_accepts_schema1_prop_diameter():
    r = Rotor.from_dict({"pos": [0, 0, 0], "axis": [0, 0, -2], "prop_diameter": 0.3, "unknown_key": 1})
    assert r.diameter == 0.3
    assert r.axis == pytest.approx([0.0, 0.0, -1.0])


# ------------------------------------------------------------------ wings
def test_wing_area_ar_and_delta():
    w = Wing.delta(0.5, 1.075, pos_ac=[-0.2, 0.0, 0.06], incidence_deg=10.0)
    assert w.root_chord == pytest.approx(2 * 0.5 / 1.075)
    assert w.tip_chord == 0.0
    assert w.area == pytest.approx(0.5)
    assert w.ar == pytest.approx(1.075 ** 2 / 0.5)
    assert w.ar == pytest.approx(2.31, abs=0.01)
    assert w.aero.model == "polhamus"
    # the apex is 2/3 of the root chord ahead of the aerodynamic centre
    assert w.pos[0] == pytest.approx(-0.2 + 2.0 * w.root_chord / 3.0)
    assert w.sweep_deg == pytest.approx(math.degrees(math.atan2(w.root_chord, 1.075 / 2)))
    # a plain rectangular wing
    rect = Wing(span=2.0, root_chord=0.25, tip_chord=0.25)
    assert rect.area == pytest.approx(0.5) and rect.ar == pytest.approx(8.0)
    rect.aspect_ratio = 6.0
    assert rect.ar == 6.0


@pytest.mark.parametrize("wing", [
    Wing.delta(0.5, 1.075, [-0.2, 0, 0.06]),
    Wing(span=1.6, root_chord=0.28, tip_chord=0.18, sweep_deg=5, dihedral_deg=4, twist_deg=-2, panels=6),
    Wing(span=0.22, root_chord=0.16, tip_chord=0.08, dihedral_deg=90, symmetric=False, panels=3),
])
def test_wing_panels_areas_sum_to_wing_area(wing):
    p = wing_panels(wing)
    assert p["area"].sum() == pytest.approx(wing.area, rel=1e-9)
    n = wing.panels * (2 if wing.symmetric else 1)
    assert p["pos"].shape == (n, 3) and p["e_n"].shape == (n, 3)
    for key in ("e_c", "e_n", "e_s"):
        assert np.allclose(np.linalg.norm(p[key], axis=1), 1.0)


def test_symmetric_wing_has_panels_on_both_sides():
    p = wing_panels(Wing(span=1.0, panels=4))
    assert sorted(set(p["side"].tolist())) == [-1.0, 1.0]
    assert (p["pos"][p["side"] > 0][:, 1] > 0).all()
    assert (p["pos"][p["side"] < 0][:, 1] < 0).all()
    assert np.allclose(p["e_n"], [0.0, 0.0, -1.0])           # flat wing: lift straight up


def test_fin_spanwise_axis_points_up():
    fin = Wing(span=0.22, root_chord=0.16, tip_chord=0.08, dihedral_deg=90.0, symmetric=False, panels=3)
    p = wing_panels(fin)
    assert len(p["area"]) == 3
    assert (p["e_s"][:, 2] < -0.99).all()                     # z negative = up in FRD
    assert np.allclose(np.abs(p["e_n"][:, 1]), 1.0, atol=1e-9)  # lift sideways
    assert (p["pos"][:, 2] < fin.pos[2]).all()


def test_wing_dict_round_trip():
    w = Wing(name="w", span=1.2, dihedral_deg=3.0, panels=5)
    w.aero.cd0 = 0.03
    w2 = Wing.from_dict(w.to_dict())
    assert w2.to_dict() == w.to_dict()


# ------------------------------------------------------------------- legs
@pytest.mark.parametrize("attach,foot", [
    ([0.1, 0.1, 0.0], [0.15, 0.18, 0.2]),
    ([0.1, -0.1, 0.0], [0.15, -0.18, 0.2]),
    ([-0.2, 0.3, 0.05], [-0.25, 0.25, 0.4]),
    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.3]),
])
def test_leg_foot_from_points_round_trip(attach, foot):
    leg = Leg.from_points(attach, foot, name="x", stiffness=1234.0)
    assert leg.stiffness == 1234.0
    assert leg.length == pytest.approx(np.linalg.norm(np.array(foot) - np.array(attach)))
    assert leg.foot() == pytest.approx(foot, abs=1e-4)
    # a straight-down leg has zero angles
    if attach[0] == foot[0] and attach[1] == foot[1]:
        assert leg.tilt_deg == 0.0 and leg.cant_deg == 0.0


def test_leg_cant_is_outward():
    right = Leg(attach=[0, 0.1, 0], length=0.2, cant_deg=20.0)
    left = Leg(attach=[0, -0.1, 0], length=0.2, cant_deg=20.0)
    assert right.foot()[1] > 0.1 and left.foot()[1] < -0.1


def test_generate_legs_reproduces_schema1_feet(schema1_dict):
    d = schema1_dict
    legs = generate_legs(d["leg_height"], d["leg_spread"], d["leg_spread"], d["landed_pitch_deg"])
    assert [l.name for l in legs] == ["FR", "FL", "RR", "RL"]
    feet = [l.foot() for l in legs]
    assert np.allclose(feet, d["leg_points"], atol=1e-3)
    # the feet lie on a plane perpendicular to gravity when the airframe stands nose-down by landed_pitch
    phi = math.radians(d["landed_pitch_deg"])
    down = np.array([-math.sin(phi), 0.0, math.cos(phi)])
    heights = [np.dot(f, down) for f in feet]
    assert np.allclose(heights, d["leg_height"])


# ------------------------------------------------------------------- mass
def test_mass_from_items_two_point_masses():
    mp = MassProperties(items=[MassItem("a", 1.0, [1.0, 0.0, 0.0]), MassItem("b", 3.0, [-1.0, 0.0, 0.0])], from_items=True)
    mp.resolve()
    assert mp.mass == pytest.approx(4.0)
    assert mp.cg == pytest.approx([-0.5, 0.0, 0.0])
    # parallel axis: 1*(1.5)^2 + 3*(0.5)^2 = 3.0 about y and z, nothing about x
    assert mp.inertia == pytest.approx([0.0, 3.0, 3.0])
    assert mp.inertia_products == pytest.approx([0.0, 0.0, 0.0])
    T = mp.tensor()
    assert np.allclose(T, np.diag([0.0, 3.0, 3.0]))


def test_mass_from_items_with_offset_and_own_inertia():
    mp = MassProperties.from_dict({"from_items": True, "items": [
        {"name": "a", "mass": 2.0, "pos": [0.0, 1.0, 0.0], "inertia": [0.1, 0.2, 0.3]},
        {"name": "b", "mass": 2.0, "pos": [0.0, 0.0, 1.0]}]})
    assert mp.mass == 4.0 and mp.cg == pytest.approx([0.0, 0.5, 0.5])
    r_a, r_b = np.array([0.0, 0.5, -0.5]), np.array([0.0, -0.5, 0.5])
    I = np.diag([0.1, 0.2, 0.3])
    for m, r in ((2.0, r_a), (2.0, r_b)):
        I = I + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    assert np.allclose(mp.tensor(), I, atol=1e-6)


def test_mass_without_from_items_is_left_alone():
    mp = MassProperties(mass=2.0, items=[MassItem("a", 1.0, [1, 0, 0])], from_items=False).resolve()
    assert mp.mass == 2.0 and mp.cg == [0.0, 0.0, 0.0]


# --------------------------------------------------------------- airframe
def test_airframe_dict_round_trip(quad, plane):
    for af in (quad, plane):
        af.px4_overrides["MC_PITCH_P"] = 5.0
        af.design["cruise_speed_kmh"] = 40
        d = af.to_dict()
        af2 = Airframe.from_dict(d)
        assert _approx_equal(af2.to_dict(), d)
        assert af2.name == af.name and len(af2.rotors) == len(af.rotors) and len(af2.legs) == len(af.legs)
        assert af2.px4_overrides == {"MC_PITCH_P": 5.0} and af2.design["cruise_speed_kmh"] == 40
        assert af2.copy().to_dict() == af2.to_dict()


def test_airframe_load_examples():
    for p in sorted(AIRFRAMES_DIR.glob("*.json")):
        af = Airframe.load(p)
        assert af.schema == 2
        assert all(r.name for r in af.rotors)


def test_migrate_schema1_fixture(schema1_dict):
    d = migrate(schema1_dict)
    assert d["schema"] == 2
    af = Airframe.from_dict(schema1_dict)
    assert len(af.rotors) == 10
    assert len(af.wings) == 1 and af.wings[0].aero.model == "polhamus"
    assert af.wings[0].area == pytest.approx(0.5) and af.wings[0].span == 1.075
    assert af.wings[0].incidence_deg == 10.0
    assert len(af.legs) == 4
    assert af.mass.mass == 13.0 and af.mass.cg == [0.0, 0.0, 0.0]
    assert af.mass.inertia == pytest.approx(schema1_dict["inertia"])
    assert af.body.size == schema1_dict["body_size"]
    assert af.hover_pitch_deg == 25.0 and af.landed_pitch_deg == -20.0
    assert af.px4_overrides == schema1_dict["px4_overrides"]
    assert af.design["cruise_speed_kmh"] == 50
    assert [r.diameter for r in af.rotors] == [0.08] * 10
    assert [r.name for r in af.rotors] == [f"M{i + 1}" for i in range(10)]
    assert np.allclose([l.foot() for l in af.legs], schema1_dict["leg_points"], atol=1e-3)
    # migrating a schema-2 dict is the identity
    assert migrate(af.to_dict()) == af.to_dict()


def test_migrated_px4_rotor_geometry_matches_schema1(schema1_dict):
    """Schema-1 rotor positions were relative to the CG (cg = 0 after migration), so the exported
    CA_ROTOR geometry is those positions and axes rotated into the hover frame, exactly as before."""
    af = Airframe.from_dict(schema1_dict)
    R = af.hover_rotation()
    p = af.px4_params_sitl()
    assert p["CA_ROTOR_COUNT"] == 10
    assert p["SENS_BOARD_Y_OFF"] == 25.0
    for i, r in enumerate(schema1_dict["rotors"]):
        pos = R @ np.array(r["pos"], float)
        ax = R @ np.array(unit(r["axis"]), float)
        assert [p[f"CA_ROTOR{i}_PX"], p[f"CA_ROTOR{i}_PY"], p[f"CA_ROTOR{i}_PZ"]] == pytest.approx(pos.tolist(), abs=1e-4)
        assert [p[f"CA_ROTOR{i}_AX"], p[f"CA_ROTOR{i}_AY"], p[f"CA_ROTOR{i}_AZ"]] == pytest.approx(ax.tolist(), abs=1e-4)
        assert p[f"CA_ROTOR{i}_KM"] == pytest.approx(r["km"])
        assert af.rotors[i].pos == pytest.approx(r["pos"])


def test_validate_flags_marginal_thrust_and_missing_legs(quad):
    assert quad.validate() == []
    heavy = quad_x(); heavy.mass.mass = 3.0                 # 32 N / 29.4 N = 1.09 < 1.2
    assert any("thrust/weight" in p for p in heavy.validate())
    two_legs = quad_x(); two_legs.legs = two_legs.legs[:2]
    assert any("fewer than 3 legs" in p for p in two_legs.validate())
    three_legs = quad_x(); three_legs.legs[0].enabled = False
    assert not any("legs" in p for p in three_legs.validate())
    none = quad_x(); none.rotors = []
    assert any("1..12 rotors" in p for p in none.validate())


def test_hover_check_quad_x_ok(quad):
    h = quad.hover_check()
    assert h["ok"] is True and h["problems"] == [] and h["negative"] == []
    assert h["shares"] == pytest.approx([1.0] * 4)
    assert sum(h["hover_thrust"]) == pytest.approx(quad.mass.mass * G)
    assert max(h["hover_utilisation"]) < 0.85


def test_hover_check_same_spin_direction_cannot_cancel_yaw():
    af = quad_x()
    for r in af.rotors:
        r.km = 0.05
    h = af.hover_check()
    assert h["ok"] is False
    assert any("yaw" in p for p in h["problems"])
    assert abs(h["residual_torque"][2]) > 2e-3


def test_hover_check_negative_thrust_when_cg_outside_footprint():
    af = quad_x()
    af.mass.cg = [0.30, 0.0, 0.0]                           # ahead of the front rotors (x = 0.177)
    h = af.hover_check()
    assert h["ok"] is False
    assert h["negative"] == [2, 4]                          # the rear rotors (PX4 numbering)
    assert any("negative thrust" in p for p in h["problems"])


def test_px4_params_sitl_quad(quad):
    p = quad.px4_params_sitl()
    assert [p[f"PWM_MAIN_FUNC{n}"] for n in range(1, 5)] == [101, 102, 103, 104]
    assert all(p[f"PWM_MAIN_FUNC{n}"] == 0 for n in range(5, 17))
    assert not any(k.startswith("HIL_ACT_FUNC") for k in p)
    assert "SYS_HITL" not in p
    assert p["CA_AIRFRAME"] == 0 and p["CA_ROTOR_COUNT"] == 4 and p["SENS_BOARD_Y_OFF"] == 0.0
    a = quad.rotors[0].pos[0]
    assert p["CA_ROTOR0_PX"] == pytest.approx(a, abs=1e-4) and p["CA_ROTOR0_PY"] == pytest.approx(a, abs=1e-4)
    assert p["CA_ROTOR0_AZ"] == -1.0 and p["CA_ROTOR0_KM"] == 0.05 and p["CA_ROTOR2_KM"] == -0.05
    hitl = quad.px4_params(hitl=True)
    assert hitl["SYS_HITL"] == 1 and hitl["HIL_ACT_FUNC1"] == 101


def test_px4_params_shift_with_cg(quad):
    p0 = quad.px4_params_sitl()
    quad.mass.cg = [0.05, -0.02, 0.01]
    p1 = quad.px4_params_sitl()
    for i in range(4):
        assert p1[f"CA_ROTOR{i}_PX"] == pytest.approx(p0[f"CA_ROTOR{i}_PX"] - 0.05, abs=1e-4)
        assert p1[f"CA_ROTOR{i}_PY"] == pytest.approx(p0[f"CA_ROTOR{i}_PY"] + 0.02, abs=1e-4)
        assert p1[f"CA_ROTOR{i}_PZ"] == pytest.approx(p0[f"CA_ROTOR{i}_PZ"] - 0.01, abs=1e-4)
        assert p1[f"CA_ROTOR{i}_AZ"] == p0[f"CA_ROTOR{i}_AZ"]


def test_px4_params_hover_pitch_rotates_geometry(quad):
    quad.hover_pitch_deg = 30.0
    p = quad.px4_params_sitl()
    assert p["SENS_BOARD_Y_OFF"] == 30.0
    # a vertical structural axis is tilted back in the hover frame
    assert p["CA_ROTOR0_AX"] == pytest.approx(-math.sin(math.radians(30.0)), abs=1e-4)
    assert p["CA_ROTOR0_AZ"] == pytest.approx(-math.cos(math.radians(30.0)), abs=1e-4)


def test_px4_overrides_are_exported_and_geometry_wins(quad):
    quad.px4_overrides = {"MC_PITCH_P": 4.0, "MC_AIRMODE": True, "CA_ROTOR_COUNT": 99}
    p = quad.px4_params_sitl()
    assert p["MC_PITCH_P"] == 4.0 and p["MC_AIRMODE"] == 1
    assert p["CA_ROTOR_COUNT"] == 4
    assert "MC_PITCH_P" not in quad.geometry_param_names()
    text = quad.px4_params_file(hitl=False)
    assert "1\t1\tCA_ROTOR_COUNT\t4\t6" in text
    assert "1\t1\tMC_PITCH_P\t4.000000\t9" in text


def test_effectiveness_matrix_shape(quad):
    E = quad.effectiveness()
    assert E.shape == (6, 4)
    assert np.allclose(E[5], -1.0)                          # unit thrust straight up
    assert np.allclose(E[2], [0.05, 0.05, -0.05, -0.05])    # yaw torque follows km


def test_estimate_inertia_positive(quad, plane):
    for af in (quad, plane):
        I = af.estimate_inertia()
        assert len(I) == 3 and all(v > 0 for v in I)
        assert I[2] > I[0] and I[2] > I[1]                  # a flat vehicle: yaw inertia is the largest
