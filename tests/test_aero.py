"""Force models: section coefficients, strip-theory wings, rotor thrust/torque/ram drag, body drag."""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.aero import BodyAero, RHO, RotorSet, WingSet, section_coefficients
from airframe_designer.aero.wing_aero import helmbold_slope
from airframe_designer.geometry import Body, Rotor, Wing
from airframe_designer.geometry.airframe import plane_quad

LINEAR = dict(model="linear", a3d=4.5, cl0=0.0, cd0=0.02, oswald=0.85, ar=8.0, stall=math.radians(15.0),
              blend=math.radians(15.0), cd_flat=1.2, kv=0.0)
POLHAMUS = dict(model="polhamus", a3d=2.5, cl0=0.0, cd0=0.02, oswald=0.85, ar=2.3, stall=math.radians(30.0),
                blend=math.radians(15.0), cd_flat=1.2, kv=math.pi)


def test_linear_section_below_stall():
    alpha = np.radians(np.linspace(-14.0, 14.0, 29))
    cl, cd = section_coefficients(alpha, **LINEAR)
    assert np.all(np.diff(cl) > 0)                                   # lift grows with alpha
    assert np.allclose(cl, LINEAR["a3d"] * alpha)
    assert np.all(cd >= LINEAR["cd0"] - 1e-12)
    assert cd[14] == pytest.approx(LINEAR["cd0"])                     # alpha = 0: parasite drag only
    assert np.allclose(cd, LINEAR["cd0"] + cl ** 2 / (math.pi * 0.85 * 8.0))


def test_linear_section_blends_to_flat_plate():
    cl, cd = section_coefficients(np.array([math.pi / 2]), **LINEAR)
    assert cl[0] == pytest.approx(0.0, abs=1e-9)
    assert cd[0] == pytest.approx(LINEAR["cd0"] + LINEAR["cd_flat"])
    # halfway through the blend the coefficients sit between the frozen pre-stall value and the flat plate
    a = LINEAR["stall"] + 0.5 * LINEAR["blend"]
    cl_mid, _ = section_coefficients(np.array([a]), **LINEAR)
    cl_stall = LINEAR["a3d"] * LINEAR["stall"]
    cl_flat = LINEAR["cd_flat"] * math.sin(a) * math.cos(a)
    assert cl_mid[0] == pytest.approx(0.5 * cl_stall + 0.5 * cl_flat)
    # continuous at the stall angle
    lo, _ = section_coefficients(np.array([LINEAR["stall"] - 1e-6]), **LINEAR)
    hi, _ = section_coefficients(np.array([LINEAR["stall"] + 1e-6]), **LINEAR)
    assert lo[0] == pytest.approx(hi[0], abs=1e-4)


def test_polhamus_symmetric_in_alpha():
    a = np.radians(np.array([2.0, 10.0, 20.0, 28.0, 40.0, 70.0]))
    cl_p, cd_p = section_coefficients(a, **POLHAMUS)
    cl_n, cd_n = section_coefficients(-a, **POLHAMUS)
    assert np.allclose(cl_p, -cl_n) and np.allclose(cd_p, cd_n)
    assert np.all(cl_p[:4] > 0) and np.all(np.diff(cl_p[:4]) > 0)
    assert np.all(cd_p >= POLHAMUS["cd0"])
    # the vortex-lift term adds lift
    cl_nv, _ = section_coefficients(a, **{**POLHAMUS, "kv": 0.0})
    assert np.all(cl_p[:4] > cl_nv[:4])
    cl90, _ = section_coefficients(np.array([math.pi / 2]), **POLHAMUS)
    assert cl90[0] == pytest.approx(0.0, abs=1e-9)


def test_helmbold_slope():
    two_pi = 2 * math.pi
    slopes = [helmbold_slope(two_pi, ar) for ar in (1.0, 2.0, 4.0, 8.0, 16.0, 100.0)]
    assert all(s < two_pi for s in slopes)
    assert all(b > a for a, b in zip(slopes, slopes[1:]))
    assert slopes[-1] == pytest.approx(two_pi, rel=0.05)
    assert helmbold_slope(two_pi, 0.0) == 0.0


# ------------------------------------------------------------------ wings
@pytest.fixture
def plane_wings():
    af = plane_quad()
    return af, WingSet(af.active_wings(), af.cg)


def test_wingset_forward_flight_lift_and_drag(plane_wings):
    af, ws = plane_wings
    assert ws.n == 6 * 2 + 3 * 2 + 3
    F, M, bd = ws.forces(np.array([15.0, 0.0, 0.0]), np.zeros(3))
    assert F[2] < 0                                   # lift up (z down in FRD)
    assert F[0] < 0                                   # drag backwards
    assert abs(F[1]) < 1e-9                           # symmetric: no side force
    assert bd["lift"] > 0 and bd["drag"] > 0 and bd["lift"] > 5 * bd["drag"]
    assert -F[2] == pytest.approx(bd["lift"], rel=0.05)
    assert len(bd["alpha"]) == 3 and bd["stalled"] is False
    assert abs(M[0]) < 1e-9 and abs(M[2]) < 1e-9     # no roll or yaw moment in symmetric flight
    # lift scales with dynamic pressure
    F2, _, _ = ws.forces(np.array([30.0, 0.0, 0.0]), np.zeros(3))
    assert F2[2] == pytest.approx(4 * F[2], rel=0.02)


def test_wingset_sideslip_dihedral_and_fin(plane_wings):
    af, ws = plane_wings
    F, M, _ = ws.forces(np.array([15.0, 3.0, 0.0]), np.zeros(3))
    assert M[0] < 0                                   # dihedral effect: +y sideslip rolls the vehicle left
    assert M[2] > 0                                   # weathercock: the fin yaws the nose into the wind
    assert F[1] < 0                                   # the fin pushes sideways against the sideslip
    F2, M2, _ = ws.forces(np.array([15.0, -3.0, 0.0]), np.zeros(3))
    assert M2[0] == pytest.approx(-M[0], rel=1e-6) and M2[2] == pytest.approx(-M[2], rel=1e-6)


def test_wingset_without_fin_has_no_weathercock():
    af = plane_quad()
    wings = [w for w in af.active_wings() if w.name != "fin"]
    ws = WingSet(wings, af.cg)
    _, M, _ = ws.forces(np.array([15.0, 3.0, 0.0]), np.zeros(3))
    assert M[0] < 0
    assert abs(M[2]) < abs(M[0]) * 0.05


def test_wingset_roll_and_pitch_damping(plane_wings):
    af, ws = plane_wings
    _, M, _ = ws.forces(np.array([15.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    assert M[0] < 0                                   # roll rate -> opposing roll moment
    _, M, _ = ws.forces(np.array([15.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0]))
    assert M[0] > 0
    _, M0, _ = ws.forces(np.array([15.0, 0.0, 0.0]), np.zeros(3))
    _, Mq, _ = ws.forces(np.array([15.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))
    assert Mq[1] < M0[1]                              # nose-up rate -> extra nose-down moment from the tailplane


def test_wingset_stall_flag_and_empty_set():
    af = plane_quad()
    ws = WingSet(af.active_wings(), af.cg)
    _, _, bd = ws.forces(np.array([5.0, 0.0, 4.0]), np.zeros(3))    # ~39 degrees angle of attack
    assert bd["stalled"] is True
    _, _, bd = ws.forces(np.zeros(3), np.zeros(3))
    assert bd["lift"] == 0.0 and bd["drag"] == 0.0
    empty = WingSet([], af.cg)
    F, M, bd = empty.forces(np.array([10.0, 0.0, 0.0]), np.zeros(3))
    assert np.all(F == 0) and np.all(M == 0) and bd["alpha"] == []


def test_wingset_mixed_models():
    delta = Wing.delta(0.5, 1.075, [-0.2, 0.0, 0.0], incidence_deg=5.0)
    tail = Wing(name="tail", pos=[-0.7, 0, 0], span=0.5, root_chord=0.12, tip_chord=0.12, panels=2)
    ws = WingSet([delta, tail], [0.0, 0.0, 0.0])
    assert ws.mixed
    F, _, bd = ws.forces(np.array([15.0, 0.0, 0.0]), np.zeros(3))
    assert F[2] < 0 and bd["lift"] > 0
    only_delta = WingSet([delta], [0.0, 0.0, 0.0])
    Fd, _, _ = only_delta.forces(np.array([15.0, 0.0, 0.0]), np.zeros(3))
    assert not only_delta.mixed and Fd[2] < 0


# ----------------------------------------------------------------- rotors
def test_rotorset_thrust_curve():
    rotors = [Rotor(pos=[0, 0, 0], max_thrust=8.0, thrust_exponent=2.0), Rotor(pos=[0, 0, 0], max_thrust=10.0, thrust_exponent=1.5)]
    rs = RotorSet(rotors, [0, 0, 0])
    assert rs.thrust(np.array([1.0, 1.0])) == pytest.approx([8.0, 10.0])
    assert rs.thrust(np.array([0.5, 0.5])) == pytest.approx([8.0 * 0.25, 10.0 * 0.5 ** 1.5])
    assert rs.thrust(np.array([1.5, -1.0])) == pytest.approx([8.0, 0.0])      # clipped to [0, 1]
    assert rs.thrust(np.zeros(2)) == pytest.approx([0.0, 0.0])


@pytest.mark.parametrize("km", [0.05, -0.05])
def test_rotorset_reaction_torque_sign_follows_km(km):
    rs = RotorSet([Rotor(pos=[0, 0, 0], km=km, max_thrust=8.0)], [0, 0, 0])
    F, M, thrust, ram = rs.forces(np.array([1.0]), np.zeros(3), np.zeros(3))
    assert F == pytest.approx([0.0, 0.0, -8.0])
    assert thrust == pytest.approx([8.0]) and ram == 0.0
    # PX4 convention: torque on the body = -km * thrust * axis; with axis (0,0,-1) the yaw torque has km's sign
    assert M[2] == pytest.approx(km * 8.0)
    assert abs(M[0]) < 1e-12 and abs(M[1]) < 1e-12


def test_rotorset_thrust_moment_about_cg():
    rs = RotorSet([Rotor(pos=[0.2, 0.0, 0.0], km=0.0, max_thrust=8.0)], [0.0, 0.0, 0.0])
    _, M, _, _ = rs.forces(np.array([1.0]), np.zeros(3), np.zeros(3))
    assert M[1] == pytest.approx(0.2 * 8.0)           # front rotor lifting: nose-up (positive pitch) moment
    rs2 = RotorSet([Rotor(pos=[0.2, 0.0, 0.0], km=0.0, max_thrust=8.0)], [0.2, 0.0, 0.0])
    _, M2, _, _ = rs2.forces(np.array([1.0]), np.zeros(3), np.zeros(3))
    assert np.allclose(M2, 0.0)                       # the CG under the rotor: no moment


def test_rotorset_ram_drag_is_momentum_drag_on_every_rotor():
    """Every rotor is a ducted jet, so every rotor pays momentum drag for the air it turns. The magnitude is
    the mass flow through the disc times the crossflow, and the direction opposes the airflow exactly."""
    duct = Rotor(pos=[0, 0, 0], max_thrust=36.0, diameter=0.08)
    v = np.array([12.0, -3.0, 0.0])
    F, _, _, ram = RotorSet([duct], [0, 0, 0]).forces(np.array([1.0]), v, np.zeros(3))
    assert ram > 0
    drag = F - np.array([0.0, 0.0, -36.0])
    assert np.dot(drag, v) < 0                          # opposes the airspeed
    assert abs(np.cross(drag, v)).max() < 1e-9          # and is parallel to it
    mdot = math.sqrt(RHO * duct.disc_area * 36.0)
    assert np.linalg.norm(drag) == pytest.approx(mdot * np.linalg.norm(v))

    # a bigger disc at the same thrust moves more air more slowly, so it drags more in crossflow
    big = Rotor(pos=[0, 0, 0], max_thrust=36.0, diameter=0.25)
    _, _, _, ram_big = RotorSet([big], [0, 0, 0]).forces(np.array([1.0]), v, np.zeros(3))
    assert ram_big > ram

    # no thrust, no mass flow, no ram drag
    _, _, _, ram0 = RotorSet([duct], [0, 0, 0]).forces(np.array([0.0]), v, np.zeros(3))
    assert ram0 == 0.0


def test_rotorset_health_scale_and_power():
    rotors = [Rotor(pos=[0, 0, 0], max_thrust=8.0), Rotor(pos=[0, 0, 0], max_thrust=8.0, diameter=0.12)]
    rs = RotorSet(rotors, [0, 0, 0])
    rs.scale = np.array([0.5, 1.0])
    assert rs.thrust(np.array([1.0, 1.0])) == pytest.approx([4.0, 8.0])
    p = rs.ideal_power(np.array([4.0, 8.0]))
    assert p > 0
    # both are ducted jets, so both use the duct's induced power: the slipstream keeps the disc's area
    expected = sum(t ** 1.5 / (2 * math.sqrt(RHO * r.disc_area)) for t, r in zip((4.0, 8.0), rotors))
    assert p == pytest.approx(expected)
    assert rs.ideal_power(np.zeros(2)) == 0.0
    empty = RotorSet([], [0, 0, 0])
    F, M, T, ram = empty.forces(np.zeros(0), np.zeros(3), np.zeros(3))
    assert np.all(F == 0) and len(T) == 0


# ------------------------------------------------------------------- body
def test_body_drag_opposes_velocity_and_damps_rotation():
    body = Body(drag_quadratic=[0.1, 0.2, 0.3], drag_angular=[0.01, 0.01, 0.01], drag_center=[0.1, 0.0, 0.0])
    ba = BodyAero(body, [0.0, 0.0, 0.0])
    v = np.array([10.0, -5.0, 2.0])
    F, M, mag = ba.forces(v, np.zeros(3))
    assert F == pytest.approx([-0.1 * 100, 0.2 * 25, -0.3 * 4])
    assert mag == pytest.approx(np.linalg.norm(F))
    assert M[2] == pytest.approx(0.1 * F[1])           # side force ahead of the CG yaws the body
    _, M, _ = ba.forces(np.zeros(3), np.array([0.0, 0.0, 2.0]))
    assert M[2] == pytest.approx(-0.04 + 0.1 * (-0.2 * 0.2 ** 2))   # angular damping + the drag of the swept centre
    centred = BodyAero(Body(drag_angular=[0.01, 0.02, 0.03]), [0.0, 0.0, 0.0])
    _, M, _ = centred.forces(np.zeros(3), np.array([1.0, -2.0, 3.0]))
    assert M == pytest.approx([-0.01, 0.08, -0.27])
