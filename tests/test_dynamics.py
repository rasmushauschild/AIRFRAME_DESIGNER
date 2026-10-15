"""The flight dynamics, on the one engine there is, plus the quaternion helpers.

These were written against the project's own Python rigid body, which no longer exists: JSBSim is the only
engine now. They have been kept and pointed at JSBSim rather than deleted, because they are the evidence that
the engine does what physics says it should, and that evidence is the point of the simulator.

Every case here is checked against something outside the code: analytic free fall, the weight a hovering
aircraft has to carry, an accelerometer's definition, the resting height the landing gear geometry implies.
Not against a previous run.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.dynamics import G, LegContacts, q_deriv, q_from_euler, q_normalize, q_to_euler, q_to_rotmat
from airframe_designer.dynamics.jsbsim_backend import JSBSimBody
from airframe_designer.geometry.airframe import quad_x

DT = 0.002


def _run(rb, seconds: float, dt: float = DT) -> None:
    for _ in range(int(round(seconds / dt))):
        rb.step(dt)


def body(af) -> JSBSimBody:
    return JSBSimBody(af)


# ------------------------------------------------------------ quaternions
@pytest.mark.parametrize("euler", [(0.0, 0.0, 0.0), (0.3, -0.2, 1.1), (-1.0, 0.4, -2.5), (0.0, 1.2, 0.0)])
def test_euler_quaternion_round_trip(euler):
    assert q_to_euler(q_from_euler(*euler)) == pytest.approx(euler, abs=1e-9)


def test_rotmat_convention_body_to_ned():
    """Nose 90 degrees to the east: the body x axis must come out pointing east in world axes."""
    R = q_to_rotmat(q_from_euler(0.0, 0.0, math.pi / 2))
    assert R @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, 1.0, 0.0], abs=1e-9)
    assert R @ R.T == pytest.approx(np.eye(3), abs=1e-12)


def test_q_deriv_and_normalize():
    q = q_from_euler(0.1, 0.2, 0.3)
    rates = np.array([0.4, -0.2, 0.7])
    dt = 1e-5
    moved = q_normalize(q + q_deriv(q, rates) * dt)
    # the body x axis must swing the way the rates say, to first order
    axis_before = q_to_rotmat(q) @ np.array([1.0, 0.0, 0.0])
    axis_after = q_to_rotmat(moved) @ np.array([1.0, 0.0, 0.0])
    expected = axis_before + np.cross(q_to_rotmat(q) @ rates, axis_before) * dt
    assert axis_after == pytest.approx(expected, abs=1e-9)
    assert np.linalg.norm(moved) == pytest.approx(1.0, abs=1e-12)


# ----------------------------------------------------------- resting on its legs
def test_rest_on_legs(quad):
    rb = body(quad)
    rest = rb.legs.rest_height(rb.rotmat)
    assert rest == pytest.approx(0.12)
    assert rb.pos[2] == pytest.approx(-rest, abs=0.02)
    _run(rb, 1.0)
    assert rb.on_ground is True and rb.feet_down >= 1
    assert rb.pos[2] == pytest.approx(-rest, abs=0.03)    # settled on its springs, within a few centimetres
    assert np.linalg.norm(rb.vel) < 0.02
    assert np.linalg.norm(rb.rates) < 0.05
    assert abs(rb.pos[0]) < 0.02 and abs(rb.pos[1]) < 0.02
    assert rb.tilt_deg == pytest.approx(0.0, abs=0.5)


def test_accel_body_is_minus_g_at_rest(quad):
    """An accelerometer measures specific force. Parked and level that is one g upward, which is -z in FRD."""
    rb = body(quad)
    assert rb.accel_body == pytest.approx([0.0, 0.0, -G])
    _run(rb, 1.0)
    assert rb.accel_body == pytest.approx([0.0, 0.0, -G], abs=0.4)


# ----------------------------------------------------------- flight
def test_hover_command_holds_position(quad):
    rb = body(quad)
    rb.reset(pos_ned=[0.0, 0.0, -2.0])
    assert rb.pos[2] == pytest.approx(-2.0, abs=0.01)
    w = math.sqrt(rb.mass * G / (4 * 8.0))                # thrust = tmax * omega^2 per rotor, four rotors
    rb.set_motor_commands([w] * 4)
    rb.omega[:] = w                                        # start spooled, so this measures the balance not the lag
    _run(rb, 2.0)
    assert rb.on_ground is False and rb.feet_down == 0
    assert abs(rb.pos[2] + 2.0) < 0.15                     # still at two metres
    assert np.linalg.norm(rb.vel) < 0.25
    assert rb.tilt_deg < 2.0
    assert rb.breakdown["thrust"] == pytest.approx(rb.mass * G, rel=1e-6)
    assert rb.breakdown["power"] > 0
    assert rb.t == pytest.approx(2.0)


def test_free_fall_from_height():
    """With the drag removed, a dropped aircraft follows v = g t and s = g t^2 / 2, and reads zero on its
    accelerometer. This is the check that gravity is applied once and only once."""
    af = quad_x()
    af.body.drag_quadratic = [0.0, 0.0, 0.0]
    rb = body(af)
    rb.reset(pos_ned=[0.0, 0.0, -10.0])
    _run(rb, 0.5)
    assert rb.vel[2] == pytest.approx(G * 0.5, rel=2e-3)
    assert rb.pos[2] == pytest.approx(-10.0 + 0.5 * G * 0.25, abs=0.02)
    assert rb.on_ground is False
    assert rb.accel_body == pytest.approx([0.0, 0.0, 0.0], abs=0.05)
    # and with the preset body drag the same fall is measurably slower
    rb2 = body(quad_x())
    rb2.reset(pos_ned=[0.0, 0.0, -10.0])
    _run(rb2, 0.5)
    assert 0.8 * G * 0.5 < rb2.vel[2] < G * 0.5


def test_thrust_above_weight_climbs_and_below_it_sinks(quad):
    rb = body(quad)
    hover = math.sqrt(rb.mass * G / (4 * 8.0))
    for factor, expect_up in ((1.15, True), (0.85, False)):
        rb.reset(pos_ned=[0.0, 0.0, -5.0])
        rb.set_motor_commands([hover * factor] * 4)
        rb.omega[:] = hover * factor
        _run(rb, 1.0)
        climbing = bool(rb.vel[2] < 0)                      # world z is down
        assert climbing is expect_up, f"at {factor:.2f} of the hover command the vertical motion is wrong"


def test_tilt_deg(quad):
    rb = body(quad)
    assert rb.tilt_deg == pytest.approx(0.0, abs=1e-6)
    rb.q = q_from_euler(math.pi / 2, 0.0, 0.0)
    assert rb.tilt_deg == pytest.approx(90.0)
    rb.q = q_from_euler(0.0, -math.pi / 6, 0.0)
    assert rb.tilt_deg == pytest.approx(30.0)
    rb.q = q_from_euler(0.0, 0.0, 1.0)                     # yaw alone is not a tilt
    assert rb.tilt_deg == pytest.approx(0.0, abs=1e-9)


def test_landed_pitch_sets_initial_attitude_and_rest_height():
    af = quad_x()
    af.legs[2].length = af.legs[3].length = 0.30           # taller rear legs: the aircraft sits nose down
    af.resolve_mass()
    rb = body(af)
    pitch = math.degrees(q_to_euler(rb.q)[1])
    assert pitch == pytest.approx(af.landed_pitch_deg, abs=1.0)
    assert rb.pos[2] == pytest.approx(-rb.legs.rest_height(rb.rotmat), abs=0.05)


def test_hover_frame_euler():
    af = quad_x()
    af.hover_pitch_deg = 20.0
    rb = body(af)
    rb.q = q_from_euler(0.0, math.radians(20.0), 0.0)
    roll, pitch, _ = rb.hover_frame_euler()
    assert pitch == pytest.approx(0.0, abs=1e-6)           # at the hover pitch the hover frame reads level
    assert roll == pytest.approx(0.0, abs=1e-6)


def test_set_rotor_health_scales_thrust(quad):
    rb = body(quad)
    rb.set_motor_commands([0.5] * 4)
    rb.omega[:] = 0.5
    rb.step(DT)
    full = rb.breakdown["thrust"]
    rb.set_rotor_health([0.0, 1.0, 1.0, 1.0])              # motor 1 dead
    rb.step(DT)
    assert rb.breakdown["thrust"] == pytest.approx(full * 0.75, rel=1e-6)


def test_motor_commands_clipped_and_spool(quad):
    rb = body(quad)
    rb.reset(pos_ned=[0, 0, -5.0])
    rb.set_motor_commands([2.0, -1.0, 0.5, 0.5])
    assert rb.cmd == pytest.approx([1.0, 0.0, 0.5, 0.5])
    assert rb.omega == pytest.approx([0.0, 0.0, 0.0, 0.0])  # the spool has not started
    _run(rb, 5 * rb.rotors.tau.max())
    assert rb.omega[0] == pytest.approx(1.0, abs=0.02)      # five time constants: there
    assert rb.omega[1] == pytest.approx(0.0, abs=1e-9)


def test_set_airframe_rebuilds_for_the_new_geometry(quad):
    """JSBSim regenerates its model, so the vehicle restarts on the ground. That is the contract, not a bug."""
    rb = body(quad)
    af = quad_x()
    af.rotors = af.rotors[:3]
    af.resolve_mass()
    rb.set_airframe(af)
    rb.reset()
    assert len(rb.cmd) == 3 and len(rb.omega) == 3
    _run(rb, 0.2)                                          # JSBSim reports contact once it has stepped
    assert rb.on_ground is True


# ----------------------------------------------------------- landing gear geometry
def test_leg_contacts_geometry():
    """LegContacts is kept for the foot geometry that JSBSim's contacts and the nose-lift sequence are built
    from. The contact forces themselves are JSBSim's now; this is the geometry the model is generated from."""
    from airframe_designer.geometry import Leg
    legs = [Leg(attach=[0.1, 0.1, 0], length=0.2), Leg(attach=[-0.1, -0.1, 0], length=0.2, foot_radius=0.05)]
    lc = LegContacts(legs, [0.0, 0.0, 0.0])
    assert lc.rest_height(np.eye(3)) == pytest.approx(0.25)   # the foot ball raises the rest height
    assert lc.n == 2
    assert lc.r.shape == (2, 3)
