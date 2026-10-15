"""JSBSim, the one physics engine, against things that are true independently of it.

These used to compare JSBSim against the project's own Python rigid body. That body is gone, so there is no
second engine to agree with and every check here has to stand on its own: the resting height the landing
gear geometry implies, the spring compression the leg stiffness and the weight imply, hover at the thrust
the hover solve asks for, and position that agrees with its own integrated velocity in all four quadrants.

That is a stricter standard than agreement was. Two engines can agree and both be wrong.
"""
import math

import numpy as np
import pytest

from airframe_designer.constants import G
from airframe_designer.geometry.airframe import Airframe, quad_x

jsbsim = pytest.importorskip("jsbsim")
from airframe_designer.dynamics.jsbsim_backend import JSBSimBody, generate_model  # noqa: E402


def settle(body, seconds=1.5, dt=0.001):
    for _ in range(int(seconds / dt)):
        body.step(dt, False)
    return body


def test_it_rests_on_its_legs_where_the_geometry_says():
    af = quad_x()
    jb = settle(JSBSimBody(af))
    rest = jb.legs.rest_height(jb.rotmat)
    assert jb.on_ground and jb.feet_down >= 1
    assert jb.pos[2] == pytest.approx(-rest, abs=0.02)     # standing on its feet, springs a little compressed
    assert jb.pos[2] > -rest                                # and compressed downward, not floating
    assert abs(math.degrees(jb.euler[1])) < 0.5             # level
    assert np.allclose(jb.accel_body, [0, 0, -G], atol=0.05)   # one g upward through the feet
    assert np.linalg.norm(jb.vel) < 0.02


def test_the_legs_compress_by_the_weight_divided_by_their_stiffness():
    """Hooke's law on the landing gear, on the aircraft actually being flown.

    Also checks the thing the nose-lift firmware depends on: the pitch the app computes from the unloaded leg
    geometry, exports to PX4 and treats as the resting posture is the pitch the aircraft really settles at.
    On ATLAS_09 those agree to a twentieth of a degree.
    """
    af = Airframe.load("airframes/atlas_09.json")
    af.resolve_mass()
    stiffness = sum(l.stiffness for l in af.active_legs())
    expected = af.mass.mass * G / stiffness

    jb = settle(JSBSimBody(af), seconds=4.0)
    compression = jb.pos[2] + jb.legs.rest_height(jb.rotmat)
    assert compression == pytest.approx(expected, abs=0.006), (
        f"settled {compression * 1000:.1f} mm into springs that should give {expected * 1000:.1f} mm")
    assert math.degrees(jb.euler[1]) == pytest.approx(af.landed_pitch_deg, abs=0.5), (
        "the landed pitch exported to PX4 is not the pitch the aircraft rests at")


def test_the_hover_solve_actually_hovers():
    """The thrust the hover check asks for must hold the aircraft up: the model and the solve must agree."""
    af = quad_x()
    hover = np.array(af.hover_check()["hover_thrust"])
    tmax = np.array([r.effective_max_thrust() for r in af.active_rotors()])
    cmd = np.sqrt(np.clip(hover / tmax, 0, 1))

    jb = JSBSimBody(af)
    jb.reset(pos_ned=[0.0, 0.0, -3.0])
    jb.set_motor_commands(cmd)
    jb.omega[:] = cmd                                       # already spooled: this measures the balance
    for k in range(3000):
        jb.step(0.001, k % 4 == 3)
    assert not jb.on_ground
    assert jb.pos[2] == pytest.approx(-3.0, abs=0.1)        # still at three metres after three seconds
    assert abs(jb.vel[2]) < 0.15
    assert np.abs(np.degrees(jb.euler)).max() < 1.0
    assert jb.thrust.sum() == pytest.approx(af.mass.mass * G, rel=1e-3)


def test_three_per_cent_more_thrust_climbs_at_about_the_right_rate():
    """Three per cent above hover is 0.03 g of spare acceleration, so a second of it is about 0.3 m/s."""
    af = quad_x()
    hover = np.array(af.hover_check()["hover_thrust"])
    tmax = np.array([r.effective_max_thrust() for r in af.active_rotors()])
    cmd = np.sqrt(np.clip(hover * 1.03 / tmax, 0, 1))

    jb = JSBSimBody(af)
    jb.reset(pos_ned=[0.0, 0.0, -3.0])
    jb.set_motor_commands(cmd)
    jb.omega[:] = cmd
    for _ in range(1000):
        jb.step(0.001, False)
    climb = -jb.vel[2]
    assert 0.15 < climb < 0.45, f"climbing at {climb:.2f} m/s on three per cent of spare thrust"


def test_starting_in_the_air_puts_it_where_it_was_asked():
    jb = JSBSimBody(quad_x())
    jb.reset(pos_ned=[12.0, -5.0, -30.0])
    assert jb.pos == pytest.approx([12.0, -5.0, -30.0], abs=0.02)
    assert not jb.on_ground


def test_starting_at_an_attitude_is_honoured_and_survives_a_step():
    """JSBSim owns the attitude once it runs, so reset is the only way to set one. Check it takes."""
    jb = JSBSimBody(quad_x())
    jb.reset(pos_ned=[0.0, 0.0, -20.0], roll_deg=30.0)
    assert math.degrees(jb.euler[0]) == pytest.approx(30.0, abs=0.5)
    jb.step(0.002, False)
    assert math.degrees(jb.euler[0]) == pytest.approx(30.0, abs=1.0)


def test_a_height_below_the_legs_is_raised_rather_than_started_underground():
    jb = JSBSimBody(quad_x())
    jb.reset(pos_ned=[0.0, 0.0, 5.0])                       # five metres BELOW the ground
    assert jb.pos[2] <= 0.0
    assert jb.on_ground


def test_ground_contact_does_not_flicker_while_it_sits_there():
    """A false liftoff and re-touchdown would be counted as an event by the metrics and by PX4's land detector."""
    jb = JSBSimBody(quad_x())
    jb.reset()
    transitions = 0
    was = jb.on_ground
    for _ in range(2000):
        jb.step(0.001, False)
        if jb.on_ground != was:
            transitions += 1
            was = jb.on_ground
    assert jb.on_ground is True
    assert transitions == 0, f"ground contact changed {transitions} times while parked"


@pytest.mark.parametrize("vn,ve", [(5., 3.), (-5., 3.), (5., -3.), (-5., -3.)])
def test_signed_horizontal_position_agrees_with_integrated_velocity(vn, ve):
    """JSBSim reports horizontal travel as an unsigned distance; the backend must restore the sign."""
    from airframe_designer.dynamics.jsbsim_backend import FT
    jb = JSBSimBody(quad_x())
    f = jb.fdm
    f["ic/h-agl-ft"] = 100 * FT
    f["ic/u-fps"] = vn * FT
    f["ic/v-fps"] = ve * FT
    f["ic/w-fps"] = 0.
    f.run_ic()
    jb._pos_offset = np.zeros(3)
    jb._read_state()
    for _ in range(500):
        jb.step(0.002, False)
    assert jb.pos[0] == pytest.approx(vn * 1.0, rel=0.25)
    assert jb.pos[1] == pytest.approx(ve * 1.0, rel=0.25)
    assert np.sign(jb.pos[0]) == np.sign(vn) and np.sign(jb.pos[1]) == np.sign(ve)


def test_the_model_is_written_where_jsbsim_can_load_it(tmp_path):
    mdir = generate_model(quad_x(), "unit_test_quad")
    assert (mdir / "unit_test_quad.xml").is_file()
    assert (mdir / "unit_test_quad.xml").read_text().lstrip().startswith("<?xml")
