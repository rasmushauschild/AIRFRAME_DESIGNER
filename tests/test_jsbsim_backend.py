"""JSBSim backend: same airframe, same interface, same rest state and hover response as the Python body."""
import math

import numpy as np
import pytest

from airframe_designer.geometry.airframe import Airframe, quad_x
from airframe_designer.dynamics import RigidBody

jsbsim = pytest.importorskip("jsbsim")
from airframe_designer.dynamics.jsbsim_backend import JSBSimBody, generate_model  # noqa: E402


def test_generated_model_loads_and_rests_like_python(tmp_path):
    af = quad_x()
    py = RigidBody(af); py.reset()
    jb = JSBSimBody(af); jb.reset()
    for _ in range(1500):
        py.step(0.001, False); jb.step(0.001, False)
    assert jb.on_ground and jb.feet_down >= 1
    assert abs(py.pos[2] - jb.pos[2]) < 0.01
    assert abs(math.degrees(py.euler[1]) - math.degrees(jb.euler[1])) < 0.2
    assert np.allclose(jb.accel_body, [0, 0, -9.80665], atol=0.05)


def test_hover_thrust_climb_matches_python():
    af = quad_x()
    hc = af.hover_check(); ht = np.array(hc["hover_thrust"]); tmax = np.array([r.effective_max_thrust() for r in af.active_rotors()])
    cmd = np.sqrt(np.clip(ht / tmax, 0, 1)) * 1.03
    py = RigidBody(af); py.reset(); jb = JSBSimBody(af); jb.reset()
    for _ in range(500):
        py.step(0.001, False); jb.step(0.001, False)
    py.set_motor_commands(cmd); jb.set_motor_commands(cmd)
    for k in range(2000):
        py.step(0.001, k % 4 == 3); jb.step(0.001, k % 4 == 3)
    assert not py.on_ground and not jb.on_ground
    assert abs(py.pos[2] - jb.pos[2]) < 0.05                 # same climb within 5 cm after 2 s
    assert abs(py.thrust.sum() - jb.thrust.sum()) < 1e-6      # identical rotor model
    assert np.abs(np.degrees(jb.euler)).max() < 0.5


def test_soft_legs_settle_to_spring_equilibrium():
    """The Python body used to hover above its spring equilibrium on soft legs (a damping hack); JSBSim exposed it."""
    af = Airframe.load("airframes/atlas_mvp_01.json")
    py = RigidBody(af); py.reset(); jb = JSBSimBody(af); jb.reset()
    for _ in range(4000):
        py.step(0.001, False); jb.step(0.001, False)
    assert abs(py.pos[2] - jb.pos[2]) < 0.02
    assert abs(math.degrees(py.euler[1]) - math.degrees(jb.euler[1])) < 0.3


@pytest.mark.parametrize("vn,ve", [(5., 3.), (-5., 3.), (5., -3.), (-5., -3.)])
def test_signed_horizontal_position_agrees_with_integrated_velocity(vn, ve):
    from airframe_designer.dynamics.jsbsim_backend import FT
    jb = JSBSimBody(quad_x())
    f = jb.fdm
    f["ic/h-agl-ft"] = 100 * FT
    f["ic/u-fps"] = vn * FT
    f["ic/v-fps"] = ve * FT
    f["ic/w-fps"] = 0.
    f.run_ic()
    jb._read_state()
    origin = jb.pos[:2].copy()
    integrated = np.zeros(2)
    for _ in range(500):
        before = jb.vel[:2].copy()
        jb.step(.001, False)
        integrated += (before + jb.vel[:2]) * .0005
    displacement = jb.pos[:2] - origin
    assert np.sign(displacement[0]) == np.sign(vn)
    assert np.sign(displacement[1]) == np.sign(ve)
    np.testing.assert_allclose(displacement, integrated, atol=.01)
