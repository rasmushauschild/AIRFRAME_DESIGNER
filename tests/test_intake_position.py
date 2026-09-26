import numpy as np

from airframe_designer.geometry.propulsion import Rotor
from airframe_designer.aero.rotor_aero import RotorSet


def test_intake_roundtrip_and_legacy():
    r = Rotor.from_dict({'kind': 'ducted', 'pos': [1, 2, 3]})
    assert r.intake_position() == r.pos
    r.intake_pos = [4, 5, 6]
    assert Rotor.from_dict(r.to_dict()).intake_position() == [4, 5, 6]


def test_intake_changes_ram_moment_without_moving_thrust():
    r = Rotor(kind='ducted', ram_drag=True, pos=[0, 0, 0], km=0)
    before = RotorSet([r], [0, 0, 0])
    r.intake_pos = [0, 0, 1]
    after = RotorSet([r], [0, 0, 0])
    omega, velocity, rates = np.ones(1), np.array([10., 0, 0]), np.zeros(3)
    f0, m0, t0, _ = before.forces(omega, velocity, rates)
    f1, m1, t1, _ = after.forces(omega, velocity, rates)
    np.testing.assert_allclose(f0, f1)
    np.testing.assert_allclose(t0, t1)
    np.testing.assert_allclose(m1 - m0, np.cross([0, 0, 1], [f1[0], 0, 0]))
    assert m1[1] < 0
    for model in (before, after):
        _, moment, _, _ = model.forces(omega, np.zeros(3), rates)
        np.testing.assert_allclose(moment, 0)


def test_explicit_colocated_intake_preserves_legacy_forces_for_random_states():
    r = Rotor(kind='ducted', ram_drag=True, pos=[.3, -.4, .2])
    legacy = RotorSet([r], [.1, .1, -.1])
    r.intake_pos = r.pos.copy()
    explicit = RotorSet([r], [.1, .1, -.1])
    rng = np.random.default_rng(23)
    for _ in range(50):
        args = (rng.random(1), rng.normal(size=3)*10, rng.normal(size=3))
        for before, after in zip(legacy.forces(*args), explicit.forces(*args)):
            np.testing.assert_allclose(before, after)
