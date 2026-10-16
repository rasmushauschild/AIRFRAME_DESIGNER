"""A rotor bolted on crooked: the simulation flies the error, PX4 never hears about it.

This is the one place in the project where the aircraft and the model of it are deliberately different, so
the tests here are mostly about what must *not* change: the export, the design's own tilt and cant, and the
file when there is no error to record.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from airframe_designer.aero.rotor_aero import RotorSet
from airframe_designer.geometry.airframe import quad_x
from airframe_designer.geometry.propulsion import Rotor

ANGLE = lambda a, b: math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))   # noqa: E731


def test_the_error_turns_the_axis_by_exactly_that_many_degrees():
    r = Rotor(pos=[0.4, 0.3, 0.0], axis=[0, 0, -1])
    assert not r.mis_mounted and r.built_axis == pytest.approx(r.axis)
    r.tilt_error_deg = 10.0
    assert ANGLE(r.built_axis, r.axis) == pytest.approx(10.0, abs=1e-3)
    assert r.built_tilt_cant[0] == pytest.approx(r.tilt_deg + 10.0, abs=1e-3)
    assert r.axis == [0.0, 0.0, -1.0], "the design was not touched"

    r.tilt_error_deg, r.cant_error_deg = 0.0, -6.0
    assert ANGLE(r.built_axis, r.axis) == pytest.approx(6.0, abs=1e-3)
    assert r.built_tilt_cant[1] == pytest.approx(r.cant_deg - 6.0, abs=1e-3)


def test_cant_error_leans_outward_on_whichever_side_the_rotor_sits():
    """Cant is measured outward, so +5 degrees of error pushes the two sides of a pair apart, not both left."""
    left = Rotor(pos=[0.0, -0.3, 0.0], axis=[0, 0, -1], cant_error_deg=5.0)
    right = Rotor(pos=[0.0, 0.3, 0.0], axis=[0, 0, -1], cant_error_deg=5.0)
    assert left.built_axis[1] < 0 < right.built_axis[1]
    assert left.built_axis[1] == pytest.approx(-right.built_axis[1])


def test_px4_is_told_the_design_and_nothing_else():
    clean = quad_x()
    built = quad_x()
    built.rotors[1].tilt_error_deg = 10.0
    built.rotors[2].cant_error_deg = -7.5
    assert built.px4_params() == clean.px4_params()
    assert built.px4_params_sitl() == clean.px4_params_sitl()
    assert built.rotors_in_px4_frame() == clean.rotors_in_px4_frame()
    assert [r.tilt_deg for r in built.rotors] == [r.tilt_deg for r in clean.rotors]


def test_the_forces_come_from_the_axis_the_aircraft_has():
    """One rotor, straight down, canted 10 degrees by a bad mount: the thrust leans with it."""
    straight = RotorSet([Rotor(pos=[0.0, 0.2, 0.0], axis=[0, 0, -1], km=0.0, max_thrust=10.0)], [0, 0, 0])
    crooked = RotorSet([Rotor(pos=[0.0, 0.2, 0.0], axis=[0, 0, -1], km=0.0, max_thrust=10.0,
                              cant_error_deg=10.0)], [0, 0, 0])
    full, still, rates = np.array([1.0]), np.zeros(3), np.zeros(3)
    F0 = straight.forces(full, still, rates)[0]
    F1 = crooked.forces(full, still, rates)[0]
    assert F0[1] == pytest.approx(0.0, abs=1e-9)
    assert F1[1] == pytest.approx(10.0 * math.sin(math.radians(10.0)), abs=1e-3)
    assert np.linalg.norm(F1) == pytest.approx(np.linalg.norm(F0), rel=1e-6), "it leans, it does not grow"
    # and the moment it makes about the CG is the one the crooked rotor really makes
    assert crooked.forces(full, still, rates)[1][0] != pytest.approx(straight.forces(full, still, rates)[1][0])


def test_jsbsim_is_given_the_crooked_direction():
    from airframe_designer.dynamics import jsbsim_backend
    af = quad_x()
    af.rotors[0].tilt_error_deg = 12.0
    mdir = jsbsim_backend.generate_model(af, "mounting_error_test")
    xml = (mdir / "mounting_error_test.xml").read_text()
    want = af.rotors[0].built_axis
    line = next(l for l in xml.splitlines() if 'name="rotor0"' in l)
    assert f"<x>{want[0]:.6g}" in line or f"<x>{round(want[0], 6)}" in line, line
    assert abs(want[0] - math.sin(math.radians(12.0))) < 1e-5


def test_a_rotor_mounted_as_designed_says_nothing_about_it():
    assert "tilt_error_deg" not in Rotor().to_dict()
    d = Rotor(tilt_error_deg=10.0).to_dict()
    assert d["tilt_error_deg"] == 10.0 and "cant_error_deg" not in d
    assert Rotor.from_dict(d).tilt_error_deg == 10.0
    assert Rotor.from_dict(Rotor().to_dict()).tilt_error_deg == 0.0


def test_the_airframe_says_so_out_loud():
    af = quad_x()
    assert not any("built off its design" in p for p in af.validate())
    af.rotors[2].tilt_error_deg = 10.0
    said = [p for p in af.validate() if "built off its design" in p]
    assert len(said) == 1 and "10° tilt" in said[0]
    assert (af.rotors[2].name or "M3") in said[0]


def test_the_margin_is_where_the_hover_actually_gives_out():
    """The number the app shows has to mean something: just inside it the aircraft still hovers, just
    outside it the trim is gone. Checked against the trim itself, not against a remembered figure."""
    af = quad_x()
    m = af.mount_margin()
    row = next(r for r in m["rotors"] if r["name"] == "M1")
    lo, hi = row["tilt_wall"]
    assert -45 <= lo < 0 < hi <= 45
    for edge in (lo, hi):
        if abs(edge) >= 45:
            continue                              # nothing broke inside the range the search looks at
        af.rotors[0].tilt_error_deg = edge * 0.97
        assert af.hover_trim_as_built()["ok"], f"{edge}° was called fine and is not"
        af.rotors[0].tilt_error_deg = edge * 1.10
        assert not af.hover_trim_as_built()["ok"], f"{edge}° was called the wall and is not"
    af.rotors[0].tilt_error_deg = 0.0
    assert row["tilt"][1] <= row["tilt_wall"][1] and row["tilt"][0] >= row["tilt_wall"][0], \
        "the margin must sit inside the wall"
    assert row["why"]


def test_asking_for_the_margin_does_not_touch_the_aircraft():
    af = quad_x()
    af.rotors[1].cant_error_deg = 6.0
    before = [r.to_dict() for r in af.rotors]
    af.mount_margin()
    assert [r.to_dict() for r in af.rotors] == before


def test_a_side_force_is_a_lean_and_a_torque_is_a_failure():
    """One rotor leaning takes the whole aircraft with it; that is allowed, up to a point, and the point
    is reported as a lean rather than as something mysterious."""
    af = quad_x()
    trim = af.hover_trim_as_built()
    assert trim["ok"] and trim["lean_deg"] == pytest.approx(0.0, abs=1e-6)
    for r in af.rotors:                            # every rotor leaning the same way: pure side force
        r.tilt_error_deg = 30.0
    tilted = af.hover_trim_as_built()
    assert not tilted["ok"] and "lean" in tilted["why"]
    assert tilted["lean_deg"] == pytest.approx(30.0, abs=0.5)
    assert tilted["residual_torque"] == pytest.approx(0.0, abs=1e-6), "nothing is out of balance, it just leans"


def test_the_browser_and_the_physics_mount_a_rotor_the_same_way(data_dir=None):
    """Half of this feature is drawn in the 3D view, so ui/scene_math.js has its own copy of the arithmetic.
    Both sides are pinned to tests/data/rotor_axes.json; tests/test_scene_math.mjs reads the other end."""
    import json
    from pathlib import Path
    cases = json.loads((Path(__file__).parent / "data" / "rotor_axes.json").read_text())
    for c in cases:
        r = Rotor(**c["rotor"])
        assert r.built_axis == pytest.approx(c["built_axis"], abs=1e-9)
        assert list(r.built_tilt_cant) == pytest.approx(c["built_tilt_cant"], abs=1e-6)
