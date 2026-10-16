import math

import numpy as np
import pytest

from airframe_designer.geometry.wings import Wing, wing_panels, wing_outline

def test_mirrored_joined_panels_preserve_area_and_orientation():
    w = Wing(span=1.2, root_chord=1, tip_chord=.4, dihedral_deg=-15)
    before = wing_panels(w)
    w.root_y = .5
    after = wing_panels(Wing.from_dict(w.to_dict()))
    delta = after["pos"] - before["pos"]
    np.testing.assert_allclose(delta[:, 1], before["side"] * .5)
    np.testing.assert_allclose(delta[:, [0,2]], 0)
    for key in ("area", "e_c", "e_n", "chord"):
        np.testing.assert_allclose(after[key], before[key])
    outlines = wing_outline(w)
    assert outlines[0][0][1] == .5
    assert outlines[1][0][1] == -.5
    assert np.all(after["e_n"][:,2] < 0)


# ---------------------------------------------------------------- a panel is flat
def test_a_panel_is_flat_and_keeps_its_area():
    """One sweep, one dihedral, chord straight from root to tip: the strips add up to the planform."""
    w = Wing(span=2.0, root_chord=0.4, tip_chord=0.2, sweep_deg=10.0, dihedral_deg=20.0, panels=5)
    p = wing_panels(w)
    assert p["area"].sum() == pytest.approx(w.area, rel=1e-12)
    tip = np.array(wing_outline(w)[0][1])                         # right tip leading edge
    assert tip[0] == pytest.approx(-math.tan(math.radians(10)) * math.cos(0), abs=1e-9)
    assert tip[1] == pytest.approx(math.cos(math.radians(20)), abs=1e-9)
    assert tip[2] == pytest.approx(-math.sin(math.radians(20)), abs=1e-9)
    for e_n, e_s in zip(p["e_n"], p["e_s"]):                      # lift is perpendicular to the span, everywhere
        assert abs(float(np.dot(e_n, e_s))) < 1e-12
        assert np.linalg.norm(e_n) == pytest.approx(1.0, abs=1e-12)
    assert np.all(p["e_n"][:, 2] < 0)                             # and points up


def test_the_root_offset_moves_both_halves_outward_not_sideways():
    """What the Y box on a symmetric wing has to do: push the two roots apart, keeping the pair centred.

    Putting that number in `pos` instead slides the whole wing sideways — both halves the same way — and
    the aircraft is no longer symmetric, which is what it looked like in the 3D view.
    """
    w = Wing(span=2.0, root_chord=0.4, tip_chord=0.4, root_y=0.5, panels=3)
    out = wing_outline(w)
    right_root, left_root = np.array(out[0][0]), np.array(out[1][0])
    assert right_root[1] == pytest.approx(0.5) and left_root[1] == pytest.approx(-0.5)
    assert (right_root[1] + left_root[1]) == pytest.approx(0.0, abs=1e-12)      # centred

    p = wing_panels(w)
    assert p["pos"][:, 1].sum() == pytest.approx(0.0, abs=1e-12)                # every strip has its mirror

    slid = Wing(span=2.0, root_chord=0.4, tip_chord=0.4, pos=[0.0, 0.5, 0.0], panels=3)
    o2 = wing_outline(slid)
    assert np.array(o2[0][0])[1] == pytest.approx(0.5) and np.array(o2[1][0])[1] == pytest.approx(0.5)
    assert wing_panels(slid)["pos"][:, 1].sum() != pytest.approx(0.0, abs=1e-6)  # off centre, as pos means
