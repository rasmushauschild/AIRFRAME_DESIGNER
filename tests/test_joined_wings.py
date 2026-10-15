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
