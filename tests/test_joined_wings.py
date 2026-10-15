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


# ---------------------------------------------------------------- wings that curve along their span
def test_a_wing_with_one_dihedral_is_still_a_flat_panel():
    """The new parameters are optional: left unset, every number comes out as it did before."""
    from airframe_designer.geometry.wings import Wing, wing_outline, wing_panels
    flat = Wing(span=2.0, root_chord=0.4, tip_chord=0.2, sweep_deg=10.0, dihedral_deg=20.0, panels=5)
    same = Wing(span=2.0, root_chord=0.4, tip_chord=0.2, sweep_deg=10.0, dihedral_deg=20.0,
                sweep_tip_deg=10.0, dihedral_tip_deg=20.0, panels=5)
    for k in ("pos", "e_c", "e_n", "e_s", "area", "chord"):
        assert np.allclose(wing_panels(flat)[k], wing_panels(same)[k], atol=1e-9), k
    assert np.allclose(wing_outline(flat), wing_outline(same), atol=1e-9)
    tip = np.array(wing_outline(flat)[0][1])                      # right tip leading edge
    assert tip[1] == pytest.approx(math.cos(math.radians(20)), abs=1e-9)
    assert tip[2] == pytest.approx(-math.sin(math.radians(20)), abs=1e-9)


def test_dihedral_that_runs_from_root_to_tip_bends_the_span_line():
    """A wing that curves: the span line is the integral of the angle, so it keeps its length and ends
    where a bent arc ends, not where a straight panel would."""
    from airframe_designer.geometry.wings import Wing, span_curve, wing_outline, wing_panels
    w = Wing(span=2.0, root_chord=0.5, tip_chord=0.5, dihedral_deg=0.0, dihedral_tip_deg=90.0, panels=40)
    off, es = span_curve(w, 1.0, 1.0)
    # a quarter circle of arc length 1 (half span): radius 2/pi, so the tip is at (2/pi, -2/pi)
    r = 2.0 / math.pi
    assert off[1] == pytest.approx(r, abs=1e-6) and off[2] == pytest.approx(-r, abs=1e-6)
    assert es == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)        # pointing straight up at the tip
    # arc length is preserved: the strips still add up to the span
    p = wing_panels(w)
    step = np.diff(p["pos"][: w.panels], axis=0)
    assert np.linalg.norm(step, axis=1).sum() == pytest.approx(w.half_span * (w.panels - 1) / w.panels, rel=2e-3)
    assert p["area"].sum() == pytest.approx(w.area, rel=1e-9)     # area is unchanged by the bend
    assert np.array(wing_outline(w)[0][1])[1] == pytest.approx(r, abs=1e-6)


def test_the_lift_direction_follows_the_curve():
    """Each strip's "up" is perpendicular to its own piece of the span, not to the root's."""
    from airframe_designer.geometry.wings import Wing, wing_panels
    w = Wing(span=2.0, root_chord=0.4, tip_chord=0.4, dihedral_deg=0.0, dihedral_tip_deg=80.0, panels=6)
    p = wing_panels(w)
    right = p["side"] > 0
    for e_n, e_s in zip(p["e_n"][right], p["e_s"][right]):
        assert abs(float(np.dot(e_n, e_s))) < 1e-9                # perpendicular to its own span
        assert np.linalg.norm(e_n) == pytest.approx(1.0, abs=1e-9)
    # the first strip sits at the middle of its own width, by then the wing has turned a twelfth of 80°
    assert p["e_n"][0][2] == pytest.approx(-math.cos(math.radians(80 / 12)), abs=1e-6)
    assert p["e_n"][right][-1][1] < -0.9                          # near-vertical at the tip: lift is sideways


def test_sweep_that_runs_to_the_tip_curves_the_leading_edge():
    from airframe_designer.geometry.wings import Wing, wing_outline
    straight = Wing(span=2.0, root_chord=0.3, tip_chord=0.3, sweep_deg=30.0, panels=4)
    curved = Wing(span=2.0, root_chord=0.3, tip_chord=0.3, sweep_deg=0.0, sweep_tip_deg=60.0, panels=4)
    sx = np.array(wing_outline(straight)[0][1])[0]
    cx = np.array(wing_outline(curved)[0][1])[0]
    assert sx == pytest.approx(-math.tan(math.radians(30)), abs=1e-9)
    assert cx < 0 and cx > -math.tan(math.radians(60))            # between the two ends it sweeps from
    assert np.array(wing_outline(curved)[0][1])[1] == pytest.approx(1.0, abs=1e-9)   # no dihedral: still flat
