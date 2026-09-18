import numpy as np
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
