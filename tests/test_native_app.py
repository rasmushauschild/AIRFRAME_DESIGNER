import math
from types import SimpleNamespace
import pytest
from airframe_designer.geometry.landing import apply_landed_pitch
from airframe_designer import app
from airframe_designer.native import NativeSimulator, NativeConnectionManager

@pytest.mark.parametrize("angle", [-20, 0, 15])
def test_landed_pitch_matches_support_plane(angle):
    slope = math.tan(math.radians(angle))
    legs = [SimpleNamespace(foot=lambda x=x, y=y: (x, y, slope*x), foot_radius=.03)
            for x, y in [(1, 0), (-1, -.5), (-1, .5)]]
    frame = SimpleNamespace(active_legs=lambda: legs, px4_overrides={})
    assert apply_landed_pitch(frame) == pytest.approx(angle, abs=.02)
    assert frame.px4_overrides["NLF_LAND_ANG"] == frame.landed_pitch_deg

def test_standard_app_uses_native_runtime():
    assert app.Simulator is NativeSimulator
    assert app.ConnectionManager is NativeConnectionManager

def test_missing_native_build_does_not_fall_back(tmp_path):
    with pytest.raises(SystemExit) as exc:
        app.main(["--px4-dir", str(tmp_path), "--no-browser"])
    assert exc.value.code == 2
