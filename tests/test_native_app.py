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


def test_a_leg_being_edited_does_not_break_the_save():
    """Two feet, or three in a line, have no resting pitch - but the airframe must still be savable: the
    last good angle stays and the reason is logged, instead of the request failing."""
    from airframe_designer.native import landed_pitch_if_possible
    from airframe_designer.geometry.airframe import quad_x
    from airframe_designer.geometry.gear import generate_legs

    af = quad_x()
    af.legs = generate_legs(height=0.25, spread_x=0.2, spread_y=0.2)
    landed_pitch_if_possible(af)
    good = af.landed_pitch_deg
    for leg in af.legs[2:]:                      # two feet left: no plane to stand on
        leg.enabled = False
    said = []
    landed_pitch_if_possible(af, said.append)
    assert af.landed_pitch_deg == good          # unchanged, and nothing raised
    assert said and 'landed pitch unchanged' in said[0]


def test_reset_puts_the_vehicle_down_at_once_and_never_refuses(monkeypatch):
    """Reset is a stop button: the vehicle is down before the call returns, and pressing it again while the
    (slow) PX4 relaunch is still going must reset again rather than answer 'already in progress'."""
    import threading, time
    relaunches, started = [], threading.Event()

    class FakeSim:
        running = False
        motor_override = [0.5]
        def __init__(self): self.resets = 0
        def reset(self): self.resets += 1

    sim = FakeSim()
    args = SimpleNamespace(baud=None, launch_px4=True, px4_dir='.', px4_model='none_iris',
                           px4_rootfs=None, px4_instance=0, qgc=False)
    conn = NativeConnectionManager(sim, args, lambda m: None)
    conn.mode = 'sitl'

    def slow_connect(launch=None):
        started.set()
        relaunches.append(time.time())
        time.sleep(1.0)                       # a real relaunch is seconds of PX4 booting
    monkeypatch.setattr(conn, 'connect_sitl', slow_connect)

    t0 = time.perf_counter()
    conn.reset_all()
    assert time.perf_counter() - t0 < 0.1, 'the click waited for PX4'
    assert sim.resets == 1 and sim.motor_override is None
    assert started.wait(1.0), 'the relaunch never started'

    t1 = time.perf_counter()
    r = conn.reset_all()                       # pressed again mid-relaunch
    assert time.perf_counter() - t1 < 0.1
    assert sim.resets == 2, 'the second press was swallowed'
    assert 'already in progress' not in ' '.join(r['steps'])
    time.sleep(1.4)
    assert len(relaunches) == 2                # and the newer press relaunched behind it


def _tripod():
    """Front foot ahead of the CG, two rear feet behind it: the V4 stand, at its CAD hard points."""
    from airframe_designer.geometry.airframe import quad_x
    from airframe_designer.geometry.gear import Leg
    af = quad_x()
    af.legs = [Leg(name="F", attach=[0.573, 0.0, -0.42], length=0.0677, foot_radius=0.01),
               Leg(name="RL", attach=[-0.395, -0.595, -0.03], length=0.03, foot_radius=0.01),
               Leg(name="RR", attach=[-0.395, 0.595, -0.03], length=0.03, foot_radius=0.01)]
    af.mass.cg = [-0.2, 0.0, -0.43]
    return af


@pytest.mark.parametrize("park", [-20, -10, 0, 10])
def test_park_pitch_moves_only_the_front_leg(park):
    """Typing a park angle stands the aircraft at it; the rear feet it rotates off do not move."""
    from airframe_designer.geometry.landing import set_park_pitch
    af = _tripod()
    rear = [l.foot() for l in af.legs[1:]]
    assert set_park_pitch(af, park) == pytest.approx(park, abs=0.02)
    assert [l.foot() for l in af.legs[1:]] == rear
    assert af.legs[0].length == pytest.approx(0.42 + 0.968 * math.tan(math.radians(park)), abs=1e-3)


def test_a_park_the_legs_cannot_reach_changes_nothing():
    from airframe_designer.geometry.landing import set_park_pitch
    af = _tripod()
    before = [l.length for l in af.legs], af.landed_pitch_deg
    with pytest.raises(ValueError, match="leg F would be"):
        set_park_pitch(af, -30)                  # the front leg would have to be negative
    assert ([l.length for l in af.legs], af.landed_pitch_deg) == before
