"""What a recording says about attitude is what PX4 says, in every mode.

A column has to mean one thing. `pitch` in a run flown here and `pitch` in a run watched over a radio
have to be the same quantity, or comparing them is a trap - and in a real flight PX4's body frame is the
only frame there is. So the recorder reports PX4's: the structural frame pitched nose-up by
``hover_pitch_deg``. The quaternion columns stay structural, because the 3D view draws the aircraft.
"""
import math

import numpy as np

from airframe_designer.dynamics.quaternion import (q_from_euler, q_from_rotmat, q_normalize, q_to_euler,
                                                   q_to_rotmat)
from airframe_designer.geometry.airframe import quad_x
from airframe_designer.sim.recorder import Recorder


def vehicle(af, roll=0.0, pitch=0.0, yaw=0.0, rates=(0.0, 0.0, 0.0)):
    """A state the recorder can sample: structural attitude, structural body rates."""
    class V:
        pass
    v = V()
    v.af = af
    v.q = q_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
    v.pos = np.zeros(3); v.vel = np.zeros(3); v.wind_ned = np.zeros(3)
    v.rates = np.array(rates, float); v.accel_body = np.array([0.0, 0.0, -9.80665])
    v.breakdown = {}; v.on_ground = False
    v.cmd = np.zeros(4); v.omega = np.zeros(4); v.thrust = np.zeros(4)
    v.euler = q_to_euler(v.q)
    v.hover_frame_euler = lambda: _hover_euler(af, v.q)
    return v


def _hover_euler(af, q):
    Rp = q_to_rotmat(q) @ af.hover_rotation().T
    return q_to_euler(q_from_rotmat(Rp))


def sample_of(af, **kw):
    r = Recorder(sample_hz=1000.0)
    v = vehicle(af, **kw)
    r.sample(0.0, v, {"armed": True}, 100.0)
    return dict(zip(r.columns, r.rows[0]))


def test_a_design_with_no_hover_pitch_records_what_it_always_did():
    af = quad_x()
    af.hover_pitch_deg = 0.0
    row = sample_of(af, pitch=7.0, roll=-3.0)
    assert abs(row["pitch"] - 7.0) < 1e-3 and abs(row["roll"] + 3.0) < 1e-3


def test_the_recorded_pitch_is_px4s_not_the_airframes():
    """Sitting at 20 degrees nose-up, an aircraft whose hover frame is 20 degrees nose-up is level."""
    af = quad_x()
    af.hover_pitch_deg = 20.0
    row = sample_of(af, pitch=20.0)
    assert abs(row["pitch"]) < 1e-3, "PX4 calls its own hover attitude level"
    assert abs(sample_of(af, pitch=25.0)["pitch"] - 5.0) < 1e-3


def test_the_rates_and_the_accelerometer_turn_with_it():
    """A vector in the body turns by the hover rotation, the way SENS_BOARD_*_OFF turns the real IMU."""
    af = quad_x()
    af.hover_pitch_deg = 90.0                      # the whole frame on its nose: x and z swap
    row = sample_of(af, rates=(0.0, 0.0, 1.0))     # a structural yaw rate is a roll rate to PX4
    assert abs(row["r"]) < 1e-6 and abs(row["p"] - math.degrees(1.0)) < 1e-3
    assert abs(row["ax"] + 9.80665) < 1e-3 and abs(row["az"]) < 1e-3


def test_a_hover_at_the_design_attitude_reads_level_to_px4():
    """The point of the whole convention: PX4's 'level' is this aircraft's hover attitude."""
    af = quad_x()
    af.hover_pitch_deg = 23.0
    v = vehicle(af, pitch=23.0)
    v.accel_body = af.hover_rotation().T @ np.array([0.0, 0.0, -9.80665])   # specific force, straight up
    r = Recorder(sample_hz=1000.0)
    r.sample(0.0, v, {"armed": True}, 100.0)
    row = dict(zip(r.columns, r.rows[0]))
    assert abs(row["pitch"]) < 1e-3
    assert abs(row["ax"]) < 1e-3 and abs(row["az"] + 9.80665) < 1e-3


def test_the_quaternion_columns_stay_the_aircrafts_own():
    """The 3D view draws the model from these: they must be where the aircraft really is."""
    af = quad_x()
    af.hover_pitch_deg = 20.0
    row = sample_of(af, pitch=20.0)
    q = np.array([row["qw"], row["qx"], row["qy"], row["qz"]])
    assert abs(math.degrees(q_to_euler(q_normalize(q))[1]) - 20.0) < 1e-3


def test_a_real_flight_records_exactly_what_the_aircraft_reported():
    """PX4 sends its own frame; the app stores the structural one and reports PX4's again. Round trip."""
    from types import SimpleNamespace
    from airframe_designer.sim.simulator import Simulator
    af = quad_x()
    af.hover_pitch_deg = 20.0
    sim = Simulator(af)
    reported = q_from_euler(0.0, math.radians(6.0), math.radians(30.0))     # what PX4 says it is
    link = SimpleNamespace(board_state={"pos": [0.0, 0.0, -3.0], "vel": [0, 0, 0], "q": reported.tolist(),
                                        "rates": [0.0, 0.2, 0.0], "t": 0.0},
                           board_motors=[], listening=True, mode="hitl", armed=True,
                           ctl_connected=True, heartbeat_time=0.0, flow=None, params={})
    sim._follow_telemetry(link, 0.05)
    row = dict(zip(sim.recorder.columns, sim.recorder.rows[-1]))
    assert abs(row["pitch"] - 6.0) < 1e-2 and abs(row["yaw"] - 30.0) < 1e-2
    assert abs(row["q"] - math.degrees(0.2)) < 1e-2          # the pitch rate PX4 reported
    # and the model is drawn where the aircraft actually is: 20 degrees further nose-up
    assert abs(math.degrees(q_to_euler(q_normalize(np.array([row["qw"], row["qx"], row["qy"], row["qz"]])))[1])
               - 26.0) < 1e-2


# ------------------------------------------------------------------ the pilot's own hands
def test_the_sticks_are_recorded_so_a_flight_replays_with_its_input():
    """Whoever holds the sticks - the USB remote in SITL, the receiver in flight - lands in rc1..rc8."""
    af = quad_x()
    r = Recorder(sample_hz=1000.0)
    r.sample(0.0, vehicle(af), {"armed": True, "rc": [1500, 1600, 1000, 1480, 2000]}, 100.0)
    row = dict(zip(r.columns, r.rows[0]))
    assert [row[f"rc{i}"] for i in range(1, 9)] == [1500, 1600, 1000, 1480, 2000, 0, 0, 0]


def test_a_flight_with_no_receiver_records_zeros_rather_than_nothing():
    """A column that is always there is a column a saved run can be read by; 0 says 'no such channel'."""
    r = Recorder(sample_hz=1000.0)
    r.sample(0.0, vehicle(quad_x()), {"armed": False}, 100.0)
    row = dict(zip(r.columns, r.rows[0]))
    assert all(row[f"rc{i}"] == 0 for i in range(1, 9))


def test_the_rotor_columns_still_come_after_the_fixed_ones():
    """Appending to COLUMNS is safe; it must not have reordered what a saved run already carries."""
    from airframe_designer.sim.recorder import COLUMNS, columns_for
    cols = columns_for(4)
    assert cols[:len(COLUMNS)] == list(COLUMNS)
    assert cols[len(COLUMNS)] == "m1_cmd" and cols[-1] == "m4_thrust"
