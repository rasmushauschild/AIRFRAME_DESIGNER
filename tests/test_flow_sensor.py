"""The H-flow sensor: an optical flow camera looking down with a rangefinder beside it.

The flow messages are checked by running PX4's own arithmetic over them (EKF2.cpp UpdateFlowSample,
optical_flow_control.cpp and optical_flow_fusion.cpp in v1.18) and asking whether what comes out is the
velocity the aircraft actually has. A measurement that survives that is one PX4 can fly on.
"""
import math

import numpy as np
import pytest
from types import SimpleNamespace

from airframe_designer.geometry.airframe import quad_x, mount_rotation, rotation_euler_deg
from airframe_designer.sensors import SensorSuite


def fake_sim(af, pos=(0.0, 0.0, -2.0), vel=(0.0, 0.0, 0.0), rates=(0.0, 0.0, 0.0), euler=(0.0, 0.0, 0.0)):
    """A vehicle state the sensor models can read: NED position and velocity, attitude, body rates."""
    R = mount_rotation([math.degrees(a) for a in euler])        # structural -> NED, same composition
    return SimpleNamespace(af=af, pos=np.array(pos, float), vel=np.array(vel, float),
                           rates=np.array(rates, float), rotmat=R, accel_body=np.zeros(3))


def with_flow(**kw):
    af = quad_x()
    af.mass.cg = [0.0, 0.0, 0.0]
    af.design["flow_sensor"] = {"enabled": True, "position": [0.0, 0.0, 0.0], "rotation": [0, 0, 0],
                                "range_m": 4.0, **kw}
    return af


def px4_flow_velocity(msg, hagl, r22=1.0, flow_rot=np.eye(2)):
    """What PX4 makes of a HIL_OPTICAL_FLOW message: the body-frame velocity it will fuse.

    EKF2 takes flow_rate = -pixel_flow/dt and gyro_rate = -delta_angle/dt, compensates the first with the
    second, and predicts (vy, -vx) * R(2,2)/hagl for that. Inverting the prediction gives the velocity the
    message claims, which is the thing worth comparing against the truth.
    """
    dt = msg["integration_time_us"] * 1e-6
    pixel = flow_rot @ np.array([msg["integrated_x"], msg["integrated_y"]])      # SENS_FLOW_ROT
    flow_rate = -pixel / dt
    gyro_rate = -np.array([msg["integrated_xgyro"], msg["integrated_ygyro"]]) / dt
    compensated = flow_rate - gyro_rate
    scale = r22 / hagl
    return np.array([-compensated[1] / scale, compensated[0] / scale])


def test_no_flow_sensor_means_no_messages_and_fusion_off():
    af = quad_x()
    s = SensorSuite()
    sim = fake_sim(af)
    assert s.hil_optical_flow(sim, 1_000_000, 0.05) is None
    assert s.distance_sensor(sim, 1_000_000) is None
    assert af.flow_sensor() is None
    p = af.px4_params()
    assert p["EKF2_OF_CTRL"] == 0 and p["EKF2_RNG_CTRL"] == 0


def test_flow_reports_the_velocity_the_aircraft_has():
    af = with_flow()
    s = SensorSuite(); s.noise.enabled = False
    sim = fake_sim(af, pos=(0, 0, -2.0), vel=(3.0, -1.0, 0.0))
    msg = s.hil_optical_flow(sim, 1_000_000, 0.05)
    assert msg["quality"] > 0 and msg["distance"] == pytest.approx(2.0)
    assert px4_flow_velocity(msg, hagl=2.0) == pytest.approx([3.0, -1.0], abs=1e-9)


def test_rotating_on_the_spot_compensates_to_no_velocity():
    af = with_flow()
    s = SensorSuite(); s.noise.enabled = False
    sim = fake_sim(af, rates=(0.3, -0.2, 0.1))
    msg = s.hil_optical_flow(sim, 1_000_000, 0.05)
    # the raw picture does move, and the gyro is what says it was rotation and not travel
    assert abs(msg["integrated_x"]) > 1e-3
    assert px4_flow_velocity(msg, hagl=2.0) == pytest.approx([0.0, 0.0], abs=1e-9)


def test_a_sensor_off_the_cg_sees_the_speed_of_its_own_mounting_point():
    af = with_flow(position=[0.0, 0.5, 0.0])
    s = SensorSuite(); s.noise.enabled = False
    # 1 rad/s nose-right yaw: a lens half a metre out on the right wing swings backwards at 0.5 m/s
    sim = fake_sim(af, rates=(0.0, 0.0, 1.0))
    msg = s.hil_optical_flow(sim, 1_000_000, 0.05)
    assert px4_flow_velocity(msg, hagl=2.0) == pytest.approx([-0.5, 0.0], abs=1e-9)


def test_tilted_flight_measures_the_slant_range_and_still_gives_the_velocity():
    af = with_flow()
    s = SensorSuite(); s.noise.enabled = False
    pitch = math.radians(20.0)
    sim = fake_sim(af, pos=(0, 0, -2.0), vel=(2.0, 0.0, 0.0), euler=(0.0, pitch, 0.0))
    msg = s.hil_optical_flow(sim, 1_000_000, 0.05)
    slant = 2.0 / math.cos(pitch)
    assert msg["distance"] == pytest.approx(slant, abs=1e-9)
    # PX4 works in body axes: the forward speed is the ground speed turned into them
    v_body = sim.rotmat.T @ sim.vel
    got = px4_flow_velocity(msg, hagl=2.0, r22=math.cos(pitch))
    assert got == pytest.approx(v_body[:2], abs=1e-9)


def test_out_of_range_is_said_to_be_out_of_range():
    af = with_flow(range_m=4.0)
    s = SensorSuite(); s.noise.enabled = False
    high = s.hil_optical_flow(fake_sim(af, pos=(0, 0, -30.0), vel=(2, 0, 0)), 1_000_000, 0.05)
    assert high["quality"] == 0 and high["distance"] == -1.0 and high["integrated_x"] == 0.0
    sideways = s.hil_optical_flow(fake_sim(af, euler=(math.radians(100), 0, 0)), 1_000_000, 0.05)
    assert sideways["quality"] == 0          # the beam points at the sky
    d = s.distance_sensor(fake_sim(af, pos=(0, 0, -30.0)), 1_000_000)
    assert d["current_distance"] > d["max_distance"]


def test_the_rangefinder_message_is_a_downward_laser_in_centimetres():
    af = with_flow(range_m=4.0)
    s = SensorSuite(); s.noise.enabled = False
    d = s.distance_sensor(fake_sim(af, pos=(0, 0, -1.5)), 2_000_000)
    assert d["orientation"] == 25 and d["type"] == 0            # MAV_SENSOR_ROTATION_PITCH_270, laser
    assert d["current_distance"] == 150 and d["max_distance"] == 400
    assert d["time_boot_ms"] == 2000


def test_a_sensor_mounted_sideways_comes_out_right_once_px4_applies_sens_flow_rot():
    af = with_flow(rotation=[0, 0, 90])
    assert af.px4_params()["SENS_FLOW_ROT"] == 2                # 2 = yaw 90
    s = SensorSuite(); s.noise.enabled = False
    sim = fake_sim(af, vel=(3.0, -1.0, 0.0))
    msg = s.hil_optical_flow(sim, 1_000_000, 0.05)
    yaw90 = np.array([[0.0, -1.0], [1.0, 0.0]])                 # what rotate_3f(ROTATION_YAW_90) does
    assert px4_flow_velocity(msg, hagl=2.0, flow_rot=yaw90) == pytest.approx([3.0, -1.0], abs=1e-9)


def test_flow_export_follows_the_cg_and_the_hover_frame():
    af = with_flow(position=[0.3, 0.1, -0.05])
    af.hover_pitch_deg = 25
    af.mass.cg = [0.1, 0.0, 0.0]
    p = af.px4_params()
    expected = af.hover_rotation() @ np.array([0.2, 0.1, -0.05])
    assert [p[f"EKF2_OF_POS_{a}"] for a in "XYZ"] == pytest.approx(expected, abs=1e-4)
    assert [p[f"EKF2_RNG_POS_{a}"] for a in "XYZ"] == pytest.approx(expected, abs=1e-4)
    assert p["EKF2_OF_CTRL"] == 1 and p["EKF2_RNG_CTRL"] == 1
    assert p["SENS_FLOW_MAXHGT"] == 4.0


def test_flow_yaw_snaps_to_the_step_px4_has_and_keeps_the_rest_in_the_measurement():
    af = with_flow(rotation=[0, 0, 50])
    assert af.px4_params()["SENS_FLOW_ROT"] == 1                # nearest step is 45 degrees
    s = SensorSuite(); s.noise.enabled = False
    sim = fake_sim(af, vel=(3.0, 0.0, 0.0))
    msg = s.hil_optical_flow(sim, 1_000_000, 0.05)
    c, sn = math.cos(math.radians(45)), math.sin(math.radians(45))
    got = px4_flow_velocity(msg, hagl=2.0, flow_rot=np.array([[c, -sn], [sn, c]]))
    assert np.linalg.norm(got) == pytest.approx(3.0, abs=1e-9)  # the speed survives
    assert got[1] == pytest.approx(3.0 * math.sin(math.radians(-5)), abs=1e-9)   # the 5 degrees do not


# ------------------------------------------------------------------ the board itself

def test_a_square_board_exports_the_hover_pitch_and_nothing_else():
    af = quad_x(); af.hover_pitch_deg = 25
    p = af.px4_params()
    assert (p["SENS_BOARD_X_OFF"], p["SENS_BOARD_Y_OFF"], p["SENS_BOARD_Z_OFF"]) == (0.0, 25.0, 0.0)


def test_a_crooked_board_reads_in_its_own_axes_and_px4_puts_it_back():
    af = quad_x(); af.hover_pitch_deg = 15
    af.design["pixhawk_rotation"] = [10.0, -20.0, 30.0]
    s = SensorSuite(); s.noise.enabled = False
    sim = fake_sim(af, pos=(0, 0, -5.0))
    sim.accel_body = np.array([0.3, -0.2, -9.81])
    sim.rates = np.array([0.1, 0.2, -0.3])
    msg = s.hil_sensor(sim, 1_000_000)
    p = af.px4_params()
    board = mount_rotation([p["SENS_BOARD_X_OFF"], p["SENS_BOARD_Y_OFF"], p["SENS_BOARD_Z_OFF"]])
    # the parameters are written to a hundredth of a degree, so the recovery is good to about that
    read = np.array([msg["xgyro"], msg["ygyro"], msg["zgyro"]])
    assert board @ read == pytest.approx(af.hover_rotation() @ sim.rates, abs=1e-4)
    read = np.array([msg["xacc"], msg["yacc"], msg["zacc"]])
    assert board @ read == pytest.approx(af.hover_rotation() @ sim.accel_body, abs=3e-3)


def test_mount_angles_survive_the_round_trip():
    for angles in ([0, 0, 0], [10, -20, 30], [-45, 0, 170], [0, 89.9, 0]):
        R = mount_rotation(angles)
        assert R @ R.T == pytest.approx(np.eye(3), abs=1e-12)
        assert rotation_euler_deg(R) == pytest.approx(angles, abs=1e-6)
