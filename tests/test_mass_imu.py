import numpy as np
import pytest
from types import SimpleNamespace
from airframe_designer.geometry.mass import MassProperties, MassItem
from airframe_designer.geometry.airframe import quad_x
from airframe_designer.sensors import SensorSuite

def test_manual_values_survive_calculation_and_serialization():
    m = MassProperties(mass=13, cg=[.1, .2, .3], inertia=[1, 2, 3], from_items=True,
                       items=[MassItem(mass=2, pos=[0, 0, 0])])
    m.resolve()
    restored = MassProperties.from_dict(m.to_dict())
    assert restored.mass == 2
    assert restored.manual == dict(mass=13, cg=[.1, .2, .3], inertia=[1, 2, 3], inertia_products=[0, 0, 0])

def test_pixhawk_export_tracks_cg_and_hover_frame():
    af = quad_x()
    af.hover_pitch_deg = 25
    af.mass.cg = [.1, 0, 0]
    af.design['pixhawk_position'] = [.3, .1, -.05]
    p = af.px4_params_sitl()
    expected = af.hover_rotation() @ np.array([.2, .1, -.05])
    assert [p['EKF2_IMU_POS_'+a] for a in 'XYZ'] == pytest.approx(expected, abs=.0001)

def test_imu_offset_rotation_acceleration_and_clock_reset():
    af = quad_x(); af.design['pixhawk_position'] = (af.cg + [1, 0, 0]).tolist()
    sim = SimpleNamespace(af=af, rates=np.array([0., 0., 2.]))
    sensor = SensorSuite()
    assert sensor.imu_lever_acceleration(sim, 10000) == pytest.approx([-4, 0, 0])
    sim.rates = np.array([0., 0., 3.])
    assert sensor.imu_lever_acceleration(sim, 20000) == pytest.approx([-9, 100, 0])
    assert sensor.imu_lever_acceleration(sim, 0) == pytest.approx([-9, 0, 0])
