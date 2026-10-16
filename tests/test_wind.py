"""The wind model: a steady vector, or gusts that are smooth, bounded and the same every run."""
import math

import numpy as np
import pytest

from airframe_designer.sim.wind import WindModel


def test_a_steady_wind_is_the_same_at_every_moment():
    w = WindModel(speed=6.0, direction_deg=90.0)          # from the east: the air goes west
    assert w.at(0.0) == pytest.approx([0.0, -6.0, 0.0], abs=1e-9)
    assert w.at(123.4) == pytest.approx([0.0, -6.0, 0.0], abs=1e-9)


def test_direction_is_where_it_comes_from():
    north = WindModel(speed=3.0, direction_deg=0.0).at(0)   # from the north: the air goes south
    assert north == pytest.approx([-3.0, 0.0, 0.0], abs=1e-9)
    west = WindModel(speed=3.0, direction_deg=270.0).at(0)  # from the west: the air goes east
    assert west == pytest.approx([0.0, 3.0, 0.0], abs=1e-9)


def test_a_vector_survives_the_round_trip():
    for v in ([4.0, -2.0, 0.5], [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]):
        assert WindModel.from_vector(*v).at(0.0) == pytest.approx(v, abs=1e-9)


def test_a_vector_becomes_a_steady_wind_with_no_gusts():
    w = WindModel.from_dict({"north": 3.0, "east": 4.0, "down": 0.0})
    assert w.gusty is False and w.speed == pytest.approx(5.0)


def test_gusts_stay_inside_the_band_and_never_blow_backwards():
    w = WindModel(speed=5.0, direction_deg=180.0, gusty=True, gust_speed=3.0, gust_period_s=6.0, gust_dir_deg=20.0)
    speeds = [w.speed_dir_at(t)[0] for t in np.arange(0, 300, 0.1)]
    dirs = [w.speed_dir_at(t)[1] for t in np.arange(0, 300, 0.1)]
    assert min(speeds) >= 0.0
    assert min(speeds) >= 5.0 - 3.0 - 1e-9 and max(speeds) <= 5.0 + 3.0 + 1e-9
    assert max(dirs) - min(dirs) > 5.0                      # it really does swing
    assert all(abs(d - 180.0) <= 20.0 + 1e-9 for d in dirs)
    assert max(speeds) - min(speeds) > 1.0                  # and the speed really does change


def test_a_gust_is_smooth_enough_to_fly_through():
    w = WindModel(speed=8.0, direction_deg=0.0, gusty=True, gust_speed=4.0, gust_period_s=5.0)
    dt = 0.004                                              # one sensor step at 250 Hz
    worst = max(abs(w.speed_dir_at(t + dt)[0] - w.speed_dir_at(t)[0]) for t in np.arange(0, 60, dt / 2))
    assert worst / dt < 15.0                                # m/s per second, not a step change


def test_a_gusty_wind_never_leaves_zero_mean_wind_blowing():
    w = WindModel(speed=0.0, gusty=True, gust_speed=2.0)
    assert min(w.speed_dir_at(t)[0] for t in np.arange(0, 120, 0.1)) >= 0.0


def test_the_same_seed_blows_the_same_wind_and_another_does_not():
    a = WindModel(speed=5, gusty=True, gust_speed=3, seed=1)
    b = WindModel(speed=5, gusty=True, gust_speed=3, seed=1)
    c = WindModel(speed=5, gusty=True, gust_speed=3, seed=2)
    ts = np.arange(0, 40, 0.5)
    assert [a.at(t).tolist() for t in ts] == [b.at(t).tolist() for t in ts]
    assert [a.at(t).tolist() for t in ts] != [c.at(t).tolist() for t in ts]


def test_the_preview_is_the_wind_that_will_blow():
    w = WindModel(speed=4, direction_deg=30, gusty=True, gust_speed=2, gust_period_s=7, gust_dir_deg=10)
    p = w.preview(seconds=10, step=0.5)
    assert len(p["t"]) == len(p["speed"]) == 21
    for t, speed, direction in zip(p["t"], p["speed"], p["dir"]):
        got_speed, got_dir, _ = w.speed_dir_at(t)
        assert speed == pytest.approx(got_speed, abs=1e-3)
        assert direction == pytest.approx(got_dir, abs=1e-2)


def test_a_dict_round_trips():
    w = WindModel(speed=3.5, direction_deg=200, down=0.4, gusty=True, gust_speed=1.5, gust_period_s=4, gust_dir_deg=25, seed=7)
    assert WindModel.from_dict(w.to_dict()) == w


def test_the_simulator_blows_the_model_and_a_plain_vector_stops_the_gusts():
    from airframe_designer.geometry.airframe import quad_x
    from airframe_designer.sim.simulator import Simulator
    simr = Simulator(quad_x(), link=None)
    simr.set_wind_model(WindModel(speed=6, direction_deg=90, gusty=True, gust_speed=3, gust_period_s=4))
    first = simr.sim.wind_ned.copy()
    simr.time_usec = 2_000_000
    simr.set_wind_model(simr.wind)                      # same model, a later moment
    assert not np.allclose(first, simr.sim.wind_ned)    # the gust has moved on
    simr.set_wind(1.0, 2.0, 0.0)
    assert simr.wind.gusty is False
    assert simr.sim.wind_ned.tolist() == pytest.approx([1.0, 2.0, 0.0])
    assert simr.wind.at(99.0).tolist() == pytest.approx([1.0, 2.0, 0.0])


def test_the_default_wind_is_calm():
    assert WindModel().at(0.0).tolist() == [0.0, 0.0, 0.0]
    assert math.isclose(WindModel().preview(2, 1)["speed"][0], 0.0)
