"""The simulator integrates the ideal motor power into energy_wh and resets it with the vehicle."""
from airframe_designer.geometry.airframe import quad_x
from airframe_designer.sim.simulator import Simulator


def test_energy_accumulates_and_resets():
    sim = Simulator(quad_x(), None, sensor_rate=250, physics_substeps=2, speed=0, lockstep=False, log=lambda s: None)
    dt = 1.0 / 250
    sim.sim.set_motor_commands([0.7] * 4)
    for _ in range(250):                       # one simulated second at 70 % command, like step_once does
        for _ in range(2):
            sim.sim.step(dt / 2, detail=True)
        sim._account_energy(dt)
    power = sim.sim.breakdown["power"]
    assert power > 0
    assert abs(sim.energy_wh - power / 3600.0) < 0.15 * power / 3600.0      # ~1 s of that power, in Wh
    assert 0 < sim.power_avg_w <= power * 1.01
    assert sim.snapshot()["energy_wh"] == round(sim.energy_wh, 4)
    sim.reset()
    assert sim.energy_wh == 0.0 and sim.power_avg_w == 0.0 and sim.snapshot()["energy_wh"] == 0.0
