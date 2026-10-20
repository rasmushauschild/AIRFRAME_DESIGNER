import math
from contextlib import nullcontext
from types import SimpleNamespace as NS
import numpy as np
import pytest
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.sim.wing_controller import WingVelocityController
from airframe_designer.dynamics.quaternion import q_to_euler


def test_cruise_rotates_structural_force_into_hover_frame():
    af=Airframe.load('airframes/atlas_mvp_08b.json')
    messages=[]
    link=NS(target_system=2,target_component=1,_ctl_lock=nullcontext(),ctl=NS(mav=NS(send=messages.append)))
    state=NS(pos=np.array([0.,0.,-10.]),vel=np.zeros(3),wind_ned=np.zeros(3),rotmat=af.hover_rotation())
    sim=NS(airframe=af,sim=state,t=0.,link=link)
    structural=af.hover_rotation().T@np.array([0.,0.,-1000.])
    table=[[v,*structural,10.] for v in [0.,10.]]
    c=WingVelocityController(sim,table);sim.t=.05;c.step(sim,0.)
    assert messages[-1].thrust_body[0]==pytest.approx(0.,abs=1e-10)
    assert q_to_euler(np.array(messages[-1].q))[1]==pytest.approx(0.,abs=1e-10)


# NoseLower and the nose_lift / nose_land phases are gone on purpose: a ground sequence that only
# exists in the simulator makes SITL fly an aircraft nobody built. The firmware module does it in both,
# reached through the `module` phase, and the tests for it are below.


def test_native_cruise_ramps_speed_and_corrects_altitude():
    from airframe_designer.sim.scenario import ScenarioRunner, Scenario
    r=ScenarioRunner(Scenario(),NS(mode='sitl'))
    r.phase={'type':'cruise_velocity','speed':50/3.6,'ramp':.6,'duration':30}
    r.phase_t0=0.;r._state={}
    calls=[];r._offboard_stream=lambda simr,**kw:calls.append(kw)
    sim=NS(t=0.,sim=NS(vel=np.zeros(3),pos=np.array([0.,0.,-10.])))
    r._p_cruise_velocity(sim)
    sim.t=.1;sim.sim.pos[2]=-9.;r._p_cruise_velocity(sim)
    assert calls[-1]['vel']==pytest.approx([.06,0.,-.6])

# ---------------------------------------------------------------- handing a phase to a firmware module
def _module_runner(phase):
    """A runner with just enough around it to step one `module` phase."""
    from airframe_designer.sim.scenario import ScenarioRunner, Scenario
    sent = []
    link = NS(armed=True, send_command_long=lambda cmd, *p: sent.append((cmd, p)))
    r = ScenarioRunner(Scenario(), link)
    r.phase = phase
    r.phase_t0 = 0.0
    r._state = {}
    r._rest_alt = 0.0
    nxt = []
    r._next = lambda simr: nxt.append(simr.t)
    r.metrics.phases[r._name()] = {}       # the runner makes this when it enters a phase
    return r, sent, nxt


def _sim(t, alt, vz=0.0, on_ground=False):
    return NS(t=t, sim=NS(pos=np.array([0.0, 0.0, -alt]), vel=np.array([0.0, 0.0, vz]), on_ground=on_ground))


def test_a_module_takeoff_asks_over_user_1_and_waits_for_the_height():
    """MAV_CMD_USER_1 is the one command PX4 leaves to the airframe's own code; the phase sends it and
    then watches for the height the module was asked for, rather than assuming it worked."""
    r, sent, nxt = _module_runner({'type': 'module', 'action': 'takeoff', 'alt': 2.0, 'settle': 1.0})
    r._p_module(_sim(0.0, 0.0))
    assert sent == [(31010, (1.0,))]                      # MAV_CMD_USER_1, param1 = 1 (take off)
    r._p_module(_sim(1.0, 0.5)); assert not nxt          # still on its way up
    r._p_module(_sim(5.0, 1.95, vz=0.05)); assert not nxt  # there, but not settled yet
    r._p_module(_sim(6.1, 1.98, vz=0.02)); assert nxt == [6.1]
    assert len(sent) == 1                                 # asked once, not once per step


def test_a_module_takeoff_that_never_climbs_fails_with_what_it_saw():
    r, sent, nxt = _module_runner({'type': 'module', 'action': 'takeoff', 'alt': 2.0, 'timeout': 10.0})
    r._p_module(_sim(0.0, 0.0))
    r._p_module(_sim(11.0, 0.1))
    assert not nxt and not r.ok
    assert any('did not reach 2.0 m' in f for f in r.failures), r.failures


def test_a_module_landing_waits_for_the_ground_and_the_disarm():
    r, sent, nxt = _module_runner({'type': 'module', 'action': 'land'})
    r._p_module(_sim(0.0, 3.0))
    assert sent == [(31010, (2.0,))]                      # param1 = 2 (land)
    r._p_module(_sim(5.0, 0.0, on_ground=True)); assert not nxt   # down, but still armed
    r.link.armed = False
    r._p_module(_sim(6.0, 0.0, on_ground=True)); assert nxt == [6.0]
