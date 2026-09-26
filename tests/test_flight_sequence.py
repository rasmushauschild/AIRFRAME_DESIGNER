import math
from contextlib import nullcontext
from types import SimpleNamespace as NS
import numpy as np
import pytest
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.sim.wing_controller import WingVelocityController, NoseLower
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


def test_landing_rejects_hardware():
    with pytest.raises(ValueError,match='SITL-only'):
        NoseLower(NS(link=NS(mode='hitl')), [0])


def test_landing_keeps_support_until_nose_settles():
    class Support:
        motors=[0];split=None;integral=0.;kq=.02;kqi=.012
        def _update_split(self,*a):pass
        def _balance_fraction(self,*a):return .4
        def _thrust_to_cmd(self,s,f):return math.sqrt(max(0.,f))
    ctl=NoseLower.__new__(NoseLower)
    ctl.support=Support();ctl.rate=3.;ctl.target=-20.;ctl.initial=np.array([.6]);ctl.t0=ctl.last=0.
    ctl.settled=ctl.fade=None;ctl.done=False;ctl.last_disarm=-10.
    calls=[]
    state=NS(rotors=NS(n=1),rates=np.zeros(3),euler=[0.,0.,0.],on_ground=True,vel=np.zeros(3))
    sim=NS(sim=state,t=2.,link=NS(armed=True,disarm=lambda:calls.append('disarm')),motor_override=None)
    ctl.step(sim)
    assert sim.motor_override[0]>0 and not ctl.done and ctl.fade is None
    assert calls==['disarm']
    state.euler[1]=math.radians(-20.);sim.link.armed=False
    for t in [3.,4.1,5.,6.2]:sim.t=t;ctl.step(sim)
    assert ctl.done and sim.motor_override is None


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

@pytest.mark.parametrize('saved,explicit,expected', [(17,None,17),(0,None,0),(None,None,10),(17,23,23)])
def test_scenario_nose_pitch_uses_simulator_setting(saved, explicit, expected):
    from airframe_designer.sim.scenario import ScenarioRunner, Scenario
    r=ScenarioRunner(Scenario(),NS(mode='sitl'))
    r.phase={'type':'nose_lift','motors':[0]};r._state={}
    if explicit is not None:r.phase['target_pitch_deg']=explicit
    af=NS(hover_pitch_deg=10,design={'nose_lift':{} if saved is None else {'target_pitch_deg':saved}})
    calls=[]
    sim=NS(airframe=af,start_nose_lift=lambda motors,target,**kw:calls.append(target))
    r._p_nose_lift(sim)
    assert calls==[expected]
