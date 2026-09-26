import asyncio
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from airframe_designer.server.scenario_params import prepare_scenario_params

def state():
    link=NS(ctl_connected=True,status=lambda:{'armed':False},set_param=Mock(side_effect=lambda n,v:{'ok':True,'value':v}))
    return NS(link=link,conn=NS(mode='sitl'),simulator=NS(paused=False),log=Mock())

def test_applies_and_verifies_both_scenario_settings():
    s=state();asyncio.run(prepare_scenario_params(s,{'MIS_TAKEOFF_ALT':10,'MPC_XY_VEL_MAX':30}))
    assert s.link.set_param.call_count==2

@pytest.mark.parametrize('blocked',['hitl','armed','paused','disconnected'])
def test_rejects_before_writing(blocked):
    s=state()
    if blocked=='hitl':s.conn.mode='hitl'
    if blocked=='armed':s.link.status=lambda:{'armed':True}
    if blocked=='paused':s.simulator.paused=True
    if blocked=='disconnected':s.link.ctl_connected=False
    with pytest.raises(ValueError):asyncio.run(prepare_scenario_params(s,{'MIS_TAKEOFF_ALT':10}))
    s.link.set_param.assert_not_called()

def test_failed_ack_stops_preparation():
    s=state();s.link.set_param.side_effect=lambda n,v:{'ok':False,'error':'timeout'}
    with pytest.raises(ValueError,match='timeout'):asyncio.run(prepare_scenario_params(s,{'A':1,'B':2}))
    assert s.link.set_param.call_count==1

def test_connection_change_stops_remaining_writes():
    s=state()
    def change(n,v):s.conn.mode='hitl';return {'ok':True,'value':v}
    s.link.set_param.side_effect=change
    with pytest.raises(ValueError,match='connection changed'):asyncio.run(prepare_scenario_params(s,{'A':1,'B':2}))
    assert s.link.set_param.call_count==1

def test_invalid_values_validated_before_any_write():
    s=state()
    with pytest.raises(ValueError):asyncio.run(prepare_scenario_params(s,{'A':1,'B':float('nan')}))
    s.link.set_param.assert_not_called()
