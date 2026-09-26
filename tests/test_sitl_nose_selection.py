from types import SimpleNamespace as NS
from unittest.mock import Mock
import asyncio
import pytest
from airframe_designer.sim.motor_selection import packed_selection
from airframe_designer.server.app import build_app

@pytest.mark.parametrize('count', [1, 6, 8, 9, 10, 12])
def test_counts_and_nonterminal_selection(count):
    rotors = [NS(enabled=True) for _ in range(count)]
    assert packed_selection(rotors, [0, count-1] if count > 1 else [0], True) == ([0,count-1] if count > 1 else [0])

def test_disabled_motors_pack_without_changing_identity():
    rotors = [NS(enabled=i not in (1,6,7)) for i in range(10)]
    assert packed_selection(rotors, [1,8,9], True) == [5,6]
    assert packed_selection(rotors, [5,6]) == [5,6]

@pytest.mark.parametrize('indices', [[], [8], [-1], [0,0], [True], [1.5]])
def test_bad_selection_rejected(indices):
    with pytest.raises(ValueError):
        packed_selection([NS(enabled=True)]*8, indices, True)

def test_only_disabled_selection_rejected():
    with pytest.raises(ValueError):
        packed_selection([NS(enabled=False), NS(enabled=True)], [0], True)

def endpoint(mode, path):
    sim=Mock(); sim.airframe.rotors=[NS(enabled=i not in (6,7)) for i in range(10)]
    sim.sim.on_ground=True; sim.airframe.hover_pitch_deg=23
    sim.start_nose_lift.return_value.status.return_value={}
    state=NS(simulator=sim,conn=NS(mode=mode),link=NS(armed=False))
    app=build_app(state)
    return next(r.endpoint for r in app.routes if r.path==path),sim

@pytest.mark.parametrize('mode',['hitl',None])
def test_hardware_and_disconnected_rejected_even_for_stop(mode):
    fn,sim=endpoint(mode,'/api/sim/nose_lift')
    for body in ({'motors':[0]}, {'stop':True}):
        assert asyncio.run(fn(body)).status_code==409
    sim.start_nose_lift.assert_not_called(); sim.stop_nose_lift.assert_not_called()

def test_endpoint_passes_packed_indices():
    fn,sim=endpoint('sitl','/api/sim/nose_lift')
    assert asyncio.run(fn({'motors':[8,9], 'index_space':'editor'}))['ok']
    assert sim.start_nose_lift.call_args.args[0]==[6,7]

def test_queued_command_cannot_reach_hardware_after_mode_change():
    fn,sim=endpoint('hitl','/api/px4/command')
    assert asyncio.run(fn({'command':'arm','expected_mode':'sitl'})).status_code==409
