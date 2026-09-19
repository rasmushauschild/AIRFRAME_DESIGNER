import time
from types import SimpleNamespace
import pytest
from airframe_designer.px4.motor_test import MotorTest

def link():
    sent=[]
    return SimpleNamespace(mode="hitl", armed=False, ctl_connected=True,
        heartbeat_time=time.time(), send_command_long=lambda *args: sent.append(args), sent=sent)

def test_mapping_timeout_and_release():
    l=link(); m=MotorTest();m.send(l,9,.05,10)
    assert l.sent[-1] == (310,.05,.5,0,0,9)
    m.send(l,9,0,10)
    assert l.sent[-1] == (310,0,0,0,0,9)
    with pytest.raises(ValueError):m.send(l,10,.05,10)
    m.deadline=0;m.send(l,10,.05,10)

@pytest.mark.parametrize("motor,value", [(0,.1),(11,.1),(True,.1),(1,.16),(1,-.1),(1,float('nan'))])
def test_invalid_requests_send_nothing(motor,value):
    l=link()
    with pytest.raises(ValueError):MotorTest().send(l,motor,value,10)
    assert not l.sent

@pytest.mark.parametrize("field,value", [('armed',True),('mode','sitl'),('ctl_connected',False),('heartbeat_time',0)])
def test_connection_gates(field,value):
    l=link();setattr(l,field,value)
    with pytest.raises(ValueError):MotorTest().send(l,1,.05,10)
    assert not l.sent
