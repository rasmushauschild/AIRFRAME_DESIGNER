import time
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.px4.real_flight import flight_params, apply_verified, check_board


def model():
    return Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1,11)],
                              "px4_overrides": {"SYS_HITL": 1, "COM_ARM_WO_GPS": 1, "CBRK_IO_SAFETY": 22027}})


def board(params):
    return SimpleNamespace(mode="hitl", ctl_connected=True, heartbeat_time=time.time(), armed=False,
                           params={k:{"value":v} for k,v in params.items()},
                           set_param=Mock(return_value={"ok":True}), preflight_storage=Mock())


def test_real_export_excludes_simulator_bypasses():
    p=flight_params(model())
    assert p["SYS_HITL"] == 0 and "CBRK_IO_SAFETY" not in p
    assert "COM_ARM_WO_GPS" not in p
    assert not any(k.startswith("HIL_ACT") for k in p)
    assert p["PWM_MAIN_FUNC8"] == 108 and p["PWM_AUX_FUNC1"] == 109 and p["PWM_AUX_FUNC2"] == 110


def test_custom_wiring_preserves_geometry_and_front_fan_roles():
    af=model(); before=flight_params(af)
    af.px4_overrides.update({k:v for k,v in before.items() if k.startswith("PWM_")})
    af.px4_overrides.update(PWM_MAIN_FUNC1=110, PWM_AUX_FUNC2=101)
    after=flight_params(af)
    assert after["PWM_AUX_FUNC2"]==101
    assert {k:v for k,v in before.items() if k.startswith(("CA_","NLF_"))} == {k:v for k,v in after.items() if k.startswith(("CA_","NLF_"))}


def test_duplicate_wiring_rejected():
    af=model(); af.px4_overrides["PWM_MAIN_FUNC1"]=101
    with pytest.raises(ValueError, match="exactly one"):
        flight_params(af)


def test_missing_aux_fails_before_any_write():
    p=flight_params(model()); link=board(p); del link.params["PWM_AUX_FUNC2"]
    with pytest.raises(ValueError, match="PWM_AUX_FUNC2"):
        apply_verified(link,p)
    link.set_param.assert_not_called()


def test_failed_write_never_enables_sequence():
    p=flight_params(model()); link=board(p); link.set_param.return_value={"ok":False}
    with pytest.raises(ValueError): apply_verified(link,p)
    assert not any(c.args==("NLF_HW_OK",1) for c in link.set_param.call_args_list)
    link.preflight_storage.assert_not_called()


def test_verified_configuration_enables_sequence_last():
    p=flight_params(model()); link=board(p)
    apply_verified(link,p)
    assert link.set_param.call_args_list[-1].args == ("NLF_HW_OK",1)
    link.preflight_storage.assert_called_once_with(True)


@pytest.mark.parametrize("field,value", [("armed",True),("ctl_connected",False),("heartbeat_time",0),("mode","sitl")])
def test_rejects_unusable_board(field,value):
    link=board({}); setattr(link,field,value)
    with pytest.raises(ValueError): check_board(link)


@pytest.mark.parametrize('method', ['send_hil_sensor','send_hil_gps','send_hil_state_quaternion'])
@pytest.mark.parametrize('mode,sys_hitl,blocked,expected', [('hitl',0,False,False),('hitl',None,False,False),('hitl',1,True,False),('hitl',1,False,True),('sitl',None,False,True)])
def test_simulator_sensor_gate(method,mode,sys_hitl,blocked,expected):
    import threading
    from airframe_designer.px4.link import PX4Link
    link=SimpleNamespace(mode=mode,params={'SYS_HITL':{'value':sys_hitl}},block_hil=blocked,
                         _write_lock=threading.Lock(),conn=SimpleNamespace(mav=Mock()))
    getattr(PX4Link,method)(link,{})
    assert bool(link.conn.mav.mock_calls) == expected


def test_staged_mode_extends_ground_disarm_window():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1, NLF_RC_THR=1)
    assert flight_params(af)["COM_DISARM_LAND"] >= 120
