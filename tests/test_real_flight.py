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
    with pytest.raises(ValueError, match="exactly one output port"):
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


def test_a_module_switched_off_takes_its_interlock_with_it():
    """A firmware that is not built with the nose-lift module has no NLF_HW_OK to write.

    The export filters a disabled module's parameters, but the interlock is added afterwards, by name,
    so it had to learn the same rule - and it must still go out, last, when the module is in.
    """
    af = model()
    assert flight_params(af)["NLF_HW_OK"] == 1
    af.design["modules_off"] = {"src/atlas_nose_lift": ["NLF_HW_OK", "NLF_ENABLE", "NLF_TARGET"]}
    p = flight_params(af)
    assert not any(k.startswith("NLF_") for k in p)
    link = board(p)
    apply_verified(link, p)
    assert not any(c.args[0] == "NLF_HW_OK" for c in link.set_param.call_args_list)
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
    # no heartbeat here: this is the parameter fallback, which is what a link that has not heard the
    # board yet has to go on. The heartbeat path is in tests/test_mode_switch.py.
    link=SimpleNamespace(mode=mode,params={'SYS_HITL':{'value':sys_hitl}},block_hil=blocked,
                         heartbeat_time=0.0,hil_enabled=False,
                         _write_lock=threading.Lock(),conn=SimpleNamespace(mav=Mock()))
    link.listening=PX4Link.listening.fget(link)
    getattr(PX4Link,method)(link,{})
    assert bool(link.conn.mav.mock_calls) == expected


def test_staged_mode_extends_ground_disarm_window():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1, NLF_RC_THR=1)
    assert flight_params(af)["COM_DISARM_LAND"] >= 120


def telemetry_asks(baud, mode="hitl", seen=()):
    """What request_flight_telemetry() would ask the aircraft for, as {message id: Hz} (0 = switched off)."""
    from airframe_designer.px4.link import PX4Link, mavlink
    sent = []
    link = PX4Link(mode, "/dev/null" if mode == "hitl" else "0.0.0.0:4560", baud=baud, log=lambda *_: None)

    def answer(cmd, *a):          # nothing is opened; the aircraft acknowledges at once, as PX4 does
        sent.append((cmd, a))
        link.last_ack = {"command": cmd, "result": 0, "t": time.time()}
    link.send_command_long = answer
    link.rx_types = {name: 10 for name in seen}      # what has actually been arriving on this link
    link.request_flight_telemetry()
    out = {}
    for cmd, a in sent:
        assert cmd == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        out[int(a[0])] = 0.0 if a[1] < 0 else round(1e6 / a[1], 2)
    return out


def test_a_radio_is_not_asked_for_more_than_it_can_carry():
    """57600 baud is about 5.7 kB/s on the wire and less in the air. Asking for the cable's streams does
    not make them arrive - it makes them arrive late, which reads as an aircraft that has stopped talking."""
    from airframe_designer.px4.link import PX4Link
    SIZES = PX4Link.MSG_BYTES
    cable = telemetry_asks(921600)
    radio = {m: hz for m, hz in telemetry_asks(57600).items() if m in SIZES}
    rate = lambda asks: sum(SIZES[m] * hz for m, hz in asks.items())                       # noqa: E731
    assert rate(cable) > 5760, "the cable's rates are the generous ones"
    # Full, the default, is 45% of the wire; half of it is the line past which a SiK link stops keeping up
    assert rate(radio) < 0.5 * 5760, f"{rate(radio):.0f} B/s will not fit down a 57600 link"
    assert radio[31] >= 10 and radio[32] >= 10, "attitude and position still have to feed the 3D view"
    assert radio[30] == 0, "ATTITUDE is dropped; the quaternion beside it says the same thing"
    # and the streams nobody reads are switched off by name, which is a third of a radio's traffic
    off = telemetry_asks(57600, seen=("RC_CHANNELS", "MISSION_CURRENT", "SERVO_OUTPUT_RAW"))
    assert off[65] == 0 and off[42] == 0 and off[36] == 0, "RC channels, mission status and servo output"
    assert 242 not in off, "a stream that is not arriving costs a round trip to switch off and saves nothing"
    assert 65 not in telemetry_asks(921600, seen=("RC_CHANNELS",)), "a cable is not worth the round trips"


def test_the_cable_is_still_asked_for_everything():
    from airframe_designer.px4.link import PX4Link
    cable = telemetry_asks(921600)
    assert cable == {m: float(hz) for m, hz in PX4Link.FLIGHT_RATES_HZ.items()}
    assert telemetry_asks(None, mode="sitl") == cable, "SITL is not a slow link"
    assert 375 in cable and 36 not in cable, "the motor columns are fed by ACTUATOR_OUTPUT_STATUS"


def test_the_port_decides_the_speed_when_nobody_says():
    """921600 is a Pixhawk's USB number. Opening a telemetry radio at it returns not one byte, which the
    app can only report as "no link" - so a port that is not a flight controller opens at 57600."""
    import time as _time
    from airframe_designer.px4.connection import ConnectionManager
    conn = ConnectionManager.__new__(ConnectionManager)          # no link, no watcher thread
    conn.args = SimpleNamespace(baud=921600)
    conn._ports_cache = (_time.time(), [{"device": "/dev/cu.usbserial-DU0E76IN", "likely_px4": False},
                                        {"device": "/dev/cu.usbmodem01", "likely_px4": True}])
    assert conn.port_baud("/dev/cu.usbserial-DU0E76IN") == 57600
    assert conn.port_baud("/dev/cu.usbmodem01") == 921600
    assert conn.port_baud("/dev/cu.unknown") == 57600            # not a known Pixhawk: treat it as a radio


def test_the_radio_talking_about_itself_is_not_the_aircraft():
    """A SiK radio sends RADIO_STATUS with nothing on the far end. Treating that as the vehicle lights the
    link green over an empty sky, and makes every command go to the radio's own system id."""
    from airframe_designer.px4.link import PX4Link
    link = PX4Link("hitl", "/dev/null", baud=57600, log=lambda *_: None)
    status = SimpleNamespace(rssi=120, remrssi=0, noise=40, txbuf=100)
    assert link._own_radio(status, "RADIO_STATUS") is True
    assert link.radio["rssi"] == 120 and link.radio["remrssi"] == 0
    assert not link.ctl_connected and link.target_system == 1, "no aircraft was found, so nothing moved"
    assert link._own_radio(SimpleNamespace(), "HEARTBEAT") is False
    assert link.status()["radio"]["age"] < 1


def test_real_flight_wiring_follows_the_motor_count():
    """It was pinned to ATLAS's ten, so a nine-rotor design could not even be looked at in real-flight
    mode: /api/px4/export went through here on every poll and raised."""
    from airframe_designer.px4.real_flight import output_keys
    assert output_keys(4) == [f"PWM_MAIN_FUNC{i}" for i in range(1, 5)]
    assert output_keys(9)[-1] == "PWM_AUX_FUNC1" and len(output_keys(9)) == 9
    assert output_keys(10)[-2:] == ["PWM_AUX_FUNC1", "PWM_AUX_FUNC2"]
    with pytest.raises(ValueError):
        output_keys(0)
    with pytest.raises(ValueError, match="MAIN \\+ AUX"):
        output_keys(17)

    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 10)]})
    p = flight_params(af)
    assert [p[k] for k in output_keys(9)] == list(range(101, 110))
    assert p["SYS_HITL"] == 0 and "PWM_AUX_FUNC2" not in p


def test_a_design_that_cannot_be_wired_is_a_sentence_not_a_500(tmp_path):
    """Two motors on one port is a real mistake, and the answer has to be readable."""
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 5)],
                             "px4_overrides": {"PWM_MAIN_FUNC1": 101, "PWM_MAIN_FUNC2": 101,
                                               "PWM_MAIN_FUNC3": 103, "PWM_MAIN_FUNC4": 104}})
    with pytest.raises(ValueError, match="exactly one output port"):
        flight_params(af)


def test_a_nan_never_takes_a_whole_response_with_it():
    from airframe_designer.server.app import SafeJSON
    body = SafeJSON({"rate": float("nan"), "deep": [1.0, float("inf"), {"x": float("-inf")}]}).body
    import json as _json
    assert _json.loads(body) == {"rate": None, "deep": [1.0, None, {"x": None}]}


def test_an_aircraft_that_has_not_located_itself_does_not_move_the_model():
    """PX4 streams its local position whether or not the estimate is valid, and an invalid one is NaN.
    Taken literally it loses the aircraft from the 3D view and makes every websocket frame unparseable."""
    import numpy as np
    from airframe_designer.sim.simulator import Simulator
    nan3 = {"pos": [float("nan")] * 3, "vel": [0.0, float("nan"), 0.0], "q": [1.0, 0.0, 0.0, float("nan")],
            "rates": [0.0, 0.0, 0.0]}
    assert Simulator._reported(nan3, "pos") is None
    assert Simulator._reported(nan3, "vel") is None
    assert Simulator._reported(nan3, "q") is None
    assert np.allclose(Simulator._reported(nan3, "rates"), [0, 0, 0])
    assert Simulator._reported({}, "pos") is None
    assert np.allclose(Simulator._reported({"pos": [1.0, 2.0, -3.0]}, "pos"), [1, 2, -3])


def test_the_export_endpoint_answers_in_words_when_the_design_cannot_be_wired():
    """What the user saw: FLIGHT on a nine-rotor design, and the Flight controller section just said
    "Internal Server Error", because this endpoint is polled and raised every time."""
    import asyncio, threading
    from airframe_designer.server.app import build_app
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 5)],
                             "px4_overrides": {"PWM_MAIN_FUNC1": 101, "PWM_MAIN_FUNC2": 101,
                                               "PWM_MAIN_FUNC3": 103, "PWM_MAIN_FUNC4": 104}})
    sim = SimpleNamespace(lock=threading.RLock(), hooks=[], t=0.0, paused=False, airframe=af,
                          reset=Mock())
    link = SimpleNamespace(mode="hitl", ctl_connected=True, params={"SYS_HITL": {"value": 0}},
                           status=lambda: {"armed": False})
    state = SimpleNamespace(simulator=sim, conn=SimpleNamespace(mode="hitl", reset_all=lambda: {"steps": []}),
                            link=link, log=Mock())
    routes = {r.path: r.endpoint for r in build_app(state).routes if hasattr(r, "endpoint")}
    r = asyncio.run(routes["/api/px4/export"]())
    assert r.status_code == 409
    assert b"exactly one output port" in r.body


def link_at(baud, share="balanced"):
    from airframe_designer.px4.link import PX4Link
    link = PX4Link("hitl", "/dev/null", baud=baud, log=lambda *_: None)
    link.share = share
    return link


def cost_of(link, rates):
    return sum(link.MSG_BYTES[m] * hz for m, hz in rates.items())


def test_the_rates_fit_the_link_whatever_it_is():
    """Every choice, on every link, has to fit inside the budget it was given - that is the whole point of
    asking for less than the cable's streams."""
    for baud in (19200, 57600, 115200):
        for share in ("light", "balanced", "full"):
            link = link_at(baud, share)
            rates = link.flight_rates()
            assert cost_of(link, rates) <= link.link_budget_bps() + 1e-6, (baud, share)
            assert rates[30] == 0, "ATTITUDE is the quaternion's duplicate and never worth the bytes"


def test_more_share_and_more_baud_both_buy_resolution():
    """What the user asks for: a dial that visibly raises the sample rate, and a faster radio that does
    the same without touching the dial."""
    pose = lambda link: link.flight_rates()[32]                          # noqa: E731
    assert pose(link_at(57600, "light")) < pose(link_at(57600, "balanced")) < pose(link_at(57600, "full"))
    assert pose(link_at(115200, "balanced")) > pose(link_at(57600, "balanced"))
    assert pose(link_at(57600, "balanced")) >= 10, "the 3D view needs a pose about ten times a second"


def test_position_and_attitude_are_the_last_things_given_up():
    """On a link too narrow for the floors, what survives is where the aircraft is and which way it faces."""
    link = link_at(9600, "light")
    rates = link.flight_rates()
    assert rates[32] > 0 and rates[31] > 0
    assert cost_of(link, rates) <= link.link_budget_bps() + 1e-6
    assert all(rates[m] == 0 for m in (105, 291, 147, 24)), "the expensive extras are gone first"


def test_a_cable_still_gets_everything():
    link = link_at(921600)
    assert link.link_budget_bps() is None
    assert link.flight_rates() == {m: float(hz) for m, hz in link.FLIGHT_RATES_HZ.items()} or \
           link.flight_rates() == link.FLIGHT_RATES_HZ


def test_a_stream_request_that_is_not_acknowledged_is_sent_again():
    """The one that mattered: twenty fire-and-forget commands into a radio, most dropped, and the streams
    keep whatever rate PX4 booted with - the link full of RC channels while the pose arrives twice a
    second. PX4 acknowledges every SET_MESSAGE_INTERVAL, so an unanswered one is sent again."""
    from airframe_designer.px4.link import PX4Link, mavlink
    link = PX4Link("hitl", "/dev/null", baud=57600, log=lambda *_: None)
    tries = []

    def flaky(cmd, *a):
        tries.append(a[0])
        if len(tries) >= 3:                      # the third attempt gets through
            link.last_ack = {"command": cmd, "result": 0, "t": time.time()}
    link.send_command_long = flaky
    assert link.set_message_interval(32, 10.0, timeout=0.05, retries=3) is True
    assert tries == [32.0, 32.0, 32.0], "it kept asking until the aircraft answered"

    tries.clear()
    link.last_ack = {}
    link.send_command_long = lambda cmd, *a: tries.append(a[0])      # never answers
    assert link.set_message_interval(32, 10.0, timeout=0.05, retries=2) is False
    assert len(tries) == 2


def test_setting_something_on_the_link_reaches_the_link():
    """The server holds the link through a proxy so it always sees the current one. Reading went through
    it and writing did not, so `link.share = "full"` landed on the proxy, was shadowed there for good, and
    the dial in the UI did nothing at all."""
    import asyncio, threading
    from airframe_designer.server.app import build_app
    real = SimpleNamespace(share="balanced", mode="hitl", ctl_connected=False, listening=True, params={},
                           SHARES={"light": 0.1, "balanced": 0.3, "full": 0.5},
                           status=lambda: {"telemetry": {"share": real.share}})
    sim = SimpleNamespace(lock=threading.RLock(), hooks=[], t=0.0, paused=False, airframe=None,
                          reset=Mock())
    state = SimpleNamespace(simulator=sim, conn=SimpleNamespace(mode="hitl", reset_all=lambda: {"steps": []}),
                            link=real, log=Mock())
    routes = {r.path: r.endpoint for r in build_app(state).routes if hasattr(r, "endpoint")}
    r = asyncio.run(routes["/api/connection/telemetry"]({"share": "full"}))
    assert real.share == "full", "the dial never reached the link"
    assert r["telemetry"]["share"] == "full"
    bad = asyncio.run(routes["/api/connection/telemetry"]({"share": "enormous"}))
    assert bad.status_code == 400 and real.share == "full"


def test_update_px4_writes_the_design_and_then_everything_else():
    """One button, one promise: the board ends up matching the simulator. The design's own parameters
    first, then every other one that differs from the SITL reference - and never the aircraft's own truth
    (calibration, RC, serial links, battery, safety, failsafes), which a laptop has no business setting."""
    import asyncio, threading
    from airframe_designer.server.app import build_app

    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 5)]})
    written: list = []
    have = {k: {"value": 0.0, "type": 9} for k in af.px4_params(hitl=True)}      # the firmware has them all
    have["SYS_HITL"] = {"value": 1, "type": 6}          # a board flying the simulation, not a real flight
    link = SimpleNamespace(mode="hitl", ctl_connected=True, armed=False, param_count=len(have),
                           params=have,
                           status=lambda: {"armed": False, "params_loaded": len(have)},
                           listening=False, link_budget_bps=lambda: None,
                           set_params=lambda vals, cb=None, timeout=2.0: [
                               (written.append(k), {"name": k, "ok": True})[1] for k in vals],
                           preflight_storage=Mock(), quiet_streams=Mock(), request_flight_telemetry=Mock())
    conn = SimpleNamespace(mode="hitl", reset_all=lambda: {"steps": []},
                           # a board running firmware, not sitting in its loader
                           detected_board=lambda: {"device": "/dev/x", "target": "px4_fmu-v6x", "bootloader": False},
                           sitl_diff=lambda: {"ok": True, "differ": [{"name": "MPC_XY_P", "board": 1, "sitl": 2}],
                                              "skipped": 7, "compared": 900},
                           sync_to_sitl=lambda: {"ok": True, "written": 1, "failed": [], "skipped": 7, "seconds": 0.2},
                           board_firmware_status=lambda: {"needs_rebuild": False, "needs_flash": False},
                           update_firmware=Mock())
    sim = SimpleNamespace(lock=threading.RLock(), hooks=[], t=0.0, paused=False, airframe=af,
                          reset=Mock(), recorder=SimpleNamespace(sample_hz=20.0))
    state = SimpleNamespace(simulator=sim, conn=conn, link=link, log=Mock(), export_log=[])
    routes = {r.path: r.endpoint for r in build_app(state).routes if hasattr(r, "endpoint")}
    r = asyncio.run(routes["/api/px4/push"]({"save": True, "firmware": False}))
    assert r["ok"] and written, "the design's parameters went out"
    assert r["rest"] == {"written": 1, "skipped": 7, "why": "", "failed": []}, "and then the rest of SITL"

    # with the board's list not downloaded there is nothing to compare, and that is said rather than guessed
    conn.sitl_diff = lambda: {"ok": False, "error": "board parameters not fully downloaded yet"}
    r2 = asyncio.run(routes["/api/px4/push"]({"save": True, "firmware": False}))
    assert r2["ok"], "the design still goes out"
    assert "not fully downloaded" in r2["rest"]["why"]


def test_every_exported_parameter_says_whether_it_reaches_a_real_aircraft():
    """The question behind the 'In flight' column: which of these will be different when I fly it?"""
    from airframe_designer.px4.real_flight import carries_over, flight_note
    af = model()
    sitl = af.px4_params_sitl()
    flight = flight_params(af)
    # Three kinds, and the table shows which is which: carried over unchanged; written out there by a
    # different rule (the HIL switch, the outputs, which come from the wiring rather than from a
    # simulated port order); or left behind, with a reason.
    for name in sitl:
        if carries_over(name):
            assert flight_note(name) == "", f"{name} crosses over, so it needs no excuse"
            assert flight.get(name) == sitl[name], f"{name} crosses over unchanged"
        else:
            assert flight_note(name), f"{name} does not cross over and must say why"
    assert sitl["SYS_HITL"] == 1 and flight["SYS_HITL"] == 0
    assert flight["PWM_MAIN_FUNC1"] == 101 and "HIL_ACT_FUNC1" not in flight
    assert flight["NLF_HW_OK"] == 1, "the interlock exists only on a real board"


def test_the_aircrafts_own_cap_is_the_ceiling():
    """PX4 will not put more than MAV_<n>_RATE on a port, whatever is asked of it, and what it drops are
    the big messages - so a board capped at 1200 B/s answers with the cheap streams and never sends the
    pose at all. The budget has to know about the cap, or the two biggest things we want are the two we
    never get."""
    from airframe_designer.px4.link import PX4Link
    link = PX4Link("hitl", "/dev/null", baud=57600, log=lambda *_: None)
    link.share = "full"
    free = link.link_budget_bps()
    link._mav_rate = 1200.0
    assert link.link_budget_bps() == 1200.0 < free, "the aircraft has the last word"
    rates = link.flight_rates()
    cost = sum(link.MSG_BYTES[m] * hz for m, hz in rates.items())
    assert cost <= 1200.0, f"{cost:.0f} B/s was asked of a board that sends 1200"
    assert rates[32] >= 5 and rates[31] == rates[32], "and the pose still gets its share of what there is"

    link._mav_rate = 0.0                 # not told: the link's own figure stands
    assert link.link_budget_bps() == free


def test_parameters_go_out_in_one_stream_and_the_refused_ones_learn_their_type():
    """A hundred parameters one round trip at a time is a minute over a radio. They go out as a stream
    and the echoes are collected; whatever PX4 refused - almost always because an integral value was sent
    as an int for a parameter the board holds as a float - is asked about and sent again."""
    import threading
    from airframe_designer.px4.link import PX4Link
    link = PX4Link("hitl", "/dev/null", baud=57600, log=lambda *_: None)
    link.ctl, link._ctl_lock = Mock(), threading.Lock()
    link.target_system = link.target_component = 1
    link.params = {}

    sent: list = []
    ON_BOARD = {"MC_PITCHRATE_P": 9, "MPC_MAN_TILT_MAX": 9}      # both are floats on the aircraft

    def board(sysid, compid, name, value, ptype):
        n = name.decode().rstrip("\x00")
        sent.append((n, ptype))
        if ptype == ON_BOARD[n]:          # a type mismatch is refused in silence, as PX4 refuses it
            link._param_set_result[n] = {"value": value, "type": ptype}
    link.ctl.mav.param_set_send = board
    link.get_param = lambda name, timeout=2.0: {"value": 0.0, "type": ON_BOARD[name]}

    out = link.set_params({"MC_PITCHRATE_P": 0.9, "MPC_MAN_TILT_MAX": 12}, timeout=0.3, pace=0.0)
    assert all(r["ok"] for r in out), out
    tilt = [p for p in sent if p[0] == "MPC_MAN_TILT_MAX"]
    assert tilt[0][1] == 6 and tilt[-1][1] == 9, "guessed int first, learned float and sent it again"
    assert len([p for p in sent if p[0] == "MC_PITCHRATE_P"]) == 1, "the one that worked was sent once"


def test_a_parameter_this_firmware_has_not_got_says_so():
    import threading
    from airframe_designer.px4.link import PX4Link
    link = PX4Link("hitl", "/dev/null", baud=57600, log=lambda *_: None)
    link.ctl, link._ctl_lock = Mock(), threading.Lock()
    link.target_system = link.target_component = 1
    link.params = {}
    link.get_param = lambda name, timeout=2.0: None            # the board does not know it
    out = link.set_params({"NLF_HW_OK": 1}, timeout=0.2, pace=0.0)
    assert out[0]["ok"] is False and "no parameter of that name" in out[0]["error"]


def test_motors_may_live_on_any_output_the_board_has():
    """A design wired to AUX 3 and AUX 4 is a wiring, not a design with motors missing.

    `flight_params` used to read only `output_keys(n)` - the first n ports - so a motor on AUX 3 was
    invisible and the check raised "assign each motor to exactly one port" against a wiring that did
    exactly that. In the push that surfaced as Internal Server Error.
    """
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 10)]})
    af.px4_overrides.update({f"PWM_MAIN_FUNC{i}": 100 + i for i in range(1, 7)})
    af.px4_overrides.update(PWM_AUX_FUNC4=107, PWM_AUX_FUNC1=108, PWM_AUX_FUNC3=109)
    p = flight_params(af)
    assert p["PWM_AUX_FUNC4"] == 107 and p["PWM_AUX_FUNC1"] == 108 and p["PWM_AUX_FUNC3"] == 109
    assert "PWM_MAIN_FUNC7" not in p and "PWM_MAIN_FUNC8" not in p      # nothing is wired there


def test_a_motor_with_no_port_says_which_one():
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 10)]})
    af.px4_overrides.update({f"PWM_MAIN_FUNC{i}": 100 + i for i in range(1, 8)})   # motors 8 and 9 nowhere
    with pytest.raises(ValueError, match=r"motor\(s\) \[8, 9\] are not on a port"):
        flight_params(af)


def test_two_motors_on_one_port_says_so():
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 4)]})
    af.px4_overrides.update(PWM_MAIN_FUNC1=101, PWM_MAIN_FUNC2=102, PWM_MAIN_FUNC3=102)
    with pytest.raises(ValueError, match="share a port"):
        flight_params(af)


def test_an_esc_on_an_aux_output_is_driven_at_400_hz():
    """A motor on AUX sets every AUX timer to 400 Hz, because the rate belongs to the timer group.

    One timer drives several pins, and which pin sits on which group is board wiring this cannot know
    offline, so all of them are set rather than one guessed. A board with fewer timers has not got the
    extra parameters; the push names them instead of writing them.
    """
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 10)]})
    af.px4_overrides.update({f"PWM_MAIN_FUNC{i}": 100 + i for i in range(1, 7)})
    af.px4_overrides.update(PWM_AUX_FUNC4=107, PWM_AUX_FUNC1=108, PWM_AUX_FUNC3=109)
    p = flight_params(af)
    assert [p[f"PWM_AUX_TIM{t}"] for t in range(4)] == [400] * 4


def test_a_design_that_uses_no_aux_output_leaves_its_timers_alone():
    """Four motors live on MAIN 1-4; nothing about AUX is this design's business."""
    p = flight_params(Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 5)]}))
    assert not any("TIM" in k for k in p)


def test_the_main_timers_are_never_written():
    """MAIN carries the first eight motors on every design; a wrong rate there is a board-level mistake
    to see rather than one for the conversion to paper over."""
    af = Airframe.from_dict({"rotors": [{"name": f"M{i}"} for i in range(1, 11)]})
    assert not any(k.startswith("PWM_MAIN_TIM") for k in flight_params(af))


def test_pushing_a_wiring_takes_back_the_ports_it_no_longer_uses():
    """One motor on three outputs is what a re-map leaves behind if the push only sets what it uses.

    The board keeps whatever it was told last, so moving a motor from MAIN 7 to AUX 4 leaves MAIN 7 still
    pointing at it: PX4 then drives both pins, and a motor test spins two fans. Every other output still
    assigned to one of this design's motors is cleared; anything else on the board is left alone.
    """
    import asyncio
    from types import SimpleNamespace
    from airframe_designer.server import app as app_mod
    written = {}
    board = {"PWM_MAIN_FUNC7": {"value": 107}, "PWM_MAIN_FUNC8": {"value": 108},
             "PWM_AUX_FUNC3": {"value": 109}, "PWM_AUX_FUNC8": {"value": 109},
             "PWM_AUX_FUNC5": {"value": 201},                      # a servo: not ours, not touched
             "PWM_AUX_FUNC1": {"value": 109}, "PWM_AUX_FUNC4": {"value": 107}}
    design = {"PWM_AUX_FUNC1": 109, "PWM_AUX_FUNC4": 107, "PWM_MAIN_FUNC1": 101}
    # the clearing rule, lifted out of the endpoint exactly as it runs there
    mine = {int(v) for k, v in design.items() if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) and int(v)}
    stale = {k: 0 for k, e in board.items()
             if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) and k not in design and int(e["value"]) in mine}
    assert stale == {"PWM_MAIN_FUNC7": 0, "PWM_AUX_FUNC3": 0, "PWM_AUX_FUNC8": 0}
    assert "PWM_AUX_FUNC5" not in stale          # the servo keeps its function
    assert "PWM_MAIN_FUNC8" not in stale         # motor 8 is not in this design's set at all
