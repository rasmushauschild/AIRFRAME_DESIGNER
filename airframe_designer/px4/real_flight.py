"""Explicit conversion from HIL to onboard sensing; never arms the aircraft."""
import time


def flight_params(airframe):
    source = airframe.px4_params(hitl=False)
    # Copy aircraft/controller settings only. Do not carry simulator arming,
    # RC, battery, GPS or failsafe overrides onto a real aircraft.
    # The sensor geometry does carry over: where the IMU and the H-flow sit, and which way they face, are
    # facts about the aircraft, not about the simulation.
    allowed = ("CA_", "NLF_", "MC_", "MPC_", "EKF2_IMU_POS_", "EKF2_OF_", "EKF2_RNG_", "SENS_FLOW_", "SENS_BOARD_")
    params = {k: v for k, v in source.items() if k.startswith(allowed) or k in
              ("THR_MDL_FAC", "COM_DISARM_LAND", "COM_DISARM_PRFLT", "MAN_OVERRIDE_SPD")}
    rotors = airframe.active_rotors()
    if len(rotors) != 10:
        raise ValueError("Real-flight setup currently requires the ten-motor ATLAS layout")
    keys = [f"PWM_MAIN_FUNC{i}" for i in range(1, 9)] + ["PWM_AUX_FUNC1", "PWM_AUX_FUNC2"]
    overrides = airframe.px4_overrides or {}
    saved = any(k in overrides for k in keys)
    mapping = {k: int(overrides.get(k, 0)) if saved else 101+i for i, k in enumerate(keys)}
    if sorted(mapping.values()) != list(range(101, 111)):
        raise ValueError("Assign each of the ten motors to exactly one PWM port in Geometry before real-flight setup")
    params.update(mapping)
    params.update(SYS_HITL=0, NLF_HW_OK=1, CBRK_SUPPLY_CHK=0)
    # Safety-button availability belongs to the physical board configuration.
    # Do not overwrite it from simulator/model settings.
    return params


def check_board(link):
    if link is None or link.mode != "hitl" or not link.ctl_connected or time.time()-link.heartbeat_time > 3:
        raise ValueError("Connect the Pixhawk and wait for a fresh heartbeat")
    if link.armed:
        raise ValueError("Disarm the aircraft before preparing real flight")


def apply_verified(link, params):
    check_board(link)
    missing = [k for k in params if k not in link.params and not k.startswith("NLF_")]
    if missing:
        raise ValueError("Required real-flight parameters are unavailable: " + ", ".join(missing))
    # No second physical output may remain assigned to one of these motors.
    params = dict(params)
    for k, entry in link.params.items():
        if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) and k not in params and 101 <= entry["value"] <= 110:
            params[k] = 0
    for k, v in params.items():
        if k == "NLF_HW_OK":
            continue  # Enable the hardware sequence only after the whole configuration succeeded.
        check_board(link)
        r = link.set_param(k, v)
        if not r.get("ok"):
            raise ValueError(f"Could not write {k}: {r.get('error', 'no acknowledgement')}")
    check_board(link)
    r = link.set_param("NLF_HW_OK", 1)
    if not r.get("ok"):
        raise ValueError("Could not enable the native ATLAS sequence")
    link.preflight_storage(True)
    return params
