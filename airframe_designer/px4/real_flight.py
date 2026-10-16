"""Explicit conversion from HIL to onboard sensing; never arms the aircraft."""
import time

MAIN_OUTPUTS = 8          # PWM_MAIN_FUNC1..8, then PWM_AUX_FUNC1..


def output_keys(n: int) -> list[str]:
    """The ports n motors are wired to by default: the first eight on MAIN, the rest on AUX.

    Nothing here is ATLAS's ten: a design with nine rotors is nine motors on nine ports, and the check
    below - every motor on exactly one port, no port left over - is what keeps that honest.
    """
    if n < 1:
        raise ValueError("This design has no enabled rotors to wire")
    aux = n - MAIN_OUTPUTS
    if aux > 8:
        raise ValueError(f"{n} motors is more outputs than a flight controller's MAIN + AUX ports")
    return ([f"PWM_MAIN_FUNC{i}" for i in range(1, min(n, MAIN_OUTPUTS) + 1)]
            + [f"PWM_AUX_FUNC{i}" for i in range(1, max(0, aux) + 1)])


# What a parameter has to be about to cross from the simulator onto an aircraft: the airframe itself, its
# controller, and where its sensors sit and what they are allowed to fuse. Everything else in the export
# exists because the simulated aircraft has no battery, no radio, no safety switch and a GPS that always
# works - and on the real machine those are the checks that stop it hurting someone.
CARRIES_OVER = ("CA_", "NLF_", "MC_", "MPC_", "EKF2_IMU_POS_", "EKF2_OF_", "EKF2_RNG_", "SENS_FLOW_", "SENS_BOARD_")
# EKF2_HGT_REF and EKF2_GPS_CTRL say which sensors this aircraft actually has to fly on - a fact about the
# machine, like where its IMU sits, not a simulator convenience - so they cross over too.
CARRIES_OVER_NAMES = frozenset({"THR_MDL_FAC", "COM_DISARM_LAND", "COM_DISARM_PRFLT", "MAN_OVERRIDE_SPD",
                                "EKF2_HGT_REF", "EKF2_GPS_CTRL"})


def carries_over(name: str) -> bool:
    """True if this parameter is written to a real aircraft as well as to the simulator."""
    return name.startswith(CARRIES_OVER) or name in CARRIES_OVER_NAMES


def flight_note(name: str) -> str:
    """Why a parameter in the export does not reach a real aircraft, in a few words. "" if it does."""
    if carries_over(name):
        return ""
    if name.startswith(("HIL_", "SYS_HITL")):
        return "the simulator's own: a real board flies on its sensors, not on these"
    if name.startswith("PWM_"):
        return "the outputs are wired from the motor count when flying for real"
    if name.startswith("CBRK_") or name.startswith("COM_ARM") or name.startswith("COM_PREARM"):
        return "a check the simulator bypasses and a real aircraft needs"
    if name.startswith(("BAT", "RC", "COM_RC", "NAV_DLL", "NAV_RCL", "GPS_", "SDLOG", "MAV_", "SER_")):
        return "the board's own: battery, radio, links and failsafes belong to the hardware"
    return "not an aircraft setting: it stays in the simulator"


def flight_params(airframe):
    source = airframe.px4_params(hitl=False)
    params = {k: v for k, v in source.items() if carries_over(k)}
    n = len(airframe.active_rotors())
    keys = output_keys(n)
    overrides = airframe.px4_overrides or {}
    saved = any(k in overrides for k in keys)
    mapping = {k: int(overrides.get(k, 0)) if saved else 101 + i for i, k in enumerate(keys)}
    if sorted(mapping.values()) != list(range(101, 101 + n)):
        raise ValueError(f"Assign each of the {n} motors to exactly one PWM port in Geometry before real-flight setup")
    params.update(mapping)
    params.update(SYS_HITL=0, CBRK_SUPPLY_CHK=0)
    # The interlock belongs to the nose-lift module. A design that has switched the module off is not
    # flying its sequence, and must not be arming it on the aircraft either - nor writing anything else
    # of that module's, which `px4_params` has already dropped but this function adds to afterwards.
    off = airframe.modules_off_params()
    if "NLF_HW_OK" not in off:
        params["NLF_HW_OK"] = 1
    for name in off:
        params.pop(name, None)
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
    motors = [v for k, v in params.items() if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC"))]
    highest = max(motors) if motors else 110
    for k, entry in link.params.items():
        if k.startswith(("PWM_MAIN_FUNC", "PWM_AUX_FUNC")) and k not in params and 101 <= entry["value"] <= highest:
            params[k] = 0
    for k, v in params.items():
        if k == "NLF_HW_OK":
            continue  # Enable the hardware sequence only after the whole configuration succeeded.
        check_board(link)
        r = link.set_param(k, v)
        if not r.get("ok"):
            raise ValueError(f"Could not write {k}: {r.get('error', 'no acknowledgement')}")
    if "NLF_HW_OK" in params:          # only when the module is part of this design
        check_board(link)
        r = link.set_param("NLF_HW_OK", 1)
        if not r.get("ok"):
            raise ValueError("Could not enable the native ATLAS sequence")
    link.preflight_storage(True)
    return params
