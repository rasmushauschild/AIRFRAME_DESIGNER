import copy
from pathlib import Path
import pytest
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.geometry.ground_sequence import ground_sequence_params

def model():
    return Airframe.load(Path(__file__).parents[1]/"firmware/atlas/atlas_07d.original.json")

def test_mass_and_cg_change_coefficients():
    af=model(); a=ground_sequence_params(af)
    assert a["NLF_CFG_OK"] == 1
    af.mass.mass *= 2; af.mass.cg[0] += .02
    b=ground_sequence_params(af)
    assert b["NLF_MASS"] == 2*a["NLF_MASS"]
    assert b["NLF_GX"] == pytest.approx(a["NLF_GX"]+.02)
    assert b["NLF_W9"] != a["NLF_W9"]

def test_changed_front_thrust_changes_pitch_authority():
    af=model(); a=ground_sequence_params(af)
    af.rotors[8].max_thrust *= 1.1
    assert ground_sequence_params(af)["NLF_MOM"] != a["NLF_MOM"]

def test_unsupported_model_is_not_approved():
    af=model(); af.rotors.pop()
    assert ground_sequence_params(af)["NLF_CFG_OK"] == 0

def test_native_takeoff_uses_gentle_thrust_ramp():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1, MPC_TKO_RAMP_T=.2)
    assert af.px4_params_sitl()["MPC_TKO_RAMP_T"] == 3
    af.px4_overrides["MPC_TKO_RAMP_T"] = 5
    assert af.px4_params_sitl()["MPC_TKO_RAMP_T"] == 5


def test_native_sequence_backs_off_auto_disarm_timers():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1)
    p=af.px4_params_sitl()
    assert p["COM_DISARM_LAND"] == 60 and p["COM_DISARM_PRFLT"] == 40 and p["THR_MDL_FAC"] == 0 and p["COM_RC_OVERRIDE"] == 3
    af.px4_overrides.update(COM_DISARM_LAND=90)
    assert af.px4_params_sitl()["COM_DISARM_LAND"] == 90
    af.px4_overrides["NLF_ENABLE"] = 0
    assert "COM_DISARM_LAND" not in af.px4_params_sitl() or af.px4_params_sitl()["COM_DISARM_LAND"] != 60


def test_lift_target_defaults_to_hover_pitch():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1); af.hover_pitch_deg = 15
    assert af.px4_params_sitl()["NLF_TARGET"] == 15
    af.px4_overrides["NLF_TARGET"] = 22
    assert af.px4_params_sitl()["NLF_TARGET"] == 22
