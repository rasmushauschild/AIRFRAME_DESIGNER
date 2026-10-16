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
    af=model(); af.px4_overrides.update(NLF_ENABLE=1)
    assert af.px4_params_sitl()["MPC_TKO_RAMP_T"] >= 0.8      # a short ramp pitches it over on its legs
    af.px4_overrides["MPC_TKO_RAMP_T"] = 5
    assert af.px4_params_sitl()["MPC_TKO_RAMP_T"] == 5
    # set below what the sequence needs and it is still written - and said out loud, not quietly fixed
    af.px4_overrides["MPC_TKO_RAMP_T"] = .2
    assert af.px4_params_sitl()["MPC_TKO_RAMP_T"] == .2
    assert any("MPC_TKO_RAMP_T" in w and "0.8" in w for w in af.override_warnings())
    assert any("MPC_TKO_RAMP_T" in p for p in af.validate())


def test_native_sequence_backs_off_auto_disarm_timers():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1)
    p=af.px4_params_sitl()
    assert p["COM_DISARM_LAND"] == 60 and p["COM_DISARM_PRFLT"] == 40 and p["THR_MDL_FAC"] == 0 and p["MAN_OVERRIDE_SPD"] == 1.0
    af.px4_overrides.update(COM_DISARM_LAND=90)
    assert af.px4_params_sitl()["COM_DISARM_LAND"] == 90
    af.px4_overrides["NLF_ENABLE"] = 0
    assert "COM_DISARM_LAND" not in af.px4_params_sitl() or af.px4_params_sitl()["COM_DISARM_LAND"] != 60


def test_lift_target_defaults_to_hover_pitch():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1); af.hover_pitch_deg = 15
    assert af.px4_params_sitl()["NLF_TARGET"] == 5            # 10 deg below hover by default
    af.px4_overrides["NLF_TARGET"] = 22
    assert af.px4_params_sitl()["NLF_TARGET"] == 22


def test_pilot_handover_keeps_px4_from_auto_disarming_and_presets_hover_thrust():
    af=model(); af.px4_overrides.update(NLF_ENABLE=1, NLF_PILOT=1)
    p=af.px4_params_sitl()
    assert p["NLF_PILOT"] == 1 and p["COM_DISARM_LAND"] == 120 and p["COM_DISARM_PRFLT"] == 120
    af.px4_overrides["MC_AIRMODE"] = 1
    # airmode cannot coexist with sitting nose-down under a manual mode. You may still set it; the app
    # tells you what the sequence needs rather than overruling you.
    assert af.px4_params_sitl()["MC_AIRMODE"] == 1
    assert any("MC_AIRMODE" in w for w in af.override_warnings())
    assert 0.2 <= p["MPC_THR_HOVER"] <= 0.8
    af.px4_overrides.update(MPC_THR_HOVER=0.6)
    assert af.px4_params_sitl()["MPC_THR_HOVER"] == 0.6
    af.px4_overrides["NLF_PILOT"] = 0; af.px4_overrides.pop("MPC_THR_HOVER")
    p=af.px4_params_sitl()
    assert p["COM_DISARM_LAND"] == 60 and "MPC_THR_HOVER" not in p


def test_hover_pattern_is_exported_for_the_pilot_handover():
    af=model(); p=ground_sequence_params(af)
    assert p["NLF_CFG_OK"] == 1
    for i in range(1, 9):
        assert 0 < p[f"NLF_U{i}"] < 1
    assert 0 < p["NLF_UF"] < 1


def test_a_module_switched_off_configures_nothing():
    """A firmware module this design does not fly with has no business setting parameters on it: its
    names leave the export, and with them the Parameters table. The values stay in the design, so
    switching it back on brings them back as they were."""
    af = model()
    af.px4_overrides.update(NLF_ENABLE=1, NLF_LAND_ANG=-16.16)
    on = af.px4_params_sitl()
    assert on["NLF_ENABLE"] == 1 and on["COM_DISARM_LAND"] >= 60, "the sequence sets its backups"

    af.design["modules_off"] = {"src/atlas_nose_lift": ["NLF_ENABLE", "NLF_LAND_ANG", "NLF_TARGET",
                                                        "NLF_DWELL", "NLF_CFG_OK", "NLF_HW_OK"]}
    off = af.px4_params_sitl()
    assert not [k for k in off if k.startswith("NLF_")], "not one of its parameters is written"
    assert "COM_DISARM_LAND" not in off, "and the sequence's own adjustments do not happen either"
    assert off["CA_ROTOR_COUNT"] == on["CA_ROTOR_COUNT"], "the aircraft is otherwise unchanged"
    assert af.px4_overrides["NLF_LAND_ANG"] == -16.16, "your value is kept, not deleted"

    del af.design["modules_off"]
    assert af.px4_params_sitl()["NLF_LAND_ANG"] == -16.16, "and it comes back with the module"
