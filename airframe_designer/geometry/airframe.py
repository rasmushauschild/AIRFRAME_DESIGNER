"""The airframe: composition of the geometry segments, JSON I/O with schema migration, validation, and the
PX4 control-allocation export.

Frames
------
Structural frame: FRD with the origin at the airframe's reference point. The CG is ``mass.cg``. PX4 wants rotor
positions relative to the CG, in the *hover* frame (the structural frame pitched nose-up by ``hover_pitch_deg``,
which PX4 treats as level), so the export does both transformations.
"""
from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from .body import Body
from .frames import unit
from .gear import Leg, generate_legs
from .mass import MassProperties, estimate_inertia
from .propulsion import Rotor
from .cad import CadModel
from .wings import Wing, wing_panels

SCHEMA_VERSION = 2
MAX_PX4_ROTORS = 12  # CA_ROTOR0 .. CA_ROTOR11
from ..constants import G


def mount_rotation(angles) -> np.ndarray:
    """Rotation taking a mounted sensor's own axes into structural axes, from (roll, pitch, yaw) degrees.

    Yaw then pitch then roll, the composition PX4 uses for SENS_BOARD_*_OFF, so a board declared here and a
    board declared to PX4 mean the same thing.
    """
    a = [float(x) for x in (list(angles or []) + [0.0, 0.0, 0.0])[:3]]
    cr, sr = math.cos(math.radians(a[0])), math.sin(math.radians(a[0]))
    cp, sp = math.cos(math.radians(a[1])), math.sin(math.radians(a[1]))
    cy, sy = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rotation_euler_deg(R: np.ndarray) -> tuple[float, float, float]:
    """(roll, pitch, yaw) degrees of a rotation built the way mount_rotation builds one."""
    pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
    if abs(math.cos(pitch)) < 1e-7:                      # straight up or down: roll and yaw are one angle
        return 0.0, math.degrees(pitch), math.degrees(math.atan2(-float(R[0, 1]), float(R[1, 1])))
    return (math.degrees(math.atan2(float(R[2, 1]), float(R[2, 2]))), math.degrees(pitch),
            math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0]))))


@dataclass
class Airframe:
    name: str = "Quad X"
    schema: int = SCHEMA_VERSION
    mass: MassProperties = field(default_factory=MassProperties)
    body: Body = field(default_factory=Body)
    rotors: list[Rotor] = field(default_factory=list)
    wings: list[Wing] = field(default_factory=list)
    legs: list[Leg] = field(default_factory=list)
    hover_pitch_deg: float = 0.0    # nose-up pitch of the structural frame in hover; PX4's "level" is this attitude
    landed_pitch_deg: float = 0.0   # nose-up pitch when standing on its legs (initial attitude of a simulation)
    px4_overrides: dict = field(default_factory=dict)   # PX4 parameters set by hand, saved with the airframe
    design: dict = field(default_factory=dict)          # optimiser / analysis settings (cruise speed, groups...)
    notes: str = ""
    cad: CadModel | None = None                         # STEP bodies with masses (geometry/cad.py)

    # ------------------------------------------------------------ helpers
    @property
    def cg(self) -> np.ndarray:
        return np.asarray(self.mass.cg, float)

    def active_rotors(self) -> list[Rotor]:
        return [r for r in self.rotors if r.enabled]

    def active_wings(self) -> list[Wing]:
        return [w for w in self.wings if w.enabled and w.area > 1e-9]

    def active_legs(self) -> list[Leg]:
        return [l for l in self.legs if l.enabled]

    def foot_points(self) -> list[list[float]]:
        return [l.foot() for l in self.active_legs()]

    def hover_rotation(self) -> np.ndarray:
        """Rotation taking structural-frame vectors into PX4's body frame (nose-up by hover_pitch_deg)."""
        phi = math.radians(self.hover_pitch_deg)
        c, s = math.cos(phi), math.sin(phi)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    def rotors_in_px4_frame(self, built: bool = False) -> list[tuple[list[float], list[float]]]:
        """(position relative to the CG, thrust axis) of every enabled rotor in PX4's hover frame.

        ``built`` asks for the axes the aircraft really has instead of the ones it was drawn with. The export
        never uses it - PX4 is told the design - but a question about what the aircraft can actually do does.
        """
        R = self.hover_rotation()
        cg = self.cg
        return [((R @ (np.asarray(r.pos, float) - cg)).tolist(),
                 (R @ np.asarray(r.built_axis if built else unit(r.axis), float)).tolist())
                for r in self.active_rotors()]

    def total_max_thrust(self) -> float:
        return sum(r.effective_max_thrust() for r in self.active_rotors())

    def leg_static(self) -> dict:
        """Static compression and damping ratio of the landing gear under the vehicle's weight (per leg share)."""
        legs = self.active_legs()
        if not legs:
            return {"compression_m": 0.0, "zeta": 1.0}
        w_leg = self.mass.mass * G / len(legs)
        m_leg = self.mass.mass / len(legs)
        comp = max(w_leg / max(l.stiffness, 1e-9) for l in legs)
        zeta = min(l.damping / (2.0 * math.sqrt(max(l.stiffness, 1e-9) * m_leg)) for l in legs)
        return {"compression_m": comp, "zeta": zeta}

    def auto_leg_constants(self, compression_m: float = 0.02, zeta: float = 0.8) -> None:
        """Size every leg's spring and damper from the mass: ``compression_m`` static sink under the weight and a
        damping ratio ``zeta`` (0.8 = settles without bouncing). Call after changing mass or legs."""
        legs = self.active_legs()
        if not legs:
            return
        w_leg = self.mass.mass * G / len(legs)
        m_leg = self.mass.mass / len(legs)
        k = w_leg / max(compression_m, 1e-4)
        c = 2.0 * zeta * math.sqrt(k * m_leg)
        for l in legs:
            l.stiffness = round(k, 1)
            l.damping = round(c, 1)

    def estimate_inertia(self, **kw) -> list[float]:
        panels = []
        for w in self.active_wings():
            p = wing_panels(w)
            panels += [(pos, a) for pos, a in zip(p["pos"], p["area"])]
        legs = [(l.attach, l.foot()) for l in self.active_legs()]
        return estimate_inertia(self.mass.mass, self.mass.cg, self.body.size, [r.pos for r in self.active_rotors()],
                                panels, legs, **kw)

    # ------------------------------------------------------------------ io
    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": SCHEMA_VERSION, "name": self.name,
            "mass": self.mass.to_dict(), "body": self.body.to_dict(),
            "rotors": [r.to_dict() for r in self.rotors], "wings": [w.to_dict() for w in self.wings],
            "legs": [l.to_dict() for l in self.legs],
            "hover_pitch_deg": self.hover_pitch_deg, "landed_pitch_deg": self.landed_pitch_deg,
            "px4_overrides": dict(self.px4_overrides), "design": copy.deepcopy(self.design), "notes": self.notes,
            "cad": self.cad.to_dict() if self.cad else None,
        }

    def resolve_mass(self) -> "Airframe":
        """Recompute mass/CG/inertia from items and CAD bodies when mass.from_items is set."""
        self.mass.resolve(self.cad.mass_items() if self.cad else None)
        return self

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Airframe":
        d = migrate(d)
        af = cls(name=str(d.get("name", "airframe")), schema=SCHEMA_VERSION,
                 mass=MassProperties.from_dict(d.get("mass", {})), body=Body.from_dict(d.get("body", {})),
                 rotors=[Rotor.from_dict(r) for r in d.get("rotors", []) or []],
                 wings=[Wing.from_dict(w) for w in d.get("wings", []) or []],
                 legs=[Leg.from_dict(l) for l in d.get("legs", []) or []],
                 hover_pitch_deg=float(d.get("hover_pitch_deg", 0.0) or 0.0),
                 landed_pitch_deg=float(d.get("landed_pitch_deg", 0.0) or 0.0),
                 px4_overrides=dict(d.get("px4_overrides", {}) or {}), design=dict(d.get("design", {}) or {}),
                 notes=str(d.get("notes", "") or ""), cad=CadModel.from_dict(d.get("cad")))
        af.resolve_mass()
        for i, r in enumerate(af.rotors):
            if not r.name:
                r.name = f"M{i + 1}"
        return af

    @classmethod
    def load(cls, path: str | Path) -> "Airframe":
        with open(path) as f:
            return cls.from_dict(json.load(f))

    def save(self, path: str | Path) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    def copy(self) -> "Airframe":
        return copy.deepcopy(self)

    # --------------------------------------------------------- validation
    def _pair_problems(self) -> list[str]:
        """A mirrored couple is two items sharing a token. One on its own means the app stopped keeping
        them mirrored without saying so, which is exactly the kind of quiet asymmetry that flies badly."""
        out = []
        for what, items in (("rotor", self.rotors), ("leg", self.legs)):
            seen: dict[str, list] = {}
            for it in items:
                tok = getattr(it, "pair", "") or ""
                if tok:
                    seen.setdefault(tok, []).append(it.name or what)
            for tok, members in seen.items():
                if len(members) != 2:
                    out.append(f"mirrored {what} group {', '.join(members)} has {len(members)} member(s), not 2")
        return out

    def validate(self) -> list[str]:
        problems = []
        n = len(self.active_rotors())
        if not (1 <= n <= MAX_PX4_ROTORS):
            problems.append(f"PX4 supports 1..{MAX_PX4_ROTORS} rotors, airframe has {n} enabled")
        if self.mass.mass <= 0:
            problems.append("mass must be positive")
        zero = [r.name or f"M{i + 1}" for i, r in enumerate(self.active_rotors()) if r.max_thrust <= 0]
        if zero:
            problems.append(f"rotor(s) {', '.join(zero)} have max thrust 0")
        # A mounting error is deliberate, so this is not a fault to fix - but it is a lie the aircraft is
        # telling its flight controller, and nobody should discover it halfway through a run.
        off = [f"{r.name or f'M{i + 1}'} {' and '.join(b for b in (f'{r.tilt_error_deg:+g}° tilt' if r.tilt_error_deg else '', f'{r.cant_error_deg:+g}° cant' if r.cant_error_deg else '') if b)}"
               for i, r in enumerate(self.rotors) if r.mis_mounted]
        if off:
            problems.append(f"built off its design on purpose: {', '.join(off)}. The simulation flies that; "
                            f"PX4 is told the design. Clear it under Mounting errors when you are done.")
        problems += self._pair_problems()
        problems += self.override_warnings()
        if any(i <= 0 for i in self.mass.inertia):
            problems.append("inertia diagonal must be positive")
        try:
            if np.linalg.eigvalsh(self.mass.tensor()).min() <= 0:
                problems.append("inertia tensor is not positive definite (check the products of inertia)")
        except Exception:
            pass
        try:
            est = self.estimate_inertia()
            ratio = min(i / e for i, e in zip(self.mass.inertia, est) if e > 0)
            if ratio < 0.05:
                problems.append(f"inertia {[round(v, 4) for v in self.mass.inertia]} is far below what this mass and size imply "
                                f"(about {[round(v, 3) for v in est]}); the simulation will be unstable. Click Estimate or enter real values.")
        except Exception:
            pass
        total = self.total_max_thrust()
        if total < self.mass.mass * G * 1.2:
            problems.append(f"thrust/weight is {total / (self.mass.mass * G):.2f}, hover will be marginal")
        if len(self.active_legs()) < 3:
            problems.append("fewer than 3 legs: the vehicle cannot stand")
        else:
            ls = self.leg_static()
            if ls["compression_m"] > 0.06:
                problems.append(f"legs too soft for {self.mass.mass:g} kg: the feet sink {ls['compression_m'] * 100:.0f} cm into the ground under "
                                f"the weight (stiffness too low). Use 'Auto k/c' on the Legs table or raise the stiffness.")
            elif ls["zeta"] < 0.3:
                problems.append(f"legs underdamped (damping ratio {ls['zeta']:.2f}): the vehicle bounces after touchdown. Use 'Auto k/c' or raise the damping.")
        for w in self.active_wings():
            if w.root_chord <= 0 and w.tip_chord <= 0:
                problems.append(f"wing '{w.name}' has no chord")
        return problems

    # -------------------------------------------------------- PX4 export
    def px4_params(self, hitl: bool = True, overrides: bool = True) -> dict[str, float | int]:
        """Parameters that make PX4 fly this exact geometry. Rotor i (enabled ones, in order) is PX4 Motor i+1.

        An override set by hand is applied *last* and wins, even over a value the geometry works out. The
        geometry is the app's opinion; a number you typed is yours, and silently overwriting it - which is
        what putting the overrides first did - makes the Parameters table a liar. ``overrides=False`` asks
        what the geometry alone would set, which is how a row that has been taken over can show what it was
        taken over from.

        Every parameter here can be overridden, including the allocator and the ground sequence's own
        minimums - it is your aircraft. What the app will not do is keep quiet about it: a row that has
        been taken over says what the geometry would have set, `validate()` names any override that goes
        against what the nose-lift sequence needs, and one click puts a row back.
        """
        ov = {k: (int(v) if isinstance(v, bool) else v) for k, v in (self.px4_overrides or {}).items()}
        # The overrides go in twice on purpose: first so the parts of the export that *read* a parameter
        # (the nose-lift block reads NLF_ENABLE) see what you set, and again at the end so nothing the
        # geometry writes can quietly beat it.
        off = self.modules_off_params()            # parameters of a firmware module this design does not use
        p: dict[str, float | int] = {k: v for k, v in ov.items() if k not in off} if overrides else {}
        # Minimums the ground sequence insists on. An override may raise one of these; it may not lower it,
        # because a takeoff ramp shorter than this pitches the aircraft over on its own legs. Every other
        # parameter in the export is yours to set outright.
        floors: dict[str, float] = {}
        forced: dict[str, float | int] = {}        # and the few an override may not touch at all
        rotors = self.active_rotors()
        p["CA_AIRFRAME"] = 0
        p["CA_ROTOR_COUNT"] = len(rotors)
        for i, (r, (pos, ax)) in enumerate(zip(rotors, self.rotors_in_px4_frame())):
            p[f"CA_ROTOR{i}_PX"] = round(pos[0], 4)
            p[f"CA_ROTOR{i}_PY"] = round(pos[1], 4)
            p[f"CA_ROTOR{i}_PZ"] = round(pos[2], 4)
            p[f"CA_ROTOR{i}_AX"] = round(ax[0], 4)
            p[f"CA_ROTOR{i}_AY"] = round(ax[1], 4)
            p[f"CA_ROTOR{i}_AZ"] = round(ax[2], 4)
            p[f"CA_ROTOR{i}_KM"] = round(r.km, 4)
        # The flight controller is mounted in the structural frame; PX4 reads its IMU in the hover frame.
        from .ground_sequence import ground_sequence_params
        if "NLF_ENABLE" not in off:                # the nose-lift module is switched off for this design
            p.update(ground_sequence_params(self))
        if p.get("NLF_ENABLE") == 1:
            # Lift target: 10 deg below the hover pitch unless a takeoff pitch was set. PX4's hover allocation wants more
            # front-fan thrust than the ground floor, so its takeover step pitches the nose up by several degrees
            # while the rear feet still touch; starting below level lands that step on level instead of past it
            # (measured on ATLAS_09: ground slide 0.36 m at level, 0.10 m at -7 deg, 0.03-0.06 m at -10 deg).
            p.setdefault("NLF_TARGET", round(float(self.hover_pitch_deg) - float(self.design.get("nose_lift_below_hover_deg", 10.0)), 2))
            # A short stock takeoff ramp can pitch the grounded aircraft past its envelope at handoff.
            p["MPC_TKO_RAMP_T"] = floors["MPC_TKO_RAMP_T"] = max(0.8, float(p.get("MPC_TKO_RAMP_T", 0.8)))   # quick rear thrust: as little time on the feet as possible
            p.setdefault("NLF_DWELL", 0.5)            # settle check after the nose reaches its target, s
            p.setdefault("MPC_TKO_SPEED", 2.0)        # climb away briskly once airborne
            # PX4's land detector reads a slow nose lift as "landed" (no thrust setpoint, near-zero velocities), so
            # the stock 2 s auto-disarm fires mid-lift; the module disarms itself after nose contact, these are backups.
            p["COM_DISARM_LAND"] = floors["COM_DISARM_LAND"] = max(60.0, float(p.get("COM_DISARM_LAND", 0.0)))
            p["COM_DISARM_PRFLT"] = floors["COM_DISARM_PRFLT"] = max(40.0, float(p.get("COM_DISARM_PRFLT", 0.0)))
            if p.get("NLF_RC_THR") == 1:
                p["COM_DISARM_LAND"] = floors["COM_DISARM_LAND"] = max(120.0, float(p["COM_DISARM_LAND"]))
            if int(p.get("NLF_PILOT", 0) or 0):
                # Pilot handover (no GPS): the nose is held up while the pilot waits with the throttle at minimum, which
                # PX4's land detector reads as "landed, not taking off". Neither auto-disarm may fire meanwhile.
                p["COM_DISARM_LAND"] = floors["COM_DISARM_LAND"] = max(120.0, float(p["COM_DISARM_LAND"]))
                p["COM_DISARM_PRFLT"] = floors["COM_DISARM_PRFLT"] = max(120.0, float(p["COM_DISARM_PRFLT"]))
                # Airmode lets the allocator trade collective thrust for pitch authority. On the legs, nose 35 deg below
                # PX4's level, that runs the front fans up and the rear fans down at any throttle: the aircraft cannot
                # lift off, the nose cannot be lowered, and the pilot's throttle stops being the collective. Off.
                p["MC_AIRMODE"] = forced["MC_AIRMODE"] = 0
                # Stabilized maps the throttle stick's centre to the hover thrust parameter (MPC_THR_CURVE 0 until the
                # hover thrust estimator has run). PX4 sees commands, not thrust (THR_MDL_FAC 0) and the motors are
                # quadratic: the rotor with the largest hover share is commanded the collective itself, so the hover
                # collective is the square root of that rotor's thrust utilisation.
                hover = self.hover_check()
                shares, util = hover.get("shares", []), hover.get("hover_utilisation", [])
                if shares and util and util[int(np.argmax(shares))]:
                    p.setdefault("MPC_THR_HOVER", round(min(0.8, max(0.2, float(np.sqrt(util[int(np.argmax(shares))])))), 2))
            # the module commands the front fans as sqrt(thrust fraction) and checks that PX4 applies no thrust curve
            p["THR_MDL_FAC"] = forced["THR_MDL_FAC"] = 0.0
            # stick override is built into this PX4: a stick moved faster than MAN_OVERRIDE_SPD (normalised travel per
            # second, 1.0 = full deflection within a second) switches Offboard/auto modes to Position; keep it enabled
            p.setdefault("MAN_OVERRIDE_SPD", 1.0)
        # The board as PX4 sees it: how it is bolted in, with the hover pitch on top, because PX4 calls the
        # hover attitude level and so the hover rotation is part of the board rotation it has to undo.
        board = self.hover_rotation() @ mount_rotation(self.design.get("pixhawk_rotation"))
        for axis, angle in zip("XYZ", rotation_euler_deg(board)):
            p[f"SENS_BOARD_{axis}_OFF"] = round(float(angle), 2)
        imu_pos = np.asarray(self.design.get("pixhawk_position", self.cg), float)
        imu_offset = self.hover_rotation() @ (imu_pos - self.cg)
        for axis, value in zip("XYZ", imu_offset):
            p[f"EKF2_IMU_POS_{axis}"] = round(float(value), 4)
        p.update(self.flow_params())
        for n in range(1, 17):
            p[f"HIL_ACT_FUNC{n}"] = 101 + (n - 1) if n <= len(rotors) else 0
        if hitl:
            p["SYS_HITL"] = 1
        object.__setattr__(self, "_sequence_wants",     # not a field: a note from the last export
                           {**{k: ("at least", v) for k, v in floors.items()},
                            **{k: ("exactly", v) for k, v in forced.items()}})
        if overrides:
            p.update(ov)
        for k in off:          # a module this design has switched off configures nothing, override or not
            p.pop(k, None)
        return p

    def flow_sensor(self) -> dict | None:
        """The H-flow sensor as numbers, or None if this design has not got one.

        ``position`` is in the structural frame; ``rotation`` (roll, pitch, yaw, degrees) turns the sensor's
        own axes into structural axes, so 0 degrees looks along the airframe's Z and a nose-up hover tilts
        the beam with the airframe.
        """
        f = self.design.get("flow_sensor") or {}
        if not f.get("enabled"):
            return None
        return {"position": np.asarray(f.get("position", self.cg), float),
                "rotation": [float(x) for x in (list(f.get("rotation") or []) + [0.0, 0.0, 0.0])[:3]],
                "range_m": max(0.5, float(f.get("range_m", 4.0)))}

    def flow_params(self) -> dict[str, float | int]:
        """EKF2 and sensor parameters for the H-flow sensor. Written whether it is fitted or not: a parameter
        left over from the last design would quietly keep fusing a sensor that is no longer there."""
        f = self.flow_sensor()
        if f is None:
            return {"EKF2_OF_CTRL": 0, "EKF2_RNG_CTRL": 0}
        offset = self.hover_rotation() @ (f["position"] - self.cg)
        # Indoors there is no GPS and there never will be, so the aircraft is told to stop waiting for one:
        # the H-flow is the horizontal aiding and the rangefinder is the height, with the barometer and the
        # IMU fused underneath them as PX4 does by default. Without this the estimator keeps the barometer
        # as its height reference and has no position source at all, which is an aircraft that cannot hold
        # still indoors however good the flow is.
        indoor = bool(self.design.get("indoor"))
        p: dict[str, float | int] = {"EKF2_OF_CTRL": 1, "EKF2_RNG_CTRL": 2 if indoor else 1}
        if indoor:
            p["EKF2_HGT_REF"] = 2        # the rangefinder is the height reference, not the barometer
            p["EKF2_GPS_CTRL"] = 0       # and nothing waits for a GPS that is not coming
        for axis, value in zip("XYZ", offset):
            p[f"EKF2_OF_POS_{axis}"] = round(float(value), 4)
            p[f"EKF2_RNG_POS_{axis}"] = round(float(value), 4)
        # PX4 knows one thing about how a flow sensor is turned: its yaw, in 45 degree steps (SENS_FLOW_ROT).
        # A yaw between two steps, or any tilt, stays in the measurement and comes out as estimator error,
        # which is what it does on the aircraft.
        p["SENS_FLOW_ROT"] = int(round(f["rotation"][2] / 45.0)) % 8
        p["SENS_FLOW_MAXHGT"] = round(min(100.0, f["range_m"]), 2)
        p["EKF2_RNG_A_HMAX"] = round(min(100.0, f["range_m"]), 2)
        return p

    def px4_params_sitl(self) -> dict[str, float | int]:
        """Same as px4_params(hitl=False) but with the SITL output driver's function names."""
        p = self.px4_params(hitl=False)
        for n in range(1, 17):
            p[f"PWM_MAIN_FUNC{n}"] = p.pop(f"HIL_ACT_FUNC{n}")
        return p

    def px4_params_file(self, hitl: bool = True) -> str:
        """QGroundControl .params file text."""
        lines = ["# Onboard parameters exported by AIRFRAME_DESIGNER", f"# Airframe: {self.name}",
                 "# Vehicle-Id Component-Id Name Value Type"]
        for k, v in (self.px4_params(True) if hitl else self.px4_params_sitl()).items():
            if isinstance(v, int):
                lines.append(f"1\t1\t{k}\t{v}\t6")
            else:
                lines.append(f"1\t1\t{k}\t{float(v):.6f}\t9")
        return "\n".join(lines) + "\n"

    def override_warnings(self) -> list[str]:
        """Overrides that contradict what the ground sequence needs. Not refused - said out loud."""
        self.px4_params(hitl=True)                      # fills _sequence_wants for this design
        out = []
        for name, (how, want) in (getattr(self, "_sequence_wants", None) or {}).items():
            got = (self.px4_overrides or {}).get(name)
            if got is None:
                continue
            if (how == "at least" and float(got) < float(want)) or (how == "exactly" and float(got) != float(want)):
                out.append(f"{name} is set to {got}, and the nose-lift sequence needs {how} {want:g}")
        return out

    def modules_off_params(self) -> set[str]:
        """Every parameter belonging to a firmware module this design has switched off.

        A module that is not part of this aircraft has no business configuring it: its parameters leave
        the export, and with them the Parameters table, until it is switched back on. Which names belong
        to which module comes from the module's own params.yaml - the server reads it and keeps the list
        with the design, so a batch run or another machine exports exactly the same thing.
        """
        off = (self.design.get("modules_off") or {})
        return {name for names in off.values() for name in (names or [])}

    def geometry_param_names(self) -> set[str]:
        return {k for k in self.px4_params(hitl=True) if k not in (self.px4_overrides or {})} | {"SYS_HITL"}

    # ------------------------------------------------- PX4 hover feasibility
    def effectiveness(self, built: bool = False) -> np.ndarray:
        """PX4's 6 x n effectiveness matrix (unit thrust): rows = torque xyz, force xyz, in the hover frame."""
        rows = []
        for r, (pos, axis) in zip(self.active_rotors(), self.rotors_in_px4_frame(built)):
            p, ax = np.array(pos, float), np.array(axis, float)
            rows.append(np.concatenate([np.cross(p, ax) - r.km * ax, ax]))
        return np.array(rows).T if rows else np.zeros((6, 0))

    # ------------------------------------------------- built, not drawn
    def hover_trim_as_built(self, lean_limit_deg: float = 15.0, thrust_limit: float = 1.0) -> dict:
        """Can the aircraft as built hold a hover, and with how much to spare?

        Four things have to come out right - no net torque about any axis, and lift equal to the weight -
        and that is exactly what a set of rotors has to give. A leftover *side* force is not a failure in
        itself: the aircraft leans into it and carries on, which is what every multirotor does all day -
        but only so far, and ``lean_limit_deg`` is how far. A leftover torque is a failure, and so is a
        thrust a motor has not got. ``thrust_limit`` is how much of a motor may go into simply holding
        still: 1.0 asks whether it can hover at all, 0.85 (the figure the hover check uses) whether it can
        hover and still have something left to fly with.
        """
        rotors = self.active_rotors()
        if not rotors:
            return {"ok": False, "why": "no rotors", "worst": float("inf"), "residual_torque": 0.0, "lean_deg": 0.0}
        name = lambda k: rotors[k].name or f"M{k + 1}"                                       # noqa: E731
        E6 = self.effectiveness(built=True)
        E = E6[[0, 1, 2, 5], :]                                      # roll, pitch, yaw, lift
        want = np.array([0.0, 0.0, 0.0, -1.0])
        u = np.linalg.pinv(E) @ want
        resid = float(np.abs(E @ u - want).max())
        lift = float(-(E[3] @ u))
        if lift <= 1e-9:
            return {"ok": False, "why": "no mix of thrusts lifts the aircraft", "worst": float("inf"),
                    "residual_torque": resid, "lean_deg": 0.0}
        F = E6[3:6] @ u                                              # what is left over sideways: a lean
        lean = math.degrees(math.atan2(float(math.hypot(F[0], F[1])), max(1e-9, float(-F[2]))))
        thrust = u * (self.mass.mass * G / lift)
        util = [float(t / r.effective_max_thrust()) if r.effective_max_thrust() > 0 else float("inf")
                for t, r in zip(thrust, rotors)]
        worst = max(util)
        low = min(range(len(util)), key=lambda k: util[k])
        high = max(range(len(util)), key=lambda k: util[k])
        if resid > 1e-3:
            why = f"no mix of thrusts balances the torque (residual {resid:.3f} per unit of lift)"
        elif util[low] < -1e-6:
            why = f"{name(low)} would have to pull downwards"
        elif worst > thrust_limit:
            why = (f"{name(high)} would need {worst:.0%} of its thrust just to hold the hover"
                   if thrust_limit >= 1.0 else
                   f"{name(high)} would be at {worst:.0%} in the hover, with nothing left to fly with")
        elif lean > lean_limit_deg:
            why = f"it would have to hover leaning {lean:.0f}° away from its hover attitude"
        else:
            why = ""
        return {"ok": not why, "why": why, "worst": worst, "residual_torque": resid, "lean_deg": lean,
                "utilisation": util, "thrust": [float(t) for t in thrust]}

    def mount_margin(self, limit_deg: float = 45.0, tol_deg: float = 0.25) -> dict:
        """How far out of true each rotor could be mounted before it costs the aircraft something.

        Two angles per direction, from the same arithmetic as the hover check, run on the aircraft as built:

        * the **margin** - past this the aircraft still hovers, but a motor is over 85% doing it and there
          is nothing left for the controller to fly with;
        * the **wall** - past this there is no mix of thrusts that holds it still at all: a motor would
          have to pull downwards, or run past its maximum, or the aircraft would have to hover leaning.

        One rotor at a time, with everything else left exactly as it is, and the two directions searched
        separately, because leaning a rotor outward is rarely as bad as leaning it in. Both are static
        limits and generous ones: PX4 is flying a model that says the error is not there, so its own trim
        and its margins go first. The wall is the wall, not the tolerance.
        """
        # Winding an error up and down is a question asked of a copy: the live airframe may be flying, and
        # nothing here is allowed to be visible from the loop.
        af = copy.copy(self)
        af.rotors = [replace(r) for r in self.rotors]

        def search(rotor, axis: str, thrust_limit: float) -> tuple[list[float], str]:
            ends, why = [], ""
            for sign in (-1.0, 1.0):
                lo, hi = 0.0, float(limit_deg)
                setattr(rotor, f"{axis}_error_deg", sign * hi)
                if af.hover_trim_as_built(thrust_limit=thrust_limit)["ok"]:
                    ends.append(sign * hi)                      # still fine at the edge of where we look
                    continue
                why = why or af.hover_trim_as_built(thrust_limit=thrust_limit)["why"]
                while hi - lo > tol_deg:
                    mid = 0.5 * (lo + hi)
                    setattr(rotor, f"{axis}_error_deg", sign * mid)
                    lo, hi = (mid, hi) if af.hover_trim_as_built(thrust_limit=thrust_limit)["ok"] else (lo, mid)
                ends.append(sign * lo)
            return [round(min(ends), 1), round(max(ends), 1)], why

        rows = []
        base = af.hover_trim_as_built()
        for i, r in enumerate(af.rotors):
            row = {"rotor": i, "name": r.name or f"M{i + 1}", "tilt_error_deg": r.tilt_error_deg,
                   "cant_error_deg": r.cant_error_deg, "tilt": None, "cant": None,
                   "tilt_wall": None, "cant_wall": None, "why": ""}
            if r.enabled is False:
                row["why"] = "disabled"
                rows.append(row)
                continue
            keep = (r.tilt_error_deg, r.cant_error_deg)
            try:
                r.tilt_error_deg = r.cant_error_deg = 0.0
                if not af.hover_trim_as_built()["ok"]:
                    row["why"] = "the aircraft does not hover even with this one mounted true"
                    rows.append(row)
                    continue
                for axis in ("tilt", "cant"):
                    r.tilt_error_deg, r.cant_error_deg = keep
                    setattr(r, f"{axis}_error_deg", 0.0)
                    row[axis], _ = search(r, axis, 0.85)
                    row[f"{axis}_wall"], why = search(r, axis, 1.0)
                    row["why"] = row["why"] or why
            finally:
                r.tilt_error_deg, r.cant_error_deg = keep
            rows.append(row)
        return {"limit_deg": limit_deg, "rotors": rows, "built": base}

    def hover_check(self, built: bool = False) -> dict:
        """Reproduce PX4's pseudo-inverse allocation for a pure-thrust hover and flag what it cannot do.

        ``built`` runs the same arithmetic over the aircraft as it really is (mounting errors and all) rather
        than over the design PX4 was given, which is how far the two have drifted apart.
        """
        rotors = self.active_rotors()
        n = len(rotors)
        if n == 0:
            return {"ok": False, "problems": ["no rotors"], "shares": []}
        E = self.effectiveness(built)
        sp = np.array([0, 0, 0, 0, 0, -1.0])
        u = np.linalg.pinv(E) @ sp
        resid = E @ u - sp
        umax = float(u.max()) if u.max() > 1e-9 else 1.0
        shares = (u / umax).tolist()
        neg = [i + 1 for i, v in enumerate(shares) if v < -1e-6]
        weight = self.mass.mass * G
        thrust_up = float(-(E[5] @ u))
        scale = weight / thrust_up if thrust_up > 1e-9 else float("inf")
        hover_thrust = [float(v * scale) for v in u]
        dead = [i + 1 for i, r in enumerate(rotors) if r.effective_max_thrust() <= 0]
        util = [t / r.effective_max_thrust() if r.effective_max_thrust() > 0 else None for t, r in zip(hover_thrust, rotors)]
        over = [i + 1 for i, x in enumerate(util) if x is not None and x > 0.85]
        problems = []
        if dead:
            problems.append(f"motor(s) {dead} have no thrust (max thrust 0 or a jetfoil bent past its loss limit)")
        force_resid = float(np.abs(resid[3:5]).max())
        yaw_resid = float(abs(resid[2]))
        rp_resid = float(np.abs(resid[:2]).max())
        unbalanced = force_resid > 1e-3 or rp_resid > 1e-3
        if unbalanced:
            problems.append(f"no motor mix gives zero net force and roll/pitch torque at {self.hover_pitch_deg:g}° nose-up "
                            f"(residual force {force_resid:.2f} per unit of lift): PX4 will have to lean away from its level "
                            f"attitude to hover. Set the hover pitch so all thrust axes are vertical in hover, or tilt rotors "
                            f"in opposing pairs, or move the CG.")
        if yaw_resid > 2e-3:
            problems.append(f"yaw torque cannot be cancelled (residual {yaw_resid:.3f} per unit of lift): alternate spin "
                            f"directions, or cant rotors left/right in opposing pairs.")
        if neg:
            problems.append(f"PX4's allocator wants negative thrust on motor(s) {neg} to hover with zero torque; it will clip "
                            f"them to zero and the vehicle will not lift off. Rebalance spin directions, move the CG, or "
                            f"set the hover pitch so the thrust axes are vertical in hover.")
        if over:
            problems.append(f"motor(s) {over} above 85% of max thrust just to hover; no control margin")
        return {"ok": not neg and not over and not unbalanced and not dead and yaw_resid <= 2e-3, "problems": problems, "shares": shares,
                "hover_thrust": hover_thrust, "hover_utilisation": util, "negative": neg,
                "residual_force": resid[3:].tolist(), "residual_torque": resid[:3].tolist()}


# ----------------------------------------------------------------- migration
def migrate(d: dict) -> dict:
    """Bring an airframe dict up to the current schema. Schema 1 is the AIRFRAME_SIMULATOR format (mass as a
    number, drag/body fields at the top level, generated leg points, delta wings positioned by their
    aerodynamic centre)."""
    d = copy.deepcopy(d)
    schema = int(d.get("schema", 1) or 1)
    if schema >= SCHEMA_VERSION:
        return d
    out: dict[str, Any] = {"schema": SCHEMA_VERSION, "name": d.get("name", "airframe")}
    out["mass"] = {"mass": float(d.get("mass", 1.5)), "cg": [0.0, 0.0, 0.0],
                   "inertia": list(d.get("inertia", [0.02, 0.02, 0.035]))}
    out["body"] = {"size": list(d.get("body_size", [0.16, 0.16, 0.06])),
                   "drag_quadratic": list(d.get("drag_quadratic", [0.1, 0.1, 0.2])),
                   "drag_angular": list(d.get("drag_angular", [0.005, 0.005, 0.005]))}
    rotors = []
    for i, r in enumerate(d.get("rotors", []) or []):
        r = dict(r)
        r.setdefault("name", f"M{i + 1}")
        if "prop_diameter" in r:
            r["diameter"] = r.pop("prop_diameter")
        rotors.append(r)
    out["rotors"] = rotors
    wings = []
    for i, w in enumerate(d.get("wings", []) or []):
        wd = Wing.delta(float(w.get("area", 0.5)), float(w.get("span", 1.0)), w.get("pos", [0, 0, 0]),
                        incidence_deg=float(w.get("incidence_deg", 0.0)), name=w.get("name") or f"wing{i + 1}",
                        cd0=float(w.get("cd0", 0.02)), stall_deg=float(w.get("stall_deg", 30.0)),
                        vortex_lift=bool(w.get("vortex_lift", True)))
        wd.enabled = bool(w.get("enabled", True))
        wings.append(wd.to_dict())
    out["wings"] = wings
    landed = float(d.get("landed_pitch_deg", 0.0) or 0.0)
    if "leg_height" in d or "leg_spread" in d:
        legs = generate_legs(float(d.get("leg_height", 0.2)), float(d.get("leg_spread", 0.2)), float(d.get("leg_spread", 0.2)), landed)
    else:
        pts = d.get("leg_points") or []
        legs = [Leg.from_points([p[0], p[1], 0.0], p, name=f"leg{i + 1}") for i, p in enumerate(pts)]
    out["legs"] = [l.to_dict() for l in legs]
    out["hover_pitch_deg"] = float(d.get("hover_pitch_deg", 0.0) or 0.0)
    out["landed_pitch_deg"] = landed
    out["px4_overrides"] = dict(d.get("px4_overrides", {}) or {})
    out["design"] = dict(d.get("design", {}) or {})
    out["notes"] = str(d.get("notes", "") or "")
    return out


# ------------------------------------------------------------------- presets
def quad_x(arm: float = 0.25, max_thrust: float = 8.0, diameter: float = 0.25) -> Airframe:
    """PX4 Quad X numbering: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.

    Every number a rotor needs is written here rather than left to the dataclass default. The default is for
    a small ducted fan, and a quarter-metre-arm quadcopter is not that; relying on it once silently shrank
    this aircraft's discs to 120 mm, which quadrupled its ram drag and stopped it cruising.
    """
    a = arm / math.sqrt(2)
    af = Airframe(name="Quad X")
    af.rotors = [Rotor(name="M1", pos=[a, a, 0.0], km=0.05, max_thrust=max_thrust, diameter=diameter, tau=0.04),
                 Rotor(name="M2", pos=[-a, -a, 0.0], km=0.05, max_thrust=max_thrust, diameter=diameter, tau=0.04),
                 Rotor(name="M3", pos=[a, -a, 0.0], km=-0.05, max_thrust=max_thrust, diameter=diameter, tau=0.04),
                 Rotor(name="M4", pos=[-a, a, 0.0], km=-0.05, max_thrust=max_thrust, diameter=diameter, tau=0.04)]
    af.legs = generate_legs(0.12, 0.10, 0.10)
    af.mass.inertia = af.estimate_inertia()
    return af


def hex_x(arm: float = 0.30, max_thrust: float = 7.0, diameter: float = 0.28) -> Airframe:
    af = Airframe(name="Hex X")
    af.mass.mass = 2.2
    layout = [(30, True), (210, True), (-30, False), (150, False), (90, False), (-90, True)]
    for i, (ang, ccw) in enumerate(layout):
        rad = math.radians(ang)
        af.rotors.append(Rotor(name=f"M{i + 1}", pos=[arm * math.cos(rad), arm * math.sin(rad), 0.0],
                               km=0.05 if ccw else -0.05, max_thrust=max_thrust,
                               diameter=diameter, tau=0.04))
    af.legs = generate_legs(0.14, 0.15, 0.15)
    af.mass.inertia = af.estimate_inertia()
    return af


def plane_quad(span: float = 1.6) -> Airframe:
    """A quad with a conventional wing and tail: exercises the strip-theory surfaces (fin, tailplane, dihedral)."""
    af = quad_x(arm=0.35, max_thrust=12.0, diameter=0.30)
    af.name = "Wing Quad"
    af.mass.mass = 3.0
    af.wings = [
        Wing(name="main", pos=[0.10, 0.0, 0.0], span=span, root_chord=0.28, tip_chord=0.18, sweep_deg=5.0,
             dihedral_deg=4.0, incidence_deg=3.0, twist_deg=-2.0, panels=6),
        Wing(name="tailplane", pos=[-0.70, 0.0, -0.02], span=0.5, root_chord=0.14, tip_chord=0.10, incidence_deg=-1.0, panels=3),
        Wing(name="fin", pos=[-0.72, 0.0, -0.02], span=0.22, root_chord=0.16, tip_chord=0.08, sweep_deg=30.0,
             dihedral_deg=90.0, symmetric=False, panels=3),
    ]
    af.body.size = [0.9, 0.12, 0.10]
    af.legs = generate_legs(0.15, 0.25, 0.15)
    af.mass.inertia = af.estimate_inertia()
    return af


def blank(name: str = "New design") -> Airframe:
    """A design from scratch: a mass, four legs to stand on, and nothing else.

    Not quite nothing, because an aircraft with no landing gear falls through the ground the moment the
    simulation starts, which is a strange thing for an empty design to do while its first rotor is being
    placed. The legs are the smallest thing that makes it a vehicle; everything else is for the designer.
    """
    af = Airframe(name=name)
    af.legs = generate_legs(0.12, 0.10, 0.10)
    af.mass.inertia = af.estimate_inertia()
    return af


PRESETS = {"quad_x": quad_x, "hex_x": hex_x, "plane_quad": plane_quad}
