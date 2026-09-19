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
from dataclasses import dataclass, field
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
G = 9.80665


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

    def rotors_in_px4_frame(self) -> list[tuple[list[float], list[float]]]:
        """(position relative to the CG, thrust axis) of every enabled rotor in PX4's hover frame."""
        R = self.hover_rotation()
        cg = self.cg
        return [((R @ (np.asarray(r.pos, float) - cg)).tolist(), (R @ np.asarray(unit(r.axis), float)).tolist())
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
    def px4_params(self, hitl: bool = True) -> dict[str, float | int]:
        """Parameters that make PX4 fly this exact geometry. Rotor i (enabled ones, in order) is PX4 Motor i+1."""
        p: dict[str, float | int] = {}
        for k, v in (self.px4_overrides or {}).items():
            p[k] = int(v) if isinstance(v, bool) else v
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
        p.update(ground_sequence_params(self))
        if p.get("NLF_ENABLE") == 1:
            # The ground sequence lifts the nose to the attitude PX4 will hold as "level" unless a takeoff pitch was
            # set explicitly; a target away from the hover pitch makes PX4 rotate the nose at the moment it takes over.
            p.setdefault("NLF_TARGET", round(float(self.hover_pitch_deg), 2))
            # A short stock takeoff ramp can pitch the grounded aircraft past its envelope at handoff.
            p["MPC_TKO_RAMP_T"] = max(3.0, float(p.get("MPC_TKO_RAMP_T", 3.0)))
            # PX4's land detector reads a slow nose lift as "landed" (no thrust setpoint, near-zero velocities), so
            # the stock 2 s auto-disarm fires mid-lift; the module disarms itself after nose contact, these are backups.
            p["COM_DISARM_LAND"] = max(60.0, float(p.get("COM_DISARM_LAND", 0.0)))
            p["COM_DISARM_PRFLT"] = max(40.0, float(p.get("COM_DISARM_PRFLT", 0.0)))
        p["SENS_BOARD_Y_OFF"] = round(float(self.hover_pitch_deg), 2)
        imu_pos = np.asarray(self.design.get("pixhawk_position", self.cg), float)
        imu_offset = self.hover_rotation() @ (imu_pos - self.cg)
        for axis, value in zip("XYZ", imu_offset):
            p[f"EKF2_IMU_POS_{axis}"] = round(float(value), 4)
        for n in range(1, 17):
            p[f"HIL_ACT_FUNC{n}"] = 101 + (n - 1) if n <= len(rotors) else 0
        if hitl:
            p["SYS_HITL"] = 1
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

    def geometry_param_names(self) -> set[str]:
        return {k for k in self.px4_params(hitl=True) if k not in (self.px4_overrides or {})} | {"SYS_HITL"}

    # ------------------------------------------------- PX4 hover feasibility
    def effectiveness(self) -> np.ndarray:
        """PX4's 6 x n effectiveness matrix (unit thrust): rows = torque xyz, force xyz, in the hover frame."""
        rows = []
        for r, (pos, axis) in zip(self.active_rotors(), self.rotors_in_px4_frame()):
            p, ax = np.array(pos, float), np.array(axis, float)
            rows.append(np.concatenate([np.cross(p, ax) - r.km * ax, ax]))
        return np.array(rows).T if rows else np.zeros((6, 0))

    def hover_check(self) -> dict:
        """Reproduce PX4's pseudo-inverse allocation for a pure-thrust hover and flag what it cannot do."""
        rotors = self.active_rotors()
        n = len(rotors)
        if n == 0:
            return {"ok": False, "problems": ["no rotors"], "shares": []}
        E = self.effectiveness()
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
def quad_x(arm: float = 0.25, max_thrust: float = 8.0) -> Airframe:
    """PX4 Quad X numbering: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW."""
    a = arm / math.sqrt(2)
    af = Airframe(name="Quad X")
    af.rotors = [Rotor(name="M1", pos=[a, a, 0.0], km=0.05, max_thrust=max_thrust),
                 Rotor(name="M2", pos=[-a, -a, 0.0], km=0.05, max_thrust=max_thrust),
                 Rotor(name="M3", pos=[a, -a, 0.0], km=-0.05, max_thrust=max_thrust),
                 Rotor(name="M4", pos=[-a, a, 0.0], km=-0.05, max_thrust=max_thrust)]
    af.legs = generate_legs(0.12, 0.10, 0.10)
    af.mass.inertia = af.estimate_inertia()
    return af


def hex_x(arm: float = 0.30, max_thrust: float = 7.0) -> Airframe:
    af = Airframe(name="Hex X")
    af.mass.mass = 2.2
    layout = [(30, True), (210, True), (-30, False), (150, False), (90, False), (-90, True)]
    for i, (ang, ccw) in enumerate(layout):
        rad = math.radians(ang)
        af.rotors.append(Rotor(name=f"M{i + 1}", pos=[arm * math.cos(rad), arm * math.sin(rad), 0.0],
                               km=0.05 if ccw else -0.05, max_thrust=max_thrust))
    af.legs = generate_legs(0.14, 0.15, 0.15)
    af.mass.inertia = af.estimate_inertia()
    return af


def plane_quad(span: float = 1.6) -> Airframe:
    """A quad with a conventional wing and tail: exercises the strip-theory surfaces (fin, tailplane, dihedral)."""
    af = quad_x(arm=0.35, max_thrust=12.0)
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


PRESETS = {"quad_x": quad_x, "hex_x": hex_x, "plane_quad": plane_quad}
