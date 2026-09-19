"""JSBSim physics backend: the same airframe, flown by JSBSim's flight dynamics model instead of the Python
rigid body, with the same interface, so a flight can be run on either engine and the two compared.

What JSBSim does here (independently of the Python code): 6-DOF integration, gravity, standard atmosphere,
ground reactions at the feet (STRUCTURE contacts), and the application of every force at its location, i.e. the
moment arms. Rotor thrust enters as one external force per rotor (fixed direction = thrust axis, location = rotor
position, in JSBSim's structural frame); JSBSim turns those into moments about its own CG. The aerodynamics of the
wings and body enter as coefficient tables (lift/drag/side/pitch/roll/yaw versus angle of attack and sideslip,
plus rate-damping derivatives) sampled from this project's aero models at a reference speed, which is how any
aircraft is described to JSBSim. Rotor spool-up, thrust curve, reaction torque and intake ram drag remain the
project's own model (JSBSim has no ducted-fan model) and are injected as body forces/moments each step.

So a Python-vs-JSBSim difference points at integration, frames, moment arms, ground contact or gravity, not at
the coefficient data, which is shared by construction.
"""
from __future__ import annotations

import math
import os
import shutil
import time
from pathlib import Path

import numpy as np

from ..aero import RotorSet, WingSet, BodyAero, RHO
from .contact import LegContacts
from ..geometry.airframe import Airframe
from ..geometry.frames import unit
from .quaternion import q_from_euler, q_to_rotmat, q_to_euler, q_normalize

G = 9.80665
FT = 3.28084          # ft per m
IN = 39.37008         # in per m
LBF = 0.2248089       # lbf per N
LBFT = 0.7375621      # lbf*ft per N*m
SLUGFT2 = 0.7375621   # slug*ft^2 per kg*m^2
LB = 2.2046226        # lb per kg
JSB_ROOT = Path(os.path.expanduser("~/.airframe_designer/jsbsim"))


def _s(v: float) -> str:
    return f"{float(v):.6g}"


# ------------------------------------------------------------------ model generation
def _wing_tables(af: Airframe, v_ref: float) -> dict:
    """Wing + body aero as JSBSim coefficient tables (about the CG, body axes), sampled from the project's own
    aero models at the reference speed: forces/moments vs alpha (and beta), and rate-damping derivatives."""
    cg = af.cg
    ws = WingSet(af.active_wings(), cg)
    S = max(sum(w.area for w in af.active_wings()), 1e-6)
    b = max((w.span for w in af.active_wings()), default=1.0)
    c = S / b
    q = 0.5 * RHO * v_ref ** 2
    alphas = np.radians(np.arange(-180.0, 180.1, 2.0))
    rows_x, rows_z, rows_m = [], [], []
    zero = np.zeros(3)
    for a in alphas:
        v = np.array([v_ref * math.cos(a), 0.0, v_ref * math.sin(a)])
        F, M = zero, zero
        if ws.n:
            for _ in range(3):
                F, M, _ = ws.forces(v, zero, False)
        rows_x.append(F[0] / (q * S)); rows_z.append(F[2] / (q * S)); rows_m.append(M[1] / (q * S * c))
    # sideslip: side force, roll and yaw coefficients per radian (small-angle fit at +-10 deg)
    def at_beta(beta):
        v = np.array([v_ref * math.cos(beta), v_ref * math.sin(beta), 0.0])
        F, M = zero, zero
        if ws.n:
            for _ in range(3):
                F, M, _ = ws.forces(v, zero, False)
        return F, M
    Fp, Mp = at_beta(math.radians(10)); Fm, Mm = at_beta(math.radians(-10))
    db = math.radians(20)
    cy_beta = (Fp[1] - Fm[1]) / db / (q * S)
    cl_beta = (Mp[0] - Mm[0]) / db / (q * S * b)
    cn_beta = (Mp[2] - Mm[2]) / db / (q * S * b)
    # rate damping: moment per unit non-dimensional rate (p b / 2V, q c / 2V, r b / 2V) at alpha = 0
    v0 = np.array([v_ref, 0.0, 0.0])
    def damp(axis, rate, ref):
        w = np.zeros(3); w[axis] = rate
        F1, M1 = zero, zero
        if ws.n:
            for _ in range(3):
                F1, M1, _ = ws.forces(v0, w, False)
        F0, M0 = zero, zero
        if ws.n:
            for _ in range(3):
                F0, M0, _ = ws.forces(v0, zero, False)
        nd = rate * ref / (2 * v_ref)
        return (M1[axis] - M0[axis]) / (q * S * ref) / nd if abs(nd) > 1e-12 else 0.0
    clp = damp(0, 0.5, b); cmq = damp(1, 0.5, c); cnr = damp(2, 0.5, b)
    return {"S": S, "b": b, "c": c, "alpha": alphas, "cx": rows_x, "cz": rows_z, "cm": rows_m,
            "cy_beta": cy_beta, "cl_beta": cl_beta, "cn_beta": cn_beta, "clp": clp, "cmq": cmq, "cnr": cnr}


def generate_model(af: Airframe, name: str, v_ref: float = 20.0) -> Path:
    """Write <JSB_ROOT>/aircraft/<name>/<name>.xml for this airframe and return the model directory."""
    root = JSB_ROOT
    for d in ("aircraft", "engine", "systems"):
        (root / d).mkdir(parents=True, exist_ok=True)
    mdir = root / "aircraft" / name
    mdir.mkdir(parents=True, exist_ok=True)
    af.resolve_mass()
    cg = af.cg
    # structural frame (JSBSim): X aft, Y right, Z up, inches. Ours: FRD metres.
    def loc(p):
        return f"<x>{_s(-p[0] * IN)}</x><y>{_s(p[1] * IN)}</y><z>{_s(-p[2] * IN)}</z>"
    I = af.mass.tensor()
    tab = _wing_tables(af, v_ref)
    rotors = af.active_rotors()
    legs = af.active_legs()
    body = af.body
    x = []
    x.append(f'<?xml version="1.0"?>\n<fdm_config name="{name}" version="2.0" release="ALPHA">')
    x.append(f'<fileheader><author>airframe_designer</author><description>generated from {af.name}</description></fileheader>')
    x.append(f'<metrics><wingarea unit="M2">{_s(tab["S"])}</wingarea><wingspan unit="M">{_s(tab["b"])}</wingspan><chord unit="M">{_s(tab["c"])}</chord>'
             f'<htailarea unit="M2">0</htailarea><htailarm unit="M">0</htailarm><vtailarea unit="M2">0</vtailarea><vtailarm unit="M">0</vtailarm>'
             f'<location name="AERORP" unit="IN">{loc(cg)}</location><location name="EYEPOINT" unit="IN">{loc(cg)}</location>'
             f'<location name="VRP" unit="IN">{loc([0, 0, 0])}</location></metrics>')
    x.append(f'<mass_balance><ixx unit="SLUG*FT2">{_s(I[0, 0] * SLUGFT2)}</ixx><iyy unit="SLUG*FT2">{_s(I[1, 1] * SLUGFT2)}</iyy><izz unit="SLUG*FT2">{_s(I[2, 2] * SLUGFT2)}</izz>'
             f'<ixy unit="SLUG*FT2">{_s(-I[0, 1] * SLUGFT2)}</ixy><ixz unit="SLUG*FT2">{_s(-I[0, 2] * SLUGFT2)}</ixz><iyz unit="SLUG*FT2">{_s(-I[1, 2] * SLUGFT2)}</iyz>'
             f'<emptywt unit="LBS">{_s(af.mass.mass * LB)}</emptywt><location name="CG" unit="IN">{loc(cg)}</location></mass_balance>')
    x.append('<ground_reactions>')
    for i, l in enumerate(legs):
        f = l.foot()
        f = [f[0], f[1], f[2] + l.foot_radius]
        x.append(f'<contact type="STRUCTURE" name="{l.name or "leg" + str(i)}"><location unit="IN">{loc(f)}</location>'
                 f'<static_friction>{_s(l.friction)}</static_friction><dynamic_friction>{_s(l.friction)}</dynamic_friction>'
                 f'<spring_coeff unit="LBS/FT">{_s(l.stiffness * LBF / FT)}</spring_coeff><damping_coeff unit="LBS/FT/SEC">{_s(l.damping * LBF / FT)}</damping_coeff></contact>')
    x.append('</ground_reactions>')
    x.append('<propulsion/>')
    x.append('<external_reactions>')
    for i, r in enumerate(rotors):
        ax = unit(r.axis)
        x.append(f'<force name="rotor{i}" frame="BODY"><location unit="IN">{loc(r.pos)}</location><direction><x>{_s(ax[0])}</x><y>{_s(ax[1])}</y><z>{_s(ax[2])}</z></direction></force>')
    for nm, d in (("extfx", (1, 0, 0)), ("extfy", (0, 1, 0)), ("extfz", (0, 0, 1))):     # injected body forces at the CG
        x.append(f'<force name="{nm}" frame="BODY"><location unit="IN">{loc(cg)}</location><direction><x>{d[0]}</x><y>{d[1]}</y><z>{d[2]}</z></direction></force>')
    x.append('</external_reactions>')
    # aerodynamics: body-axis force tables (X, Z) and pitch moment vs alpha; side/roll/yaw from beta; damping
    def table(vals):
        return "".join(f"{_s(a)} {_s(v)}\n" for a, v in zip(tab["alpha"], vals))
    x.append('<aerodynamics>')
    x.append(f'<axis name="X"><function name="aero/force/x-wing"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>'
             f'<table><independentVar lookup="row">aero/alpha-rad</independentVar><tableData>\n{table(tab["cx"])}</tableData></table></product></function>'
             f'<function name="aero/force/x-injected"><property>custom/ext-mx-fx-lbs</property></function></axis>')
    x.append(f'<axis name="Y"><function name="aero/force/y-beta"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>aero/beta-rad</property><value>{_s(tab["cy_beta"])}</value></product></function>'
             f'<function name="aero/force/y-injected"><property>custom/ext-mx-fy-lbs</property></function></axis>')
    x.append(f'<axis name="Z"><function name="aero/force/z-wing"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property>'
             f'<table><independentVar lookup="row">aero/alpha-rad</independentVar><tableData>\n{table(tab["cz"])}</tableData></table></product></function>'
             f'<function name="aero/force/z-injected"><property>custom/ext-mx-fz-lbs</property></function></axis>')
    x.append(f'<axis name="ROLL"><function name="aero/moment/roll-beta"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/bw-ft</property><property>aero/beta-rad</property><value>{_s(tab["cl_beta"])}</value></product></function>'
             f'<function name="aero/moment/roll-damp"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/bw-ft</property><property>aero/bi2vel</property><property>velocities/p-aero-rad_sec</property><value>{_s(tab["clp"])}</value></product></function>'
             f'<function name="aero/moment/roll-injected"><property>custom/ext-mx-l-lbsft</property></function></axis>')
    x.append(f'<axis name="PITCH"><function name="aero/moment/pitch-wing"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property>'
             f'<table><independentVar lookup="row">aero/alpha-rad</independentVar><tableData>\n{table(tab["cm"])}</tableData></table></product></function>'
             f'<function name="aero/moment/pitch-damp"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property><property>aero/ci2vel</property><property>velocities/q-aero-rad_sec</property><value>{_s(tab["cmq"])}</value></product></function>'
             f'<function name="aero/moment/pitch-injected"><property>custom/ext-mx-m-lbsft</property></function></axis>')
    x.append(f'<axis name="YAW"><function name="aero/moment/yaw-beta"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/bw-ft</property><property>aero/beta-rad</property><value>{_s(tab["cn_beta"])}</value></product></function>'
             f'<function name="aero/moment/yaw-damp"><product><property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/bw-ft</property><property>aero/bi2vel</property><property>velocities/r-aero-rad_sec</property><value>{_s(tab["cnr"])}</value></product></function>'
             f'<function name="aero/moment/yaw-injected"><property>custom/ext-mx-n-lbsft</property></function></axis>')
    x.append('</aerodynamics>')
    x.append('</fdm_config>')
    (mdir / f"{name}.xml").write_text("\n".join(x))
    return mdir


# ------------------------------------------------------------------ the body
class JSBSimBody:
    """Drop-in replacement for dynamics.RigidBody backed by JSBSim."""

    backend = "jsbsim"

    def __init__(self, airframe: Airframe, home_alt_m: float = 10.0, lat_deg: float = 55.6761, lon_deg: float = 12.5683,
                 v_ref: float = 20.0, log=None):
        import jsbsim
        self._jsbsim = jsbsim
        self.log = log or (lambda s: None)
        self.wind_ned = np.zeros(3)
        self.home_alt = float(home_alt_m); self.lat = float(lat_deg); self.lon = float(lon_deg)
        self.v_ref = float(v_ref)
        self.set_airframe(airframe)
        self.reset()

    # -- configuration
    def set_airframe(self, airframe: Airframe) -> None:
        self.af = airframe
        airframe.resolve_mass()
        self.mass = float(airframe.mass.mass)
        self.I = airframe.mass.tensor(); self.I_inv = np.linalg.inv(self.I)
        cg = airframe.cg
        rotors = airframe.active_rotors()
        self.rotors = RotorSet(rotors, cg)          # thrust curve / spool / torque / ram drag: the project's model
        self.wings = WingSet(airframe.active_wings(), cg)   # only for the breakdown readout (lift, alpha)
        self.body = BodyAero(airframe.body, cg)
        self.legs = LegContacts(airframe.active_legs(), cg)   # geometry only (feet relative to the CG); JSBSim does the contact
        n = len(rotors)
        if not hasattr(self, "omega") or len(self.omega) != n:
            self.omega = np.zeros(n); self.cmd = np.zeros(n); self.thrust = np.zeros(n)
        self.model_name = "af_" + "".join(ch if ch.isalnum() else "_" for ch in airframe.name.lower())[:40]
        self.model_dir = generate_model(airframe, self.model_name, self.v_ref)
        self._build_fdm()
        self.breakdown = {}

    def _build_fdm(self) -> None:
        fdm = self._jsbsim.FGFDMExec(str(JSB_ROOT))
        fdm.set_debug_level(0)
        try:
            fdm.disable_output()
        except Exception:
            pass
        if not fdm.load_model(self.model_name):
            raise RuntimeError(f"JSBSim could not load the generated model {self.model_dir}")
        self.fdm = fdm
        self._dt = None

    def _apply_dt(self, dt: float) -> None:
        if self._dt != dt:
            self.fdm.set_dt(dt)
            self._dt = dt

    def set_rotor_health(self, scales) -> None:
        s = np.ones(self.rotors.n)
        for i, v in enumerate(scales[: self.rotors.n]):
            s[i] = float(v)
        self.rotors.scale = s

    def reset(self, yaw: float = 0.0, pos_ned=None) -> None:
        self.t = 0.0
        fdm = self.fdm
        pitch = float(self.af.landed_pitch_deg)
        # rest height: lowest foot on the ground (JSBSim settles it exactly on its own contacts afterwards)
        R = q_to_rotmat(q_from_euler(0.0, math.radians(pitch), yaw))
        feet = np.array([l.foot() for l in self.af.active_legs()], float).reshape(-1, 3) - self.af.cg
        h = float(np.max((feet @ R.T)[:, 2])) if len(feet) else 0.0
        fdm["ic/lat-gc-deg"] = self.lat; fdm["ic/long-gc-deg"] = self.lon
        fdm["ic/terrain-elevation-ft"] = self.home_alt * FT
        fdm["ic/h-agl-ft"] = (h + 0.002) * FT
        fdm["ic/phi-deg"] = 0.0; fdm["ic/theta-deg"] = pitch; fdm["ic/psi-true-deg"] = math.degrees(yaw) % 360.0
        for k in ("ic/u-fps", "ic/v-fps", "ic/w-fps", "ic/p-rad_sec", "ic/q-rad_sec", "ic/r-rad_sec"):
            fdm[k] = 0.0
        for i in range(self.rotors.n):
            fdm[f"external_reactions/rotor{i}/magnitude"] = 0.0
        for k in ("custom/ext-mx-fx-lbs", "custom/ext-mx-fy-lbs", "custom/ext-mx-fz-lbs", "custom/ext-mx-l-lbsft", "custom/ext-mx-m-lbsft", "custom/ext-mx-n-lbsft"):
            fdm[k] = 0.0
        fdm.run_ic()
        self.omega = np.zeros(self.rotors.n); self.cmd = np.zeros(self.rotors.n); self.thrust = np.zeros(self.rotors.n)
        self._read_state()
        self._vel_prev = self.vel.copy()
        self.accel_body = np.array([0.0, 0.0, -G])
        self.breakdown = {"thrust": 0.0, "lift": 0.0, "wing_drag": 0.0, "ram_drag": 0.0, "body_drag": 0.0, "power": 0.0,
                          "alpha": [], "stalled": False, "airspeed": 0.0, "wing_forces": []}
        if pos_ned is not None:
            self.log("[jsbsim] reset to an arbitrary position is not supported; started on the ground")

    def set_motor_commands(self, cmd) -> None:
        c = np.asarray(cmd, dtype=float)[: len(self.cmd)]
        self.cmd[: len(c)] = np.clip(c, 0.0, 1.0)

    # -- state
    def _read_state(self) -> None:
        f = self.fdm
        # The distance-from-start lat/lon properties are unsigned distances.
        # Use signed local coordinates so south/west motion agrees with velocity and GPS.
        n = f["position/from-start-neu-n-ft"] / FT
        e = f["position/from-start-neu-e-ft"] / FT
        d = -(f["position/h-agl-ft"] / FT)
        self.pos = np.array([n, e, d])
        self.vel = np.array([f["velocities/v-north-fps"], f["velocities/v-east-fps"], f["velocities/v-down-fps"]]) / FT
        phi, theta, psi = f["attitude/phi-rad"], f["attitude/theta-rad"], f["attitude/psi-rad"]
        self.q = q_from_euler(phi, theta, psi)
        self.rates = np.array([f["velocities/p-rad_sec"], f["velocities/q-rad_sec"], f["velocities/r-rad_sec"]])
        # feet in contact: STRUCTURE contacts expose their compression (BOGEY ones also a WOW flag)
        feet = 0
        for i in range(self.legs.n):
            try:
                feet += 1 if f[f"gear/unit[{i}]/compression-ft"] > 1e-5 else 0
            except Exception:
                try:
                    feet += 1 if f[f"gear/unit[{i}]/WOW"] > 0.5 else 0
                except Exception:
                    pass
        if feet == 0:
            try:
                if abs(f["forces/fbz-gear-lbs"]) > 1e-3:
                    feet = 1
            except Exception:
                pass
        self.feet_down = feet
        self.on_ground = feet > 0

    # -- step
    def step(self, dt: float, detail: bool = True) -> None:
        self._apply_dt(dt)
        f = self.fdm
        R = q_to_rotmat(self.q)
        v_air = R.T @ (self.vel - self.wind_ned)
        # rotor model (project): spool, thrust curve, reaction torque, ram drag
        self.omega = np.clip(self.omega + (self.cmd - self.omega) / self.rotors.tau * dt, 0.0, 1.0)
        F_r, M_r, thrust, ram = self.rotors.forces(self.omega, v_air, self.rates, detail)
        tv = thrust @ self.rotors.axis
        M_thrust = np.cross(self.rotors.r, thrust[:, None] * self.rotors.axis).sum(axis=0) if self.rotors.n else np.zeros(3)
        F_rest = F_r - tv                    # ram drag (thrust itself goes in per rotor below)
        M_rest = M_r - M_thrust              # reaction torque + ram-drag moment
        F_b, M_b, body_drag = self.body.forces(v_air, self.rates, detail)
        F_inj = F_rest + F_b; M_inj = M_rest + M_b
        for i in range(self.rotors.n):
            f[f"external_reactions/rotor{i}/magnitude"] = float(thrust[i]) * LBF
        f["custom/ext-mx-fx-lbs"] = float(F_inj[0]) * LBF; f["custom/ext-mx-fy-lbs"] = float(F_inj[1]) * LBF; f["custom/ext-mx-fz-lbs"] = float(F_inj[2]) * LBF
        f["custom/ext-mx-l-lbsft"] = float(M_inj[0]) * LBFT; f["custom/ext-mx-m-lbsft"] = float(M_inj[1]) * LBFT; f["custom/ext-mx-n-lbsft"] = float(M_inj[2]) * LBFT
        wn, we, wd = (float(v) for v in self.wind_ned)
        f["atmosphere/wind-north-fps"] = wn * FT; f["atmosphere/wind-east-fps"] = we * FT; f["atmosphere/wind-down-fps"] = wd * FT
        vel_before = self.vel
        f.run()
        self._read_state()
        R = q_to_rotmat(self.q)
        dv = (self.vel - vel_before) / dt
        dv[2] -= G
        self.accel_body = R.T @ dv
        self.thrust = thrust
        self.t += dt
        if detail:
            v_air = R.T @ (self.vel - self.wind_ned)
            _, _, wb = self.wings.forces(v_air, self.rates, True) if self.wings.n else (None, None, {"lift": 0.0, "drag": 0.0, "alpha": [], "stalled": False, "wings": []})
            self.breakdown = {"thrust": float(thrust.sum()), "lift": wb["lift"], "wing_drag": wb["drag"], "ram_drag": ram,
                              "body_drag": body_drag, "alpha": wb["alpha"], "stalled": wb["stalled"], "airspeed": float(np.sqrt(v_air @ v_air)),
                              "power": self.rotors.ideal_power(thrust), "wing_forces": wb.get("wings", [])}

    # -- convenience (same as RigidBody)
    def is_sane(self) -> bool:
        if not (np.isfinite(self.pos).all() and np.isfinite(self.vel).all() and np.isfinite(self.q).all() and np.isfinite(self.rates).all()):
            return False
        return bool((self.vel @ self.vel) < 300.0 ** 2 and (self.rates @ self.rates) < 300.0 ** 2 and abs(self.pos[2]) < 50000.0)

    @property
    def euler(self):
        return q_to_euler(self.q)

    @property
    def rotmat(self):
        return q_to_rotmat(self.q)

    @property
    def tilt_deg(self) -> float:
        up = self.rotmat @ np.array([0.0, 0.0, -1.0])
        return math.degrees(math.acos(max(-1.0, min(1.0, -up[2]))))

    def hover_frame_euler(self):
        Rh = self.af.hover_rotation()
        Rp = self.rotmat @ Rh.T
        w = math.sqrt(max(0.0, 1 + Rp[0, 0] + Rp[1, 1] + Rp[2, 2])) / 2
        if w < 1e-9:
            return q_to_euler(self.q)
        x = (Rp[2, 1] - Rp[1, 2]) / (4 * w); y = (Rp[0, 2] - Rp[2, 0]) / (4 * w); z = (Rp[1, 0] - Rp[0, 1]) / (4 * w)
        return q_to_euler(q_normalize(np.array([w, x, y, z])))
