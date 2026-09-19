"""6-DOF rigid body: the vehicle's translational state at the CG (NED), attitude quaternion (body FRD -> NED),
body rates, and the rotor spool states. Forces come from the aero segment (about the CG, structural frame),
gravity and the leg contacts.

The structural frame is the body frame: the CG offset is handled inside the force models (they take the CG at
construction), so the state here is always that of the CG."""
from __future__ import annotations

import math

import numpy as np

from ..aero import RotorSet, WingSet, BodyAero, RHO
from ..dynamics.contact import LegContacts
from ..geometry.airframe import Airframe
from .quaternion import q_normalize, q_to_rotmat, q_from_euler, q_to_euler, q_deriv
from ..aero.fastmath import cross3

G = 9.80665


class RigidBody:
    def __init__(self, airframe: Airframe):
        self.wind_ned = np.zeros(3)
        self.set_airframe(airframe)
        self.reset()

    # -- configuration
    def set_airframe(self, airframe: Airframe) -> None:
        self.af = airframe
        airframe.resolve_mass()
        rotors = airframe.active_rotors()
        n = len(rotors)
        self.mass = float(airframe.mass.mass)
        self.I = airframe.mass.tensor()
        self.I_inv = np.linalg.inv(self.I)
        cg = airframe.cg
        self.rotors = RotorSet(rotors, cg)
        self.wings = WingSet(airframe.active_wings(), cg)
        self.body = BodyAero(airframe.body, cg)
        self.legs = LegContacts(airframe.active_legs(), cg)
        if not hasattr(self, "omega") or len(self.omega) != n:
            self.omega = np.zeros(n)
            self.cmd = np.zeros(n)
            self.thrust = np.zeros(n)
        self.breakdown = {}

    def set_rotor_health(self, scales) -> None:
        """Per-rotor thrust scale (motor failure injection)."""
        s = np.ones(self.rotors.n)
        for i, v in enumerate(scales[: self.rotors.n]):
            s[i] = float(v)
        self.rotors.scale = s

    def reset(self, yaw: float = 0.0, pos_ned=None) -> None:
        self.t = 0.0
        self.pos = np.zeros(3) if pos_ned is None else np.asarray(pos_ned, float).copy()
        self.vel = np.zeros(3)
        pitch = math.radians(float(self.af.landed_pitch_deg))
        self.q = q_from_euler(0.0, pitch, yaw)
        self.rates = np.zeros(3)
        n = self.rotors.n
        self.omega = np.zeros(n); self.cmd = np.zeros(n); self.thrust = np.zeros(n)
        self.accel_body = np.array([0.0, 0.0, -G])
        self.on_ground = True
        self.feet_down = 0
        if pos_ned is None:
            self.pos[2] = -self.legs.rest_height(q_to_rotmat(self.q))   # lowest foot on the ground
        self.breakdown = {"thrust": 0.0, "lift": 0.0, "wing_drag": 0.0, "ram_drag": 0.0, "body_drag": 0.0, "power": 0.0,
                          "alpha": [], "stalled": False}

    def set_motor_commands(self, cmd) -> None:
        c = np.asarray(cmd, dtype=float)[: len(self.cmd)]
        self.cmd[: len(c)] = np.clip(c, 0.0, 1.0)

    # -- forces
    def _forces(self, pos, vel, q, rates, omega, detail: bool = True, dt: float | None = None):
        R = q_to_rotmat(q)
        v_air = R.T @ (vel - self.wind_ned)
        F_r, M_r, thrust, ram = self.rotors.forces(omega, v_air, rates, detail)
        F_w, M_w, wb = self.wings.forces(v_air, rates, detail)
        F_b, M_b, body_drag = self.body.forces(v_air, rates, detail)
        F_body = F_r + F_w + F_b
        M_body = M_r + M_w + M_b
        F_ned = R @ F_body
        F_ned[2] += self.mass * G
        F_c, M_c, on_ground, feet = self.legs.forces(pos, vel, R, rates, dt, self.mass, self.I_inv)
        F_ned = F_ned + F_c
        M_body = M_body + M_c
        bd = None
        if detail:
            bd = {"thrust": float(thrust.sum()), "lift": wb["lift"], "wing_drag": wb["drag"], "ram_drag": ram,
                  "body_drag": body_drag, "alpha": wb["alpha"], "stalled": wb["stalled"], "airspeed": float(np.sqrt(v_air @ v_air)),
                  "power": self.rotors.ideal_power(thrust), "wing_forces": wb.get("wings", [])}
        return F_ned, M_body, thrust, on_ground, feet, R, bd

    # -- integrate: semi-implicit Euler at a small dt (the loop runs 500-1000 Hz)
    def step(self, dt: float, detail: bool = True) -> None:
        """Advance by dt. ``detail`` also refreshes the force breakdown (skip it on inner sub-steps)."""
        F_ned, M_body, thrust, on_ground, feet, R, bd = self._forces(self.pos, self.vel, self.q, self.rates, self.omega, detail, dt)
        acc = F_ned / self.mass
        ang_acc = self.I_inv @ (M_body - cross3(self.rates, self.I @ self.rates))
        d_omega = (self.cmd - self.omega) / self.rotors.tau
        vel_before = self.vel
        self.omega = np.clip(self.omega + d_omega * dt, 0.0, 1.0)
        self.vel = vel_before + acc * dt
        self.pos = self.pos + self.vel * dt
        self.rates = self.rates + ang_acc * dt
        self.q = q_normalize(self.q + q_deriv(self.q, self.rates) * dt)
        # the accelerometer reads the specific force of the motion actually made this step (see the reference
        # simulator: a force-derived value disagrees with the motion on the ground and confuses the EKF)
        dv = (self.vel - vel_before) / dt
        dv[2] -= G
        self.accel_body = R.T @ dv
        # A numerical deadband only: when parked with the motors idle and the motion is below 1 mm/s and 0.06 deg/s,
        # zero it so the vehicle does not drift on friction noise. (The previous version halved anything below
        # 5 cm/s every step, which acted as a huge damper and left soft-legged aircraft parked far above their
        # spring equilibrium, a discrepancy the JSBSim cross-check exposed.)
        if on_ground and (self.vel @ self.vel) < 1e-6 and (self.rates @ self.rates) < 1e-6 and thrust.sum() < 0.03 * self.mass * G:
            self.vel = self.vel * 0.0
            self.rates = self.rates * 0.0
        self.thrust = thrust
        self.on_ground = on_ground
        self.feet_down = feet
        if bd is not None:
            self.breakdown = bd
        self.t += dt

    # -- convenience
    def is_sane(self) -> bool:
        """False when the state is non-finite or physically absurd (a diverging integration)."""
        if not (np.isfinite(self.pos).all() and np.isfinite(self.vel).all() and np.isfinite(self.q).all() and np.isfinite(self.rates).all()):
            return False
        return bool((self.vel @ self.vel) < 300.0 ** 2 and (self.rates @ self.rates) < 300.0 ** 2 and abs(self.pos[2]) < 50000.0)

    @property
    def euler(self) -> tuple[float, float, float]:
        return q_to_euler(self.q)

    @property
    def rotmat(self) -> np.ndarray:
        return q_to_rotmat(self.q)

    @property
    def tilt_deg(self) -> float:
        """Angle between the body 'up' and the world 'up' (structural frame)."""
        up = self.rotmat @ np.array([0.0, 0.0, -1.0])
        return math.degrees(math.acos(max(-1.0, min(1.0, -up[2]))))

    def hover_frame_euler(self) -> tuple[float, float, float]:
        """Attitude as PX4 sees it (its level = the structural frame pitched by hover_pitch_deg)."""
        Rh = self.af.hover_rotation()
        Rp = self.rotmat @ Rh.T
        w = math.sqrt(max(0.0, 1 + Rp[0, 0] + Rp[1, 1] + Rp[2, 2])) / 2
        if w < 1e-9:
            return q_to_euler(self.q)
        x = (Rp[2, 1] - Rp[1, 2]) / (4 * w); y = (Rp[0, 2] - Rp[2, 0]) / (4 * w); z = (Rp[1, 0] - Rp[0, 1]) / (4 * w)
        return q_to_euler(q_normalize(np.array([w, x, y, z])))
