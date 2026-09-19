"""Static force/moment model of an airframe: PX4's allocation for hover, and a steady-cruise trim solve with
the same wing, ram-drag and body-drag models the simulation uses (aero segment), so what the analysis says
about a design is what the simulation will show, minus the dynamics."""
from __future__ import annotations

import math

import numpy as np

from ..aero import RotorSet, WingSet, BodyAero
from ..geometry.airframe import Airframe

G = 9.80665


def pitch_rotation(theta: float) -> np.ndarray:
    """World <- body for a nose-up pitch theta (level flight along world x, z down)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class VehicleModel:
    def __init__(self, af: Airframe):
        self.af = af
        af.resolve_mass()
        from ..aero.airfoils import ensure_polars
        ensure_polars(af)
        cg = af.cg
        rotors = af.active_rotors()
        self.n = len(rotors)
        self.rotors = RotorSet(rotors, cg)
        self.wings = WingSet(af.active_wings(), cg)
        self.body = BodyAero(af.body, cg)
        self.tmax = self.rotors.tmax
        self.mass = float(af.mass.mass)
        self.E = af.effectiveness()
        self.E_pinv = np.linalg.pinv(self.E) if self.n else np.zeros((0, 6))

    def allocate(self, sp: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """PX4's pseudo-inverse allocation for [torque xyz, force xyz] in the hover frame: raw and clipped thrusts."""
        u = self.E_pinv @ sp
        return u, np.clip(u, 0.0, self.tmax)

    def forces(self, theta: float, thrust: np.ndarray, airspeed: float) -> dict:
        """Net force and moment (structural frame, about the CG) in steady flight at nose-up pitch ``theta``
        and airspeed along world x."""
        Rwb = pitch_rotation(theta)
        v = Rwb.T @ np.array([airspeed, 0.0, 0.0])
        zero = np.zeros(3)
        omega = np.power(np.clip(thrust / np.maximum(self.tmax, 1e-9), 0, 1), 1.0 / self.rotors.exponent) if self.n else np.zeros(0)
        F_r, M_r, _, ram = self.rotors.forces(omega, v, zero)
        F_w, M_w, wb = self.wings.forces(v, zero)
        F_b, M_b, body_drag = self.body.forces(v, zero)
        F = F_r + F_w + F_b + Rwb.T @ np.array([0.0, 0.0, self.mass * G])
        M = M_r + M_w + M_b
        return {"F": F, "M": M, "ram": ram, "body_drag": body_drag, "lift": wb["lift"], "wing_drag": wb["drag"],
                "alpha": (max(wb["alpha"], key=abs) if wb["alpha"] else None), "stalled": wb["stalled"]}

    # ------------------------------------------------------------ hover
    def hover(self) -> dict:
        weight = self.mass * G
        if self.n == 0:
            return {"ok": False, "max_util": float("inf"), "problems": ["no rotors"], "thrust": [], "util": [],
                    "negative": [], "power": 0.0, "authority": {}, "waste": 0.0, "total_thrust": 0.0}
        u_raw, _ = self.allocate(np.array([0, 0, 0, 0, 0, -1.0]))
        thrust_up = float(-(self.E[5] @ u_raw))
        if thrust_up <= 1e-9:
            return {"ok": False, "max_util": float("inf"), "problems": ["no lift"], "thrust": [], "util": [],
                    "negative": [], "power": 0.0, "authority": {}, "waste": 0.0, "total_thrust": 0.0}
        u_raw = u_raw * (weight / thrust_up)
        resid = self.E @ u_raw - np.array([0, 0, 0, 0, 0, -weight])
        negative = [i + 1 for i, v in enumerate(u_raw) if v < -1e-6]
        thrust = np.clip(u_raw, 0.0, None)
        util = thrust / np.maximum(self.tmax, 1e-9)
        auth = {}
        for k, name in enumerate(("roll", "pitch", "yaw")):
            sp = np.zeros(6); sp[k] = 1.0
            du = self.E_pinv @ sp
            lim = []
            for sign in (1.0, -1.0):
                d = du * sign
                with np.errstate(divide="ignore", invalid="ignore"):
                    room = np.where(d > 1e-9, (self.tmax - thrust) / d, np.where(d < -1e-9, -thrust / d, np.inf))
                lim.append(float(room.min()) if len(room) else 0.0)
            auth[name] = min(lim)
        force_resid = float(np.abs(resid[3:5]).max()); rp_resid = float(np.abs(resid[:2]).max()); yaw_resid = float(abs(resid[2]))
        problems = []
        if force_resid > 1e-3 * weight or rp_resid > 1e-3 * weight:
            problems.append("hover needs a net force or torque no motor mix gives")
        if negative:
            problems.append(f"allocator wants negative thrust on motor(s) {negative}")
        if yaw_resid > 2e-3 * weight:
            problems.append("no yaw authority")
        return {"ok": not problems, "problems": problems, "thrust": thrust.tolist(), "util": util.tolist(),
                "max_util": float(util.max()), "negative": negative, "power": self.rotors.ideal_power(thrust),
                "total_thrust": float(thrust.sum()), "waste": float(thrust.sum() / weight - 1.0),
                "authority": auth, "residual_force": force_resid, "residual_yaw": yaw_resid}

    # ----------------------------------------------------------- cruise
    def cruise(self, airspeed: float, tilt_limit_deg: float = 45.0) -> dict:
        """Steady level flight at ``airspeed``: solve body pitch, collective and pitch torque for zero net force
        and pitching moment with PX4's allocation in the loop."""
        if self.n == 0:
            return {"ok": False, "problems": ["no rotors"], "converged": False}
        weight = self.mass * G
        hover_phi = math.radians(self.af.hover_pitch_deg)

        def residual(x):
            theta, tz, tau = x
            u_raw, u = self.allocate(np.array([0.0, tau, 0.0, 0.0, 0.0, -tz]))
            f = self.forces(theta, u, airspeed)
            return np.array([f["F"][0], f["F"][2], f["M"][1]]), u_raw, u, f

        best = None
        for theta0 in (hover_phi - 0.4, hover_phi, hover_phi - 0.8, hover_phi + 0.3):
            x = np.array([theta0, weight * 0.8, 0.0])
            lam = 1e-2
            r, *_ = residual(x)
            stall_count = 0
            for _ in range(40):
                J = np.zeros((3, 3))
                h = np.array([1e-4, 1e-2, 1e-3])
                for j in range(3):
                    xp = x.copy(); xp[j] += h[j]
                    J[:, j] = (residual(xp)[0] - r) / h[j]
                try:
                    dx = -np.linalg.solve(J.T @ J + lam * np.diag(np.diag(J.T @ J) + 1e-9), J.T @ r)
                except np.linalg.LinAlgError:
                    break
                xn = x + dx
                xn[0] = float(np.clip(xn[0], -math.pi / 2, math.pi / 2)); xn[1] = max(0.0, xn[1])
                rn, *_ = residual(xn)
                if np.linalg.norm(rn) < np.linalg.norm(r):
                    x, r, lam = xn, rn, max(lam / 3, 1e-6); stall_count = 0
                else:
                    lam = min(lam * 5, 1e3); stall_count += 1
                if np.linalg.norm(r) < 1e-3 * weight or stall_count >= 6:
                    break
            if best is None or np.linalg.norm(r) < np.linalg.norm(best[1]):
                best = (x.copy(), r.copy())
            if np.linalg.norm(best[1]) < 1e-3 * weight:
                break
        x, r = best
        r, u_raw, u, f = residual(x)
        theta, tz, tau = (float(v) for v in x)
        converged = bool(np.linalg.norm(r) < 5e-3 * weight)
        px4_pitch = theta - hover_phi
        util = u / np.maximum(self.tmax, 1e-9)
        saturated = [i + 1 for i in range(self.n) if u_raw[i] > self.tmax[i] + 1e-6]
        negative = [i + 1 for i in range(self.n) if u_raw[i] < -1e-6]
        problems = []
        if not converged:
            problems.append("no steady state found at this speed")
        if saturated:
            problems.append(f"motor(s) {saturated} saturated in cruise")
        if negative:
            problems.append(f"allocator wants negative thrust on motor(s) {negative} in cruise")
        if abs(math.degrees(px4_pitch)) > tilt_limit_deg:
            problems.append(f"PX4 pitch {math.degrees(px4_pitch):.0f}° exceeds MPC_TILTMAX_AIR ({tilt_limit_deg:g}°)")
        alpha = f["alpha"]
        if f["stalled"]:
            problems.append(f"wing stalled ({math.degrees(alpha):.0f}° angle of attack)")
        return {"ok": not problems, "converged": converged, "problems": problems, "airspeed": airspeed,
                "pitch_deg": math.degrees(theta), "px4_pitch_deg": math.degrees(px4_pitch), "collective": tz,
                "pitch_torque": tau, "thrust": u.tolist(), "util": util.tolist(), "max_util": float(util.max()),
                "total_thrust": float(u.sum()), "power": self.rotors.ideal_power(u), "lift": f["lift"],
                "lift_share": f["lift"] / weight, "wing_drag": f["wing_drag"], "ram_drag": f["ram"], "body_drag": f["body_drag"],
                "alpha_deg": (math.degrees(alpha) if alpha is not None else None), "saturated": saturated,
                "negative": negative, "residual": float(np.linalg.norm(r))}


def analyse(af: Airframe, airspeed: float | None = None, tilt_limit_deg: float = 45.0) -> dict:
    m = VehicleModel(af)
    speed = airspeed if airspeed is not None else float(af.design.get("cruise_speed_kmh", 50.0)) / 3.6
    h = m.hover()
    c = m.cruise(speed, tilt_limit_deg)
    c["power_ratio"] = (c["power"] / h["power"]) if (h.get("power") and c.get("power") is not None) else None
    return {"hover": h, "cruise": c, "airspeed": speed}
