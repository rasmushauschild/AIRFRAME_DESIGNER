"""Rotor forces: thrust along the axis, PX4-consistent reaction torque, first-order spool-up, and the momentum
(ram) drag of ducted fans. Vectorised over all rotors."""
from __future__ import annotations

import numpy as np

from ..geometry.propulsion import Rotor
from .atmosphere import RHO


class RotorSet:
    def __init__(self, rotors: list[Rotor], cg):
        from .fastmath import skew_stack
        cg = np.asarray(cg, float)
        self.n = len(rotors)
        self.r = np.array([r.pos for r in rotors], float).reshape(self.n, 3) - cg
        # the axis the aircraft was built with, not the one it was drawn with (Rotor.built_axis)
        axes = np.array([r.built_axis for r in rotors], float).reshape(self.n, 3)
        norms = np.linalg.norm(axes, axis=1, keepdims=True); norms[norms < 1e-9] = 1.0
        self.axis = axes / norms
        self.km = np.array([r.km for r in rotors], float)
        self.tmax = np.array([r.effective_max_thrust() for r in rotors], float)
        self.tau = np.array([max(r.tau, 1e-3) for r in rotors], float)
        self.exponent = np.array([r.thrust_exponent for r in rotors], float)
        self.square = bool(self.n and np.all(np.abs(self.exponent - 2.0) < 1e-9))
        self.ram = np.ones(self.n, bool)          # every rotor is a ducted jet: the duct always swallows a mass flow
        self.area = np.array([r.disc_area for r in rotors], float)
        self.mdot_coef = np.sqrt(RHO * self.area) * self.ram          # mdot = coef * sqrt(thrust)
        self.any_ram = bool(self.ram.any())
        self.scale = np.ones(self.n)                                  # per-rotor health (1 = fine, 0 = dead)
        self.S = skew_stack(self.r)                                   # r x (.)
        self.intake_S = skew_stack(np.array([r.pos for r in rotors], float).reshape(self.n, 3) - cg)
        self.MA = np.cross(self.r, self.axis) - self.km[:, None] * self.axis   # moment per unit thrust
        # Ducted jet: the duct holds the slipstream's area, so the induced power is half an open rotor's for
        # the same disc. Every rotor here is ducted, so there is one coefficient and no branch.
        self.power_coef = 1.0 / (2 * np.sqrt(RHO * np.maximum(self.area, 1e-9)))

        # Calibrate the T^1.5 curve to electrical input at full thrust.
        for i, rotor in enumerate(rotors):
            if np.isfinite(rotor.max_power_w) and rotor.max_power_w > 0 and self.tmax[i] > 0:
                self.power_coef[i] = rotor.max_power_w / self.tmax[i] ** 1.5

    def thrust(self, omega: np.ndarray) -> np.ndarray:
        om = np.clip(omega, 0.0, 1.0)
        return self.tmax * self.scale * (om * om if self.square else np.power(om, self.exponent))

    def forces(self, omega: np.ndarray, v_air_body: np.ndarray, rates: np.ndarray, detail: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """Force, moment about the CG, per-rotor thrust, and the total ram-drag magnitude."""
        if self.n == 0:
            return np.zeros(3), np.zeros(3), np.zeros(0), 0.0
        thrust = self.thrust(omega)
        F = thrust @ self.axis
        M = thrust @ self.MA
        ram = 0.0
        if self.any_ram:
            # the inlet swallows mdot = sqrt(rho A T) of air arriving with the local airspeed; that momentum is lost
            mdot = self.mdot_coef * np.sqrt(thrust)
            V = v_air_body - np.einsum("nij,j->ni", self.intake_S, rates)
            f = -mdot[:, None] * V
            fs = f.sum(axis=0)
            F = F + fs
            M = M + np.einsum("nij,nj->i", self.intake_S, f)
            if detail:
                ram = float(np.sqrt(fs @ fs))
        return F, M, thrust, ram

    def ideal_power(self, thrust: np.ndarray) -> float:
        """Estimated power: rated electrical T^1.5 curve when supplied, otherwise ideal momentum theory."""
        t = np.maximum(thrust, 0.0)
        return float((t * np.sqrt(t) * self.power_coef).sum())
