"""Propulsion segment: rotors - open propellers, ducted fans, and ducted fans whose jet is bent by a jetfoil.

Conventions (identical to PX4's control allocation):
  * ``axis`` is the unit thrust direction in the structural frame; a rotor pushing the vehicle up has axis (0,0,-1).
  * ``km`` is CA_ROTORn_KM: reaction torque on the body = -km * thrust * axis. km > 0 spins CCW seen from above.
  * Rotor i is PX4 "Motor i+1" (output function 101 + i).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

from .frames import unit, tilt_cant_to_axis, axis_to_tilt_cant

ROTOR_KINDS = {
    # defaults applied when a rotor is switched to this kind (km keeps its sign = spin direction)
    "prop": {"km": 0.05, "tau": 0.04, "diameter": 0.25, "thrust_exponent": 2.0, "ram_drag": False},
    # Electric ducted fan: stator vanes cancel most of the swirl (tiny reaction torque), a small heavy rotor at
    # high rpm spools slower, and the duct swallows a mass flow that produces momentum ("ram") drag in crossflow.
    "ducted": {"km": 0.01, "tau": 0.12, "diameter": 0.12, "thrust_exponent": 2.0, "ram_drag": True},
}


@dataclass
class Rotor:
    name: str = ""
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])      # structural frame, m
    axis: list[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])    # unit thrust direction
    km: float = 0.05             # moment coefficient, signed (spin direction)
    max_thrust: float = 8.0      # N at full command (before jetfoil turning loss)
    max_power_w: float = 0.0    # electrical watts at full thrust; 0 retains ideal-power estimate
    tau: float = 0.04            # spool-up time constant, s
    diameter: float = 0.25       # prop / fan diameter, m (disc area for ram drag and momentum-theory power)
    thrust_exponent: float = 2.0  # thrust = max_thrust * omega_norm ** exponent
    kind: str = "prop"           # "prop" | "ducted"
    ram_drag: bool = False       # momentum drag of the inlet mass flow
    duct_axis: list[float] | None = None   # physical fan axis when a jetfoil bends the jet to ``axis`` (None: fan along the jet)
    turn_loss: float = 0.1       # fraction of thrust lost when the jetfoil bends the jet by 90 degrees
    enabled: bool = True
    intake_pos: list[float] | None = None  # structural frame; None follows thrust origin (legacy)

    def intake_position(self) -> list[float]:
        return self.intake_pos if self.kind == "ducted" and self.intake_pos is not None else self.pos

    # ------------------------------------------------------------ derived
    def deflection_deg(self) -> float:
        """Angle the jetfoil bends the jet: between the duct axis and the thrust axis (0 without a jetfoil)."""
        if not self.duct_axis:
            return 0.0
        a, d = unit(self.axis), unit(self.duct_axis)
        return math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, d))))))

    def effective_max_thrust(self) -> float:
        if not self.enabled:
            return 0.0
        return self.max_thrust * max(0.0, 1.0 - self.turn_loss * self.deflection_deg() / 90.0)

    @property
    def disc_area(self) -> float:
        return math.pi * (self.diameter / 2.0) ** 2

    @property
    def ccw(self) -> bool:
        return self.km >= 0.0

    @property
    def side(self) -> float:
        return -1.0 if self.pos[1] < 0 else 1.0

    # tilt / cant are virtual attributes over ``axis`` so optimisers and the UI can drive them directly
    @property
    def tilt_deg(self) -> float:
        return axis_to_tilt_cant(self.axis, self.side)[0]

    @tilt_deg.setter
    def tilt_deg(self, v: float) -> None:
        self.axis = tilt_cant_to_axis(float(v), self.cant_deg, self.side)

    @property
    def cant_deg(self) -> float:
        return axis_to_tilt_cant(self.axis, self.side)[1]

    @cant_deg.setter
    def cant_deg(self, v: float) -> None:
        self.axis = tilt_cant_to_axis(self.tilt_deg, float(v), self.side)

    # ------------------------------------------------------------ editing
    def set_kind(self, kind: str) -> "Rotor":
        d = ROTOR_KINDS.get(kind, ROTOR_KINDS["prop"])
        self.kind = kind
        self.km = (1.0 if self.km >= 0 else -1.0) * d["km"]
        self.tau, self.diameter, self.thrust_exponent, self.ram_drag = d["tau"], d["diameter"], d["thrust_exponent"], d["ram_drag"]
        if kind != "ducted":
            self.duct_axis = None
        return self

    def normalized(self) -> "Rotor":
        self.axis = unit(self.axis)
        if self.duct_axis:
            self.duct_axis = unit(self.duct_axis, (1.0, 0.0, 0.0))
        return self

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Rotor":
        d = dict(d)
        if "prop_diameter" in d and "diameter" not in d:     # schema 1 name
            d["diameter"] = d.pop("prop_diameter")
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__}).normalized()
