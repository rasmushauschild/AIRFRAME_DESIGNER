"""Propulsion segment: rotors. Every rotor is a ducted jet. There is nothing else to choose.

Conventions (identical to PX4's control allocation):
  * ``axis`` is the unit thrust direction in the structural frame; a rotor pushing the vehicle up has axis (0,0,-1).
  * ``km`` is CA_ROTORn_KM: reaction torque on the body = -km * thrust * axis. km > 0 spins CCW seen from above.
  * Rotor i is PX4 "Motor i+1" (output function 101 + i).

A duct always swallows a mass flow, so momentum ("ram") drag in crossflow is always modelled; it is not a
switch. ``max_thrust`` is the thrust the rotor actually delivers along ``axis``, with no hidden derating.

Earlier versions had a ``kind`` (open propeller or ducted fan) and an optional jetfoil that bent the jet away
from the fan's own axis, charging a ``turn_loss`` for the bend. Both are gone. A file written by that version
is migrated on load: the bend's thrust penalty is folded into ``max_thrust``, so the aircraft keeps exactly
the thrust it had, and the geometry that described the bend is dropped. ``axis`` was always the direction the
thrust actually pointed, so nothing is re-aimed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

from .frames import unit, tilt_cant_to_axis, axis_to_tilt_cant

@dataclass
class Rotor:
    name: str = ""
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])      # structural frame, m
    axis: list[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])    # unit thrust direction
    km: float = 0.01             # moment coefficient, signed (spin direction)
    max_thrust: float = 8.0      # N delivered along ``axis`` at full command
    max_power_w: float = 0.0     # electrical watts at full thrust; 0 retains ideal-power estimate
    tau: float = 0.12            # spool-up time constant, s
    diameter: float = 0.12       # fan diameter, m (disc area for ram drag and momentum-theory power)
    thrust_exponent: float = 2.0  # thrust = max_thrust * omega_norm ** exponent
    enabled: bool = True
    pair: str = ""               # two rotors sharing a token are a left/right couple: the app keeps their
                                 # geometry mirrored about y. Each still has its own output, name and spin.
    # How the rotor really sits, when that is not how it was drawn: degrees added to the design's tilt and
    # cant. The simulation flies the sum; nothing else knows. See ``built_axis``.
    tilt_error_deg: float = 0.0
    cant_error_deg: float = 0.0

    # ------------------------------------------------------------ derived
    def effective_max_thrust(self) -> float:
        return self.max_thrust if self.enabled else 0.0

    @property
    def disc_area(self) -> float:
        return math.pi * (self.diameter / 2.0) ** 2

    @property
    def ccw(self) -> bool:
        return self.km >= 0.0

    @property
    def side(self) -> float:
        return -1.0 if self.pos[1] < 0 else 1.0

    @property
    def mis_mounted(self) -> bool:
        return abs(self.tilt_error_deg) > 1e-9 or abs(self.cant_error_deg) > 1e-9

    @property
    def built_axis(self) -> list[float]:
        """The thrust axis the aircraft actually has: the design turned by the mounting error.

        ``axis`` is the design. It is what the table edits, what an optimiser moves, what the analysis
        reasons about and - the point of keeping the two apart - what PX4 is told, because a motor bolted on
        ten degrees out is not a different design, it is the same design built wrong. Only the physics reads
        this one, so a controller tuned for a perfect aircraft flies the imperfect one, as it does in a hangar.
        """
        if not self.mis_mounted:
            return unit(self.axis)
        tilt, cant = axis_to_tilt_cant(self.axis, self.side)
        return tilt_cant_to_axis(tilt + self.tilt_error_deg, cant + self.cant_error_deg, self.side)

    @property
    def built_tilt_cant(self) -> tuple[float, float]:
        """Tilt and cant of the axis the aircraft really has, degrees."""
        return axis_to_tilt_cant(self.built_axis, self.side)

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
    def normalized(self) -> "Rotor":
        self.axis = unit(self.axis)
        return self

    def to_dict(self) -> dict:
        d = asdict(self)
        for k in ("tilt_error_deg", "cant_error_deg"):     # a rotor mounted as designed says nothing about it
            if not d[k]:
                d.pop(k)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Rotor":
        d = dict(d)
        if "prop_diameter" in d and "diameter" not in d:     # schema 1 name
            d["diameter"] = d.pop("prop_diameter")
        # A jetfoil rotor from an earlier version: its thrust was derated for the bend, and `axis` already
        # pointed where that bent jet pushed. Fold the derating into max_thrust so the aircraft is unchanged,
        # then drop the jetfoil fields. Without this the aircraft would quietly gain thrust on load.
        duct = d.pop("duct_axis", None)
        loss = float(d.pop("turn_loss", 0.1) or 0.0)
        d.pop("kind", None)
        d.pop("ram_drag", None)
        d.pop("intake_pos", None)
        if duct:
            a, b = unit(d.get("axis", (0.0, 0.0, -1.0))), unit(duct, (1.0, 0.0, 0.0))
            bend = math.degrees(math.acos(max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))))
            if bend > 1e-9 and "max_thrust" in d:
                d["max_thrust"] = float(d["max_thrust"]) * max(0.0, 1.0 - loss * bend / 90.0)
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__}).normalized()
