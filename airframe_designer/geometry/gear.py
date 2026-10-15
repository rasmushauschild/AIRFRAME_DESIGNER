"""Landing gear segment: individual legs, each with its own attachment point, length, angles and contact
properties. The foot is where the leg meets the ground; the simulator applies a spring-damper with friction
there, so an asymmetric or too-short set of legs tips the vehicle over as it would in reality."""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np


@dataclass
class Leg:
    name: str = "leg"
    attach: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # where the leg meets the structure
    length: float = 0.2           # m, attachment to foot
    tilt_deg: float = 0.0         # lean of the leg forward (+) / backward (-) from straight down
    cant_deg: float = 0.0         # lean outward (+) from the centreline / inward (-)
    foot_radius: float = 0.0      # m, contact starts this far above the foot point (a ball or skid tube)
    stiffness: float = 3000.0     # N/m
    damping: float = 150.0        # N/(m/s)
    friction: float = 0.8         # Coulomb coefficient at the foot
    enabled: bool = True
    pair: str = ""                # two legs sharing a token are a left/right couple, kept mirrored about y

    @property
    def side(self) -> float:
        return -1.0 if self.attach[1] < 0 else 1.0

    def direction(self) -> np.ndarray:
        """Unit vector from the attachment to the foot, structural frame (down when both angles are zero)."""
        t, c = math.radians(self.tilt_deg), math.radians(self.cant_deg) * self.side
        return np.array([math.sin(t), math.sin(c) * math.cos(t), math.cos(c) * math.cos(t)])

    def foot(self) -> list[float]:
        return (np.asarray(self.attach, float) + self.length * self.direction()).tolist()

    @classmethod
    def from_points(cls, attach, foot, name: str = "leg", **kw) -> "Leg":
        """A leg that reaches from ``attach`` to ``foot``."""
        a, f = np.asarray(attach, float), np.asarray(foot, float)
        d = f - a
        length = float(np.linalg.norm(d))
        leg = cls(name=name, attach=a.tolist(), length=length, **kw)
        if length > 1e-9:
            u = d / length
            tilt = math.degrees(math.asin(max(-1.0, min(1.0, u[0]))))
            cant = math.degrees(math.atan2(u[1], u[2])) if math.hypot(u[1], u[2]) > 1e-9 else 0.0
            leg.tilt_deg = round(tilt, 3)
            leg.cant_deg = round(cant * leg.side, 3)
        return leg

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Leg":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


def generate_legs(height: float, spread_x: float, spread_y: float, landed_pitch_deg: float = 0.0,
                  attach_z: float = 0.0, cg=(0.0, 0.0, 0.0), mass: float | None = None, **kw) -> list[Leg]:
    """Four legs whose feet lie on a plane perpendicular to gravity when the airframe stands at
    ``landed_pitch_deg`` nose-up, ``height`` below the CG along the landed 'down'. Attachment points sit at the
    body corners (±spread) at ``attach_z``. This reproduces the schema-1 generated feet."""
    phi = math.radians(landed_pitch_deg)
    c, s = math.cos(phi), math.sin(phi)
    cg = np.asarray(cg, float)
    legs = []
    if mass is not None and "stiffness" not in kw:
        # size the springs for 2 cm of static sink and a damping ratio of 0.8 (no bouncing)
        w_leg = mass * 9.80665 / 4.0
        kw["stiffness"] = round(w_leg / 0.02, 1)
        kw["damping"] = round(2.0 * 0.8 * math.sqrt(kw["stiffness"] * mass / 4.0), 1)
    for name, (x, y) in zip(("FR", "FL", "RR", "RL"), ((spread_x, spread_y), (spread_x, -spread_y), (-spread_x, spread_y), (-spread_x, -spread_y))):
        foot = cg + np.array([c * x - s * height, y, s * x + c * height])   # landed frame (x, y, h) -> structural
        attach = cg + np.array([x, y, attach_z])
        legs.append(Leg.from_points(attach, foot, name=name, **kw))
    return legs
