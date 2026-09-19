"""Mass segment: total mass, centre of gravity, inertia tensor.

The CG lives here (structural frame) so that every other segment can keep its positions fixed while the CG
is moved, e.g. when a battery slides forward. When ``items`` are given and ``from_items`` is set, the mass,
CG and inertia are computed from those point masses (plus the structure's own contribution)."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict

import numpy as np


@dataclass
class MassItem:
    name: str = "item"
    mass: float = 0.0                                      # kg
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # structural frame, m
    inertia: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # own inertia about its own CG, kg m^2
    inertia_products: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # Ixy Ixz Iyz of that own inertia


@dataclass
class MassProperties:
    mass: float = 1.5                                               # kg, total
    cg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # centre of gravity, structural frame
    inertia: list[float] = field(default_factory=lambda: [0.02, 0.02, 0.035])   # Ixx Iyy Izz about the CG, body axes
    inertia_products: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # Ixy Ixz Iyz (with the -sign convention of the tensor)
    items: list[MassItem] = field(default_factory=list)            # optional component masses
    from_items: bool = False                                        # derive mass/cg/inertia from ``items``

    # ------------------------------------------------------------------ tensor
    def tensor(self) -> np.ndarray:
        ixx, iyy, izz = (float(v) for v in self.inertia)
        ixy, ixz, iyz = (float(v) for v in self.inertia_products)
        return np.array([[ixx, -ixy, -ixz], [-ixy, iyy, -iyz], [-ixz, -iyz, izz]])

    def resolve(self, extra: list["MassItem"] | None = None) -> "MassProperties":
        """Apply ``from_items``: totals from the component list plus ``extra`` items (e.g. the CAD bodies of
        geometry/cad.py); returns self for chaining."""
        items = list(self.items) + list(extra or [])
        if self.from_items and items:
            m = sum(max(0.0, i.mass) for i in items)
            if m > 0:
                cg = sum(np.array(i.pos, float) * i.mass for i in items) / m
                I = np.zeros((3, 3))
                for it in items:
                    r = np.array(it.pos, float) - cg
                    ixy, ixz, iyz = (list(it.inertia_products) + [0, 0, 0])[:3]
                    own = np.diag(it.inertia) - np.array([[0, ixy, ixz], [ixy, 0, iyz], [ixz, iyz, 0]], float)
                    I += own + it.mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
                self.mass = float(m)
                self.cg = [round(float(v), 5) for v in cg]
                self.inertia = [round(float(I[0, 0]), 6), round(float(I[1, 1]), 6), round(float(I[2, 2]), 6)]
                self.inertia_products = [round(float(-I[0, 1]), 6), round(float(-I[0, 2]), 6), round(float(-I[1, 2]), 6)]
        return self

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "MassProperties":
        d = dict(d or {})
        items = [MassItem(**{k: v for k, v in i.items() if k in MassItem.__dataclass_fields__}) for i in d.pop("items", []) or []]
        mp = cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})
        mp.items = items
        return mp.resolve()


def estimate_inertia(mass: float, cg, body_size, rotor_positions, wing_panels=None, legs=None,
                     body_fraction: float = 0.55, motor_mass: float | None = None) -> list[float]:
    """Rough inertia about the CG: a central body block, point-mass motors at the rotor positions, the wings as
    thin plates (their panel areas at their panel positions) and the legs as thin rods.

    The split of the mass is heuristic (body_fraction of the total in the body block, the rest shared between
    motors, wings and legs by count) - good enough to start a simulation, refine from CAD when available."""
    cg = np.asarray(cg, float)
    n_rot = max(1, len(rotor_positions))
    m_body = mass * body_fraction
    rest = mass - m_body
    wing_panels = wing_panels or []
    legs = legs or []
    n_extra = n_rot + (1 if wing_panels else 0) + len(legs)
    m_rot = (rest / n_extra) if motor_mass is None else motor_mass
    bx, by, bz = body_size
    I = np.array([m_body * (by ** 2 + bz ** 2) / 12, m_body * (bx ** 2 + bz ** 2) / 12, m_body * (bx ** 2 + by ** 2) / 12])

    def point(m, p):
        r = np.asarray(p, float) - cg
        return m * np.array([r[1] ** 2 + r[2] ** 2, r[0] ** 2 + r[2] ** 2, r[0] ** 2 + r[1] ** 2])

    for p in rotor_positions:
        I += point(m_rot, p)
    if wing_panels:
        m_w = rest / n_extra
        total_area = sum(a for _, a in wing_panels) or 1.0
        for p, a in wing_panels:
            I += point(m_w * a / total_area, p)
    for leg in legs:
        a, f = np.asarray(leg[0], float), np.asarray(leg[1], float)
        I += point(rest / n_extra, (a + f) / 2)
    return [round(float(v), 6) for v in I]
