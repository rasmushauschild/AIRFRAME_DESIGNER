"""Wing segment: lifting surfaces described by planform and orientation, evaluated by strip theory.

A wing is a trapezoidal panel (or a mirrored pair of them) attached at its root leading edge:
  pos            root leading-edge point on the centreline (structural frame)
  span           tip-to-tip for a symmetric wing; panel length for a single panel (a fin)
  root_chord / tip_chord, sweep_deg (leading edge, positive = swept back)
  dihedral_deg   tips up positive; 90 with symmetric=False is a vertical fin
  incidence_deg  root chord angle above the structural x axis (leading edge up)
  twist_deg      extra incidence at the tip (washout negative)
  panels         spanwise strips per half; each strip carries its own angle of attack, so dihedral effect,
                 roll damping and pitch damping from tail surfaces come out of the model by themselves.

``aero`` holds the coefficients. Two section models:
  linear     CL = cl0 + a*alpha with a from Helmbold (2-D slope cl_alpha and the aspect ratio), induced drag
             CL^2 / (pi e AR), flat-plate blend past the stall angle
  polhamus   sharp-leading-edge delta: potential + vortex lift, drag CL tan(alpha) (the ATLAS delta)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np

from .frames import rodrigues, rot_y


@dataclass
class WingAeroCoefficients:
    model: str = "linear"        # "linear" | "polhamus" | "polar" (airfoil section data: root/tip polars vs alpha and Re)
    airfoil_root: str = ""       # polar model: section at the root, e.g. "naca23006" or a UIUC name ("e423")
    airfoil_tip: str = ""        # polar model: section at the tip (default: same as root)
    polar_source: str = "auto"   # "auto" (XFOIL if installed, else NeuralFoil) | "xfoil" | "neuralfoil"
    ncrit: float = 9.0           # transition criterion for the polar (9 = clean wind tunnel, lower = rough air)
    cl_alpha: float = 6.2832     # 2-D lift slope, 1/rad (2 pi thin airfoil)
    cl0: float = 0.0             # lift at zero geometric angle of attack (cambered sections)
    cd0: float = 0.02            # parasite drag
    oswald: float = 0.85         # span efficiency for induced drag (linear model)
    stall_deg: float = 15.0      # angle of attack where the section stalls
    stall_blend_deg: float = 15.0  # width of the blend into flat-plate behaviour
    cd_flat: float = 1.2         # flat-plate drag coefficient past the stall
    cm0: float = 0.0             # pitching moment about the quarter chord (nose-up positive)
    vortex_lift: bool = True     # polhamus: add the vortex-lift term


@dataclass
class Wing:
    name: str = "wing"
    enabled: bool = True
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])   # root leading edge, structural frame
    root_y: float = 0.0         # mirrored lateral root offset for joined, non-overlapping wing segments
    span: float = 1.0
    root_chord: float = 0.3
    tip_chord: float = 0.3
    sweep_deg: float = 0.0
    dihedral_deg: float = 0.0
    incidence_deg: float = 0.0   # section angle about the span line at the root, LE up positive
    twist_deg: float = 0.0
    pitch_deg: float = 0.0       # rigid rotation of the whole wing about the aircraft pitch axis through ``pos``, nose-up positive
    symmetric: bool = True       # left + right halves; False = one panel (fin, strake)
    panels: int = 6              # strips per half
    aspect_ratio: float | None = None   # override the geometric aspect ratio used for lift slope / induced drag
    aero: WingAeroCoefficients = field(default_factory=WingAeroCoefficients)

    # ------------------------------------------------------------ planform
    @property
    def half_span(self) -> float:
        return self.span / 2.0 if self.symmetric else self.span

    @property
    def area(self) -> float:
        """Total planform area (both halves for a symmetric wing)."""
        return (self.root_chord + self.tip_chord) / 2.0 * self.half_span * (2 if self.symmetric else 1)

    @property
    def ar(self) -> float:
        if self.aspect_ratio:
            return float(self.aspect_ratio)
        a = self.area
        b = self.span if self.symmetric else 2.0 * self.span   # a fin on a body behaves like half of a mirrored pair
        return (b * b / a) if a > 1e-9 else 1.0

    @property
    def mean_chord(self) -> float:
        return (self.root_chord + self.tip_chord) / 2.0

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "Wing":
        d = dict(d)
        aero = d.pop("aero", None) or {}
        w = cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})
        w.aero = WingAeroCoefficients(**{k: v for k, v in aero.items() if k in WingAeroCoefficients.__dataclass_fields__})
        w.panels = max(1, int(w.panels))
        return w

    @classmethod
    def delta(cls, area: float, span: float, pos_ac, incidence_deg: float = 0.0, name: str = "delta", **aero) -> "Wing":
        """A sharp-edged delta from its area and span, positioned by its aerodynamic centre (2/3 root chord
        behind the apex). This is the schema-1 wing."""
        c_r = 2.0 * area / span if span > 0 else 0.0
        sweep = math.degrees(math.atan2(c_r, span / 2.0)) if span > 0 else 0.0
        apex = [float(pos_ac[0]) + 2.0 * c_r / 3.0, float(pos_ac[1]), float(pos_ac[2])]
        w = cls(name=name, pos=apex, span=span, root_chord=c_r, tip_chord=0.0, sweep_deg=sweep,
                incidence_deg=incidence_deg, panels=6)
        w.aero = WingAeroCoefficients(model="polhamus", **aero)
        return w


def wing_panels(w: Wing) -> dict:
    """Strip geometry of one wing as numpy arrays (all in the structural frame):
    pos (n,3) quarter-chord points, e_c (n,3) chordwise unit vectors (forward, incidence applied),
    e_n (n,3) lift-direction unit vectors ("up" of the panel), e_s (n,3) spanwise (root to tip), pitch_axis (n,3)
    the axis a nose-up pitching moment acts about, area (n,), chord (n,), side (n,) +1 right / -1 left."""
    halves = [1.0, -1.0] if w.symmetric else [1.0]
    n = max(1, int(w.panels))
    dih = math.radians(w.dihedral_deg)
    swp = math.radians(w.sweep_deg)
    hs = w.half_span
    pos, e_c, e_n, e_s, pax, area, chord, side, eta = [], [], [], [], [], [], [], [], []
    root = np.asarray(w.pos, float)
    Rp = rot_y(math.radians(w.pitch_deg)) if abs(w.pitch_deg) > 1e-12 else None
    for sd in halves:
        es = np.array([0.0, sd * math.cos(dih), -math.sin(dih)])
        en0 = np.array([0.0, -sd * math.sin(dih), -math.cos(dih)])
        ec0 = np.array([1.0, 0.0, 0.0])
        pitch_axis = sd * es          # ~ +y for both halves: nose-up positive
        for k in range(n):
            s0, s1 = hs * k / n, hs * (k + 1) / n
            sm = 0.5 * (s0 + s1)
            c0 = w.root_chord + (w.tip_chord - w.root_chord) * s0 / hs if hs > 0 else w.root_chord
            c1 = w.root_chord + (w.tip_chord - w.root_chord) * s1 / hs if hs > 0 else w.root_chord
            cm = 0.5 * (c0 + c1)
            inc = math.radians(w.incidence_deg + w.twist_deg * (sm / hs if hs > 0 else 0.0))
            R = rodrigues(pitch_axis, inc)
            ec, en = R @ ec0, R @ en0
            le = root + np.array([0.0, sd * w.root_y, 0.0]) + es * sm - np.array([math.tan(swp) * sm, 0.0, 0.0])
            qc = le - ec * (cm / 4.0)
            if Rp is not None:                      # whole-wing pitch about the aircraft y axis through the root point
                qc = root + Rp @ (qc - root); ec = Rp @ ec; en = Rp @ en
            pos.append(qc); e_c.append(ec); e_n.append(en); e_s.append(Rp @ es if Rp is not None else es); pax.append(Rp @ pitch_axis if Rp is not None else pitch_axis)
            area.append((s1 - s0) * cm); chord.append(cm); side.append(sd); eta.append(sm / hs if hs > 0 else 0.0)
    return {"pos": np.array(pos).reshape(-1, 3), "e_c": np.array(e_c).reshape(-1, 3), "e_n": np.array(e_n).reshape(-1, 3),
            "e_s": np.array(e_s).reshape(-1, 3), "pitch_axis": np.array(pax).reshape(-1, 3),
            "area": np.array(area, float), "chord": np.array(chord, float), "side": np.array(side, float), "eta": np.array(eta, float)}


def wing_outline(w: Wing) -> list[list[list[float]]]:
    """Corner points of each half (root LE, tip LE, tip TE, root TE) for drawing, structural frame."""
    out = []
    dih, swp = math.radians(w.dihedral_deg), math.radians(w.sweep_deg)
    hs = w.half_span
    root = np.asarray(w.pos, float)
    Rp = rot_y(math.radians(w.pitch_deg))
    for sd in ([1.0, -1.0] if w.symmetric else [1.0]):
        es = np.array([0.0, sd * math.cos(dih), -math.sin(dih)])
        pitch_axis = sd * es
        pts = []
        for s, c, inc in ((0.0, w.root_chord, w.incidence_deg), (hs, w.tip_chord, w.incidence_deg + w.twist_deg)):
            R = rodrigues(pitch_axis, math.radians(inc))
            ec = R @ np.array([1.0, 0.0, 0.0])
            le = root + np.array([0.0, sd * w.root_y, 0.0]) + es * s - np.array([math.tan(swp) * s, 0.0, 0.0])
            le, te = root + Rp @ (le - root), root + Rp @ (le - ec * c - root)
            pts.append((le, te))
        (rle, rte), (tle, tte) = pts
        out.append([rle.tolist(), tle.tolist(), tte.tolist(), rte.tolist()])
    return out
