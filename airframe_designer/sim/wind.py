"""The wind in the simulation: either a steady vector or one that keeps gusting.

Direction is where the wind comes FROM, degrees clockwise from north, as every weather report gives it; the
vector it becomes is where the air is going, NED, which is what the aerodynamics want.

A gust here is not a random number per step. Sampling noise at the sensor rate gives the airframe a shaking
that no real air has and that the estimator reads as vibration; real gusty wind is a slow, smooth wander. So
the model is a sum of three sines with incommensurate periods, which is smooth, repeats only after a long
time, and - being a function of time, not a random walk - gives the same wind to every run of the same
scenario. That is what makes a gusty run worth comparing with another gusty run.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# the three sines, as fractions of the gust period, and their weights
_HARMONICS = ((1.0, 0.60), (0.37, 0.27), (2.30, 0.26))
_NORM = sum(w for _, w in _HARMONICS)


@dataclass
class WindModel:
    """The wind the simulation blows. ``at(t)`` is the only thing the loop asks for."""
    speed: float = 0.0            # mean speed, m/s
    direction_deg: float = 0.0    # where it comes from, degrees clockwise from north
    down: float = 0.0             # steady vertical component, m/s, positive downwards
    gusty: bool = False
    gust_speed: float = 0.0       # how much the speed swings either side of the mean, m/s
    gust_period_s: float = 8.0    # roughly how long one gust takes
    gust_dir_deg: float = 0.0     # how far the direction swings either side, degrees
    seed: int = 1

    def _wave(self, t: float, which: int) -> float:
        """A smooth signal in [-1, 1], different for each `which`, repeating only after a long time."""
        rng = np.random.default_rng((int(self.seed) & 0xFFFF) * 7 + which)
        period = max(0.5, float(self.gust_period_s))
        total = 0.0
        for (f, w), phase in zip(_HARMONICS, rng.uniform(0, 2 * math.pi, len(_HARMONICS))):
            total += w * math.sin(2 * math.pi * t / (period * f) + float(phase))
        return total / _NORM

    def speed_dir_at(self, t: float) -> tuple[float, float, float]:
        """(speed m/s, direction it comes from in degrees, vertical m/s) at simulated time t."""
        if not self.gusty:
            return max(0.0, self.speed), self.direction_deg, self.down
        speed = max(0.0, self.speed + self.gust_speed * self._wave(t, 0))
        direction = self.direction_deg + self.gust_dir_deg * self._wave(t, 1)
        # a gust that only pushes sideways is half a gust: the same air moves up and down as it tumbles
        down = self.down + 0.25 * self.gust_speed * self._wave(t, 2)
        return speed, direction, down

    def at(self, t: float) -> np.ndarray:
        """The wind vector in NED, m/s: where the air is going."""
        speed, direction, down = self.speed_dir_at(t)
        a = math.radians(direction)
        return np.array([-speed * math.cos(a), -speed * math.sin(a), down], float)

    def preview(self, seconds: float = 60.0, step: float = 0.25) -> dict:
        """The speed and direction this model will blow over the next `seconds`, for the UI to draw."""
        n = max(2, int(seconds / max(0.01, step)) + 1)
        ts = [i * step for i in range(n)]
        sd = [self.speed_dir_at(t) for t in ts]
        return {"t": ts, "speed": [round(s, 3) for s, _, _ in sd], "dir": [round(d, 2) for _, d, _ in sd],
                "down": [round(w, 3) for _, _, w in sd]}

    def to_dict(self) -> dict:
        return {"speed": self.speed, "direction_deg": self.direction_deg, "down": self.down,
                "gusty": bool(self.gusty), "gust_speed": self.gust_speed,
                "gust_period_s": self.gust_period_s, "gust_dir_deg": self.gust_dir_deg, "seed": int(self.seed)}

    @classmethod
    def from_dict(cls, d: dict | None) -> "WindModel":
        d = dict(d or {})
        if "north" in d or "east" in d:            # a plain vector: a steady wind, no gusts
            return cls.from_vector(float(d.get("north", 0.0)), float(d.get("east", 0.0)), float(d.get("down", 0.0)))
        return cls(speed=max(0.0, float(d.get("speed", 0.0))), direction_deg=float(d.get("direction_deg", 0.0)),
                   down=float(d.get("down", 0.0)), gusty=bool(d.get("gusty", False)),
                   gust_speed=max(0.0, float(d.get("gust_speed", 0.0))),
                   gust_period_s=max(0.5, float(d.get("gust_period_s", 8.0))),
                   gust_dir_deg=max(0.0, float(d.get("gust_dir_deg", 0.0))),
                   seed=int(d.get("seed", 1)))

    @classmethod
    def from_vector(cls, north: float, east: float, down: float = 0.0) -> "WindModel":
        speed = float(math.hypot(north, east))
        direction = 0.0 if speed < 1e-9 else (math.degrees(math.atan2(-east, -north)) + 360.0) % 360.0
        return cls(speed=speed, direction_deg=direction, down=float(down))
