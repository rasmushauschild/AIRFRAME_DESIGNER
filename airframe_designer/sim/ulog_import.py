"""A flight controller's own log, read into the same columns everything else in this app is plotted in.

The .ulg on the board's SD card is the real record of a flight: every topic at the rate the controller
publishes it, with nothing dropped by a radio. What it is not is this app's recording format, so this maps
one onto the other - the same `COLUMNS`, the same per-rotor triples - and the result is a run like any
other: it lands in the run list, the graph plots it, and the 3D view scrubs through it.

Where a quantity does not exist in the log it stays at zero rather than being invented. Two in particular
are honestly absent: a real aircraft reports its motor *outputs*, not the thrust they made, so the thrust
column of an imported run is zero; and the wind it flew through is nowhere in its log at all.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .recorder import COLUMNS, columns_for

#: topic -> the fields taken from it. Nothing else is read, so an unusual logging profile costs nothing.
WANTED = {
    "vehicle_local_position": ("x", "y", "z", "vx", "vy", "vz", "heading"),
    "vehicle_attitude": ("q[0]", "q[1]", "q[2]", "q[3]"),
    "vehicle_angular_velocity": ("xyz[0]", "xyz[1]", "xyz[2]"),
    "sensor_combined": ("accelerometer_m_s2[0]", "accelerometer_m_s2[1]", "accelerometer_m_s2[2]",
                        "gyro_rad[0]", "gyro_rad[1]", "gyro_rad[2]"),
    "vehicle_air_data": ("baro_alt_meter",),
    "battery_status": ("remaining", "voltage_v", "current_a"),
    "vehicle_status": ("arming_state", "nav_state"),
    "actuator_outputs": tuple(f"output[{i}]" for i in range(16)),
    "actuator_motors": tuple(f"control[{i}]" for i in range(12)),
    "vehicle_air_data_0": (),
    "airspeed_validated": ("true_airspeed_m_s",),
    "optical_flow": ("pixel_flow_x_integral", "pixel_flow_y_integral", "integration_timespan", "quality"),
    "distance_sensor": ("current_distance",),
}

#: PX4's nav_state (as the log records it) -> the number this app plots: main mode, with Auto's sub-mode
#: behind the decimal point, exactly as `Simulator._link_status` unpacks it from a heartbeat. The two have
#: to agree, or an imported log's mode staircase would read differently from a recorded one's.
NAV_STATE_MODE = {
    0: 1.0,      # MANUAL
    1: 2.0,      # ALTCTL
    2: 3.0,      # POSCTL
    3: 4.04,     # AUTO_MISSION
    4: 4.03,     # AUTO_LOITER  (Hold)
    5: 4.05,     # AUTO_RTL
    6: 3.0,      # POSITION_SLOW
    10: 5.0,     # ACRO
    12: 4.06,    # DESCEND      (reported as Auto/Land)
    13: 10.0,    # TERMINATION
    14: 6.0,     # OFFBOARD
    15: 7.0,     # STABILIZED
    17: 4.02,    # AUTO_TAKEOFF
    18: 4.06,    # AUTO_LAND
    19: 4.08,    # AUTO_FOLLOW_TARGET
    20: 4.09,    # AUTO_PRECLAND
    21: 4.03,    # ORBIT
    22: 4.02,    # AUTO_VTOL_TAKEOFF
}


def _series(data, name: str, field: str):
    """(times in seconds, values) for one field, or None when the log has not got it."""
    for d in data:
        if d.name != name:
            continue
        if field not in d.data:
            return None
        t = np.asarray(d.data["timestamp"], dtype=float) / 1e6
        return t, np.asarray(d.data[field], dtype=float)
    return None


def _at(series, grid: np.ndarray) -> np.ndarray:
    """A field sampled onto the grid, holding the last value - the same thing the live recorder does."""
    if series is None:
        return np.zeros_like(grid)
    t, v = series
    if not len(t):
        return np.zeros_like(grid)
    idx = np.searchsorted(t, grid, side="right") - 1
    return np.where(idx < 0, v[0], v[np.clip(idx, 0, len(v) - 1)])


def _euler_deg(q: np.ndarray) -> np.ndarray:
    """(roll, pitch, yaw) degrees from rows of (w, x, y, z)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(np.column_stack([roll, pitch, yaw]))


def ulog_to_run(path: str | Path, rate_hz: float = 50.0, max_rows: int = 200_000) -> dict:
    """Read a .ulg and return a run: {columns, rows, duration, samples, sample_hz, source}.

    ``rate_hz`` is the grid the log is resampled onto. The log itself holds more - 250 Hz and up for the
    IMU - but a graph that can draw ten minutes needs a grid, and 50 Hz is finer than any radio will ever
    manage while still being a few hundred thousand numbers rather than a few million.
    """
    from pyulog import ULog

    path = Path(path)
    ulog = ULog(str(path), message_name_filter_list=list(WANTED), disable_str_exceptions=True)
    data = ulog.data_list
    if not data:
        raise ValueError("this log has no data in it (an empty or truncated .ulg)")

    t_min = min(float(d.data["timestamp"][0]) for d in data if len(d.data["timestamp"])) / 1e6
    t_max = max(float(d.data["timestamp"][-1]) for d in data if len(d.data["timestamp"])) / 1e6
    span = max(0.0, t_max - t_min)
    n = int(min(max_rows, max(1, round(span * rate_hz)) + 1))
    grid = np.linspace(t_min, t_min + span, n)
    g = grid - t_min

    get = lambda topic, field: _series(data, topic, field)                      # noqa: E731
    sample = lambda topic, field: _at(get(topic, field), grid)                  # noqa: E731

    pos = np.column_stack([sample("vehicle_local_position", k) for k in ("x", "y", "z")])
    vel = np.column_stack([sample("vehicle_local_position", k) for k in ("vx", "vy", "vz")])
    q = np.column_stack([sample("vehicle_attitude", f"q[{i}]") for i in range(4)])
    norm = np.linalg.norm(q, axis=1, keepdims=True)
    q = np.where(norm > 1e-6, q / np.where(norm > 1e-6, norm, 1.0), np.array([1.0, 0, 0, 0]))
    rpy = _euler_deg(q)
    rates = np.degrees(np.column_stack([sample("vehicle_angular_velocity", f"xyz[{i}]") for i in range(3)]))
    accel = np.column_stack([sample("sensor_combined", f"accelerometer_m_s2[{i}]") for i in range(3)])
    gyro = np.degrees(np.column_stack([sample("sensor_combined", f"gyro_rad[{i}]") for i in range(3)]))
    baro = sample("vehicle_air_data", "baro_alt_meter")
    baro = baro - (baro[0] if len(baro) else 0.0)
    battery = sample("battery_status", "remaining") * 100.0
    if not battery.any():
        battery = np.full(n, 100.0)
    armed = (sample("vehicle_status", "arming_state") >= 2).astype(float)       # 2 = ARMED in PX4
    nav = sample("vehicle_status", "nav_state")
    mode = np.array([NAV_STATE_MODE.get(int(v), 0.0) for v in nav]) if len(nav) else np.zeros(n)
    airspeed = sample("airspeed_validated", "true_airspeed_m_s")
    rng = get("distance_sensor", "current_distance")
    flow_range = _at(rng, grid) if rng else np.full(n, -1.0)
    flow_q = sample("optical_flow", "quality")

    # motors: actuator_motors is 0..1 per motor and is what the allocator asked for; actuator_outputs is
    # what the driver sent, in PWM microseconds. Prefer the first and fall back to the second.
    motors: list[np.ndarray] = []
    for i in range(12):
        s = get("actuator_motors", f"control[{i}]")
        if s is None:
            break
        v = _at(s, grid)
        if not np.isfinite(v).any():
            break
        motors.append(np.nan_to_num(np.clip(v, 0.0, 1.0)))
    if not motors:
        for i in range(16):
            s = get("actuator_outputs", f"output[{i}]")
            if s is None:
                break
            v = _at(s, grid)
            if not v.any():
                continue
            motors.append(np.clip((v - 1000.0) / 1000.0, 0.0, 1.0))

    height = -pos[:, 2]
    speed = np.linalg.norm(vel, axis=1)
    on_ground = (height < 0.4).astype(float)
    zeros = np.zeros(n)
    cols = columns_for(len(motors))
    by_name = {
        "t": g, "n": pos[:, 0], "e": pos[:, 1], "d": pos[:, 2],
        "vn": vel[:, 0], "ve": vel[:, 1], "vd": vel[:, 2],
        "qw": q[:, 0], "qx": q[:, 1], "qy": q[:, 2], "qz": q[:, 3],
        "roll": rpy[:, 0], "pitch": rpy[:, 1], "yaw": rpy[:, 2],
        "p": rates[:, 0], "q": rates[:, 1], "r": rates[:, 2],
        "height": height, "speed": speed, "airspeed": airspeed,
        "thrust": zeros, "power": zeros, "lift": zeros,
        "battery": battery, "wind": zeros, "armed": armed, "on_ground": on_ground,
        "ax": accel[:, 0], "ay": accel[:, 1], "az": accel[:, 2],
        "gx": gyro[:, 0], "gy": gyro[:, 1], "gz": gyro[:, 2],
        "alt_baro": baro, "flow_x": zeros, "flow_y": zeros,
        "flow_range": flow_range, "flow_q": flow_q, "mode": mode,
    }
    for i, m in enumerate(motors):
        by_name[f"m{i + 1}_cmd"] = m
        by_name[f"m{i + 1}_spool"] = m
        by_name[f"m{i + 1}_thrust"] = zeros

    table = np.column_stack([by_name.get(c, zeros) for c in cols])
    table = np.nan_to_num(table, nan=0.0, posinf=0.0, neginf=0.0)
    rows = [[round(float(v), 5) for v in row] for row in table]
    # how fast the board itself was logging the fastest thing in the file: the honest comparison against
    # whatever a radio managed for the same flight
    logged_hz = max((len(d.data["timestamp"]) / span for d in data if span > 0 and len(d.data["timestamp"])), default=0.0)
    return {"columns": cols, "rows": rows, "duration": round(float(g[-1]) if n else 0.0, 3),
            "samples": n, "sample_hz": rate_hz, "source": path.name,
            "motors": len(motors), "topics": len({d.name for d in data}), "logged_hz": round(logged_hz, 1)}
