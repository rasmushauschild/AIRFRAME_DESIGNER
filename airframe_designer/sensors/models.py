"""Sensor models that turn true vehicle state into what PX4 expects in HIL_SENSOR / HIL_GPS.

Units follow the MAVLink HIL messages:
  accel  m/s^2 (body FRD, specific force)
  gyro   rad/s (body FRD)
  mag    gauss (body FRD)
  baro   hPa, pressure altitude m, temperature degC
  gps    lat/lon 1e7 deg, alt mm, velocities cm/s
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

R_EARTH = 6371000.0
FIELDS_ALL = 0x1FFF          # accel | gyro | mag | baro | diff-press (see SimulatorMavlink SensorSource)
FIELDS_NO_DIFF = 0x1FFF & ~0b10000000000  # PX4 expects diff-press bit only if you supply airspeed


@dataclass
class Home:
    lat: float = 55.6761      # deg   (Copenhagen by default; change to taste)
    lon: float = 12.5683
    alt: float = 10.0         # m AMSL


@dataclass
class SensorNoise:
    accel: float = 0.02       # m/s^2 std
    gyro: float = 0.002       # rad/s std
    mag: float = 0.002        # gauss std
    baro: float = 0.05        # hPa std
    gps_pos: float = 0.08     # m std (horizontal); keep small, PX4's preflight drift check dislikes white noise
    gps_alt: float = 0.15     # m std
    gps_vel: float = 0.03     # m/s std
    accel_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro_bias: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    enabled: bool = True


def magnetic_field_ned(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Cheap dipole approximation of the Earth field, gauss, NED.

    Inclination = atan(2 tan(lat)), intensity ~0.5 G, declination assumed small.
    PX4's EKF fetches its own declination from its WMM table using GPS, and the
    heading it derives from this field is self-consistent with the simulation.
    """
    lat = math.radians(lat_deg)
    incl = math.atan(2.0 * math.tan(lat))
    decl = math.radians(0.0)
    intensity = 0.5
    h = intensity * math.cos(incl)
    return np.array([h * math.cos(decl), h * math.sin(decl), intensity * math.sin(incl)])


class SensorSuite:
    def __init__(self, home: Home | None = None, noise: SensorNoise | None = None, seed: int = 1):
        self.home = home or Home()
        self.noise = noise or SensorNoise()
        self.rng = np.random.default_rng(seed)
        self.mag_ned = magnetic_field_ned(self.home.lat, self.home.lon)

    def set_home(self, lat: float, lon: float, alt: float) -> None:
        self.home = Home(lat, lon, alt)
        self.mag_ned = magnetic_field_ned(lat, lon)

    def _n(self, std: float, n: int = 3) -> np.ndarray:
        if not self.noise.enabled or std <= 0:
            return np.zeros(n)
        return self.rng.normal(0.0, std, n)

    def imu_lever_acceleration(self, sim, time_usec):
        """Rigid-body acceleration at the IMU, relative to the CG, in structural axes."""
        rates = np.asarray(sim.rates, float)
        previous = getattr(self, "_imu_previous", None)
        self._imu_previous = (time_usec, rates.copy(), id(sim))
        alpha = np.zeros(3)
        if previous and previous[2] == id(sim) and 0 < time_usec - previous[0] < 100000:
            alpha = (rates - previous[1]) / ((time_usec - previous[0]) * 1e-6)
        af = getattr(sim, "af", None)
        if af is None:
            return np.zeros(3)
        offset = np.asarray(af.design.get("pixhawk_position", af.cg), float) - af.cg
        return np.cross(alpha, offset) + np.cross(rates, np.cross(rates, offset))

    # ---------------------------------------------------------- HIL_SENSOR
    def hil_sensor(self, sim, time_usec: int) -> dict:
        R = sim.rotmat
        accel = sim.accel_body + self.imu_lever_acceleration(sim, time_usec) + np.asarray(self.noise.accel_bias) + self._n(self.noise.accel)
        gyro = sim.rates + np.asarray(self.noise.gyro_bias) + self._n(self.noise.gyro)
        mag = R.T @ self.mag_ned + self._n(self.noise.mag)

        alt = self.home.alt - sim.pos[2]
        # ISA troposphere
        temp_c = 15.0 - 0.0065 * alt
        pressure = 1013.25 * (1.0 - 2.25577e-5 * alt) ** 5.25588 + float(self._n(self.noise.baro, 1)[0])
        # pressure altitude back from the noisy pressure
        pressure_alt = (1.0 - (pressure / 1013.25) ** (1.0 / 5.25588)) / 2.25577e-5

        return dict(
            time_usec=int(time_usec),
            xacc=float(accel[0]), yacc=float(accel[1]), zacc=float(accel[2]),
            xgyro=float(gyro[0]), ygyro=float(gyro[1]), zgyro=float(gyro[2]),
            xmag=float(mag[0]), ymag=float(mag[1]), zmag=float(mag[2]),
            abs_pressure=float(pressure), diff_pressure=0.0,
            pressure_alt=float(pressure_alt), temperature=float(temp_c),
            fields_updated=FIELDS_NO_DIFF, id=0,
        )

    # ------------------------------------------------------------- HIL_GPS
    def hil_gps(self, sim, time_usec: int) -> dict:
        n, e, d = sim.pos
        pn = self._n(self.noise.gps_pos, 2)
        n += pn[0]
        e += pn[1]
        d += float(self._n(self.noise.gps_alt, 1)[0])
        lat = self.home.lat + math.degrees(n / R_EARTH)
        lon = self.home.lon + math.degrees(e / (R_EARTH * math.cos(math.radians(self.home.lat))))
        alt = self.home.alt - d
        v = sim.vel + self._n(self.noise.gps_vel)
        vh = math.hypot(v[0], v[1])
        cog = math.degrees(math.atan2(v[1], v[0])) % 360.0
        _, _, yaw = sim.euler
        yaw_cdeg = int(round(math.degrees(yaw) % 360.0 * 100))
        return dict(
            time_usec=int(time_usec), fix_type=3,
            lat=int(round(lat * 1e7)), lon=int(round(lon * 1e7)), alt=int(round(alt * 1000)),
            # report the accuracy the noise model actually delivers (a modern receiver, not a 1.5 m one): PX4's
            # EKF weights GPS by eph/epv, and an under-reported accuracy leaves touchdown transients in the
            # vertical estimate for tens of seconds, which the land detector then reads as vertical movement
            eph=int(round(max(0.3, 3 * self.noise.gps_pos) * 100)), epv=int(round(max(0.5, 3 * self.noise.gps_alt) * 100)),
            vel=int(round(vh * 100)),
            vn=int(round(v[0] * 100)), ve=int(round(v[1] * 100)), vd=int(round(v[2] * 100)),
            cog=int(round(cog * 100)), satellites_visible=12, id=0,
            yaw=yaw_cdeg if yaw_cdeg != 0 else 36000,
        )

    # ------------------------------------------------ HIL_STATE_QUATERNION
    def hil_state_quaternion(self, sim, time_usec: int) -> dict:
        n, e, d = sim.pos
        lat = self.home.lat + math.degrees(n / R_EARTH)
        lon = self.home.lon + math.degrees(e / (R_EARTH * math.cos(math.radians(self.home.lat))))
        alt = self.home.alt - d
        v = sim.vel
        return dict(
            time_usec=int(time_usec),
            attitude_quaternion=[float(x) for x in sim.q],
            rollspeed=float(sim.rates[0]), pitchspeed=float(sim.rates[1]), yawspeed=float(sim.rates[2]),
            lat=int(round(lat * 1e7)), lon=int(round(lon * 1e7)), alt=int(round(alt * 1000)),
            vx=int(round(v[0] * 100)), vy=int(round(v[1] * 100)), vz=int(round(v[2] * 100)),
            ind_airspeed=0, true_airspeed=int(round(float(np.linalg.norm(v)) * 100)),
            xacc=int(round(sim.accel_body[0] / 9.80665 * 1000)),
            yacc=int(round(sim.accel_body[1] / 9.80665 * 1000)),
            zacc=int(round(sim.accel_body[2] / 9.80665 * 1000)),
        )
