"""Sensor models that turn true vehicle state into what PX4 expects in HIL_SENSOR / HIL_GPS.

Units follow the MAVLink HIL messages:
  accel  m/s^2 (body FRD, specific force)
  gyro   rad/s (body FRD)
  mag    gauss (body FRD)
  baro   hPa, pressure altitude m, temperature degC
  gps    lat/lon 1e7 deg, alt mm, velocities cm/s
  flow   rad integrated over the frame, plus the slant range to the ground in m
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..geometry.airframe import mount_rotation

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
    flow: float = 0.05        # rad/s std on the measured line-of-sight rate
    range: float = 0.01       # m std on the rangefinder
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

    @staticmethod
    def board_rotation(af) -> np.ndarray:
        """Sensor axes -> structural axes for the flight controller, as it is bolted into this airframe."""
        return np.eye(3) if af is None else mount_rotation(af.design.get("pixhawk_rotation"))

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
        accel = sim.accel_body + self.imu_lever_acceleration(sim, time_usec)
        gyro = np.asarray(sim.rates, float)
        mag = R.T @ self.mag_ned
        # A board bolted in crooked reads in its own axes; PX4 turns it back with SENS_BOARD_*_OFF, which the
        # export writes from the same angles. Declare it wrong and the attitude comes out wrong, as it would.
        board = self.board_rotation(getattr(sim, "af", None))
        if not np.array_equal(board, np.eye(3)):
            accel, gyro, mag = board.T @ accel, board.T @ gyro, board.T @ mag
        accel = accel + np.asarray(self.noise.accel_bias) + self._n(self.noise.accel)
        gyro = gyro + np.asarray(self.noise.gyro_bias) + self._n(self.noise.gyro)
        mag = mag + self._n(self.noise.mag)

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

    # ----------------------------------------------------------- H-flow
    FLOW_MIN_RANGE = 0.08      # the HereFlow's VL53L1X sees nothing closer than this
    FLOW_MIN_COS = 0.17        # beam more than ~80 deg off vertical: no ground in the picture

    def _flow_state(self, sim):
        """Geometry of the H-flow sensor right now, or None if this design has not got one.

        Returns the slant range along the beam (NaN when it misses the ground), the velocity and angular rate
        in the sensor's own axes, the angular rate in PX4's body axes, and how square the beam is to the ground.
        """
        af = getattr(sim, "af", None)
        f = af.flow_sensor() if af is not None else None
        if f is None:
            return None
        R_ns = np.asarray(sim.rotmat, float)                      # structural -> NED
        R_sm = mount_rotation(f["rotation"])                      # sensor -> structural
        R_nsens = R_ns @ R_sm                                     # sensor -> NED
        arm = np.asarray(f["position"], float) - af.cg            # structural, from the CG
        rates = np.asarray(sim.rates, float)
        height = -float(sim.pos[2] + (R_ns @ arm)[2])             # the lens above flat ground
        cos_tilt = float(R_nsens[2, 2])                           # the beam's share of straight down
        rng = height / cos_tilt if cos_tilt > self.FLOW_MIN_COS and height > 0.0 else float("nan")
        return dict(range=rng, cos_tilt=cos_tilt, max_range=f["range_m"],
                    v_sensor=R_nsens.T @ (np.asarray(sim.vel, float) + R_ns @ np.cross(rates, arm)),
                    w_sensor=R_sm.T @ rates, w_body=af.hover_rotation() @ rates)

    def hil_optical_flow(self, sim, time_usec: int, dt_s: float) -> dict | None:
        """One frame of a downward optical flow camera with its rangefinder, as HIL_OPTICAL_FLOW.

        The signs are PX4's, read back from v1.18: EKF2 takes ``flow_rate = -pixel_flow / dt`` and
        ``gyro_rate = -delta_angle / dt``, compensates with ``flow_rate - gyro_rate.xy()``, and predicts
        ``(v_y, -v_x) / range`` for the sensor's own velocity. So the camera reports the line-of-sight rate
        it sees, negated and integrated over the frame.

        The integrated gyro goes out in PX4's body axes, not the sensor's: PX4 turns the pixel flow by
        SENS_FLOW_ROT but leaves the delta angle alone (it is the vehicle's own gyro when a sensor reports
        none), so the body frame is what makes the two agree after the rotation.
        """
        st = self._flow_state(sim)
        if st is None or dt_s <= 0:
            return None
        rng, quality = st["range"], 0
        if np.isfinite(rng) and self.FLOW_MIN_RANGE <= rng <= st["max_range"]:
            # a slanted view of the ground is a poorer picture, and the far end of the range is grainier
            quality = int(round(255 * min(1.0, st["cos_tilt"]) * (1.0 - 0.4 * rng / st["max_range"])))
        los = np.zeros(2)
        if quality:
            v, w = st["v_sensor"], st["w_sensor"]
            los = np.array([v[1] / rng + w[0], -v[0] / rng + w[1]]) + self._n(self.noise.flow, 2)
        w_body = st["w_body"]
        return dict(
            time_usec=int(time_usec), sensor_id=0, integration_time_us=int(round(dt_s * 1e6)),
            integrated_x=float(-los[0] * dt_s), integrated_y=float(-los[1] * dt_s),
            integrated_xgyro=float(-w_body[0] * dt_s), integrated_ygyro=float(-w_body[1] * dt_s),
            integrated_zgyro=float(-w_body[2] * dt_s),
            temperature=2000, quality=quality, time_delta_distance_us=int(round(dt_s * 1e6)),
            distance=float(rng + self._n(self.noise.range, 1)[0]) if quality else -1.0,
        )

    def distance_sensor(self, sim, time_usec: int) -> dict | None:
        """The H-flow's rangefinder on its own, as DISTANCE_SENSOR: the slant range PX4 then de-tilts."""
        st = self._flow_state(sim)
        if st is None:
            return None
        rng, max_range = st["range"], st["max_range"]
        reading = rng + float(self._n(self.noise.range, 1)[0]) if np.isfinite(rng) else max_range + 1.0
        return dict(
            time_boot_ms=int(time_usec // 1000), min_distance=int(round(self.FLOW_MIN_RANGE * 100)),
            max_distance=int(round(max_range * 100)),
            current_distance=int(round(max(0.0, min(reading, max_range + 1.0)) * 100)),
            type=0, id=0, orientation=25,            # laser, facing down (MAV_SENSOR_ROTATION_PITCH_270)
            covariance=int(round((self.noise.range * 100) ** 2)) or 1,
            horizontal_fov=0.0, vertical_fov=0.0, quaternion=[0.0, 0.0, 0.0, 0.0], signal_quality=0,
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
