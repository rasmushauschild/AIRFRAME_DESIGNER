"""MAVLink link to PX4: the Simulator MAVLink API plus the parameter protocol.

One class serves both modes, with two logical channels:
  hil  - HIL_SENSOR / HIL_GPS out, HIL_ACTUATOR_CONTROLS in
  ctl  - parameters, commands, vehicle status (HEARTBEAT, STATUSTEXT)

  SITL  - hil: we listen on TCP 4560(+instance), PX4's simulator_mavlink connects to us and lockstep is driven
               by our HIL_SENSOR timestamps. That module speaks *only* the HIL messages, so
          ctl: a UDP client on PX4's onboard MAVLink instance (PX4 sends to 14540+instance, listens on 14580+instance).
  HITL  - hil and ctl are the same connection: the Pixhawk's USB serial port. Everything the vehicle says is
          optionally proxied to QGroundControl over UDP so QGC can be used at the same time.

One reader thread per connection drains it; the simulator loop and the parameter client only write.
"""
from __future__ import annotations

import os
import socket
import struct
import queue
import threading
import time
from collections import deque
from typing import Callable

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")
from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import common as mavlink  # noqa: E402

PARAM_TYPE_INT32 = mavlink.MAV_PARAM_TYPE_INT32
PARAM_TYPE_REAL32 = mavlink.MAV_PARAM_TYPE_REAL32


def _float_to_int_bits(f: float) -> int:
    return struct.unpack("<i", struct.pack("<f", f))[0]


def _int_bits_to_float(i: int) -> float:
    return struct.unpack("<f", struct.pack("<i", int(i)))[0]


class PX4Link:
    def __init__(self, mode: str, address: str = "0.0.0.0:4560", baud: int = 921600,
                 qgc_proxy: str | None = None, ctl_address: str = "udpin:127.0.0.1:14540",
                 log: Callable[[str], None] | None = None):
        assert mode in ("sitl", "hitl")
        self.mode = mode
        self.address = address
        self.ctl_address = ctl_address if mode == "sitl" else address
        self.baud = baud
        self.qgc_proxy = qgc_proxy
        self.log = log or (lambda s: print(s, flush=True))
        self.target_system = 1
        self.target_component = 1

        self.conn = None          # hil channel
        self.ctl = None           # control channel (== conn in HITL)
        self.connected = False    # hil channel alive
        self.ctl_connected = False
        self._write_lock = threading.Lock()      # hil channel
        self._ctl_lock = threading.Lock()        # ctl channel
        self._stop = threading.Event()

        # actuators
        self.actuators = [0.0] * 16
        self.actuator_seq = 0
        self.actuator_armed = False
        self.actuator_time_usec = 0
        self._act_cond = threading.Condition()

        # vehicle status from HEARTBEAT
        self.heartbeat_time = 0.0
        self.armed = False
        self.custom_mode = 0
        self.mav_type = 0
        self.hil_enabled = False

        # parameters
        self.params: dict[str, dict] = {}     # name -> {"value":, "type":, "index":}
        self.param_count = 0
        self._param_event = threading.Event()
        self._param_set_result: dict[str, dict] = {}

        self.statustext = deque(maxlen=200)
        self.firmware: dict = {}      # from AUTOPILOT_VERSION
        self.rx_types: dict[str, int] = {}
        self.last_ack: dict = {}
        self.event_decoder = None            # events.EventDecoder, set by the app
        self.board_imu: dict = {}            # what the autopilot says its IMU sees (HIGHRES_IMU)
        self.board_sys: dict = {}            # SYS_STATUS: comm drop rate etc.
        self.rc: dict = {}                   # RC_CHANNELS from the autopilot
        self.board_att: dict = {}            # ATTITUDE as PX4 estimates it (in its own, possibly rotated, frame)
        self.board_state: dict = {}          # where the aircraft is, as it reports it: LOCAL_POSITION_NED + the quaternion
        self.board_motors: list[float] = []  # what the outputs are doing (ESC_STATUS / ACTUATOR_OUTPUT_STATUS)
        self.block_hil = False               # listening to a real flight: never inject sensors
        self.manual_last: dict = {}          # last MANUAL_CONTROL we sent (USB joystick)
        self._shell_buf = bytearray()
        self._shell_lock = threading.Lock()
        self.recent_events = deque(maxlen=60)
        self._last_event_text: dict[str, float] = {}
        self.rx_count = 0
        self.rx_bytes = 0
        self.asked_rates: dict = {}          # what this link last asked the aircraft to send
        self._rate: dict = {"msg_s": 0.0, "bytes_s": 0.0}     # what is actually arriving
        self._rate_mark = (time.time(), 0, 0)
        self.radio: dict = {}                # RADIO_STATUS from the telemetry radio on this computer
        self._last_interval_id = 0           # which stream the last SET_MESSAGE_INTERVAL was about
        self._mav_rate: float | None = None  # MAV_<n>_RATE: the aircraft's own cap on this port
        self.board_flow: dict = {}           # the aircraft's own H-flow, as it reports it
        self.board_range: dict = {}          # and its rangefinder
        self.board_alt: dict = {}            # ALTITUDE: height above where it started, from the board
        self.log_entries: "queue.Queue" = queue.Queue()   # LOG_ENTRY / LOG_DATA, drained by px4/logs.py
        self.log_chunks: "queue.Queue" = queue.Queue()
        self.last_rx_time = 0.0
        self._qgc_sock = None
        self._qgc_addr = None
        self._readers: list[threading.Thread] = []
        self.ctl_rx_time = 0.0

    # ----------------------------------------------------------------- lifecycle
    def open(self) -> None:
        if self.mode == "sitl":
            host, port = self.address.rsplit(":", 1)
            self.log(f"[link] SITL: listening for PX4 simulator link on tcp://{host}:{port}")
            self.conn = mavutil.mavlink_connection(f"tcpin:{host}:{port}", source_system=1, source_component=51,
                                                   dialect="common")
            self.log(f"[link] SITL: parameters/commands via PX4 onboard link {self.ctl_address}")
            self.ctl = mavutil.mavlink_connection(self.ctl_address, source_system=255, source_component=190,
                                                  dialect="common")
        else:
            self.log(f"[link] HITL: opening serial {self.address} @ {self.baud}")
            # Speak as a GCS (sysid 255): PX4 then counts us as the ground station (no "GCS not connected"
            # arming complaint) and accepts commands the same way it does from QGroundControl.
            self.conn = mavutil.mavlink_connection(self.address, baud=self.baud, source_system=255, source_component=190,
                                                   dialect="common", autoreconnect=True)
            if self.qgc_proxy:
                host, port = self.qgc_proxy.rsplit(":", 1)
                self._qgc_target = (host, int(port))
                self._qgc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._qgc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._qgc_sock.bind(("0.0.0.0", 0))
                self._qgc_sock.setblocking(False)
                self.log(f"[link] proxying vehicle MAVLink to QGroundControl at udp://{host}:{port}")
            self.ctl = self.conn
            self._ctl_lock = self._write_lock
        self._readers = [threading.Thread(target=self._read_loop, args=(self.conn, True, self.ctl is self.conn),
                                          name="mavlink-hil-reader", daemon=True)]
        if self.ctl is not self.conn:
            self._readers.append(threading.Thread(target=self._read_loop, args=(self.ctl, False, True),
                                                  name="mavlink-ctl-reader", daemon=True))
        for t in self._readers:
            t.start()

    def close(self) -> None:
        """Stop the readers and release the port. pymavlink's serial autoreconnect would otherwise reopen the
        device from the reader thread right after close(), which keeps it busy for e.g. a firmware upload."""
        self._stop.set()
        conns = list({id(self.conn): self.conn, id(self.ctl): self.ctl}.values())
        for c in conns:
            if c is not None and hasattr(c, "autoreconnect"):
                c.autoreconnect = False
        for t in self._readers:
            if t.is_alive() and t is not threading.current_thread():
                t.join(1.5)
        for c in conns:
            try:
                if c:
                    c.close()
            except Exception:
                pass
        self.connected = False
        self.ctl_connected = False

    # ------------------------------------------------------------------- write
    def send_hil_sensor(self, s: dict) -> None:
        if self.listening:
            return  # Never inject simulator sensors into a real-flight board.
        with self._write_lock:
            self.conn.mav.hil_sensor_send(**s)

    def send_hil_gps(self, g: dict) -> None:
        if self.listening:
            return  # Never inject simulator sensors into a real-flight board.
        with self._write_lock:
            self.conn.mav.hil_gps_send(**g)

    def send_hil_optical_flow(self, f: dict) -> None:
        if self.listening:
            return  # Never inject simulator sensors into a real-flight board.
        with self._write_lock:
            self.conn.mav.hil_optical_flow_send(**f)

    def send_distance_sensor(self, d: dict) -> None:
        if self.listening:
            return  # Never inject simulator sensors into a real-flight board.
        with self._write_lock:
            self.conn.mav.distance_sensor_send(**d)

    def send_hil_state_quaternion(self, st: dict) -> None:
        if self.listening:
            return  # Never inject simulator sensors into a real-flight board.
        with self._write_lock:
            self.conn.mav.hil_state_quaternion_send(**st)

    def send_heartbeat(self) -> None:
        with self._write_lock:
            self.conn.mav.heartbeat_send(mavlink.MAV_TYPE_GCS if self.mode == "hitl" else mavlink.MAV_TYPE_GENERIC,
                                         mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.ctl is not self.conn:
            with self._ctl_lock:
                self.ctl.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def send_command_long(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        with self._ctl_lock:
            self.ctl.mav.command_long_send(self.target_system, self.target_component, command, 0, *p[:7])

    # Board -> host telemetry the USB instance streams by default (about 400 messages/s on an FMU v6X). In HITL that
    # traffic shares the link with HIL_SENSOR/HIL_ACTUATOR_CONTROLS and adds latency to the control loop, so the
    # heavy streams are throttled or switched off; the UI reads ATTITUDE, HIGHRES_IMU and the status messages.
    TELEMETRY_RATES_HZ = {30: 20, 105: 10, 31: 0, 33: 5, 32: 0, 331: 0, 65: 10, 83: 0, 36: 0, 141: 5, 24: 5, 74: 5, 85: 0,
                          12901: 0}   # ATTITUDE, HIGHRES_IMU, ATT_QUAT, GLOBAL_POS, LOCAL_POS, ODOMETRY, RC_CH, ATT_TGT, SERVO_RAW, ALT, GPS_RAW, VFR_HUD, POS_TGT, ODID

    # Listening to a real flight: the streams that say where the aircraft is and what its motors are doing,
    # at a rate the graph can use. Nothing is written to the vehicle beyond asking for these.
    # A SiK radio reports its own signal strength over the same link. It is not the aircraft.
    RADIO_OWN = ("RADIO_STATUS", "RADIO")

    # On a cable there is no reason to be shy: USB carries ~92 kB/s and this is ~12. Pose at 50 Hz is as
    # fast as the recorder keeps it, so nothing below is thrown away.
    FLIGHT_RATES_HZ = {30: 50, 31: 50, 32: 50, 105: 50, 375: 20, 106: 20, 132: 10, 141: 5,
                       291: 5, 147: 2, 74: 5, 24: 2, 33: 5}
    #                  ..., OPTICAL_FLOW_RAD, DISTANCE_SENSOR, ALTITUDE
    #                  ATTITUDE, ATT_QUAT, LOCAL_POS, HIGHRES_IMU, ACTUATOR_OUT, ESC_STATUS, BATTERY, VFR_HUD, GPS, GLOBAL_POS
    # 375 is ACTUATOR_OUTPUT_STATUS, which is what `board_motors` reads and therefore what fills the motor
    # columns of a real flight. SERVO_OUTPUT_RAW (36) was asked for instead and is not read by anything.

    # What each of those costs on the wire, bytes per message including MAVLink v2 framing. Close enough to
    # divide a link up with: the point is the ratio between a 148-byte actuator report and a 32-byte VFR_HUD.
    MSG_BYTES = {30: 40, 31: 46, 32: 40, 105: 74, 375: 148, 106: 56, 132: 50, 141: 44,
                 291: 60, 147: 66, 74: 32, 24: 64, 33: 40}

    # When there is not enough link for all of it, this is the order things are given up in: the last one
    # standing is where the aircraft is and which way it is pointing, because that is the 3D view and the
    # graph. ATTITUDE goes first and always - the quaternion beside it says the same thing.
    GIVE_UP_ORDER = [30, 24, 291, 147, 105, 141, 74, 33, 132, 106, 375, 31, 32]

    # Enough of each to be worth having at all, and then the order the rest of the link is spent in.
    # Position and attitude rise together, because a pose is both of them at the same instant.
    # Enough of each to be worth having at all. Kept small, because a floor is money taken from the pose
    # before anything is decided, and on a radio there is not much money.
    FLOOR_HZ = {30: 0, 32: 5, 31: 5, 106: 2, 375: 1, 132: 1, 147: 1, 141: 1, 74: 1, 33: 1, 105: 0, 291: 0, 24: 0}

    # Then the rest of the link is spent in this order, each step up to the rate beside it (None = as much
    # as a cable would get). Position and attitude rise together - a pose is both at the same instant -
    # first to something you can fly a view with, then the H-flow to something you can see tracking in,
    # then the motors, and only after all that does the pose climb towards the cable's rate.
    CORE = (32, 31)                 # where it is and which way it is pointing: the pose, and it rises as one
    SPEND_ORDER = [((32, 31), 10), ((106,), 10), ((375,), 2), ((132,), 2), ((32, 31), None), ((105,), None),
                   ((33,), None), ((74,), None), ((141,), None), ((291,), None), ((147,), None), ((24,), None)]

    # What PX4 sends down a telemetry link without being asked, and this app never reads. On a radio it is
    # a third of the traffic - the graph is choppy because the link is busy carrying RC channels and
    # mission status - so when the budget is tight these are switched off by name. Anything the app does
    # use (STATUSTEXT, COMMAND_ACK, PARAM_VALUE, HEARTBEAT, the streams above) is never touched.
    SILENCE_ON_RADIO = {65: "RC_CHANNELS", 36: "SERVO_OUTPUT_RAW", 42: "MISSION_CURRENT",
                        245: "EXTENDED_SYS_STATE", 29: "SCALED_PRESSURE", 230: "ESTIMATOR_STATUS",
                        241: "VIBRATION", 83: "ATTITUDE_TARGET", 85: "POSITION_TARGET_LOCAL_NED",
                        87: "POSITION_TARGET_GLOBAL_INT", 62: "NAV_CONTROLLER_OUTPUT", 331: "ODOMETRY",
                        242: "HOME_POSITION", 26: "SCALED_IMU", 140: "ACTUATOR_CONTROL_TARGET",
                        25: "GPS_STATUS", 250: "DEBUG_VECT", 61: "ATTITUDE_QUATERNION_COV"}

    SLOW_LINK_BAUD = 115200          # at or below this, the far end is a radio, not a board on a cable
    SHARES = {"light": 0.12, "balanced": 0.28, "full": 0.45}
    # As much of a narrow link as the telemetry can have without the link falling over. The measured rate
    # sits next to the dial, so when it starts falling short of what was asked you can see it and step
    # down; starting low only hides resolution that was there for the taking.
    share = "full"

    def board_send_cap(self) -> float:
        """Bytes per second the *aircraft* will put on this port, whatever we ask for.

        PX4 caps each MAVLink instance at `MAV_<n>_RATE` and simply does not send what does not fit; the
        streams that lose are the big ones, so asking a board capped at 1200 B/s for a 2.5 kB/s pose
        leaves you with the cheap messages arriving and the position and attitude never turning up at all.
        Read once per link: it is a parameter, not a measurement.
        """
        if self._mav_rate is None and self.ctl_connected:
            try:
                # Which MAVLink instance is ours is not something the protocol says, and guessing wrong
                # reads another port's cap: the board's own status names the one that can hear us.
                import re
                out = self.shell("mavlink status", timeout=6.0)
                mine = None
                for n, body in re.findall(r"instance #(\d+):(.*?)(?=instance #|\Z)", out, re.S):
                    if "GCS heartbeat valid" in body:
                        mine = int(n)
                        break
                if mine is None:
                    return 0.0                      # no answer: ask again next time rather than assume
                rate = self.get_param(f"MAV_{mine}_RATE", timeout=3.0)
                if rate is None:
                    return 0.0
                # 0 means PX4 picks baud/20 for itself, which is more than a radio carries anyway
                self._mav_rate = float(rate["value"]) if float(rate["value"]) > 0 else 0.0
                if self._mav_rate:
                    self.log(f"[link] the aircraft will send at most {self._mav_rate:.0f} B/s on this port "
                             f"(MAV_{mine}_RATE); asking for no more than that")
                else:
                    self.log(f"[link] the aircraft sets no limit of its own on this port (MAV_{mine}_RATE = 0)")
            except Exception:
                pass
        return self._mav_rate or 0.0

    def link_budget_bps(self) -> float | None:
        """Bytes per second the telemetry may use, or None for a cable (which is not the bottleneck).

        A serial line carries baud/10 bytes a second and a radio carries rather less than its serial side
        suggests, so only a fraction of it is handed out. Fill the line and nothing is lost exactly - it
        queues, and arrives a second late, which is worse: the graph lies about when things happened.
        """
        if not (self.mode == "hitl" and self.baud and int(self.baud) <= self.SLOW_LINK_BAUD):
            return None
        ours = (int(self.baud) / 10.0) * self.SHARES.get(self.share, self.SHARES["balanced"])
        cap = float(self._mav_rate or 0.0)   # what the aircraft is willing to send, when it has told us
        return min(ours, cap) if cap else ours

    def flight_rates(self) -> dict[int, float]:
        """The streams to ask the aircraft for, scaled to what the link can actually carry."""
        budget = self.link_budget_bps()
        if budget is None:
            return dict(self.FLIGHT_RATES_HZ)
        wish = self.FLIGHT_RATES_HZ
        rates = {m: 0.0 for m in wish}
        cost = lambda: sum(self.MSG_BYTES[m] * hz for m, hz in rates.items())      # noqa: E731
        # The pose comes first and comes whole: a position without the attitude of the same instant is not
        # a pose, and the two have to be asked for at the same rate or the 3D view stutters between them.
        for m in self.CORE:
            rates[m] = float(min(self.FLOOR_HZ.get(m, 1), wish[m]))
        if cost() > budget:                            # narrower than even that: shrink the pair together
            k = budget / cost()
            for m in self.CORE:
                rates[m] = max(0.5, int(rates[m] * k * 2) / 2)      # down to the half hertz, never up
        for m in reversed(self.GIVE_UP_ORDER):         # then the extras, inside their share of the link
            hz = float(min(self.FLOOR_HZ.get(m, 1), wish[m]))
            if m in self.CORE or not hz:
                continue
            if cost() + self.MSG_BYTES[m] * hz <= 0.6 * budget:
                rates[m] = hz
        for group, cap in self.SPEND_ORDER:            # then the rest of the link, in that order
            top = {m: (wish[m] if cap is None else min(cap, wish[m])) for m in group}
            step = 0.5 * sum(self.MSG_BYTES[m] for m in group)
            while cost() + step <= budget and all(rates[m] + 0.5 <= top[m] for m in group):
                for m in group:
                    rates[m] += 0.5
        return rates

    def set_message_interval(self, msg_id: int, hz: float, timeout: float = 1.5, retries: int = 3) -> bool:
        """Ask for one stream at one rate, and make sure the aircraft heard.

        Over a radio this matters more than anything else here. The commands used to go out in a burst of
        twenty in half a second; a radio drops most of them, there is no retry in MAV_CMD_SET_MESSAGE_
        INTERVAL, and a stream that never heard keeps whatever rate PX4 booted with - so the link stays
        full of things nobody reads while the pose arrives twice a second. PX4 acknowledges every one of
        these, so each is sent, waited for, and sent again if the answer does not come.
        """
        interval = -1.0 if hz <= 0 else 1e6 / hz
        self._last_interval_id = int(msg_id)
        for _ in range(retries):
            self.last_ack = {}
            self.send_command_long(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, float(msg_id), interval)
            t0 = time.time()
            while time.time() - t0 < timeout:
                ack = self.last_ack
                if ack.get("command") == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    return ack.get("result") == 0
                time.sleep(0.02)
        return False

    def request_flight_telemetry(self) -> None:
        if self.mode == "hitl" and self.baud and int(self.baud) <= self.SLOW_LINK_BAUD:
            self.board_send_cap()          # ask the aircraft its own limit before dividing the link up
        rates = self.flight_rates()
        budget = self.link_budget_bps()
        wanted = dict(rates)
        if budget is not None:
            # Silence only what is actually arriving and nobody reads. A blind list of twenty is twenty
            # round trips down a link that has no room for them, and most of it was never being sent.
            seen = {t for t, n in self.rx_types.items() if n}
            wanted.update({m: 0.0 for m, name in self.SILENCE_ON_RADIO.items()
                           if m not in rates and name in seen})
        missed = [m for m, hz in wanted.items() if not self.set_message_interval(m, hz)]
        self.asked_rates = {m: hz for m, hz in wanted.items() if m in self.MSG_BYTES}
        asked = sum(self.MSG_BYTES[m] * hz for m, hz in self.asked_rates.items())
        self.log("[link] asked the aircraft for position, attitude and motor telemetry"
                 + (f" at {self.share} radio rates: {asked:.0f} B/s of the {budget:.0f} B/s this link is given"
                    if budget else "")
                 + (f"; {len(missed)} of {len(wanted)} requests went unanswered" if missed else ""))

    def measured_rate(self) -> dict:
        """What is actually arriving, averaged over the last second or so. Asked-for and received are two
        different numbers over a radio, and the gap between them is the whole story."""
        t, n, b = self._rate_mark
        dt = time.time() - t
        if dt >= 1.0:
            self._rate = {"msg_s": round((self.rx_count - n) / dt, 1), "bytes_s": round((self.rx_bytes - b) / dt)}
            self._rate_mark = (time.time(), self.rx_count, self.rx_bytes)
        return dict(self._rate)

    def quiet_streams(self) -> None:
        """Stop the telemetry for a moment. A parameter write is a question and an answer, and over a radio
        it waits behind whatever is already queued; nothing is worth more than the write going through."""
        for msg_id, hz in (self.asked_rates or self.FLIGHT_RATES_HZ).items():
            if hz:
                self.set_message_interval(msg_id, 0, timeout=1.0, retries=2)
        self.log("[link] telemetry paused while the parameters go out")

    def _own_radio(self, msg, t: str) -> bool:
        """The telemetry radio on this computer talking about itself, rather than the aircraft.

        A SiK radio sends RADIO_STATUS whether or not anything is on the far end. Taking that for the
        aircraft would light the link green over an empty sky and - worse - address every command to the
        radio's own system id. Its signal numbers are worth keeping, so they are kept here.
        """
        if t not in self.RADIO_OWN:
            return False
        self.radio = {"rssi": getattr(msg, "rssi", None), "remrssi": getattr(msg, "remrssi", None),
                      "noise": getattr(msg, "noise", None), "txbuf": getattr(msg, "txbuf", None),
                      "t": time.time()}
        return True


    @property
    def listening(self) -> bool:
        """True when a board is flying on its own sensors and this app is only watching it.

        The question is whether the board wants our sensors, and the board answers it in every
        heartbeat: MAV_MODE_FLAG_HIL_ENABLED. `SYS_HITL` is the same answer in slower form, and it is
        only *available* once the parameter list has been downloaded - which over a cable takes a few
        seconds and over a radio never happens unless asked. Reading the parameter first meant that a
        board in HITL, freshly connected, was treated as a real flight: every HIL send was dropped, and
        the board sat there reporting no accelerometer, no gyro and no attitude, exactly as if its
        sensors had failed. So the heartbeat decides whenever it has spoken, and the parameter is the
        fallback for a link that has not heard one yet.
        """
        if self.mode != "hitl":
            return False
        if self.block_hil:                       # a deliberate decision: FLIGHT, or a board about to fly
            return True
        if self.heartbeat_time and time.time() - self.heartbeat_time < 3.0:
            return not self.hil_enabled
        return self.params.get("SYS_HITL", {}).get("value") != 1

    def trim_telemetry(self) -> None:
        """Throttle the board's default telemetry streams (HITL); 0 Hz disables a stream."""
        for msg_id, hz in self.TELEMETRY_RATES_HZ.items():
            interval = -1.0 if hz <= 0 else 1e6 / hz
            self.send_command_long(mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, float(msg_id), interval)
            time.sleep(0.02)
        self.log("[link] board telemetry throttled for HITL")

    def send_manual_control(self, roll: float, pitch: float, throttle: float, yaw: float, buttons: int = 0,
                            aux: list[float] | None = None) -> None:
        """MAVLink MANUAL_CONTROL, the same message QGroundControl sends for a joystick.
        roll/pitch/yaw in [-1, 1] (pitch forward positive), throttle in [0, 1], aux channels in [-1, 1]."""
        c = lambda v: int(max(-1000, min(1000, round(v * 1000))))
        aux = list(aux or [])[:6] + [0.0] * (6 - len(aux or []))
        ext = sum(1 << i for i in range(6) if i < len(aux or [])) if aux else 0
        z = int(max(0, min(1000, round(throttle * 1000))))
        with self._ctl_lock:
            self.ctl.mav.manual_control_send(self.target_system, c(pitch), c(roll), z, c(yaw), int(buttons), 0, ext,
                                             0, 0, *[c(v) for v in aux])
        self.manual_last = {"roll": roll, "pitch": pitch, "throttle": throttle, "yaw": yaw, "buttons": int(buttons),
                            "aux": aux, "t": time.time()}

    # ---------------------------------------------------------- scripted flight helpers (all non-blocking)
    # PX4 custom mode encoding: main_mode << 16 | sub_mode << 24
    MODES = {"manual": (1, 0), "altitude": (2, 0), "position": (3, 0), "acro": (5, 0), "offboard": (6, 0),
             "stabilized": (7, 0), "takeoff": (4, 2), "hold": (4, 3), "mission": (4, 4), "rtl": (4, 5), "land": (4, 6)}

    def set_mode(self, name: str) -> None:
        main, sub = self.MODES[name]
        self.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, float(mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(main), float(sub))

    def mode_is(self, name: str) -> bool:
        main, sub = self.MODES[name]
        return ((self.custom_mode >> 16) & 0xFF) == main and (sub == 0 or ((self.custom_mode >> 24) & 0xFF) == sub)

    @property
    def main_mode(self) -> int:
        return (self.custom_mode >> 16) & 0xFF

    def arm(self, force: bool = False) -> None:
        self.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0 if force else 0.0)

    def disarm(self, force: bool = False) -> None:
        self.send_command_long(mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0 if force else 0.0)

    def send_setpoint_local(self, time_boot_ms: int, pos=None, vel=None, yaw: float | None = 0.0, yaw_rate: float | None = None) -> None:
        """SET_POSITION_TARGET_LOCAL_NED for offboard control. Give ``pos`` (NED, m) or ``vel`` (NED, m/s)."""
        mask = 0
        x = y = z = vx = vy = vz = 0.0
        if pos is None:
            mask |= 1 | 2 | 4
        else:
            x, y, z = (float(v) for v in pos)
        if vel is None:
            mask |= 8 | 16 | 32
        else:
            vx, vy, vz = (float(v) for v in vel)
        mask |= 64 | 128 | 256                      # no acceleration setpoint
        if yaw is None:
            mask |= 1024
        if yaw_rate is None:
            mask |= 2048
        with self._ctl_lock:
            self.ctl.mav.set_position_target_local_ned_send(int(time_boot_ms), self.target_system, self.target_component,
                                                            mavlink.MAV_FRAME_LOCAL_NED, mask, x, y, z, vx, vy, vz,
                                                            0.0, 0.0, 0.0, float(yaw or 0.0), float(yaw_rate or 0.0))

    def param_set_nowait(self, name: str, value: float | int) -> bool:
        """Send PARAM_SET without waiting; the echoed PARAM_VALUE lands in self.params. Needs the type (from a
        previous download or metadata); returns False when the type is unknown."""
        info = self.params.get(name)
        ptype = info["type"] if info is not None else getattr(self, "param_types", {}).get(name)
        if ptype is None:
            return False
        wire = _int_bits_to_float(int(round(value))) if ptype == PARAM_TYPE_INT32 else float(value)
        with self._ctl_lock:
            self.ctl.mav.param_set_send(self.target_system, self.target_component, name.encode()[:16], wire, ptype)
        return True

    def can_arm_modes(self) -> str:
        """PX4's last arming-check summary: the modes it would arm in ('' while unknown)."""
        for x in reversed(list(self.recent_events)):
            if x.get("name") == "commander_arming_check_summary":
                d = dict(zip(x.get("arg_names", []), x.get("args", [])))
                return str(d.get("can_arm", ""))
        return ""

    def clear_actuators(self) -> None:
        """Forget the last motor commands (e.g. on reset or while PX4 reboots) so the physics does not keep
        flying on stale values."""
        with self._act_cond:
            self.actuators = [0.0] * 16
            self.actuator_armed = False

    # --------------------------------------------------------------- lockstep
    def wait_for_actuators(self, seq_before: int, timeout: float) -> bool:
        """Block until a HIL_ACTUATOR_CONTROLS newer than seq_before arrives (lockstep)."""
        with self._act_cond:
            if self.actuator_seq > seq_before:
                return True
            self._act_cond.wait_for(lambda: self.actuator_seq > seq_before, timeout=timeout)
            return self.actuator_seq > seq_before

    # ------------------------------------------------------------------ reader
    def _guard_serial_writes(self, conn) -> None:
        """A USB serial write blocks forever while the board reboots (the CDC buffer never drains), which would
        freeze the sensor loop while it holds the write lock and with it every command on the link. Give the
        port a write timeout; the senders already catch and throttle the resulting exceptions."""
        port = getattr(conn, "port", None)
        if port is not None and hasattr(port, "write_timeout") and getattr(port, "_airframe_guarded", None) is not port:
            try:
                port.write_timeout = 0.3
                port._airframe_guarded = port
            except Exception:
                pass

    def _read_loop(self, conn, is_hil: bool, is_ctl: bool) -> None:
        name = "simulator link" if (is_hil and not is_ctl) else ("vehicle link" if is_ctl and is_hil else "control link")
        while not self._stop.is_set():
            if self.mode == "hitl":
                self._guard_serial_writes(conn)   # re-applied after pymavlink's autoreconnect swaps the port object
            try:
                msg = conn.recv_match(blocking=True, timeout=0.05)
            except Exception as e:  # serial unplugged etc.
                if self._stop.is_set():
                    break
                if time.time() - getattr(self, "_last_read_err", 0.0) > 5.0:
                    self.log(f"[link] {name} unavailable ({str(e)[:60]}), waiting for it to come back")
                    self._last_read_err = time.time()
                if is_hil:
                    self.clear_actuators()
                time.sleep(0.5)
                continue
            if is_hil:
                self._poll_qgc()
            if msg is None:
                if is_hil and self.connected and time.time() - self.last_rx_time > 5.0 and self.mode == "sitl":
                    self.connected = False
                    self.log(f"[link] {name} lost" + (" (the simulation is paused: PX4 receives no sensor data while paused)"
                                                     if getattr(self, "paused_hint", lambda: False)() else ""))
                if is_ctl and self.ctl_connected and time.time() - self.ctl_rx_time > 5.0:
                    self.ctl_connected = False
                    self.log(f"[link] {name} lost")
                continue
            self.rx_count += 1
            try:
                self.rx_bytes += len(msg.get_msgbuf())
            except Exception:
                pass
            t = msg.get_type()
            self.rx_types[t] = self.rx_types.get(t, 0) + 1
            if self._own_radio(msg, t):
                continue
            if is_hil:
                self.last_rx_time = time.time()
                if not self.connected:
                    self.connected = True
                    self.log(f"[link] PX4 {name} up (sysid {msg.get_srcSystem()} compid {msg.get_srcComponent()})")
            if is_ctl:
                self.ctl_rx_time = time.time()
                if not self.ctl_connected:
                    self.ctl_connected = True
                    self.target_system = msg.get_srcSystem()
                    self.log(f"[link] PX4 {name} up (sysid {msg.get_srcSystem()} compid {msg.get_srcComponent()})")
            if t == "HIL_ACTUATOR_CONTROLS":
                with self._act_cond:
                    self.actuators = list(msg.controls)
                    self.actuator_armed = bool(msg.mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.actuator_time_usec = msg.time_usec
                    self.actuator_seq += 1
                    self._act_cond.notify_all()
            elif t == "HEARTBEAT" and is_ctl:
                if msg.get_srcComponent() == 1 and msg.autopilot == mavlink.MAV_AUTOPILOT_PX4:
                    self.target_system = msg.get_srcSystem()
                    self.heartbeat_time = time.time()
                    self.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.hil_enabled = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_HIL_ENABLED)
                    self.custom_mode = msg.custom_mode
                    self.mav_type = msg.type
            elif t == "PARAM_VALUE":
                self._handle_param_value(msg)
            elif t == "UNKNOWN_410":         # EVENT: PX4's arming/health/mode messages (pymavlink lacks it)
                self._handle_event(msg)
            elif t == "SERIAL_CONTROL":
                if msg.device == mavlink.SERIAL_CONTROL_DEV_SHELL:
                    with self._shell_lock:
                        self._shell_buf += bytes(msg.data[:msg.count])
            elif t == "ATTITUDE":
                self.board_att = {"roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw, "t": time.time()}
                self.board_state.update(rates=[msg.rollspeed, msg.pitchspeed, msg.yawspeed],
                                        euler=[msg.roll, msg.pitch, msg.yaw], t_att=time.time())
            elif t == "ATTITUDE_QUATERNION":
                self.board_state.update(q=[msg.q1, msg.q2, msg.q3, msg.q4],
                                        rates=[msg.rollspeed, msg.pitchspeed, msg.yawspeed], t_att=time.time())
            elif t == "LOCAL_POSITION_NED":
                self.board_state.update(pos=[msg.x, msg.y, msg.z], vel=[msg.vx, msg.vy, msg.vz], t_pos=time.time())
            elif t == "OPTICAL_FLOW_RAD":
                # The same quantity the simulator sends as HIL_OPTICAL_FLOW, turned into a rate the same
                # way (sensors/models.py), so the flow columns mean one thing whether a run was flown here
                # or out there: the negated line-of-sight rate, in the sensor's own axes.
                span = max(1e-6, float(msg.integration_time_us) * 1e-6)
                self.board_flow = {"rate": (-float(msg.integrated_x) / span, -float(msg.integrated_y) / span),
                                   "quality": float(msg.quality), "range": float(msg.distance),
                                   "gyro": [float(msg.integrated_xgyro) / span, float(msg.integrated_ygyro) / span,
                                            float(msg.integrated_zgyro) / span], "t": time.time()}
            elif t == "DISTANCE_SENSOR":
                self.board_range = {"m": float(msg.current_distance) / 100.0, "t": time.time(),
                                    "min": float(msg.min_distance) / 100.0, "max": float(msg.max_distance) / 100.0}
            elif t == "ALTITUDE":
                self.board_alt = {"relative": float(msg.altitude_relative), "amsl": float(msg.altitude_amsl),
                                  "terrain": float(msg.altitude_terrain), "t": time.time()}
            elif t == "LOG_ENTRY":
                self.log_entries.put({"id": msg.id, "num_logs": msg.num_logs, "last_log_num": msg.last_log_num,
                                      "time_utc": msg.time_utc, "size": msg.size})
            elif t == "LOG_DATA":
                self.log_chunks.put({"id": msg.id, "ofs": msg.ofs, "data": bytes(bytearray(msg.data)[:msg.count])})
            elif t == "ACTUATOR_OUTPUT_STATUS":
                self.board_motors = [float(v) for v in list(msg.actuator)[:max(0, int(msg.active))]]

            elif t == "RC_CHANNELS":
                n = int(msg.chancount)
                vals = [getattr(msg, f"chan{i}_raw") for i in range(1, 19)]
                self.rc = {"count": n, "channels": vals[:max(n, 0)], "rssi": int(msg.rssi), "t": time.time(),
                           "time_boot_ms": int(msg.time_boot_ms)}
            elif t == "SYS_STATUS":
                self.board_sys = {"drop_rate_comm": msg.drop_rate_comm, "errors_comm": msg.errors_comm, "load": msg.load,
                                  "t": time.time()}
            elif t == "HIGHRES_IMU":
                self.board_imu = {"xacc": msg.xacc, "yacc": msg.yacc, "zacc": msg.zacc, "xgyro": msg.xgyro,
                                  "xmag": msg.xmag, "zmag": msg.zmag, "abs_pressure": msg.abs_pressure,
                                  "fields_updated": msg.fields_updated, "t": time.time()}
            elif t == "COMMAND_ACK":
                names = {0: "accepted", 1: "temporarily rejected", 2: "denied", 3: "unsupported", 4: "failed",
                         5: "in progress", 6: "cancelled"}
                cmds = {400: "arm/disarm", 176: "set mode", 22: "takeoff", 21: "land", 246: "reboot", 245: "save params"}
                self.last_ack = {"command": msg.command, "result": msg.result, "t": time.time()}
                if msg.result != 0 and msg.command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    # A stream this firmware has not got. Worth one quiet line naming it, not an error:
                    # the app asks for a few that only newer PX4 builds publish.
                    self.log(f"[link] this firmware does not stream message {self._last_interval_id} "
                             f"({names.get(msg.result, msg.result)})")
                elif msg.result != 0:
                    self.log(f"[px4] command {cmds.get(msg.command, msg.command)} {names.get(msg.result, msg.result)}")
            elif t == "AUTOPILOT_VERSION":
                v = msg.flight_sw_version
                ver = f"{(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF}"
                typ = {0: "dev", 64: "alpha", 128: "beta", 192: "rc", 255: "release"}.get(v & 0xFF, "")
                gh = bytes(msg.flight_custom_version).hex()[:8]
                self.firmware = {"version": f"{ver} {typ}".strip(), "git": gh, "board": msg.board_version,
                                 "vendor_id": msg.vendor_id, "product_id": msg.product_id}
                self.log(f"[link] firmware PX4 v{ver} {typ} ({gh})")
            elif t == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
                self.statustext.append((time.time(), msg.severity, text))
                self.log(f"[px4] {text}")
            elif t == "BAD_DATA":
                continue
            # HITL: forward everything the vehicle says to QGC
            if self._qgc_sock is not None and t != "BAD_DATA":
                try:
                    self._qgc_sock.sendto(msg.get_msgbuf(), self._qgc_target)
                except OSError:
                    pass

    def _handle_event(self, msg) -> None:
        dec = self.event_decoder
        if dec is None:
            return
        try:
            raw = bytes(msg.get_msgbuf())
            ev = dec.parse_message(raw)
            f = dec.format(ev) if ev else None
        except Exception as e:
            self._event_debug(f"decode error {e}")
            return
        if not f:
            self._event_debug(f"unknown event id {ev['id'] if ev else '?'} seq {ev['seq'] if ev else '?'} raw {raw[:24].hex()}")
            return
        f["t"] = time.time()
        self.recent_events.append(f)
        if f["level"] <= 6 and f["group"] != "protocol":
            key = f["text"]
            if time.time() - self._last_event_text.get(key, 0.0) > 5.0:
                self._last_event_text[key] = time.time()
                tag = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERROR", 4: "WARN", 5: "NOTICE", 6: "INFO"}.get(f["level"], "")
                self.log(f"[px4] {tag} {f['text']}".replace("  ", " "))

    _event_debug_count = 0

    def _event_debug(self, text: str) -> None:
        if self._event_debug_count < 5:
            self._event_debug_count += 1
            self.log(f"[events] {text}")

    def _poll_qgc(self) -> None:
        if self._qgc_sock is None:
            return
        for _ in range(32):
            try:
                data, addr = self._qgc_sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                return
            self._qgc_addr = addr
            with self._write_lock:
                try:
                    self.conn.write(data)
                except Exception:
                    pass

    # -------------------------------------------------------------- parameters
    def _decode_param(self, msg) -> float | int:
        if msg.param_type in (mavlink.MAV_PARAM_TYPE_INT32, mavlink.MAV_PARAM_TYPE_INT16, mavlink.MAV_PARAM_TYPE_INT8,
                              mavlink.MAV_PARAM_TYPE_UINT32, mavlink.MAV_PARAM_TYPE_UINT16, mavlink.MAV_PARAM_TYPE_UINT8):
            return _float_to_int_bits(msg.param_value)
        return float(msg.param_value)

    def _handle_param_value(self, msg) -> None:
        name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
        name = name.rstrip("\x00")
        value = self._decode_param(msg)
        self.params[name] = {"value": value, "type": int(msg.param_type), "index": int(msg.param_index)}
        self.param_count = int(msg.param_count)
        self._param_set_result[name] = self.params[name]
        self._param_event.set()

    def param_request_list(self) -> None:
        with self._ctl_lock:
            self.ctl.mav.param_request_list_send(self.target_system, self.target_component)

    def param_request_read(self, name: str, index: int = -1) -> None:
        with self._ctl_lock:
            self.ctl.mav.param_request_read_send(self.target_system, self.target_component,
                                                 name.encode()[:16], index)

    def fetch_all_params(self, timeout: float = 30.0) -> dict[str, dict]:
        """Blocking full parameter download with retry of missing indices."""
        self.params.clear()
        self.param_count = 0
        self.param_request_list()
        deadline = time.time() + timeout
        last_n = -1
        stall = time.time()
        while time.time() < deadline:
            self._param_event.wait(0.5)
            self._param_event.clear()
            n = len(self.params)
            if self.param_count and n >= self.param_count:
                break
            if n != last_n:
                last_n = n
                stall = time.time()
            elif time.time() - stall > 1.5:
                # retry missing indices
                have = {p["index"] for p in self.params.values()}
                missing = [i for i in range(self.param_count) if i not in have][:50]
                if not missing:
                    break
                for i in missing:
                    self.param_request_read("", i)
                stall = time.time()
        self.log(f"[link] parameters: {len(self.params)}/{self.param_count}")
        return self.params

    def get_param(self, name: str, timeout: float = 2.0) -> dict | None:
        self._param_set_result.pop(name, None)
        self.param_request_read(name)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if name in self._param_set_result:
                return self._param_set_result[name]
            time.sleep(0.01)
        return None

    def set_param(self, name: str, value: float | int, timeout: float = 2.0, retries: int = 3) -> dict:
        """Set a parameter with PX4's byte-wise int encoding; verifies the echoed PARAM_VALUE."""
        info = self.params.get(name)
        if info is None:
            info = self.get_param(name)
        if info is None:
            return {"name": name, "ok": False, "error": "unknown parameter"}
        ptype = info["type"]
        if ptype == PARAM_TYPE_INT32:
            v_int = int(round(value))
            wire = _int_bits_to_float(v_int)
            want = v_int
        else:
            wire = float(value)
            want = float(value)
        for _ in range(retries):
            self._param_set_result.pop(name, None)
            with self._ctl_lock:
                self.ctl.mav.param_set_send(self.target_system, self.target_component, name.encode()[:16], wire, ptype)
            t0 = time.time()
            while time.time() - t0 < timeout:
                res = self._param_set_result.get(name)
                if res is not None:
                    got = res["value"]
                    ok = (got == want) if ptype == PARAM_TYPE_INT32 else abs(float(got) - want) <= 1e-5 * max(1.0, abs(want))
                    if ok:
                        return {"name": name, "ok": True, "value": got}
                    break
                time.sleep(0.005)
        return {"name": name, "ok": False, "error": "no matching PARAM_VALUE ack",
                "value": self._param_set_result.get(name, {}).get("value")}

    def set_params(self, values: dict[str, float | int], progress: Callable[[str, dict], None] | None = None,
                   timeout: float = 2.0, pace: float | None = None) -> list[dict]:
        """Write them all, pipelined.

        One at a time is one round trip each, and over a radio a hundred parameters is a minute of
        waiting with nothing to look at. PX4 echoes every accepted PARAM_SET as a PARAM_VALUE whether or
        not anyone is waiting for it, so the writes go out as a paced stream and the echoes are collected
        as they arrive; whatever has not echoed when the stream ends is sent again, twice. Same result,
        a fraction of the wall clock, and `progress` fires as each one lands so it can be counted.

        A type has to be right or PX4 refuses the write (it answers with an error, not an echo): the one
        this link has already seen is used, and otherwise int for an int and float for a float, which is
        the same rule the .params export has always used.
        """
        want: dict[str, tuple[float, int, float]] = {}
        done: dict[str, dict] = {}
        pace = pace if pace is not None else (0.04 if self.link_budget_bps() else 0.004)

        def plan(name, ptype):
            """How this one goes on the wire: PX4 carries an int32 in the float's bits."""
            v = values[name]
            if ptype == PARAM_TYPE_INT32:
                target = int(round(float(v)))
                want[name] = (_int_bits_to_float(target), ptype, target)
            else:
                want[name] = (float(v), ptype, float(v))

        for name, v in values.items():
            info = self.params.get(name)
            plan(name, info["type"] if info else (PARAM_TYPE_INT32 if isinstance(v, (bool, int)) else PARAM_TYPE_REAL32))

        todo = list(values)
        for attempt in range(3):
            if not todo:
                break
            if attempt:
                # Whatever did not echo was most likely refused for its type: PX4 rejects a float sent as
                # an int outright (MPC_MAN_TILT_MAX set to 12 is an int in Python and a float on the
                # board). Ask what it really is - only for the few that failed - and send it again.
                for name in list(todo):
                    info = self.get_param(name, timeout=2.0)
                    if info is None:
                        done[name] = {"name": name, "ok": False,
                                      "error": "this firmware has no parameter of that name"}
                        if progress:
                            progress(name, done[name])
                        todo.remove(name)
                        continue
                    plan(name, info["type"])
            for name in todo:
                wire, ptype, _ = want[name]
                self._param_set_result.pop(name, None)
                with self._ctl_lock:
                    self.ctl.mav.param_set_send(self.target_system, self.target_component,
                                                name.encode()[:16], wire, ptype)
                time.sleep(pace)
            deadline = time.time() + timeout + pace * len(todo)
            while time.time() < deadline and len(done) < len(values):
                for name in list(todo):
                    if name in done:
                        continue
                    res = self._param_set_result.get(name)
                    if res is None:
                        continue
                    _, ptype, target = want[name]
                    got = res["value"]
                    ok = (got == target) if ptype == PARAM_TYPE_INT32 else abs(float(got) - target) <= 1e-5 * max(1.0, abs(target))
                    done[name] = ({"name": name, "ok": True, "value": got} if ok else
                                  {"name": name, "ok": False, "value": got, "error": f"the aircraft kept {got}"})
                    if progress:
                        progress(name, done[name])
                if all(n in done for n in todo):
                    break
                time.sleep(0.01)
            todo = [n for n in todo if n not in done]
        for name in todo:
            done[name] = {"name": name, "ok": False, "error": "no answer from the aircraft (unknown parameter, "
                                                             "read-only, or the wrong type for this firmware)"}
            if progress:
                progress(name, done[name])
        return [done[name] for name in values]

    # --------------------------------------------------------------- commands
    def shell(self, command: str, timeout: float = 3.0, idle: float = 0.4) -> str:
        """Run a NuttX/PX4 shell command over MAVLink SERIAL_CONTROL and return its output."""
        with self._shell_lock:
            self._shell_buf.clear()
        data = (command.strip() + "\n").encode()
        flags = mavlink.SERIAL_CONTROL_FLAG_RESPOND | mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE | mavlink.SERIAL_CONTROL_FLAG_MULTI
        for i in range(0, max(len(data), 1), 70):
            chunk = data[i:i + 70]
            with self._ctl_lock:
                self.ctl.mav.serial_control_send(mavlink.SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(chunk),
                                                 chunk + b"\x00" * (70 - len(chunk)))
        t0 = time.time()
        last_len, last_change = 0, time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.05)
            with self._shell_lock:
                n = len(self._shell_buf)
            if n != last_len:
                last_len, last_change = n, time.time()
            elif n and time.time() - last_change > idle:
                break
        with self._shell_lock:
            out = bytes(self._shell_buf).decode(errors="replace")
        return out.replace("\r", "")

    def restart_estimator(self) -> bool:
        """Restart EKF2 on the vehicle (after a board-rotation change, or when it initialised on bad data).
        Returns False when the shell gave no sign the module came back, so callers can fall back to a reboot."""
        self.log("[px4] restarting the estimator (ekf2 stop / start)")
        self.shell("ekf2 stop", timeout=2.0)
        time.sleep(1.0)
        self.shell("ekf2 start", timeout=2.0)
        # Verify by data rather than by shell text (replies are slow or lost on a saturated USB link): PX4 only
        # streams ATTITUDE while EKF2 publishes, so a running estimator shows as a steady flow of them.
        time.sleep(1.5)
        n0 = self.rx_types.get("ATTITUDE", 0)
        time.sleep(1.0)
        ok = self.rx_types.get("ATTITUDE", 0) - n0 >= 5
        if not ok:
            self.log("[px4] estimator restart: EKF2 is not publishing an attitude")
        return ok

    def request_autopilot_version(self) -> None:
        self.send_command_long(mavlink.MAV_CMD_REQUEST_MESSAGE, float(mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION))

    # ----------------------------------------------------------------- the board's own flight logs
    def send_log_request_list(self, start: int = 0, end: int = 0xFFFF) -> None:
        with self._ctl_lock:
            self.ctl.mav.log_request_list_send(self.target_system, self.target_component, start, end)

    def send_log_request_data(self, log_id: int, ofs: int, count: int) -> None:
        with self._ctl_lock:
            self.ctl.mav.log_request_data_send(self.target_system, self.target_component, log_id, ofs, count)

    def send_log_request_end(self) -> None:
        with self._ctl_lock:
            self.ctl.mav.log_request_end_send(self.target_system, self.target_component)

    def preflight_storage(self, save: bool = True) -> None:
        """MAV_CMD_PREFLIGHT_STORAGE: 1 = write params to storage."""
        self.send_command_long(mavlink.MAV_CMD_PREFLIGHT_STORAGE, 1.0 if save else 0.0)

    def reboot(self) -> None:
        """Reboot the autopilot. The commander refuses the MAVLink reboot in some states (e.g. flight
        termination), so fall back to the nsh `reboot` command over the MAVLink shell."""
        self.last_ack = {}
        self.send_command_long(mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1.0)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            ack = self.last_ack
            if ack.get("command") == mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                if ack.get("result") == 0:
                    return
                break
            time.sleep(0.05)
        self.log("[px4] reboot command not accepted, rebooting through the shell")
        try:
            self.shell("reboot", timeout=1.0)
        except Exception as e:
            self.log(f"[px4] shell reboot failed: {e}")

    def status(self) -> dict:
        return {
            "mode": self.mode,
            "address": self.address,
            "connected": self.connected and (time.time() - self.last_rx_time < 3.0),
            "ctl_connected": self.ctl_connected and (time.time() - self.ctl_rx_time < 3.0),
            "ctl_address": self.ctl_address,
            "target_system": self.target_system,
            "armed": self.armed,
            "hil_enabled": self.hil_enabled,
            "custom_mode": self.custom_mode,
            "rx_count": self.rx_count,
            "param_count": self.param_count,
            "params_loaded": sum(1 for v in self.params.values() if v["index"] < self.param_count),
            "actuator_seq": self.actuator_seq,
            "actuators": [round(float(a), 3) for a in self.actuators],
            "actuator_armed": self.actuator_armed,
            "rx_types": dict(sorted(self.rx_types.items(), key=lambda kv: -kv[1])[:25]),
            "events": [e for e in list(self.recent_events)[-12:]],
            "listening": self.listening,
            "radio": dict(self.radio, age=round(time.time() - self.radio["t"], 2)) if self.radio else None,
            "telemetry": {**self.measured_rate(), "share": self.share, "budget_bps": self.link_budget_bps(),
                          "board_cap_bps": self._mav_rate or 0,
                          "asked_bps": round(sum(self.MSG_BYTES.get(m, 40) * hz for m, hz in self.asked_rates.items())),
                          "asked": {str(m): hz for m, hz in self.asked_rates.items() if hz}},
            # how long ago the aircraft last said where it is: a real flight is only "working" while this
            # keeps arriving, and over a radio it is the first thing to go
            "telemetry_age": round(time.time() - max(self.board_state.get("t_pos", 0.0),
                                                     self.board_state.get("t_att", 0.0)), 2)
            if self.board_state else None,
            "board_imu": self.board_imu,
            "board_sys": self.board_sys,
            "board_att": self.board_att,
            "manual": self.manual_last if (self.manual_last and time.time() - self.manual_last.get("t", 0) < 1.0) else {},
            "rc": self.rc if (self.rc and time.time() - self.rc.get("t", 0) < 3.0) else {},
            "last_ack": self.last_ack,
            "qgc_proxy": self.qgc_proxy,
        }
