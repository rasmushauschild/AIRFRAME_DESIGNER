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
        self.manual_last: dict = {}          # last MANUAL_CONTROL we sent (USB joystick)
        self._shell_buf = bytearray()
        self._shell_lock = threading.Lock()
        self.recent_events = deque(maxlen=60)
        self._last_event_text: dict[str, float] = {}
        self.rx_count = 0
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
        with self._write_lock:
            self.conn.mav.hil_sensor_send(**s)

    def send_hil_gps(self, g: dict) -> None:
        with self._write_lock:
            self.conn.mav.hil_gps_send(**g)

    def send_hil_state_quaternion(self, st: dict) -> None:
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
            t = msg.get_type()
            self.rx_types[t] = self.rx_types.get(t, 0) + 1
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
                if msg.result != 0:
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

    def set_params(self, values: dict[str, float | int], progress: Callable[[str, dict], None] | None = None) -> list[dict]:
        results = []
        for k, v in values.items():
            r = self.set_param(k, v)
            results.append(r)
            if progress:
                progress(k, r)
        return results

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
            "board_imu": self.board_imu,
            "board_sys": self.board_sys,
            "board_att": self.board_att,
            "manual": self.manual_last if (self.manual_last and time.time() - self.manual_last.get("t", 0) < 1.0) else {},
            "rc": self.rc if (self.rc and time.time() - self.rc.get("t", 0) < 3.0) else {},
            "last_ack": self.last_ack,
            "qgc_proxy": self.qgc_proxy,
        }
