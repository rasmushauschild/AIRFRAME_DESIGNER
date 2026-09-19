"""Short, bounded actuator-test pulses. No arming or persistent output overrides."""
import math
import threading
import time
ACTUATOR_TEST_COMMAND = 310  # PX4 VehicleCommand.msg; absent from this pymavlink common dialect

class MotorTest:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = None
        self.deadline = 0.0

    def send(self, link, motor, value, count):
        with self.lock:
            if isinstance(motor, bool) or not isinstance(motor, int) or not 1 <= motor <= count:
                raise ValueError("Invalid motor number")
            if not math.isfinite(value) or not 0 <= value <= .15:
                raise ValueError("Motor test is limited to 0–15% command")
            if not link.ctl_connected or time.time()-link.heartbeat_time > 2:
                raise ValueError("Fresh flight-controller heartbeat required")
            if link.mode != "hitl" or link.armed:
                raise ValueError("Hardware motor tests require a disarmed flight controller")
            now = time.monotonic()
            if value > 0 and self.active not in (None, motor) and now < self.deadline:
                raise ValueError("Release the previous motor and wait one second before switching")
            # A pulse expires on the flight controller even if the browser/server/USB disappears.
            link.send_command_long(ACTUATOR_TEST_COMMAND, value, .5 if value > 0 else 0, 0, 0, motor)
            if value > 0:
                self.active, self.deadline = motor, now + .6
            # Keep the previous deadline after a stop: a lost stop cannot overlap two motors.
            return {"ok": True, "sent": True, "motor": motor, "command": value, "timeout_s": .5}
