"""Experimental SITL outer loop for independently commanded body thrust.

Uses ideal simulator position/velocity as feedback and an explicitly supplied,
offline trim table. PX4 still controls attitude, rates and motor allocation.
This is a design test harness, not an onboard controller or a sensor validation.
"""
import math
import struct
import numpy as np
from ..px4.link import mavlink


class BodyThrustMessage(mavlink.MAVLink_set_attitude_target_message):
    """MAVLink2 thrust_body extension, absent from older installed pymavlink."""
    def __init__(self, time_ms, system, component, q, thrust_body):
        super().__init__(time_ms, system, component, 7 | 32, q, 0., 0., 0., 0.)
        self.thrust_body = list(thrust_body)

    def pack(self, mav, force_mavlink1=False):
        if force_mavlink1:
            raise ValueError('Body-vector thrust requires MAVLink2')
        payload = self.unpacker.pack(self.time_boot_ms, *self.q, self.body_roll_rate,
                                    self.body_pitch_rate, self.body_yaw_rate, self.thrust,
                                    self.target_system, self.target_component, self.type_mask)
        return self._pack(mav, self.crc_extra, payload + struct.pack('<3f', *self.thrust_body))


class WingVelocityController:
    def __init__(self, simr, table):
        af = simr.airframe
        self.hover_rotation = af.hover_rotation()
        self.hover_pitch = float(af.hover_pitch_deg)
        self.table = np.asarray(table, float)
        if self.table.ndim != 2 or self.table.shape[1] not in (4, 5) or len(self.table) < 2:
            raise ValueError('trim_table requires rows [speed, Fx, Fy, Fz, optional structural pitch degrees]')
        if not np.all(np.isfinite(self.table)) or np.any(np.diff(self.table[:,0]) <= 0):
            raise ValueError('trim_table must be finite and ordered by increasing speed')
        # Reproduce PX4 pseudo-inverse thrust normalization using exported CT values.
        rotors = af.active_rotors()
        capacity = np.array([r.effective_max_thrust() for r in rotors])
        mix = np.linalg.pinv(af.effectiveness() * capacity)
        self.scale = np.array([(np.abs(mix[:,j])[np.abs(mix[:,j])>1e-7].mean() if np.any(np.abs(mix[:,j])>1e-7) else 0.) for j in [3,4,5]])
        if not np.all(np.isfinite(self.scale)) or self.scale[0] <= 0 or self.scale[2] <= 0:
            raise ValueError('Aircraft must provide finite XZ thrust normalization')
        self.mass = af.mass.mass
        self.alt = -float(simr.sim.pos[2])
        self.integral = np.zeros(3)
        self.reference = float(simr.sim.vel[0])
        self.last_time = simr.t
        self.y = float(simr.sim.pos[1])

    def step(self, simr, target, ramp=1.0):
        dt = min(.1, max(.001, simr.t-self.last_time)); self.last_time = simr.t
        s=simr.sim
        self.reference += float(np.clip(target-self.reference,-ramp*dt,ramp*dt))
        vz = np.clip((-s.pos[2]-self.alt)*.65,-2.,2.)
        desired=np.array([self.reference,np.clip((self.y-s.pos[1])*.4,-1.5,1.5),vz])
        error=desired-s.vel
        self.integral=np.clip(self.integral+error*dt,-5,5)
        acceleration=np.array([.8,1.2,1.5])*error+np.array([.18,.2,.35])*self.integral
        speed=float(np.linalg.norm(s.vel-s.wind_ned))
        force=np.array([np.interp(speed,self.table[:,0],self.table[:,j]) for j in [1,2,3]])
        # Trim forces are structural FRD; PX4 allocation uses the rotated hover frame.
        force += s.rotmat.T @ (self.mass*acceleration)
        normalized=np.clip((self.hover_rotation @ force)*self.scale,-.95,.95)
        roll=float(np.clip(acceleration[1]/9.80665,-math.radians(8),math.radians(8)))
        normalized[1]=0.
        from ..dynamics.quaternion import q_from_euler
        pitch = np.interp(speed, self.table[:,0], self.table[:,4]) if self.table.shape[1] == 5 else self.hover_pitch
        q=q_from_euler(roll, math.radians(pitch-self.hover_pitch), 0.).tolist()
        msg=BodyThrustMessage(int(simr.t*1000),simr.link.target_system,simr.link.target_component,q,normalized)
        with simr.link._ctl_lock:
            simr.link.ctl.mav.send(msg)


class NoseLower:
    """Ground support controller for simulator tests only, never sent to hardware."""
    def __init__(self, simr, motors, rate=3.):
        from .nose_lift import NoseLift
        if simr.link.mode != "sitl":
            raise ValueError("Nose lowering is SITL-only")
        if not motors or any(i < 0 or i >= simr.sim.rotors.n for i in motors):
            raise ValueError("Valid front motor indices required")
        self.support = NoseLift(motors, simr.airframe.landed_pitch_deg, rate_deg_s=rate)
        self.rate = abs(rate)
        self.target = float(simr.airframe.landed_pitch_deg)
        self.initial = np.asarray(simr.link.actuators[:simr.sim.rotors.n], float)
        self.t0 = self.last = simr.t
        self.settled = None
        self.fade = None
        self.done = False
        self.last_disarm = -10.
        simr.nose_lift = None

    def step(self, simr):
        s = simr.sim
        dt = max(.001, min(.1, simr.t-self.last)); self.last=simr.t
        nl = self.support
        nl._update_split(s, s.rates)
        pitch = math.degrees(s.euler[1]); q = math.degrees(s.rates[1])
        desired = float(np.clip((self.target-pitch)*.8, -self.rate, self.rate))
        err = desired-q
        nl.integral = float(np.clip(nl.integral+err*dt, -10, 10))
        frac = nl._balance_fraction(s)+nl.kq*err+nl.kqi*nl.integral
        cmd = nl._thrust_to_cmd(s, frac)
        blend = min(1., (simr.t-self.t0)/1.5)
        support = np.zeros(s.rotors.n)
        for k,i in enumerate(nl.motors):
            support[i] = np.clip(cmd*(nl.split[k] if nl.split else 1.),0,1)
        output = (1-blend)*self.initial+blend*support
        stable = s.on_ground and abs(pitch-self.target)<1.5 and abs(q)<1.5 and abs(s.vel[2])<.1
        self.settled = (self.settled if self.settled is not None else simr.t) if stable else None
        if self.fade is None and self.settled is not None and simr.t-self.settled>1.:
            self.fade=simr.t
            self.fade_output=output.copy()
        if self.fade is not None:
            output=self.fade_output*max(0.,1-(simr.t-self.fade)/2.)
        simr.motor_override=output.tolist()
        # PX4 must relinquish attitude control; simulator support remains until touchdown settles.
        if simr.t-self.last_disarm>1. and simr.link.armed:
            simr.link.disarm()
            self.last_disarm=simr.t
        if self.fade is not None and simr.t-self.fade>=2. and not simr.link.armed:
            simr.motor_override=None
            self.done=True
