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
        if abs(af.hover_pitch_deg) > 1e-6:
            raise ValueError('Experimental body-vector loop currently requires a zero-degree hover frame')
        self.table = np.asarray(table, float)
        if self.table.ndim != 2 or self.table.shape[1] != 4 or len(self.table) < 2:
            raise ValueError('trim_table requires rows [speed, Fx, Fy, Fz]')
        if not np.all(np.isfinite(self.table)) or np.any(np.diff(self.table[:,0]) <= 0):
            raise ValueError('trim_table must be finite and ordered by increasing speed')
        # Reproduce PX4 pseudo-inverse thrust normalization using exported CT values.
        rotors = af.active_rotors()
        ct = np.array([af.px4_overrides.get(f'CA_ROTOR{i}_CT', 6.5) for i in range(len(rotors))])
        physical_per_ct = np.array([r.effective_max_thrust() for r in rotors]) / ct
        if np.ptp(physical_per_ct) > 1e-3:
            raise ValueError('CT values must be proportional to effective thrust')
        mix = np.linalg.pinv(af.effectiveness() * ct)
        self.scale = np.array([(np.abs(mix[:,j])[np.abs(mix[:,j])>1e-7].mean() if np.any(np.abs(mix[:,j])>1e-7) else 0.) for j in [3,4,5]]) / physical_per_ct.mean()
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
        force += s.rotmat.T @ (self.mass*acceleration)
        normalized=np.clip(force*self.scale,-.95,.95)
        roll=float(np.clip(acceleration[1]/9.80665,-math.radians(8),math.radians(8)))
        normalized[1]=0.
        q=[math.cos(roll/2),math.sin(roll/2),0.,0.]
        msg=BodyThrustMessage(int(simr.t*1000),simr.link.target_system,simr.link.target_component,q,normalized)
        with simr.link._ctl_lock:
            simr.link.ctl.mav.send(msg)
