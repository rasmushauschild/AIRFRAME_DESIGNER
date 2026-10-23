"""Yaw around takeoff: what the aircraft did, what PX4 asked for, what the allocator could give, ground contact."""
import sys, json
import numpy as np
from pyulog import ULog

p = sys.argv[1]
u = ULog(p); t0 = float(u.start_timestamp)
D = {}
for d in u.data_list:
    D.setdefault(d.name, {})[d.multi_id] = d.data
T = lambda d: (d["timestamp"].astype(float) - t0) / 1e6
P = u.initial_parameters
tk = [(m.timestamp - t0) / 1e6 for m in u.logged_messages if "Takeoff detected" in m.message][0]
vs = D["vehicle_status"][0]; tv = T(vs)
arm = tv[np.argmax(vs["arming_state"] == 2)]
def at(name, field, t, mid=0):
    d = D[name][mid]; return float(np.interp(t, T(d), d[field].astype(float)))
sc = D["sensor_combined"][0]; tsc = T(sc); gz = sc["gyro_rad[2]"]
def rmean(t, w=0.25):
    m = (tsc >= t - w / 2) & (tsc < t + w / 2); return float(np.degrees(gz[m].mean())) if m.any() else np.nan
am = D["actuator_motors"][0]; tam = T(am)
mot = np.nan_to_num(np.stack([am["control[%d]" % k] for k in range(9)], 1))
ld = D["vehicle_land_detected"][0]
print("== %s  armed %.2f, takeoff detected %.2f (%.1f s after arming); nav_state at takeoff %d" % (
    p.split('/')[-1][:7], arm, tk, tk - arm, int(np.interp(tk, tv, vs["nav_state"]))))
print("   t-tk  r(°/s)  r_sp  tau_z_sp  unalloc_z  yaw_integ  landed maybe ground  | m1-6 mean  m7   m8   m9 | heading(°)")
va = D["vehicle_attitude"][0]; tva = T(va); q = np.stack([va["q[%d]" % i] for i in range(4)], 1)
yaw = np.degrees(np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))))
rcs = D.get("rate_ctrl_status", {}).get(0)
for dt in np.arange(arm - tk - 0.5, 9.01, 0.5):
    t = tk + dt
    n = np.argmin(abs(tam - t))
    yi = float(np.interp(t, T(rcs), rcs["yawspeed_integ"])) if rcs is not None else np.nan
    print("  %5.1f  %6.1f %6.1f  %7.3f   %7.3f   %7.3f    %d     %d     %d    |  %.2f   %.2f %.2f %.2f | %6.0f" % (
        dt, rmean(t), np.degrees(at("vehicle_rates_setpoint", "yaw", t)), at("vehicle_torque_setpoint", "xyz[2]", t),
        at("control_allocator_status", "unallocated_torque[2]", t), yi,
        round(at("vehicle_land_detected", "landed", t)), round(at("vehicle_land_detected", "maybe_landed", t)),
        round(at("vehicle_land_detected", "ground_contact", t)), mot[n, :6].mean(), mot[n, 6], mot[n, 7], mot[n, 8], np.interp(t, tva, yaw)))
