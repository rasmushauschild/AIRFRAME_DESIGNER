import sys, glob, numpy as np
from pyulog import ULog
T = ["vehicle_angular_velocity", "vehicle_rates_setpoint", "vehicle_torque_setpoint", "vehicle_thrust_setpoint",
     "control_allocator_status", "actuator_motors", "vehicle_local_position", "vehicle_land_detected", "vehicle_status", "vehicle_attitude"]
def get(u, name, field, i=0):
    for d in u.data_list:
        if d.name == name and d.multi_id == i and field in d.data:
            return d.data["timestamp"] / 1e6, np.asarray(d.data[field], float)
    return None, None
for f in sorted(glob.glob("results/logs/log_42[0-5]_*.ulg")):
    u = ULog(f, message_name_filter_list=T)
    tl, landed = get(u, "vehicle_land_detected", "landed")
    tz, z = get(u, "vehicle_local_position", "z")
    tr, r = get(u, "vehicle_angular_velocity", "xyz[2]")
    if tr is None or tz is None: print(f, "missing"); continue
    air = tl[np.where(landed == 0)[0]] if tl is not None else []
    if not len(air): print(f.split('/')[-1], "never airborne"); continue
    t0 = air[0]
    tth, thr = get(u, "vehicle_thrust_setpoint", "xyz[2]")
    tu, unz = get(u, "control_allocator_status", "unallocated_torque[2]")
    ttq, tqz = get(u, "vehicle_torque_setpoint", "xyz[2]")
    tm = None
    mot = [get(u, "actuator_motors", f"control[{k}]") for k in range(9)]
    def seg(t, v, a, b):
        m = (t >= a) & (t < b); return v[m] if t is not None else np.array([])
    zt = np.interp(t0 + np.arange(0, 40, 0.5), tz, -z)
    zmax = zt.max(); t_reach = t0 + np.arange(0, 40, 0.5)[np.argmax(zt > 0.9 * zmax)]
    print(f"{f.split('/')[-1]}: airborne at {t0 - tz[0]:.1f}s, climb to {zmax:.2f} m reached after {t_reach - t0:.1f}s")
    for name, a, b in (("ground (5 s before lift)", t0 - 5, t0), ("climb", t0, t_reach), ("hover (next 10 s)", t_reach, t_reach + 10)):
        yr = np.degrees(seg(tr, r, a, b)); th = -seg(tth, thr, a, b); uz = seg(tu, unz, a, b); q = seg(ttq, tqz, a, b)
        mm = np.array([seg(t, v, a, b).mean() if t is not None and len(seg(t, v, a, b)) else np.nan for t, v in mot])
        mx = np.array([seg(t, v, a, b).max() if t is not None and len(seg(t, v, a, b)) else np.nan for t, v in mot])
        print(f"   {name:26} yaw rate mean {yr.mean() if len(yr) else np.nan:7.1f} max|{np.abs(yr).max() if len(yr) else np.nan:6.1f}| deg/s | thrust sp {th.mean() if len(th) else np.nan:.2f} "
              f"| yaw torque sp {q.mean() if len(q) else np.nan:+.3f} | unallocated yaw {uz.mean() if len(uz) else np.nan:+.3f} | motor max {np.nanmax(mx):.2f} mean {np.round(mm,2)}")
