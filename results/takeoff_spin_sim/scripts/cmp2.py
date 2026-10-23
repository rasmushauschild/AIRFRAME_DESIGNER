import sys, json, numpy as np
from pyulog import ULog
exec(open("cmp.py").read().split("if __name__")[0])
for p in sys.argv[1:]:
    n = p.split("/")[-1][:-4]
    m = metrics(p)
    res = json.load(open(p[:-4] + ".json"))
    u = ULog(p, ["actuator_motors", "control_allocator_status", "vehicle_attitude"]); t0 = float(u.start_timestamp); D = {d.name: d.data for d in u.data_list}
    tk = [(x.timestamp - t0) / 1e6 for x in u.logged_messages if "Takeoff detected" in x.message]
    if not tk or m is None: print("%-28s no takeoff" % n); continue
    tk = tk[0]; T = lambda d: (d["timestamp"].astype(float) - t0) / 1e6
    am = D["actuator_motors"]; ta = T(am); M = np.nan_to_num(np.stack([am["control[%d]" % i] for i in range(9)], 1)); w = (ta > tk) & (ta < tk + 15)
    pinned = ((M[w] >= 0.99) | (M[w] <= 0.01)).any(1).mean()
    ca = D["control_allocator_status"]; tc = T(ca); wc = (tc > tk) & (tc < tk + 15)
    va = D["vehicle_attitude"]; tv = T(va); q = np.stack([va["q[%d]" % i] for i in range(4)], 1)
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    print("%-28s peak %4.0f deg/s at +%.1f | turned %+5.0f in 8 s | torque achieved %3.0f%% | any motor pinned %3.0f%% | max tilt %4.1f deg | takeoff %s" % (
        n, m["peak"], m["t_peak"], m["turned"], 100 * ca["torque_setpoint_achieved"][wc].mean(), 100 * pinned, tilt[(tv > tk) & (tv < tk + 15)].max(),
        "; ".join(f[:60] for f in res.get("failures", [])) or "ok"))
