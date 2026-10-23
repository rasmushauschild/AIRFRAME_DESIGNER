"""Measure the yaw plant from flight: Izz*rdot = k*tau_sp + d + c*r. Then the integrator state and the hover drift."""
import sys, glob, json, numpy as np
from pyulog import ULog
from scipy import signal
R = sys.argv[1]; IZZ = 1.73274
out = {}
allrows = []
for n in (409, 413, 414, 423, 424):
    p = glob.glob("%s/log_%d_*.ulg" % (R, n))[0]; res = json.load(open("res/%d.json" % n))
    u = ULog(p, ["sensor_combined", "vehicle_torque_setpoint", "rate_ctrl_status", "trajectory_setpoint", "vehicle_status"]); t0 = float(u.start_timestamp)
    D = {d.name: d.data for d in u.data_list}; T = lambda d: (d["timestamp"].astype(float) - t0) / 1e6
    tk, tkill = res["takeoff"], res["kill"]
    sc = D["sensor_combined"]; ts = T(sc); b, a = signal.butter(2, 3, fs=269)
    r = signal.filtfilt(b, a, sc["gyro_rad[2]"]); rdot = np.gradient(r, ts)
    tq = D["vehicle_torque_setpoint"]; tt = T(tq); tau = signal.filtfilt(b, a, tq["xyz[2]"])
    rc = D["rate_ctrl_status"]; tr = T(rc); integ = rc["yawspeed_integ"]
    tr_sp = D["trajectory_setpoint"]; ttr = T(tr_sp); hold = ~np.isnan(tr_sp["yaw"])
    t_hold = ttr[hold & (ttr > tk)][0] if (hold & (ttr > tk)).any() else np.nan
    g = np.arange(tk + 0.3, tkill - 0.3, 0.02)
    y = np.interp(g, ts, rdot) * IZZ; X = np.column_stack([np.interp(g, tt, tau), np.ones(len(g)), np.interp(g, ts, r)])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    r2 = 1 - ((y - X @ c) ** 2).sum() / ((y - y.mean()) ** 2).sum()
    allrows.append(np.column_stack([y, X]))
    hov = (g > t_hold + 2) if np.isfinite(t_hold) else (g > tk + 10)
    t_int = tr[(tr > tk) & (integ >= 0.27)]
    out[n] = dict(k=c[0], d=c[1], damp=c[2], r2=r2, heading_hold_from=t_hold - tk, peak=np.degrees(np.abs(r[(ts > tk) & (ts < tk + 8)]).max()),
                  t_peak=ts[(ts > tk) & (ts < tk + 8)][np.argmax(np.abs(r[(ts > tk) & (ts < tk + 8)]))] - tk,
                  integ_hover=np.interp(g[hov], tr, integ).mean(), integ_reach_027=(t_int[0] - tk) if len(t_int) else np.nan,
                  tau_hover=np.interp(g[hov], tt, tau).mean(), drift_hover=np.degrees(np.interp(g[hov], ts, r).mean()))
    print("log_%d: k %.2f N*m/unit, disturbance d %+.2f N*m, damping %.2f, R2 %.2f | heading hold from +%.1f s | peak spin %.0f deg/s at +%.1f s | integrator reaches 0.27 at +%.1f s, hover %.3f (limit 0.30) | hover torque sp %.3f, drift %+.0f deg/s" % (
        n, c[0], c[1], c[2], r2, out[n]["heading_hold_from"], out[n]["peak"], out[n]["t_peak"], out[n]["integ_reach_027"], out[n]["integ_hover"], out[n]["tau_hover"], out[n]["drift_hover"]))
A = np.vstack(allrows); c, *_ = np.linalg.lstsq(A[:, 1:], A[:, 0], rcond=None)
r2 = 1 - ((A[:, 0] - A[:, 1:] @ c) ** 2).sum() / ((A[:, 0] - A[:, 0].mean()) ** 2).sum()
print("ALL: k %.2f N*m per unit torque, d %+.2f N*m, damping %.2f N*m per rad/s, R2 %.2f" % (c[0], c[1], c[2], r2))
json.dump({"k": c[0], "d": c[1], "damp": c[2], "per": {str(k): v for k, v in out.items()}}, open("yawplant.json", "w"), default=float)
