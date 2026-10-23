"""Same yaw metrics from any PX4 log, real or simulated: reference = PX4's 'Takeoff detected'."""
import sys, json, numpy as np
from pyulog import ULog
from scipy import signal
def metrics(p):
    u = ULog(p, ["sensor_combined", "rate_ctrl_status", "trajectory_setpoint", "vehicle_torque_setpoint", "vehicle_local_position"]); t0 = float(u.start_timestamp)
    D = {d.name: d.data for d in u.data_list}; T = lambda d: (d["timestamp"].astype(float) - t0) / 1e6
    tk = [(m.timestamp - t0) / 1e6 for m in u.logged_messages if "Takeoff detected" in m.message]
    if not tk: return None
    tk = tk[0]; sc = D["sensor_combined"]; ts = T(sc)
    fs = 1 / np.median(np.diff(ts)); b, a = signal.butter(2, 2, fs=fs); r = np.degrees(signal.filtfilt(b, a, sc["gyro_rad[2]"]))
    g = np.arange(0, 10.01, 0.05); rr = np.interp(g + tk, ts, r)
    rc = D["rate_ctrl_status"]; tr = T(rc); it = rc["yawspeed_integ"]; hit = tr[(tr > tk) & (it >= 0.27)]
    tsp = D["trajectory_setpoint"]; tt = T(tsp); hold = tt[(tt > tk) & ~np.isnan(tsp["yaw"])]
    m8 = g <= 8; pk = np.argmax(np.abs(rr[m8]))
    P = u.initial_parameters
    return dict(grid=g, r=rr, peak=float(abs(rr[m8][pk])), t_peak=float(g[m8][pk]), turned=float(np.trapezoid(rr[m8], g[m8])), r8=float(rr[np.argmin(abs(g - 8))]),
                integ027=float(hit[0] - tk) if len(hit) else np.nan, hold=float(hold[0] - tk) if len(hold) else np.nan,
                km=[round(P.get("CA_ROTOR%d_KM" % i, np.nan), 4) for i in (0, 6, 7)], yawP=P.get("MC_YAWRATE_P"), yawI=P.get("MC_YAWRATE_I"))
if __name__ == "__main__":
    for p in sys.argv[1:]:
        m = metrics(p)
        if m is None: print("%-34s no takeoff" % p.split("/")[-1]); continue
        print("%-34s peak %4.0f deg/s at +%.1f s | turned %+5.0f deg in 8 s | rate at +8 s %+4.0f | integrator 0.27 at +%4.1f s | heading hold from +%4.1f s | PX4 KM %s yaw P/I %.3g/%.3g" % (
            p.split("/")[-1][:34], m["peak"], m["t_peak"], m["turned"], m["r8"], m["integ027"], m["hold"], m["km"], m["yawP"], m["yawI"]))
