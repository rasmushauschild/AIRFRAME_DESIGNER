import sys, numpy as np
from pyulog import ULog
from scipy import signal
u = ULog(sys.argv[1], ["actuator_motors", "vehicle_angular_velocity", "sensor_combined"]); t0 = float(u.start_timestamp)
D = {d.name: d.data for d in u.data_list}
am = D["actuator_motors"]; tam = (am["timestamp"].astype(float) - t0) / 1e6
mot = np.nan_to_num(np.stack([am["control[%d]" % i] for i in range(9)], 1)); fsm = 1 / np.median(np.diff(tam))
sc = D["sensor_combined"]; tsc = (sc["timestamp"].astype(float) - t0) / 1e6; fs = 1 / np.median(np.diff(tsc))
g = np.degrees(np.stack([sc["gyro_rad[%d]" % i] for i in range(3)], 1))
for a, b in [tuple(map(float, w.split(":"))) for w in sys.argv[2:]]:
    m = (tam >= a) & (tam <= b); f, P = signal.welch(mot[m], fsm, nperseg=min(256, m.sum()), axis=0, detrend="linear")
    mm = f > 3; Ps = P[mm].sum(1)
    ms = (tsc >= a) & (tsc <= b); fg, Pg = signal.welch(g[ms], fs, nperseg=min(256, ms.sum()), axis=0, detrend="linear"); mg = fg > 3
    print("  %.1f-%.1f s: motor ripple peak %.1f Hz, rms 3-60 Hz per motor %s | gyro x,y peak %.1f / %.1f Hz, rms 10-40 Hz x,y %s deg/s | frac motors at 0 or 1: %.2f" % (
        a, b, f[mm][np.argmax(Ps)], np.round(np.sqrt(P[(f > 3) & (f < 60)].sum(0) * (f[1] - f[0])), 2),
        fg[mg][np.argmax(Pg[mg, 0])], fg[mg][np.argmax(Pg[mg, 1])],
        np.round(np.sqrt(Pg[(fg > 10) & (fg < 40), :2].sum(0) * (fg[1] - fg[0])), 1), np.mean((mot[m] <= 0.01) | (mot[m] >= 0.99))))
