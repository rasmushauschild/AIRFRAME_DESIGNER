"""Yaw snap in log_399: real rotation or estimator reset, and what the mag / accel / vibration were doing."""
import sys
import numpy as np
from pyulog import ULog
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path, out = sys.argv[1], sys.argv[2]
OFF = 1.40  # the app run is 1.4 s behind boot time (the reset is at 46.62 boot, 45.22 in the run)
W0, W1 = 43.0 - OFF, 48.0 - OFF
u = ULog(path)
D = {}
for d in u.data_list:
    D.setdefault(d.name, {})[d.multi_id] = d.data
B = lambda d, k="timestamp": d[k].astype(float) / 1e6 - OFF  # the app's time axis
win = lambda t: (t >= W0) & (t <= W1)

def yaw_of(q):
    return np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))

va = D["vehicle_attitude"][0]; tva = B(va)
q = np.stack([va["q[%d]" % i] for i in range(4)], 1)
yaw = np.unwrap(np.radians(yaw_of(q)))
print("vehicle_attitude fields:", [k for k in va if "reset" in k])
rc = va["quat_reset_counter"]
for i in np.flatnonzero(np.diff(rc)):
    dq = np.array([va["delta_q_reset[%d]" % j][i + 1] for j in range(4)])
    print("  quat reset #%d at t=%.3f: delta yaw %.1f deg" % (rc[i + 1], tva[i + 1], yaw_of(dq[None])[0]))

# biggest attitude-yaw step vs what the gyro integrated over the same interval
sc = D["sensor_combined"][0]; tsc = B(sc)
gz = sc["gyro_rad[2]"]; gx, gy = sc["gyro_rad[0]"], sc["gyro_rad[1]"]
dyaw = np.diff(np.degrees(yaw)); dt = np.diff(tva)
i = np.argmax(np.abs(dyaw) * win(tva[1:]))
a, b = tva[i], tva[i + 1]
m = (tsc >= a) & (tsc <= b)
print("largest attitude yaw step in window: %.1f deg between t=%.3f and %.3f (%.0f ms); gyro z integrated over it: %.2f deg" % (
    dyaw[i], a, b, 1e3 * (b - a), np.degrees(np.trapezoid(gz[m], tsc[m])) if m.sum() > 1 else 0.0))
# over 0.5 s around it: attitude yaw change vs gyro (body z ~ yaw near level)
for lo, hi in ((a - .5, a), (a - .05, b + .05), (b, b + .5), (W0, W1)):
    mv = (tva >= lo) & (tva <= hi); mg = (tsc >= lo) & (tsc <= hi)
    print("  %.2f-%.2f: attitude yaw change %+.1f deg, gyro z integral %+.1f deg" % (
        lo, hi, np.degrees(yaw[mv][-1] - yaw[mv][0]), np.degrees(np.trapezoid(gz[mg], tsc[mg]))))
T_SNAP = b

# estimator: status flags and mag fusion around the snap
if "estimator_status_flags" in D:
    f = D["estimator_status_flags"][0]; tf = B(f)
    keys = [k for k in f if k.startswith(("cs_", "fs_", "reset_count", "fault")) or "reset" in k]
    for k in keys:
        v = f[k]
        ch = np.flatnonzero(np.diff(v.astype(float)))
        if len(ch):
            print("  flag %-26s changes at %s" % (k, [(round(tf[j + 1], 2), int(v[j]), int(v[j + 1])) for j in ch][:8]))
es = D["estimator_status"][0]; tes = B(es)
for k in ("mag_test_ratio", "hdg_test_ratio", "heading_test_ratio"):
    if k in es:
        mm = win(tes); print("  %s in window: %s" % (k, np.round(es[k][mm], 2).tolist()))
for k in ("reset_count_quat", "quat_reset_counter"):
    if k in es: print("  estimator_status %s:" % k, np.unique(es[k]))
print("  estimator_status fields with yaw/mag/reset:", [k for k in es if any(s in k for s in ("yaw", "mag", "reset", "hdg"))])

am_ = D["estimator_aid_src_mag"][0]; tam_ = B(am_)
mm = (tam_ >= W0 - 3) & (tam_ <= W1 + 3)
print("\nestimator_aid_src_mag (%.1f Hz):" % (1 / np.median(np.diff(tam_))))
for j in np.flatnonzero(mm):
    print("  t=%.2f innov %s test_ratio %s fused %d rejected %d" % (tam_[j], np.round([am_["innovation[%d]" % k][j] for k in range(3)], 3),
          np.round([am_["test_ratio[%d]" % k][j] for k in range(3)], 2), am_["fused"][j], am_["innovation_rejected"][j]))

# magnetometer
vm = D["vehicle_magnetometer"][0]; tvm = B(vm)
mag = np.stack([vm["magnetometer_ga[%d]" % k] for k in range(3)], 1); nrm = np.linalg.norm(mag, axis=1)
vs = D["vehicle_status"][0]; tv = B(vs); armed_t = tv[vs["arming_state"] == 2]
am = D["actuator_motors"][0]; tm = B(am); mot = np.nan_to_num(np.stack([am["control[%d]" % k] for k in range(9)], 1)); tot = mot.sum(1)
tot_at = np.interp(tvm, tm, tot)
pre = tvm < armed_t[0]
print("\nmagnetometer (%.1f Hz): |B| before arming %.3f +/- %.3f G; armed min %.3f max %.3f" % (
    1 / np.median(np.diff(tvm)), nrm[pre].mean(), nrm[pre].std(), nrm[~pre & (tvm < armed_t[-1])].min(), nrm[~pre & (tvm < armed_t[-1])].max()))
ok = ~pre & (tvm < armed_t[-1])
print("  corr(|B|, total motor command) while armed: %.2f" % np.corrcoef(nrm[ok], tot_at[ok])[0, 1])
# heading from mag in body frame (tilt-compensated) vs attitude yaw
roll_ = np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
pitch_ = np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1))
r_ = np.interp(tvm, tva, roll_); p_ = np.interp(tvm, tva, pitch_)
mx, my, mz = mag.T
Xh = mx * np.cos(p_) + my * np.sin(r_) * np.sin(p_) + mz * np.cos(r_) * np.sin(p_)
Yh = my * np.cos(r_) - mz * np.sin(r_)
mhdg = np.degrees(np.arctan2(-Yh, Xh))
att_at = np.degrees(np.interp(tvm, tva, yaw))
diff = (mhdg - att_at + 180) % 360 - 180
print("  mag heading minus attitude yaw (deg), t from %.1f:" % W0)
for j in np.flatnonzero((tvm >= W0 - 2) & (tvm <= W1 + 1)):
    print("    t=%.2f |B|=%.3f motors_sum=%.2f mag-att=%+.0f" % (tvm[j], nrm[j], tot_at[j], diff[j]))

# accelerometer and vibration in short windows across the flight
fs = 1 / np.median(np.diff(tsc))
ac = np.stack([sc["accelerometer_m_s2[%d]" % k] for k in range(3)], 1)
print("\nper 0.5 s: gyro 10-40 Hz rms (deg/s, x/y/z), accel 10-60 Hz rms (m/s2, |xyz|), |a| max, yaw rate mean")
rows = []
for s in np.arange(41.5 - OFF, 53.5 - OFF, 0.5):
    m = (tsc >= s) & (tsc < s + .5)
    if m.sum() < 64: continue
    f, Pg = signal.welch(gx[m][:, None] * 0 + np.stack([gx[m], gy[m], gz[m]], 1), fs, nperseg=64, axis=0)
    f, Pa = signal.welch(ac[m], fs, nperseg=64, axis=0)
    bg = (f >= 10) & (f <= 40); ba = (f >= 10) & (f <= 60); df = f[1] - f[0]
    gr = np.degrees(np.sqrt(Pg[bg].sum(0) * df)); ar = np.sqrt(Pa[ba].sum() * df)
    rows.append((s, *gr, ar))
    print("  %.1f  gyro %5.1f %5.1f %5.1f   accel %4.2f   |a|max %5.1f   r %+6.1f %s" % (
        s, *gr, ar, np.linalg.norm(ac[m], axis=1).max(), np.degrees(gz[m].mean()), "<-- snap" if s <= T_SNAP < s + .5 else ""))
rows = np.array(rows)

# yaw control around the snap
sp = D["vehicle_attitude_setpoint"][0]; tsp = B(sp)
rs = D["vehicle_rates_setpoint"][0]; trs = B(rs)
print("\nyaw setpoint / rate setpoint / m5 m6 around the snap:")
for s in np.arange(T_SNAP - .3, T_SNAP + .6, .1):
    j = np.argmin(abs(tsp - s)); k = np.argmin(abs(trs - s)); n = np.argmin(abs(tm - s)); v = np.argmin(abs(tva - s))
    print("  t=%.2f yaw %+7.1f yaw_sp %+7.1f yawrate_sp %+6.1f deg/s  m5 %.2f m6 %.2f" % (
        s, (np.degrees(yaw[v]) + 180) % 360 - 180, (yaw_of(np.array([[sp["q_d[%d]" % z][j] for z in range(4)]]))[0]), np.degrees(rs["yaw"][k]), mot[n, 4], mot[n, 5]))

# figure
fig, axs = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
m = (tva > 41.5 - OFF) & (tva < 53.5 - OFF)
axs[0].plot(tva[m], np.degrees(yaw[m]), label="attitude yaw (estimator)")
mg = (tsc > 41.5 - OFF) & (tsc < 53.5 - OFF)
gint = np.degrees(np.cumsum(gz[mg]) / fs); gint += np.degrees(yaw[m][0]) - gint[0]
axs[0].plot(tsc[mg], gint, "--", label="gyro z integrated (what the aircraft did)")
mv = (tvm > 41.5 - OFF) & (tvm < 53.5 - OFF)
axs[0].plot(tvm[mv], np.degrees(np.interp(tvm[mv], tva, yaw)) + diff[mv], "o", ms=3, label="magnetometer heading")
axs[0].set_ylabel("yaw, unwrapped deg"); axs[0].legend(fontsize=8)
axs[1].plot(tsc[mg], np.degrees(gz[mg]), lw=.5, label="gyro z"); ms = (trs > 41.5 - OFF) & (trs < 53.5 - OFF)
axs[1].plot(trs[ms], np.degrees(rs["yaw"][ms]), "k--", lw=.8, label="yaw rate setpoint"); axs[1].set_ylabel("deg/s"); axs[1].legend(fontsize=8)
axs[2].plot(tvm[mv], nrm[mv], "o-", ms=3, label="|B| (G)"); ax2 = axs[2].twinx()
mm2 = (tm > 41.5 - OFF) & (tm < 53.5 - OFF); ax2.plot(tm[mm2], tot[mm2], "tab:red", lw=.5, alpha=.6, label="sum of motor commands")
axs[2].set_ylabel("|B| gauss"); ax2.set_ylabel("motor sum", color="tab:red"); axs[2].legend(fontsize=8, loc="upper left")
axs[3].plot(rows[:, 0] + .25, rows[:, 1], "o-", label="gyro x 10-40 Hz"); axs[3].plot(rows[:, 0] + .25, rows[:, 2], "o-", label="gyro y 10-40 Hz")
axs[3].plot(rows[:, 0] + .25, rows[:, 3], "o-", label="gyro z 10-40 Hz"); axs[3].set_ylabel("deg/s rms"); axs[3].legend(fontsize=8)
ax3 = axs[3].twinx(); ax3.plot(rows[:, 0] + .25, rows[:, 4], "s-", color="k", label="accel 10-60 Hz"); ax3.set_ylabel("accel m/s2 rms")
for k in (4, 5): axs[4].plot(tm[mm2], mot[mm2, k], lw=.7, label="m%d" % (k + 1))
axs[4].set_ylabel("motor cmd"); axs[4].legend(fontsize=8)
for ax in axs: ax.axvline(T_SNAP, color="r", lw=.8)
axs[4].set_xlabel("s, the app's time axis (boot time - 1.4 s); red = the snap")
plt.tight_layout(); plt.savefig(out + "_snap.png", dpi=105)
print("figure written")
