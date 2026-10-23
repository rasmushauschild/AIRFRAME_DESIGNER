"""Second pass: per-span timeline, fundamental, rate tracking, high-frequency (motor) lines, clipping timing, figure."""
import sys
import numpy as np
from pyulog import ULog
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path, out = sys.argv[1], sys.argv[2]
u = ULog(path, ["vehicle_status", "actuator_motors", "vehicle_imu_status", "sensor_accel_fifo", "sensor_gyro_fifo",
                "vehicle_land_detected", "vehicle_local_position", "vehicle_angular_velocity", "vehicle_rates_setpoint",
                "vehicle_torque_setpoint", "control_allocator_status", "rate_ctrl_status", "battery_status"])
t0 = float(u.start_timestamp)
D = {}
for d in u.data_list:
    D.setdefault(d.name, {})[d.multi_id] = d.data
T = lambda d, k="timestamp": (d[k].astype(float) - t0) / 1e6

vs = D["vehicle_status"][0]; tv = T(vs); armed = vs["arming_state"] == 2
ld = D["vehicle_land_detected"][0]; tl = T(ld); landed = ld["landed"].astype(bool)
def spans_of(t, m):
    sp, s = [], None
    for i in range(len(t)):
        if m[i] and s is None: s = t[i]
        if not m[i] and s is not None: sp.append((s, t[i])); s = None
    if s is not None: sp.append((s, t[-1]))
    return sp
arm_sp = spans_of(tv, armed); air_sp = spans_of(tl, ~landed)
inside = lambda t, sp: np.any([(t >= a) & (t <= b) for a, b in sp], 0) if sp else np.zeros_like(t, bool)
print("armed", [(round(a, 1), round(b, 1)) for a, b in arm_sp]); print("airborne", [(round(a, 1), round(b, 1)) for a, b in air_sp])
print("nav_state per armed span:", [sorted(set(vs["nav_state"][(tv >= a) & (tv <= b)].tolist())) for a, b in arm_sp])

def fifo(name):
    f = D[name][0]; dt = np.median(f["dt"]) * 1e-6; n = f["samples"].astype(int)
    xs = [np.stack([f["%s[%d]" % (a, j)] for j in range(32)], 1) for a in "xyz"]
    ts = T(f, "timestamp_sample")
    tt, vv = [], []
    for i in range(len(ts)):
        k = n[i]
        tt.append(ts[i] - (k - 1 - np.arange(k)) * dt)
        vv.append(np.stack([x[i, :k] for x in xs], 1) * f["scale"][i])
    return 1 / dt, np.concatenate(tt), np.concatenate(vv)

fs_g, tg, g = fifo("sensor_gyro_fifo")
fs_a, ta_, a = fifo("sensor_accel_fifo")
print("gyro fifo %.0f Hz %d samples; accel fifo %.0f Hz %d samples (%.1f s worth)" % (fs_g, len(tg), fs_a, len(ta_), len(ta_) / fs_a))

def psd_in(t, x, fs, sp, nper):
    """Welch over contiguous runs inside spans."""
    acc, n = 0, 0
    gaps = np.flatnonzero(np.diff(t) > 3 / fs)
    bounds = np.r_[0, gaps + 1, len(t)]
    for i0, i1 in zip(bounds[:-1], bounds[1:]):
        m = inside(t[i0:i1], sp)
        if m.sum() < nper: continue
        seg = x[i0:i1][m]
        f, P = signal.welch(seg, fs, nperseg=nper, axis=0, detrend="linear")
        w = len(seg) // nper
        acc = acc + P * w; n += w
    return (f, acc / n, n) if n else (None, None, 0)

ground_arm = [(a_, b_) for a_, b_ in arm_sp]  # refined below
f, Pg_air, n1 = psd_in(tg, g, fs_g, air_sp, int(fs_g * 2))
land_arm = [(max(a1, a2), min(b1, b2)) for a1, b1 in arm_sp for a2, b2 in spans_of(tl, landed) if min(b1, b2) > max(a1, a2)]
_, Pg_gnd, n2 = psd_in(tg, g, fs_g, land_arm, int(fs_g * 2))
df = f[1] - f[0]
print("\nGYRO airborne (%d x 2 s) vs armed-on-ground (%d x 2 s), 0.5 Hz bins" % (n1, n2))
for lo, hi in ((0.5, 3), (3, 10), (10, 30), (30, 60), (60, 150), (150, 400), (400, 1500)):
    m = (f >= lo) & (f < hi)
    ra = np.sqrt(Pg_air[m].sum(0) * df); rg = np.sqrt(Pg_gnd[m].sum(0) * df) if n2 else [np.nan] * 3
    print("  %5.1f-%6.1f Hz  air rms deg/s x,y,z %s   ground %s" % (lo, hi, np.round(np.degrees(ra), 1), np.round(np.degrees(rg), 1)))
m = (f > 3) & (f < 60)
Ps = Pg_air[:, :2].sum(1)
pk, _ = signal.find_peaks(Ps[m], prominence=Ps[m].max() * 0.05)
print("  airborne gyro peaks 3-60 Hz:", [(round(float(f[m][i]), 1), round(float(np.degrees(np.sqrt(Ps[m][i] * df))), 1)) for i in pk[np.argsort(Ps[m][pk])[::-1][:10]]])
m = (f > 60) & (f < 1500)
pk, _ = signal.find_peaks(Ps[m], prominence=Ps[m].max() * 0.1)
print("  airborne gyro peaks 60-1500 Hz:", sorted([(round(float(f[m][i]), 1), round(float(np.degrees(np.sqrt(Ps[m][i] * df))), 2)) for i in pk[np.argsort(Ps[m][pk])[::-1][:8]]]))

# accel: short contiguous windows
fa, Pa_air, n3 = psd_in(ta_, a, fs_a, air_sp, 1024)
if n3:
    dfa = fa[1] - fa[0]
    print("\nACCEL airborne (%d x %.2f s windows, %.1f Hz bins)" % (n3, 1024 / fs_a, dfa))
    for lo, hi in ((0, 30), (30, 100), (100, 300), (300, 1000), (1000, 4000)):
        mm = (fa >= lo) & (fa < hi)
        print("  %4d-%4d Hz rms m/s2 x,y,z %s" % (lo, hi, np.round(np.sqrt(Pa_air[mm].sum(0) * dfa), 2)))
    Ps = Pa_air.sum(1); mm = fa > 30
    pk, _ = signal.find_peaks(Ps[mm], prominence=Ps[mm].max() * 0.1)
    print("  accel peaks >30 Hz:", sorted([(round(float(fa[mm][i])), round(float(np.sqrt(Ps[mm][i] * dfa)), 2)) for i in pk[np.argsort(Ps[mm][pk])[::-1][:8]]]))
    print("  raw accel |a| max airborne %.1f m/s2" % np.abs(a[inside(ta_, air_sp)]).max())

# rate tracking (55 Hz logged)
av = D["vehicle_angular_velocity"][0]; rs = D["vehicle_rates_setpoint"][0]
tav, trs = T(av), T(rs)
for ax, sk in ((0, "roll"), (1, "pitch"), (2, "yaw")):
    act = av["xyz[%d]" % ax]; sp = np.interp(tav, trs, rs[sk])
    m = inside(tav, air_sp)
    print("rate %-5s airborne: actual rms %5.1f deg/s, setpoint rms %5.1f, error rms %5.1f, max |actual| %5.0f" % (
        sk, np.degrees(np.std(act[m])), np.degrees(np.std(sp[m])), np.degrees(np.sqrt(np.mean((act - sp)[m] ** 2))), np.degrees(np.abs(act[m]).max())))

# allocator saturation
cas = D["control_allocator_status"][0]; tc = T(cas); m = inside(tc, air_sp)
print("allocator airborne: torque_setpoint_achieved %.0f%% of samples; unallocated torque rms %s" % (
    100 * cas["torque_setpoint_achieved"][m].mean(), np.round([np.sqrt(np.mean(cas["unallocated_torque[%d]" % i][m] ** 2)) for i in range(3)], 3)))
sat = np.stack([cas["actuator_saturation[%d]" % i] for i in range(9)], 1)[m]
print("  per-motor saturation airborne (+1 upper/-1 lower) share upper:", np.round((sat > 0).mean(0), 2), "lower:", np.round((sat < 0).mean(0), 2))

am = D["actuator_motors"][0]; tam = T(am); mot = np.stack([am["control[%d]" % i] for i in range(9)], 1)
mm = inside(tam, air_sp)
print("motor cmd airborne mean", np.round(np.nanmean(mot[mm], 0), 2), "share >=0.99", np.round(np.mean(mot[mm] >= 0.99, 0), 2))

# clipping timing
for k, s in D["vehicle_imu_status"].items():
    ts = T(s); c = sum(s["accel_clipping[%d]" % j].astype(float) for j in range(3))
    inc = np.flatnonzero(np.diff(c) > 0)
    print("imu%d clipping increments at t =" % k, [(round(ts[i + 1], 1), int(np.diff(c)[i]), "air" if inside(np.array([ts[i + 1]]), air_sp)[0] else "gnd") for i in inc][:20])

bs = D["battery_status"][0]; tb = T(bs); mb = inside(tb, arm_sp)
print("battery while armed: V %.1f-%.1f, I max %.0f A" % (bs["voltage_v"][mb].min(), bs["voltage_v"][mb].max(), bs["current_a"][mb].max()))

# figure
lp = D["vehicle_local_position"][0]; tlp = T(lp)
fig, axs = plt.subplots(len(arm_sp), 1, figsize=(13, 4.2 * len(arm_sp)))
for row, (a0, b0) in enumerate(arm_sp):
    ax = axs[row]; lo, hi = a0 - 1, b0 + 1
    m = (tg >= lo) & (tg <= hi)
    ax.plot(tg[m], np.degrees(g[m, 0]), lw=.4, label="gyro x (roll) deg/s")
    ax.plot(tg[m], np.degrees(g[m, 1]), lw=.4, label="gyro y (pitch) deg/s")
    m2 = (trs >= lo) & (trs <= hi)
    ax.plot(trs[m2], np.degrees(rs["roll"][m2]), "k--", lw=.8, label="roll rate setpoint")
    for a1, b1 in air_sp:
        if b1 > lo and a1 < hi: ax.axvspan(max(a1, lo), min(b1, hi), color="tab:green", alpha=.08)
    ax.set_ylabel("deg/s"); ax.set_xlim(lo, hi); ax.set_title("armed span %d (green = airborne)" % (row + 1))
    ax2 = ax.twinx(); m3 = (tam >= lo) & (tam <= hi)
    for i in range(9): ax2.plot(tam[m3], mot[m3, i], lw=.8, alpha=.6)
    ax2.set_ylim(-.05, 1.6); ax2.set_ylabel("motor cmd (9 lines)")
    if row == 0: ax.legend(loc="upper left", fontsize=8)
plt.tight_layout(); plt.savefig(out + "_timeline.png", dpi=110)

fig, ax = plt.subplots(1, 2, figsize=(13, 4))
ax[0].semilogy(f, np.degrees(np.sqrt(Pg_air[:, 0])), lw=.8, label="roll, airborne")
ax[0].semilogy(f, np.degrees(np.sqrt(Pg_air[:, 1])), lw=.8, label="pitch, airborne")
if n2: ax[0].semilogy(f, np.degrees(np.sqrt(Pg_gnd[:, 1])), lw=.8, label="pitch, armed on ground")
ax[0].set_xlim(0, 80); ax[0].set_xlabel("Hz"); ax[0].set_ylabel("deg/s/sqrt(Hz)"); ax[0].legend(); ax[0].grid(alpha=.3)
ax[0].set_title("Gyro spectrum, 0-80 Hz")
ax[1].semilogy(f, np.degrees(np.sqrt(Pg_air[:, :2].sum(1))), lw=.6); ax[1].set_xlim(0, 1500); ax[1].set_xlabel("Hz")
ax[1].set_title("Gyro spectrum airborne, 0-1500 Hz"); ax[1].grid(alpha=.3)
plt.tight_layout(); plt.savefig(out + "_spectrum.png", dpi=110)
print("figures written")
