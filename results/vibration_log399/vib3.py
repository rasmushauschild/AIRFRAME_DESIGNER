"""Vibration pass for a default-profile .ulg (no FIFO): sensor_combined + filtered rates + motors at ~250-290 Hz."""
import sys
import numpy as np
from pyulog import ULog
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path, out = sys.argv[1], sys.argv[2]
u = ULog(path)
t0 = float(u.start_timestamp)
D = {}
for d in u.data_list:
    D.setdefault(d.name, {})[d.multi_id] = d.data
T = lambda d: (d["timestamp"].astype(float) - t0) / 1e6
def spans_of(t, m):
    sp, s = [], None
    for i in range(len(t)):
        if m[i] and s is None: s = t[i]
        if not m[i] and s is not None: sp.append((s, t[i])); s = None
    if s is not None: sp.append((s, t[-1]))
    return sp
inside = lambda t, sp: np.any([(t >= a) & (t <= b) for a, b in sp], 0) if sp else np.zeros_like(t, bool)

vs = D["vehicle_status"][0]; tv = T(vs)
arm_sp = spans_of(tv, vs["arming_state"] == 2)
ld = D["vehicle_land_detected"][0]; tl = T(ld)
air_sp = [(max(a, arm_sp[0][0]), b) for a, b in spans_of(tl, ~ld["landed"].astype(bool))]
_tk = [(m.timestamp - t0) / 1e6 for m in u.logged_messages if "Takeoff detected" in m.message]
_kl = [(m.timestamp - t0) / 1e6 for m in u.logged_messages if "Kill engaged" in m.message]
if _tk and _kl: air_sp = [(_tk[0], _kl[0])]  # powered flight only
gnd_arm = [(a, b) for a, b in spans_of(tl, ld["landed"].astype(bool))]
gnd_arm = [(max(a, x), min(b, y)) for a, b in gnd_arm for x, y in arm_sp if min(b, y) > max(a, x)]
print("armed", [(round(a, 1), round(b, 1)) for a, b in arm_sp], "airborne", [(round(a, 1), round(b, 1)) for a, b in air_sp],
      "armed on ground", [(round(a, 1), round(b, 1)) for a, b in gnd_arm])
print("nav_state during armed:", sorted(set(vs["nav_state"][inside(tv, arm_sp)].tolist())))
for m in u.logged_messages:
    print("  msg %.1f %s" % ((m.timestamp - t0) / 1e6, m.message))

sc = D["sensor_combined"][0]; tsc = T(sc)
fs = 1 / np.median(np.diff(tsc))
gy = np.stack([sc["gyro_rad[%d]" % i] for i in range(3)], 1)
ac = np.stack([sc["accelerometer_m_s2[%d]" % i] for i in range(3)], 1)
print("sensor_combined %.0f Hz" % fs)

def psd(t, x, sp, nper):
    m = inside(t, sp)
    if m.sum() < nper: return None, None
    # split at span boundaries
    acc, n = 0, 0
    for a, b in sp:
        mm = (t >= a) & (t <= b)
        if mm.sum() < nper: continue
        f, P = signal.welch(x[mm], fs, nperseg=nper, axis=0, detrend="linear")
        w = mm.sum() / nper; acc = acc + P * w; n += w
    return f, acc / n

nper = int(fs)  # ~1 Hz bins
for lab, x, unit, conv in (("gyro", gy, "deg/s", np.degrees), ("accel", ac, "m/s2", lambda v: v)):
    fa, Pa = psd(tsc, x, air_sp, nper)
    fg, Pg = psd(tsc, x, gnd_arm, min(nper, 128))
    df = fa[1] - fa[0]
    print("\n%s airborne vs armed-on-ground, rms %s x,y,z" % (lab, unit))
    for lo, hi in ((0.3, 3), (3, 10), (10, 30), (30, 60), (60, fs / 2)):
        m = (fa >= lo) & (fa < hi)
        sa = conv(np.sqrt(Pa[m].sum(0) * df))
        if Pg is not None:
            mg = (fg >= lo) & (fg < hi); sg = conv(np.sqrt(Pg[mg].sum(0) * (fg[1] - fg[0])))
        else:
            sg = [np.nan] * 3
        print("  %5.1f-%5.1f Hz  air %s   ground %s" % (lo, hi, np.round(sa, 2), np.round(sg, 2)))
    Ps = Pa[:, :2].sum(1) if lab == "gyro" else Pa.sum(1)
    m = fa > 2
    pk, _ = signal.find_peaks(Ps[m], prominence=Ps[m].max() * 0.05)
    best = pk[np.argsort(Ps[m][pk])[::-1][:8]]
    print("  peaks:", sorted([(round(float(fa[m][i]), 1), round(float(conv(np.sqrt(Ps[m][i] * df))), 2)) for i in best]))
    np.save("%s_%s_psd.npy" % (out, lab), np.column_stack([fa, Pa]))
    if Pg is not None: np.save("%s_%s_psdg.npy" % (out, lab), np.column_stack([fg, Pg]))

_m = inside(tsc, air_sp); _i = np.argmax(np.abs(ac[_m]).max(1))
print("raw accel max |a| powered %.1f m/s2 at t=%.1f, gyro max %.0f deg/s; after kill max |a| %.1f" % (np.abs(ac[_m]).max(), tsc[_m][_i], np.degrees(np.abs(gy[_m]).max()), np.abs(ac[tsc > air_sp[-1][1]]).max()))
for k, s in D["vehicle_imu_status"].items():
    ts = T(s); m = inside(ts, air_sp)
    print("imu%d (1 Hz) acc_vib air mean %.2f max %.2f | gyro_vib mean %.3f max %.3f | clipping acc %s" % (
        k, s["accel_vibration_metric"][m].mean(), s["accel_vibration_metric"][m].max(), s["gyro_vibration_metric"][m].mean(),
        s["gyro_vibration_metric"][m].max(), [int(s["accel_clipping[%d]" % j][-1]) for j in range(3)]))

av = D["vehicle_angular_velocity"][0]; tav = T(av); rs = D["vehicle_rates_setpoint"][0]; trs = T(rs)
for ax, sk in ((0, "roll"), (1, "pitch"), (2, "yaw")):
    act = av["xyz[%d]" % ax]; sp = np.interp(tav, trs, rs[sk]); m = inside(tav, air_sp)
    print("rate %-5s airborne: actual rms %5.1f deg/s, setpoint rms %5.1f, error rms %5.1f, max |actual| %4.0f" % (
        sk, np.degrees(np.std(act[m])), np.degrees(np.std(sp[m])), np.degrees(np.sqrt(np.mean((act - sp)[m] ** 2))), np.degrees(np.abs(act[m]).max())))
    # loop oscillation: spectrum of the error
    fe, Pe = signal.welch((act - sp)[m], 1 / np.median(np.diff(tav)), nperseg=512)
    mm = fe > 1
    print("      error spectrum peak %.1f Hz" % fe[mm][np.argmax(Pe[mm])])

va = D["vehicle_attitude"][0]; tva = T(va)
q = np.stack([va["q[%d]" % i] for i in range(4)], 1)
roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))
pitch = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))
m = inside(tva, air_sp)
print("attitude airborne: roll %.0f..%.0f deg, pitch %.0f..%.0f deg" % (roll[m].min(), roll[m].max(), pitch[m].min(), pitch[m].max()))

am = D["actuator_motors"][0]; tam = T(am); n = int(np.sum([("control[%d]" % i) in am for i in range(12)]))
nm = 9
mot = np.stack([am["control[%d]" % i] for i in range(nm)], 1); m = inside(tam, air_sp)
print("motor cmd airborne mean", np.round(np.nanmean(mot[m], 0), 2), "\n  share >=0.99", np.round(np.mean(mot[m] >= 0.99, 0), 2),
      "\n  share <=0.01", np.round(np.mean(mot[m] <= 0.01, 0), 2))
fsm = 1 / np.median(np.diff(tam))
fm, Pm = signal.welch(np.nan_to_num(mot[m]), fsm, nperseg=512, axis=0)
mm = fm > 1
print("  motor command spectrum peak per motor (Hz):", np.round(fm[mm][np.argmax(Pm[mm], 0)], 1),
      " rms 3-60 Hz:", np.round(np.sqrt(Pm[(fm > 3) & (fm < 60)].sum(0) * (fm[1] - fm[0])), 3))

if "control_allocator_status" in D:
    cas = D["control_allocator_status"][0]; tc = T(cas); m = inside(tc, air_sp)
    print("allocator airborne: torque achieved %.0f%%; unallocated torque rms %s" % (100 * cas["torque_setpoint_achieved"][m].mean(),
          np.round([np.sqrt(np.mean(cas["unallocated_torque[%d]" % i][m] ** 2)) for i in range(3)], 3)))
if "hover_thrust_estimate" in D:
    h = D["hover_thrust_estimate"][0]; th = T(h); m = inside(th, air_sp) & h["valid"].astype(bool)
    if m.any(): print("hover thrust estimate airborne: %.2f (valid samples %d)" % (np.median(h["hover_thrust"][m]), m.sum()))
mc = D["manual_control_setpoint"][0]; tm_ = T(mc); m = inside(tm_, air_sp)
print("sticks airborne: roll rms %.2f pitch rms %.2f yaw rms %.2f throttle %.2f..%.2f" % tuple(
    [np.std(mc[k][m]) for k in ("roll", "pitch", "yaw")] + [mc["throttle"][m].min(), mc["throttle"][m].max()]))

# figure
lo, hi = arm_sp[0][0] - 0.5, arm_sp[-1][1] + 0.5
fig, axs = plt.subplots(4, 1, figsize=(13, 12), sharex=True)
m = (tav >= lo) & (tav <= hi); m2 = (trs >= lo) & (trs <= hi)
for i, (k, c) in enumerate((("roll", "tab:blue"), ("pitch", "tab:orange"))):
    axs[i].plot(tav[m], np.degrees(av["xyz[%d]" % i][m]), c, lw=.7, label="%s rate" % k)
    axs[i].plot(trs[m2], np.degrees(rs[k][m2]), "k--", lw=.8, label="setpoint")
    axs[i].set_ylabel("deg/s"); axs[i].legend(loc="upper left", fontsize=8)
m = (tva >= lo) & (tva <= hi)
axs[2].plot(tva[m], roll[m], label="roll"); axs[2].plot(tva[m], pitch[m], label="pitch"); axs[2].set_ylabel("deg"); axs[2].legend(fontsize=8)
m = (tam >= lo) & (tam <= hi)
for i in range(nm): axs[3].plot(tam[m], mot[m, i], lw=.7, label="m%d" % (i + 1))
axs[3].set_ylabel("motor cmd"); axs[3].legend(ncol=9, fontsize=7, loc="upper left"); axs[3].set_ylim(-.05, 1.25)
for ax in axs:
    for a, b in air_sp: ax.axvspan(a, b, color="tab:green", alpha=.07)
    for mm_ in u.logged_messages:
        if "Kill" in mm_.message or "Takeoff" in mm_.message: ax.axvline((mm_.timestamp - t0) / 1e6, color="r", lw=.6)
axs[3].set_xlabel("s (green = airborne, red = takeoff / kill)")
plt.tight_layout(); plt.savefig(out + "_timeline.png", dpi=105)
print("figure written")
