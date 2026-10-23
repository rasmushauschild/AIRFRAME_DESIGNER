"""One flight, powered phase (takeoff -> kill): vibration, compass, heading resets, yaw. Prints one JSON object.

  flight.py <ulg> [stage]    stage: vib | mag | yaw | all
"""
import sys, json
import numpy as np
from pyulog import ULog
from scipy import signal

p = sys.argv[1]; stage = sys.argv[2] if len(sys.argv) > 2 else "all"
u = ULog(p); t0 = float(u.start_timestamp)
D = {}
for d in u.data_list:
    D.setdefault(d.name, {})[d.multi_id] = d.data
T = lambda d: (d["timestamp"].astype(float) - t0) / 1e6
P = u.initial_parameters
msgs = [((m.timestamp - t0) / 1e6, m.message.strip()) for m in u.logged_messages]
tk = [t for t, m in msgs if "Takeoff detected" in m]
kl = [t for t, m in msgs if "Kill engaged" in m or "Disarmed" in m]
A = tk[0] if tk else 0.0
B = min([t for t in kl if t > A], default=(u.last_timestamp - t0) / 1e6)
win = lambda t: (t >= A) & (t <= B)
out = {"log": p.split("/")[-1][:7], "takeoff": round(A, 2), "kill": round(B, 2), "powered_s": round(B - A, 1),
       "params": {k: P.get(k) for k in ("EKF2_MAG_TYPE", "MC_PITCHRATE_P", "MC_ROLLRATE_P", "MC_YAWRATE_P", "IMU_GYRO_CUTOFF",
                                         "IMU_DGYRO_CUTOFF", "MC_YAWRATE_MAX", "SYS_HITL", "EKF2_HGT_REF")}}

sc = D["sensor_combined"][0]; tsc = T(sc); fs = 1 / np.median(np.diff(tsc))
g = np.stack([sc["gyro_rad[%d]" % i] for i in range(3)], 1)
ac = np.stack([sc["accelerometer_m_s2[%d]" % i] for i in range(3)], 1)
m = win(tsc)

if stage in ("vib", "all"):
    f, Pg = signal.welch(g[m], fs, nperseg=256, axis=0, detrend="linear")
    f, Pa = signal.welch(ac[m], fs, nperseg=256, axis=0, detrend="linear")
    df = f[1] - f[0]; b = (f >= 10) & (f <= 40); ba = (f >= 10) & (f <= 60)
    out["vib_gyro_dps"] = round(float(np.degrees(np.sqrt(Pg[b, :2].sum() * df))), 2)
    out["vib_accel_ms2"] = round(float(np.sqrt(Pa[ba].sum() * df)), 2)
    pb = (f >= 10) & (f <= 60)
    out["osc_hz"] = round(float(f[pb][np.argmax(Pg[pb, :2].sum(1))]), 1)
    # worst 0.5 s window, and when
    worst, at = 0, None
    for s in np.arange(A, B - 0.5, 0.25):
        mm = (tsc >= s) & (tsc < s + .5)
        if mm.sum() < 64: continue
        ff, PP = signal.welch(g[mm, :2], fs, nperseg=64, axis=0)
        v = np.degrees(np.sqrt(PP[(ff >= 10) & (ff <= 40)].sum() * (ff[1] - ff[0])))
        if v > worst: worst, at = v, s
    out["vib_burst_dps"] = round(float(worst), 1); out["vib_burst_at"] = round(float(at - A), 1) if at is not None else None
    out["accel_max_ms2"] = round(float(np.linalg.norm(ac[m], axis=1).max()), 1)
    st = D["vehicle_imu_status"][0]; ts = T(st)
    out["accel_clip"] = int(sum(st["accel_clipping[%d]" % j][win(ts)].max() - st["accel_clipping[%d]" % j][ts <= A].max() if (ts <= A).any() else 0 for j in range(3)))

va = D["vehicle_attitude"][0]; tva = T(va)
q = np.stack([va["q[%d]" % i] for i in range(4)], 1)
yaw = np.unwrap(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))
roll_ = np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
pitch_ = np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1))

if stage in ("mag", "all"):
    rc = va["quat_reset_counter"]; resets = []
    for i in np.flatnonzero(np.diff(rc)):
        if A - 1 <= tva[i + 1] <= B:
            dq = [va["delta_q_reset[%d]" % j][i + 1] for j in range(4)]
            dy = np.degrees(np.arctan2(2 * (dq[0] * dq[3] + dq[1] * dq[2]), 1 - 2 * (dq[2] ** 2 + dq[3] ** 2)))
            resets.append({"t": round(float(tva[i + 1] - A), 2), "dyaw_deg": round(float(dy), 1)})
    out["resets"] = len(resets); out["reset_list"] = resets
    f = D.get("estimator_status_flags", {}).get(0)
    if f is not None:
        tf = T(f); k = f["cs_mag_aligned_in_flight"]
        ch = np.flatnonzero(np.diff(k.astype(int)) > 0)
        out["mag_inflight_align_at"] = [round(float(tf[i + 1] - A), 2) for i in ch]
    vm = D["vehicle_magnetometer"][0]; tvm = T(vm)
    mag = np.stack([vm["magnetometer_ga[%d]" % k] for k in range(3)], 1); nrm = np.linalg.norm(mag, axis=1)
    am = D["actuator_motors"][0]; tam = T(am)
    mot = np.nan_to_num(np.stack([am["control[%d]" % k] for k in range(9)], 1))
    tot = np.interp(tvm, tam, mot.sum(1))
    rest = nrm[(tvm < A) & (tot < 0.2)]
    rest = rest if len(rest) else nrm[tvm < A]
    B0 = float(np.median(rest)) if len(rest) else float("nan")
    pw = win(tvm)
    out["mag_rest_G"] = round(B0, 3); out["mag_powered_median_G"] = round(float(np.median(nrm[pw])), 3)
    out["mag_drop_pct"] = round(100 * (1 - np.median(nrm[pw]) / B0), 0)
    out["mag_corr_motor"] = round(float(np.corrcoef(nrm[pw], tot[pw])[0, 1]), 2)
    r_ = np.interp(tvm, tva, roll_); p_ = np.interp(tvm, tva, pitch_)
    mx, my, mz = mag.T
    Xh = mx * np.cos(p_) + my * np.sin(r_) * np.sin(p_) + mz * np.cos(r_) * np.sin(p_)
    Yh = my * np.cos(r_) - mz * np.sin(r_)
    mh = np.unwrap(np.arctan2(-Yh, Xh))
    # gyro-tracked heading: integrate body z rate (approx earth yaw rate near level), anchored at takeoff
    gz = np.interp(tvm, tsc, np.cumsum(g[:, 2]) / fs)
    gyro_h = gz - np.interp(A, tvm, gz) + np.interp(A, tvm, mh)
    err = np.degrees((mh - gyro_h + np.pi) % (2 * np.pi) - np.pi)
    out["mag_err_deg"] = round(float(np.abs(err[pw]).max()), 0)
    out["mag_err_at_s"] = round(float(tvm[pw][np.argmax(np.abs(err[pw]))] - A), 1)
    # vibration vs compass error over time (0.5 s windows): is the error bigger when it shakes?
    rows = []
    for s in np.arange(A, B - 0.5, 0.5):
        mm = (tsc >= s) & (tsc < s + .5); mv = (tvm >= s) & (tvm < s + .5)
        if mm.sum() < 64 or not mv.any(): continue
        ff, PP = signal.welch(g[mm, :2], fs, nperseg=64, axis=0)
        v = np.degrees(np.sqrt(PP[(ff >= 10) & (ff <= 40)].sum() * (ff[1] - ff[0])))
        rows.append((v, np.abs(err[mv]).mean(), tot[mv].mean(), nrm[mv].mean()))
    rows = np.array(rows)
    if len(rows) > 4:
        out["corr_err_vs_vib"] = round(float(np.corrcoef(rows[:, 0], rows[:, 1])[0, 1]), 2)
        out["corr_field_vs_motor"] = round(float(np.corrcoef(rows[:, 2], rows[:, 3])[0, 1]), 2)
        out["corr_field_vs_vib"] = round(float(np.corrcoef(rows[:, 0], rows[:, 3])[0, 1]), 2)

if stage in ("yaw", "all"):
    am = D["actuator_motors"][0]; tam = T(am); mm = win(tam)
    mot = np.nan_to_num(np.stack([am["control[%d]" % k] for k in range(9)], 1))
    ax4 = P.get("CA_ROTOR4_AX"); ax5 = P.get("CA_ROTOR5_AX")
    out["tilted_pair"] = "m5/m6" if (ax4 and ax4 > 0.3 and ax5 and ax5 > 0.3) else "check geometry (AX4=%s AX5=%s)" % (ax4, ax5)
    sat = (mot[mm, 4] >= 0.99) | (mot[mm, 4] <= 0.01) | (mot[mm, 5] >= 0.99) | (mot[mm, 5] <= 0.01)
    out["yaw_sat_pct"] = round(100 * float(sat.mean()), 0)
    out["m5_full_pct"] = round(100 * float((mot[mm, 4] >= 0.99).mean()), 0)
    out["m6_zero_pct"] = round(100 * float((mot[mm, 5] <= 0.01).mean()), 0)
    av = D["vehicle_angular_velocity"][0]; tav = T(av); rs = D["vehicle_rates_setpoint"][0]; trs = T(rs)
    w = win(tav); r = np.degrees(av["xyz[2]"][w]); sp = np.degrees(np.interp(tav[w], trs, rs["yaw"]))
    out["yaw_rate_mean_dps"] = round(float(r.mean()), 1); out["yaw_rate_max_dps"] = round(float(np.abs(r).max()), 0)
    out["yaw_sp_rms_dps"] = round(float(np.sqrt((sp ** 2).mean())), 0)
    out["yaw_turned_deg"] = round(float(np.degrees(np.trapezoid(g[m, 2], tsc[m]))), 0)
    cas = D.get("control_allocator_status", {}).get(0)
    if cas is not None:
        tc = T(cas); wc = win(tc)
        out["torque_achieved_pct"] = round(100 * float(cas["torque_setpoint_achieved"][wc].mean()), 0)
print(json.dumps(out, default=lambda o: round(float(o), 4)))
