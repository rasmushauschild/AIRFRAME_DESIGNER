"""Which part of the yaw torque is wrong? Fit Izz * yaw acceleration against the pieces of the yaw torque, powered flight.

Torques in N*m from the motor commands, using the DESIGN's real geometry (front pair tilted 30 deg sideways),
the board's positions (relative to the CG, hover frame) and thrust = command * 29.13 N:
  lever_rear : rear six fans' thrust x lever arm (the tilted pair M5/M6 mostly)
  lever_front: front pair M7/M8 sideways thrust x lever arm (what the board models with KM 0.27 / -0.22)
  react      : the fans' own spin torque if all nine spin as declared (km = +0.0134 each, PX4's sign)
  px4        : the yaw torque PX4 believed it was making, from the board's own effectiveness (CT, KM, axes)
"""
import sys, glob, json
import numpy as np
from pyulog import ULog
from scipy import signal

IZZ, TMAX = 1.73274, 29.13
R = sys.argv[1]
rows = []
for n in (409, 413, 414, 423, 424):
    p = glob.glob("%s/log_%d_*.ulg" % (R, n))[0]; res = json.load(open("res/%d.json" % n))
    A, B = res["takeoff"] - 0.3, res["kill"] - 0.3
    u = ULog(p, ["actuator_motors", "sensor_combined", "vehicle_torque_setpoint"]); t0 = u.start_timestamp; P = u.initial_parameters
    D = {d.name: d.data for d in u.data_list}
    am = D["actuator_motors"]; ta = (am["timestamp"].astype(float) - t0) / 1e6
    U = np.nan_to_num(np.stack([am["control[%d]" % i] for i in range(9)], 1)).clip(0, 1)
    pos = np.array([[P["CA_ROTOR%d_P%s" % (i, c)] for c in "XYZ"] for i in range(9)])
    ax_b = np.array([[P["CA_ROTOR%d_A%s" % (i, c)] for c in "XYZ"] for i in range(9)])
    ax_b /= np.linalg.norm(ax_b, axis=1)[:, None]
    ax_d = ax_b.copy()
    # design: front pair M7/M8 tilted sideways 30 deg (design axis y = +0.5 / -0.5), in the hover frame
    for i, s in ((6, 1), (7, -1)):
        ax_d[i] = [ax_b[i, 0] * 0.866 / np.hypot(ax_b[i, 0], ax_b[i, 2]) * 1.0, s * 0.5, ax_b[i, 2] * 0.866 / np.hypot(ax_b[i, 0], ax_b[i, 2])]
        ax_d[i] /= np.linalg.norm(ax_d[i])
    lev = np.cross(pos, ax_d)[:, 2]               # yaw lever per newton of thrust
    km = np.array([P["CA_ROTOR%d_KM" % i] for i in range(9)])
    ct = np.array([P["CA_ROTOR%d_CT" % i] for i in range(9)])
    T = U * TMAX
    lever_rear = (T[:, :6] * lev[:6]).sum(1); lever_front = (T[:, 6:8] * lev[6:8]).sum(1); lever_m9 = T[:, 8] * lev[8]
    react = (T * 0.0134 * (-ax_d[:, 2])).sum(1)
    px4 = (U * ct * (np.cross(pos, ax_b)[:, 2] - km * ax_b[:, 2])).sum(1)  # PX4's own units (normalised by CT)
    sc = D["sensor_combined"]; ts = (sc["timestamp"].astype(float) - t0) / 1e6
    b, a = signal.butter(2, 4, fs=269); r = signal.filtfilt(b, a, sc["gyro_rad[2]"])
    rdot = np.gradient(r, ts)
    tq = np.arange(A, B, 0.02)
    f = lambda t, x: np.interp(tq, t, x)
    lp = lambda x: signal.filtfilt(b, a, x)
    rows.append(np.column_stack([f(ts, rdot) * IZZ, f(ta, lp(lever_rear)), f(ta, lp(lever_front)), f(ta, lp(lever_m9)), f(ta, lp(react)),
                                 f(ts, r), f(ta, lp(px4)), np.full(len(tq), n), tq - res["takeoff"]]))
X = np.vstack(rows)
y = X[:, 0]
names = ["lever_rear", "lever_front", "react", "yaw_rate_damping", "constant"]
M = np.column_stack([X[:, 1], X[:, 2], X[:, 4], X[:, 5], np.ones(len(y))])
c, *_ = np.linalg.lstsq(M, y, rcond=None)
pred = M @ c; r2 = 1 - ((y - pred) ** 2).sum() / ((y - y.mean()) ** 2).sum()
print("fit Izz*rdot = sum(c_i * piece_i), all flights, R2 %.2f" % r2)
for nme, v in zip(names, c): print("   %-18s %+.3f" % (nme, v))
print("   (a correct model gives 1.0 for each torque piece)")
# average torque budget in steady hover (2 s after takeoff to kill)
m = X[:, 8] > 2
print("\nmean torques in hover, N*m:  rear levers %+.2f | front pair levers %+.2f | M9 %+.2f | fans' spin torque (as declared) %+.2f | Izz*rdot %+.2f" % (
    X[m, 1].mean(), X[m, 2].mean(), X[m, 3].mean(), X[m, 4].mean(), X[m, 0].mean()))
print("PX4 believed its yaw torque averaged %+.3f (its own units) in hover" % X[m, 6].mean())
# first 3 s after takeoff
m = (X[:, 8] > 0.5) & (X[:, 8] < 3.5)
print("first 3 s after takeoff, N*m:  rear levers %+.2f | front pair levers %+.2f | spin torque %+.2f | Izz*rdot %+.2f" % (
    X[m, 1].mean(), X[m, 2].mean(), X[m, 4].mean(), X[m, 0].mean()))
for sgn, lab in ((1, "as declared"), (-1, "opposite to declared")):
    tot = X[:, 1] + X[:, 2] + X[:, 3] + sgn * X[:, 4]
    resid = y - tot - c[3] * X[:, 5]
    print("if the fans spin %-20s: torque left unexplained in hover %+.2f N*m" % (lab, resid[X[:, 8] > 2].mean()))
