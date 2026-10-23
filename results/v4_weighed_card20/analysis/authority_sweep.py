"""V4 with the fixed hover-20 CA_ROTOR card: per-axis authority vs hover pitch (SENS_BOARD_Y_OFF) and park angle.

Physics: the app's airframe (rotor positions, jet axes, km, max thrust, mass, CG, inertia). PX4: the card's
effectiveness (rows Mx, My, Mz, Fz) and its pseudo-inverse, as PX4's allocator uses it. Thrust fraction f in [0, 1]
per motor (THR_MDL_FAC = 1, thrust ~ command^2, so PX4's allocation is linear in f).

Hover: the aircraft settles where PX4's allocation zeroes the true torque and the true force balances gravity; that
attitude does not depend on SENS_BOARD_Y_OFF (PX4 allocates with the card whatever frame it measures in). The hover
pitch only sets (a) the lean PX4 holds = natural pitch - hover pitch and (b) the angle between PX4's roll/yaw axes
and the card's, which mixes roll into yaw.

Authority per axis: PX4 asks for torque on one of its axes; the card turns that into motor changes; the true
angular acceleration is measured in PX4's frame. Usable range = how far from the hover trim before any motor hits 0
or 1, each direction. Park: on the rear feet, the most nose-up moment about the feet PX4 can produce with pitch and
thrust requests only, against gravity (> 0 means it can rotate off the stand).
"""
import json, math, sys
import numpy as np
sys.path.insert(0, sys.argv[1])
from airframe_designer.geometry.airframe import Airframe

G = 9.80665
B = "/Users/stefanosfragkoulis/AIRFRAME_REPRO/v4_weighed_card20"
af = Airframe.load(f"{B}/airframe_V4_fusion_v8_full.json")
mp = af.mass; m = mp.mass; cg = np.array(mp.cg)
I = np.array(mp.tensor() if hasattr(mp, "tensor") else np.diag(mp.inertia))
P = json.load(open(f"{B}/px4_params_V3_with_V4card20.json"))
n = len(af.rotors)

def ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

# true effectiveness per unit thrust fraction, body frame, about the CG
Bt = np.zeros((6, n))
for i, r in enumerate(af.rotors):
    a = np.array(r.axis, float); a /= np.linalg.norm(a); rr = np.array(r.pos) - cg; T = r.max_thrust
    Bt[0:3, i] = T * (np.cross(rr, a) - r.km * a); Bt[3:6, i] = T * a
# card effectiveness (PX4 frame of the card) and PX4's allocation matrix
E = np.zeros((4, n))
for i in range(n):
    p = np.array([P[f"CA_ROTOR{i}_P{k}"] for k in "XYZ"]); a = np.array([P[f"CA_ROTOR{i}_A{k}"] for k in "XYZ"])
    ct, km = P[f"CA_ROTOR{i}_CT"], P[f"CA_ROTOR{i}_KM"]
    E[0:3, i] = ct * (np.cross(p, a) - km * a); E[3, i] = ct * a[2]
A = np.linalg.pinv(E)          # f = A @ [Mx, My, Mz, Fz]

def down_body(pitch_deg):
    t = math.radians(pitch_deg); return np.array([-math.sin(t), 0.0, math.cos(t)])

# ---- hover equilibrium (independent of SENS_BOARD_Y_OFF)
best = None
for th in np.arange(-10, 45.001, 0.01):
    rhs = np.concatenate([np.zeros(3), -m * G * down_body(th)])
    M = Bt @ A
    c, *_ = np.linalg.lstsq(M, rhs, rcond=None)
    res = np.linalg.norm(M @ c - rhs)
    if best is None or res < best[0]: best = (res, th, c)
res, th_n, c_tr = best
f_tr = A @ c_tr
print(f"natural hover pitch (body, nose up): {th_n:.2f} deg   residual {res:.3f} N   mass {m:.2f} kg")
print("trim thrust fraction per motor:", np.round(f_tr, 3), " (rear M1-M6, front M7-M9)")
feasible = (f_tr.min() >= 0) and (f_tr.max() <= 1)
print("trim feasible:", feasible)
Ii = np.linalg.inv(I)

def authority(h):
    out = {}
    for j, ax in enumerate(("roll", "pitch", "yaw")):
        e = np.zeros(4); e[j] = 1.0
        df = A @ e
        alpha_h = ry(h) @ (Ii @ (Bt[0:3] @ df))         # true angular accel, in PX4's frame, per unit request
        kp = min([(1 - f_tr[i]) / df[i] if df[i] > 1e-12 else (0 - f_tr[i]) / df[i] if df[i] < -1e-12 else np.inf for i in range(n)])
        km_ = min([(f_tr[i] - 0) / df[i] if df[i] > 1e-12 else (f_tr[i] - 1) / df[i] if df[i] < -1e-12 else np.inf for i in range(n)])
        main = alpha_h[j]
        out[ax] = {"plus": kp * main, "minus": km_ * main, "min": min(abs(kp * main), abs(km_ * main)),
                   "coupling": float(np.linalg.norm(np.delete(alpha_h, j)) / abs(main))}
    return out

print("\nHOVER PITCH sweep (card fixed at 20 deg). Authority = usable angular accel each way, rad/s^2; coupling = off-axis / on-axis")
print(f"{'hover':>6} {'lean':>6} | {'roll -/+':>15} {'cpl':>5} | {'pitch -/+':>15} {'cpl':>5} | {'yaw -/+':>15} {'cpl':>5} | {'min axis':>8}")
rows = []
for h in np.arange(10, 30.01, 1.0):
    a = authority(h); lean = th_n - h
    rows.append((h, lean, a))
    print(f"{h:6.1f} {lean:6.2f} | {a['roll']['minus']:7.2f}/{a['roll']['plus']:<7.2f} {a['roll']['coupling']:5.2f} | "
          f"{a['pitch']['minus']:7.2f}/{a['pitch']['plus']:<7.2f} {a['pitch']['coupling']:5.2f} | {a['yaw']['minus']:7.2f}/{a['yaw']['plus']:<7.2f} {a['yaw']['coupling']:5.2f} | "
          f"{min(a[x]['min'] for x in a):8.2f}")

# ---- park: rotation off the rear feet
rear = [np.array(l.foot()) for l in af.legs if l.name in ("RL", "RR")]
piv = np.mean(rear, axis=0)
Iyy_piv = I[1, 1] + m * ((cg - piv)[0] ** 2 + (cg - piv)[2] ** 2)
Mpiv = np.zeros(n)
for i, r in enumerate(af.rotors):
    a = np.array(r.axis, float); a /= np.linalg.norm(a)
    Mpiv[i] = r.max_thrust * (np.cross(np.array(r.pos) - piv, a) - r.km * a)[1]
print(f"\nPARK sweep. Pivot = rear feet at x {piv[0]:.3f} z {piv[2]:.3f}; CG {cg[0]-piv[0]:+.3f} m ahead of it.")
print("max nose-up moment PX4 can make with pitch + thrust requests (roll/yaw zero), net of gravity, and as rotation accel")
grid = [(ty, T) for ty in np.linspace(-20, 20, 401) for T in np.linspace(-30, 0, 301)]
cols = np.array([A @ np.array([0, ty, 0, T]) for ty, T in grid])
ok = np.all((cols >= -1e-9) & (cols <= 1 + 1e-9), axis=1)
mom_thr = cols[ok] @ Mpiv
best_thr = mom_thr.max(); f_best = cols[ok][mom_thr.argmax()]
for park in np.arange(-25, 20.01, 5.0):
    grav = (m * G * np.cross(cg - piv, down_body(park)))[1]
    net = best_thr + grav
    L = 0.42 + 0.968 * math.tan(math.radians(park))
    print(f"park {park:6.1f} deg: front leg {L:6.3f} m | gravity {grav:7.1f} N m, best thrust {best_thr:6.1f} N m -> net {net:7.1f} N m "
          f"= {net / Iyy_piv:6.2f} rad/s^2 | PX4 starts {park - 20:+.0f} deg off level")
print("motor fractions for that best moment:", np.round(f_best, 2))
json.dump({"natural_hover_pitch": th_n, "trim": f_tr.tolist(), "hover_rows": [(h, l, a) for h, l, a in rows]},
          open(f"{B}/analysis/authority_sweep.json", "w"), indent=1, default=float)
