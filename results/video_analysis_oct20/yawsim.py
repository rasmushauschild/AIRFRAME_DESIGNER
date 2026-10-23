"""Yaw loop with the plant measured in flight: Izz*rdot = k*tau + d(t) + c*r. PX4-style rate PI with clamp, optional heading hold."""
import json, numpy as np
pl = json.load(open("yawplant.json")); IZZ = 1.73274; K, C = pl["k"], pl["damp"]
def run(P=0.07, I=0.035, lim=0.30, d=pl["d"], hold_from=8.0, ramp=1.0, delay=0.0, yawP=2.8, w=0.4, T=15, dt=0.002, tau_max=1.0):
    r = yaw = integ = 0.0; out = []; tgt = 0.0
    for i in range(int(T / dt)):
        t = i * dt
        dist = d * np.clip((t - delay) / ramp, 0.0, 1.0)  # torque builds with thrust through the takeoff ramp
        if t < hold_from: tgt = yaw; rsp = 0.0           # takeoff: no heading target, the setpoint follows the nose
        else: rsp = np.clip(yawP * w * (tgt - yaw), -np.radians(200), np.radians(200))
        e = rsp - r
        integ = np.clip(integ + I * e * dt, -lim, lim)
        tau = np.clip(P * e + integ, -tau_max, tau_max)
        r += (K * tau + dist + C * r) / IZZ * dt; yaw += r * dt
        out.append((t, np.degrees(r), np.degrees(yaw), tau, integ))
    return np.array(out)
def summary(o):
    t, r, yaw, tau, integ = o.T
    pk = np.argmax(np.abs(r)); settle = t[np.flatnonzero(np.abs(r) > 10)[-1]] if (np.abs(r) > 10).any() else 0
    return "peak %4.0f deg/s at %4.1f s | turned %5.0f deg in 8 s | |r| below 10 deg/s after %4.1f s | hover drift %+5.1f deg/s | max |tau| %.2f" % (
        abs(r[pk]), t[pk], yaw[np.argmin(abs(t - 8))], settle, r[t > 12].mean(), np.abs(tau).max())
if __name__ == "__main__":
    obs = pl["per"]
    print("observed: peak %s deg/s at %s s; integrator 0.27 at %s s" % (
        [round(v["peak"]) for v in obs.values()], [round(v["t_peak"], 1) for v in obs.values()], [round(v["integ_reach_027"], 1) for v in obs.values()]))
    cases = [("as flown (P 0.07, I 0.035, limit 0.30)", {}),
             ("yaw model fixed, same gains (disturbance -> -0.15 N*m)", dict(d=-0.15)),
             ("integrator limit 0.6, model unchanged", dict(lim=0.6)),
             ("Oct 19 gains P 0.2 / I 0.1, model unchanged", dict(P=0.2, I=0.1)),
             ("P 0.2 / I 0.1 + limit 0.6, model unchanged", dict(P=0.2, I=0.1, lim=0.6)),
             ("heading held from liftoff, model unchanged", dict(hold_from=0.0)),
             ("model fixed + P 0.15 / I 0.07", dict(d=-0.15, P=0.15, I=0.07))]
    for name, kw in cases: print("%-58s %s" % (name, summary(run(**kw))))
