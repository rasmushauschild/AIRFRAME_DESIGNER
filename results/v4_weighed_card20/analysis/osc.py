"""Oscillation after take-off in the current recording (port 8087): period, amplitude by signal, decay."""
import json, sys, urllib.request, numpy as np
d = json.load(urllib.request.urlopen("http://127.0.0.1:8087/api/runs/current", timeout=60)); c = d["columns"]; i = {k: n for n, k in enumerate(c)}
R = np.array([r for r in d["rows"] if r[i["armed"]]], float)
t = R[:, i["t"]]; t = t - t[0]
air = t > 8                                         # after lift-off and the climb transient
def amp(col, a, b):
    m = (t >= a) & (t < b); x = R[m, i[col]]; return (x.max() - x.min()) / 2 if m.any() else np.nan
sig = ["r", "yaw", "p", "q", "roll", "pitch"] + [f"m{n}_cmd" for n in range(1, 10)]
x = R[air, i["r"]] - R[air, i["r"]].mean(); ta = t[air]
f = np.fft.rfftfreq(len(x), d=np.median(np.diff(ta))); P = np.abs(np.fft.rfft(x)); k = 1 + np.argmax(P[1:])
print(f"{sys.argv[1] if len(sys.argv)>1 else ''}: {t[-1]:.0f} s armed; dominant yaw-rate period {1/f[k]:.1f} s")
for a, b in ((8, 30), (30, 60), (60, 90), (90, 120), (120, 160)):
    print(f"  {a:3d}-{b:3d}s  half-amplitude: yaw rate {amp('r', a, b):5.1f} deg/s, yaw {amp('yaw', a, b):5.1f} deg, roll {amp('roll', a, b):4.2f}, pitch {amp('pitch', a, b):4.2f}, "
          f"motors {', '.join(f'{amp(f'm{n}_cmd', a, b):.2f}' for n in range(1, 10))}")
