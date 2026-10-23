"""Per-axis wobble in a saved run: amplitude and dominant frequency for take-off and hover windows."""
import json, sys, numpy as np
import urllib.request
src = sys.argv[1]
d = json.load(urllib.request.urlopen("http://127.0.0.1:8087/api/runs/current", timeout=60)) if src == "current" else json.load(open(src))
c = d["columns"]; i = {k: n for n, k in enumerate(c)}
R = np.array(d["rows"], float); t = R[:, 0] - R[0, 0]
arm = R[:, i["armed"]] > 0; ta = t[arm][0]
h0 = R[arm][0, i["height"]]
lift = t[(t > ta) & (R[:, i["height"]] > h0 + 0.1)][0]
hmax = R[t > ta, i["height"]].max()
reach = t[(t > lift) & (R[:, i["height"]] > h0 + 0.9 * (hmax - h0))][0]
def stats(col, a, b):
    m = (t >= a) & (t < b); x = R[m, i[col]]; x = x - np.polyval(np.polyfit(t[m], x, 1), t[m])
    if len(x) < 8: return None
    f = np.fft.rfftfreq(len(x), d=np.median(np.diff(t[m]))); P = np.abs(np.fft.rfft(x)); k = 1 + np.argmax(P[1:])
    return round(float(np.std(x) * 2 ** 0.5), 2), round(float(f[k]), 2)
print(f"armed {ta:.1f}s, lift-off {lift:.1f}s, reached {hmax - h0 + h0:.2f} m at {reach:.1f}s, end {t[-1]:.1f}s")
for name, a, b in (("take-off", lift, reach + 1), ("hover", reach + 1, t[-1])):
    print(f"[{name} {a:.1f}-{b:.1f}s]")
    for col, unit in (("roll", "deg"), ("pitch", "deg"), ("yaw", "deg"), ("p", "deg/s"), ("q", "deg/s"), ("r", "deg/s"), ("height", "m"), ("vd", "m/s")):
        s = stats(col, a, b); print(f"   {col:6} amplitude {s[0]:7.2f} {unit:5}  dominant {s[1]:5.2f} Hz" if s else f"   {col}: too short")
    cm = [stats(f"m{n}_cmd", a, b) for n in range(1, 10)]
    print("   motor cmd amplitude", [x[0] for x in cm], " max cmd", round(float(max(R[(t >= a) & (t < b), i[f'm{n}_cmd']].max() for n in range(1, 10))), 2))
