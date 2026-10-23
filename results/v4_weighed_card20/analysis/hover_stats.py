import json, sys, urllib.request, statistics as st, math
d = json.load(urllib.request.urlopen("http://127.0.0.1:8087/api/runs/current", timeout=60)); c = d["columns"]; i = {k: n for n, k in enumerate(c)}
rows = [r for r in d["rows"] if r[i["armed"]]]
t_end = rows[-1][0]; hv = [r for r in rows if r[0] >= t_end - 20]
g = lambda k: [r[i[k]] for r in hv]
n0, e0 = hv[0][i["n"]], hv[0][i["e"]]
drift = max(math.hypot(r[i["n"]] - n0, r[i["e"]] - e0) for r in hv)
out = {"label": sys.argv[1], "window_s": round(hv[-1][0] - hv[0][0], 1), "height_mean": round(st.mean(g("height")), 2), "height_sd": round(st.pstdev(g("height")), 3),
       "pitch_mean": round(st.mean(g("pitch")), 2), "pitch_sd": round(st.pstdev(g("pitch")), 2), "roll_mean": round(st.mean(g("roll")), 2), "roll_sd": round(st.pstdev(g("roll")), 2),
       "yaw_drift": round(g("yaw")[-1] - g("yaw")[0], 1), "pos_drift_m": round(drift, 2), "power_W": round(st.mean(g("power"))),
       "cmd_mean": [round(st.mean(g(f"m{n}_cmd")), 2) for n in range(1, 10)]}
print(json.dumps(out)); open("/Users/stefanosfragkoulis/AIRFRAME_REPRO/v4_weighed_card20/analysis/sitl_hover.jsonl", "a").write(json.dumps(out) + "\n")
