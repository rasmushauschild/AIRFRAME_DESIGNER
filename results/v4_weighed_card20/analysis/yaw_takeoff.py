"""Scripted takeoff on port 8087; yaw rate through the climb and the first 10 s of hover."""
import json, sys, time, urllib.request, statistics as st
U = "http://127.0.0.1:8087"
def get(p): return json.load(urllib.request.urlopen(U + p, timeout=60))
def post(p, d): return json.load(urllib.request.urlopen(urllib.request.Request(U + p, json.dumps(d).encode(), {"Content-Type": "application/json"}), timeout=60))
label = sys.argv[1]
fly = len(sys.argv) < 3 or sys.argv[2] != "nofly"
t0 = 0.0
if fly:
    time.sleep(20)
    t0 = get("/api/runs/current")["rows"][-1][0]
    post("/api/px4/command", {"command": "takeoff"}); time.sleep(2)
    for _ in range(6):
        post("/api/px4/command", {"command": "arm"}); time.sleep(2)
        if get("/api/status").get("armed"): break
    post("/api/px4/command", {"command": "takeoff"}); time.sleep(35)
d = get("/api/runs/current"); c = d["columns"]; i = {k: n for n, k in enumerate(c)}
rows = [r for r in d["rows"] if r[0] >= t0 and r[i["armed"]]]
h0 = rows[0][i["height"]]; hmax = max(r[i["height"]] for r in rows)
lift = next(k for k, r in enumerate(rows) if r[i["height"]] > h0 + 0.1)
top = next(k for k, r in enumerate(rows) if r[i["height"]] > h0 + 0.9 * (hmax - h0))
climb, hover = rows[lift:top + 1], [r for r in rows[top:] if r[0] <= rows[top][0] + 10]
yr = lambda s: [r[i["r"]] for r in s]                       # deg/s already
yaw_turn = lambda s: s[-1][i["yaw"]] - s[0][i["yaw"]]
mot = max(max(r[i[f"m{n}_cmd"]] for n in range(1, 10)) for r in climb)
out = {"label": label, "climb_s": round(climb[-1][0] - climb[0][0], 1), "h_max": round(hmax, 2),
       "climb_yawrate_mean": round(st.mean(yr(climb)), 1), "climb_yawrate_absmax": round(max(abs(x) for x in yr(climb)), 1),
       "climb_yaw_turned_deg": round(yaw_turn(climb), 1), "hover_yawrate_absmax": round(max(abs(x) for x in yr(hover)), 1),
       "climb_motor_max": round(mot, 2)}
print(json.dumps(out)); open("/Users/stefanosfragkoulis/AIRFRAME_REPRO/v4_weighed_card20/analysis/yaw_runs.jsonl", "a").write(json.dumps(out) + "\n")
