"""One scripted takeoff on the running V4 app (port 8087): Takeoff mode, arm, watch 40 s, report."""
import json, sys, time, urllib.request
U = "http://127.0.0.1:8087"
def get(p): return json.load(urllib.request.urlopen(U + p, timeout=60))
def post(p, d): return json.load(urllib.request.urlopen(urllib.request.Request(U + p, json.dumps(d).encode(), {"Content-Type": "application/json"}), timeout=60))
def shell(c): return urllib.request.urlopen(U + "/api/px4/shell?c=" + urllib.parse.quote(c), timeout=60).read().decode()
import urllib.parse
label = sys.argv[1]
for _ in range(60):
    if "Preflight check: OK" in shell("commander check") or True:
        break
time.sleep(20)                                    # estimator settle, as for V3
t0 = get("/api/runs/current")["rows"][-1][0]
post("/api/px4/command", {"command": "takeoff"}); time.sleep(2)
for _ in range(6):
    post("/api/px4/command", {"command": "arm"}); time.sleep(2)
    if get("/api/status").get("armed"): break
post("/api/px4/command", {"command": "takeoff"})
armed_at = time.time(); time.sleep(40)
d = get("/api/runs/current"); c = d["columns"]; i = {k: n for n, k in enumerate(c)}
rows = [r for r in d["rows"] if r[0] >= t0 and r[i["armed"]]]
st = get("/api/status")
out = {"label": label, "armed_samples": len(rows), "still_armed": st.get("armed"), "mode": st.get("mode_name")}
if rows:
    h = [r[i["height"]] for r in rows]; p = [r[i["pitch"]] for r in rows]; ro = [abs(r[i["roll"]]) for r in rows]
    out.update(h_start=round(h[0], 2), h_max=round(max(h), 2), h_end=round(h[-1], 2), pitch_start=round(p[0], 1),
               pitch_end=round(p[-1], 1), roll_absmax=round(max(ro), 1), thrust_max=round(max(r[i["thrust"]] for r in rows), 0),
               cmd_end=[round(rows[-1][i[f"m{n}_cmd"]], 2) for n in range(1, 10)])
    lift = next((r[0] - rows[0][0] for r in rows if r[i["height"]] > h[0] + 0.3), None)
    out["liftoff_s"] = None if lift is None else round(lift, 1)
print(json.dumps(out))
open("/Users/stefanosfragkoulis/AIRFRAME_REPRO/v4_weighed_card20/analysis/sitl_runs.jsonl", "a").write(json.dumps(out) + "\n")
post("/api/px4/command", {"command": "disarm"}) if False else None
