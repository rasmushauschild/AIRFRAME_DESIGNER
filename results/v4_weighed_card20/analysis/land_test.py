"""Take off, hover 15 s, Land; measure the descent, the touchdown and the shutdown (port 8087)."""
import json, sys, time, urllib.request
U = "http://127.0.0.1:8087"
def get(p): return json.load(urllib.request.urlopen(U + p, timeout=60))
def post(p, d): return json.load(urllib.request.urlopen(urllib.request.Request(U + p, json.dumps(d).encode(), {"Content-Type": "application/json"}), timeout=60))
label = sys.argv[1]
if len(sys.argv) < 3 or sys.argv[2] != "landonly":
    time.sleep(20)
    post("/api/px4/command", {"command": "takeoff"}); time.sleep(2)
    for _ in range(6):
        post("/api/px4/command", {"command": "arm"}); time.sleep(2)
        if get("/api/status").get("armed"): break
    post("/api/px4/command", {"command": "takeoff"}); time.sleep(30)
t_land = get("/api/runs/current")["rows"][-1][0]
post("/api/px4/command", {"command": "mode", "mode": "land"}); time.sleep(25)
d = get("/api/runs/current"); c = d["columns"]; i = {k: n for n, k in enumerate(c)}
R = [r for r in d["rows"] if r[0] >= t_land]
td = next(k for k, r in enumerate(R) if r[i["on_ground"]])
thr = lambda r: [r[i[f"m{n}_thrust"]] for n in range(1, 10)]
before = R[max(0, td - 10):td]
spike = max(max(abs(a - b) for a, b in zip(thr(r), thr(R[td - 1]))) for r in R[td:td + 15])
off = next((k for k, r in enumerate(R[td:], td) if max(thr(r)) < 1.0), None)
pitch_at_off = R[off][i["pitch"]] if off else None
pitch_after = R[min(len(R) - 1, off + 25)][i["pitch"]] if off else None
desc = [r for r in R[:td] if r[i["vd"]] > 0.05]
out = {"label": label, "descent_s": round(R[td][0] - (desc[0][0] if desc else R[0][0]), 1),
       "speed_last_0.3m": round(max(r[i["vd"]] for r in before), 2), "touchdown_vd": round(R[td - 1][i["vd"]], 2),
       "touchdown_pitch": round(R[td][i["pitch"]], 1), "thrust_jump_N": round(spike, 1),
       "touchdown_to_motors_off_s": round(R[off][0] - R[td][0], 1) if off else None,
       "nose_drop_deg": round(pitch_at_off - pitch_after, 1) if off else None, "armed_end": get("/api/status").get("armed")}
print(json.dumps(out)); open("/Users/stefanosfragkoulis/AIRFRAME_REPRO/v4_weighed_card20/analysis/land_runs.jsonl", "a").write(json.dumps(out) + "\n")
