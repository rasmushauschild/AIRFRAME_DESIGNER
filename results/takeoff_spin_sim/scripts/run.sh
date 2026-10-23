#!/bin/zsh
# run.sh <name> [extra --set/--param args...]   flies the takeoff scenario on the as-built sim, prints the yaw summary
S=/private/tmp/claude-501/-Users-stefanosfragkoulis-Documents-UtopiaLabs-new-AIRFRAME-DESIGNER/0c01eb96-338b-485f-8320-52b846b16409/scratchpad
N=$1; shift
cd /Users/stefanosfragkoulis/Documents/UtopiaLabs/new/AIRFRAME_DESIGNER
.venv/bin/python -m airframe_designer run --airframe airframes/small_scale_v3.json --scenario $S/simruns/takeoff_spin.json --px4-dir $S/px4priv --instance ${INST:-12} \
  --out $S/simruns/$N.json --timeseries $S/simruns/${N}_ts.json --timeout 280 ${=$(cat $S/simruns/asbuilt.args)} ${=$(cat $S/simruns/board_flight_params_gps.args)} \
  --param SDLOG_MODE=0 --param SDLOG_PROFILE=1 "$@" > $S/simruns/$N.log 2>&1
L=$(find ~/.airframe_designer/px4_instance_${INST:-12} -name '*.ulg' -mmin -3 | sort | tail -1); [ -n "$L" ] && cp $L $S/simruns/$N.ulg
.venv/bin/python - $S/simruns/${N}_ts.json $N <<'PY'
import json, sys, numpy as np
d = json.load(open(sys.argv[1]))["timeseries"]; c = d["columns"]; R = np.array(d["rows"], float); t = R[:, 0]
a = R[:, c.index("airborne")] > 0.5
if not a.any(): print("%-28s never airborne" % sys.argv[2]); sys.exit()
t0 = t[np.argmax(a)]; m = (t >= t0) & (t <= t0 + 8)
r = np.degrees(R[:, c.index("r")]); yaw = np.unwrap(R[:, c.index("yaw")])
pk = np.argmax(np.abs(r[m]))
print("%-28s peak %4.0f deg/s at +%.1f s | turned %+5.0f deg in 8 s | rate at +8 s %+4.0f | max alt %.1f m" % (sys.argv[2], abs(r[m][pk]), t[m][pk] - t0,
      np.degrees(yaw[m][-1] - yaw[m][0]), r[np.argmin(abs(t - t0 - 8))], (-R[:, c.index("d")]).max()))
PY
