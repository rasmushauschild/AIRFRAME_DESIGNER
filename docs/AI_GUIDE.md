# Running simulations from an AI agent (or any script)

Everything the interactive app can fly, a script can fly headless, with PX4 in the loop, as fast as PX4 allows,
in parallel. The contract is JSON in, JSON out.

## One run

```bash
.venv/bin/python -m airframe_designer run --airframe airframes/atlas_08.json --scenario hover --out r.json
.venv/bin/python -m airframe_designer run --airframe airframes/atlas_08.json --scenario cruise \
    --set "rotors[0:8].tilt_deg=30" --set "hover_pitch_deg=20" --set "px4.MC_PITCHRATE_P=0.12" --out r.json
```

`--set path=value` applies a [parameter path](SCHEMA.md#parameter-paths) before the run; `--param NAME=VALUE`
adds a raw PX4 parameter; `--timeseries ts.json` also saves the 50 Hz state history.

What happens: a private PX4 SITL instance boots from a fresh working directory with **every parameter pre-seeded**
(geometry export + `px4_overrides` + scenario parameters + batch defaults) in its parameter file, the lockstep loop
runs at `--speed 0` (unthrottled), the scenario drives PX4 over MAVLink, metrics are collected in simulation time,
PX4 is stopped. Nothing touches an interactive session running on instance 0.

The result (`r.json`):

```jsonc
{
  "ok": true, "status": "done", "failures": [],
  "scenario": "hover", "airframe_name": "ATLAS_08", "variables": {"rotors[0:8].tilt_deg": 30},
  "timing": {"wall_s": 4.1, "px4_boot_s": 0.4, "sim_s": 21.0, "rtf": 6.9, "lockstep_timeouts": 8, "instance": 1},
  "px4_params_verified": {"CA_ROTOR_COUNT": {"seeded": 10, "vehicle": 10, "ok": true}, ...},
  "metrics": {
    "crashed": false, "crash_reason": "", "max_tilt_deg": 33, "max_alt_m": 3.3, "energy_wh": 16.6,
    "saturation_fraction": 0.016, "touchdown_speed": null, "flight_time": 16.8, "final_alt": 3.2,
    "events": [{"t": 4.2, "text": "takeoff altitude reached after 6.8s"}],
    "phases": {
      "takeoff": {"duration": 6.8, "time_to_alt": 6.8, "alt_err_rms": 2.0, "util_max": 0.86, ...},
      "hover":   {"duration": 10.0, "alt_mean": 3.18, "alt_std": 0.04, "pos_std_xy": 0.13, "pos_drift": 0.2,
                  "roll_rms_deg": 0.26, "pitch_rms_deg": 2.5, "pitch_mean_deg": -2.5, "yaw_drift_deg": 1.2,
                  "rates_rms_deg_s": 3.1, "tilt_max_deg": 6.0, "util_max": 0.73, "util_mean": 0.6,
                  "power_mean": 3738, "thrust_to_weight_mean": 1.06, "lift_share_mean": 0.0, "airspeed_mean": 0.1, ...}
    }
  }
}
```

Every phase gets the same statistics (see `airframe_designer/sim/metrics.py`): altitude, position scatter and drift,
speed, roll/pitch mean/RMS/max in PX4's hover frame, yaw drift, rate RMS, tilt, thrust/weight, momentum-theory
power, wing lift share, airspeed, busiest-motor utilisation, saturation. Offboard phases add `vel_err_rms` /
`pos_err_rms` / `time_to_target`; takeoff adds `time_to_alt`; phases named `hover*` define `power_ratio_vs_hover`.

## Scenarios

Bundled in `scenarios/` (`airframe-designer scenarios` lists them): `hover`, `takeoff_hover_land`, `cruise`
(offboard velocity 12 m/s), `box` (offboard position, 20 m square), `gust` (6 m/s wind switches on), `motor_out`
(rotor 1 dies), `manual_push` (full forward stick in Position mode), `nose_lift_takeoff` (nose-up hoverer:
raise the nose with the front motors before arming). Write your own: a JSON with phases
`wait_ready`, `nose_lift`, `takeoff`, `hold`, `land`, `offboard_velocity`, `offboard_position`, `manual`, `wait`,
`wind`, `motor_failure`, `param`, `arm`, `mode` (documented in `airframe_designer/sim/scenario.py`).

`nose_lift` (`{"type": "nose_lift", "motors": [8, 9], "target_pitch_deg": 25, "rate_deg_s": 8, "assist_motors": [...],
"assist_cmd": 0.2}`) is a ground sequence for aircraft that park nose-down but hover nose-up: while PX4 is still
disarmed the simulator drives the listed motors to raise the nose to the hover pitch, the next `takeoff` phase arms
PX4, and the sequence keeps holding the nose until PX4's own commands take over, then fades out. PX4 needs no
changes (it only sees its attitude change, as if tilted by hand). The metrics report `time_to_pitch` and `max_cmd`
(how close to full thrust the lifting motors came). The same sequence is available interactively on the Flight tab
and, when enabled for an airframe (`design.nose_lift`), the Takeoff button runs it first. Give phases a
`"name"` to group metrics; scenario `"params"` seed PX4 parameters for that scenario.

## Many runs in parallel

```bash
.venv/bin/python -m airframe_designer batch --tasks tasks.json --workers 6 --out results.jsonl
```

`tasks.json` = `[{"id": "a", "airframe": "...json", "scenario": "hover", "set": {"mass.cg[0]": 0.03}}, ...]`.
Each worker process owns one PX4 instance (1..9); results stream to `results.jsonl` as they finish. On an M2 Max a
hover scenario (21 s of flight) takes about 4.5 s of wall time per worker including PX4 boot: four workers finished
eight runs in 13 s (about 40 per minute), six workers give roughly 55. `--instances 2,3,4,5` pins the PX4 instances
when several batches run at once.

## Optimisation studies

```bash
.venv/bin/python -m airframe_designer study --spec studies/atlas08_hover_tilt.json --workers 6
```

A study is JSON: airframe, scenario(s), variables (paths + ranges), an objective expression over the metrics,
constraints, and an algorithm (`random`, `grid`, `cmaes`, `nelder_mead`) with a budget. Results land in
`results/<name>/`: `trials.jsonl` (every evaluation with its metrics), `best.json`, `best_airframe.json` (load it in
the UI or run it again), `summary.json`. Interrupted studies resume from `trials.jsonl`.

```jsonc
{
  "name": "atlas08_hover_tilt",
  "airframe": "airframes/atlas_08.json",
  "scenario": "hover",
  "variables": [{"path": "rotors[0:8].tilt_deg", "range": [10, 45]}, {"path": "hover_pitch_deg", "range": [0, 35]}],
  "objective": "phases.hover.pos_std_xy + 0.05 * phases.hover.pitch_rms_deg + 0.5 * phases.hover.util_max",
  "constraints": ["not metrics.crashed", "ok", "phases.hover.util_max < 0.9"],
  "penalty": 100,
  "algorithm": {"name": "cmaes", "budget": 24, "population": 6, "sigma": 0.3, "seed": 1},
  "workers": 6,
  "sim": {"rate": 250, "substeps": 2, "noise": true, "speed": 0}
}
```

Expressions see the result dict: `ok`, `metrics.*` (or just `phases.*`, `crashed`, `energy_wh`), and with several
scenarios `scenarios.<name>.metrics.*`. Allowed: arithmetic, comparisons, `and/or/not`, `abs min max sqrt exp log
clip hypot`, conditional expressions. A missing value or a violated constraint adds `penalty`.

## From Python

```python
from airframe_designer.batch import run_once, run_many, run_study
from airframe_designer.geometry import Airframe, apply_variables

r = run_once("airframes/atlas_08.json", "hover", variables={"mass.cg[0]": 0.03})
print(r["ok"], r["metrics"]["phases"]["hover"]["pos_std_xy"])

rs = run_many([{"id": f"cg{i}", "airframe": "airframes/atlas_08.json", "scenario": "hover",
                "variables": {"mass.cg[0]": x}} for i, x in enumerate([-0.05, 0, 0.05])], workers=3)
```

## Static analysis (no PX4, milliseconds)

`airframe-designer analyse --airframe X --speed-kmh 50` gives PX4's hover allocation (busiest motor, wasted thrust,
control authority, negative-thrust problems), a steady cruise trim (pitch, power vs hover, wing lift share, angle
of attack) and validation problems. `airframe-designer optimise --airframe X --spec spec.json` searches the same
model over variable paths in seconds; use it to pre-screen before spending closed-loop runs.

## Practical notes for agents

* Read `px4_params_verified`: if any entry is not ok, PX4 did not take the seeded parameters (a wrong name, or
  a parameter that needs a reboot).
* `status` is `done`, `aborted` (crash / termination / max_time) or `error` (PX4 or link problem: see `failures`
  and `log`); `ok` is false whenever a phase failed.
* Keep the interactive app on instance 0 and port 8080; batch runs use instances 1..9 automatically, or pass
  `--instance` / `--instances` explicitly when several batches run at once.
* Simulation time is deterministic per seed, wall time is not; compare metrics, not wall time.
* A run that never becomes "ready" within 45 s of simulation time usually means PX4 refuses to arm: check
  `failures` for the last arming-check summary.

## The nose lift on a real aircraft

The simulator's nose lift is a controller *outside* PX4: while PX4 is disarmed it drives the front motors to raise
the nose to the hover pitch, holds it while PX4 arms and spools up, and fades out once PX4's own commands exceed
its hold. PX4 only ever sees its attitude change. To do the same on hardware you need something that can command
motors before PX4's own control loop takes over. Three ways, in order of how well they match the simulation:

1. **PX4 external mode (ROS 2 / uXRCE-DDS) on a companion computer** — the closest match. The PX4 ROS 2 Interface
   Library (`px4_ros2`) lets a companion register a custom flight mode that PX4 arms in and that publishes
   *direct actuator setpoints* (`DirectActuatorsSetpointType`), i.e. per-motor commands exactly like the
   simulator's floor. The mode runs the same loop (geometric balance feed-forward + rate loop + yaw split, all in
   `airframe_designer/sim/nose_lift.py`, about 150 lines to port), then requests Takeoff/Position mode at the target
   pitch. The motors are already running at the handover, so PX4's allocator takes over from a live state instead
   of from zero. Needs PX4 >= 1.15 and a companion (Raspberry Pi / Jetson) or a laptop on the DDS link.
2. **A custom PX4 module** in the firmware that does the same as an internal mode and publishes `actuator_motors`
   directly. Best latency and no companion, but firmware work (mode registration, arming state, output enabling).
3. **`MAV_CMD_ACTUATOR_TEST` from a script** (companion or ground station over MAVLink): PX4 drives individual
   motors while *disarmed* for bench tests, so a script can run the lift loop through it, then arm and take off.
   Simple, but at arming the test outputs stop and PX4 spools from zero, so the nose drops until PX4 catches it
   (in simulation the same handover without the hold swings the nose about 15 degrees). Fine for ground trials,
   not for a gentle liftoff.

Whichever path, the simulation tells you the command profile to expect (for ATLAS_07D: about 96% of the front
jets at the parked attitude falling to 57% at 25 degrees, 8 to 15 degrees per second) and the margins (the front
pair only just holds the nose; a 20% idle on the rear jets or the CG 6 cm aft gives headroom). HITL with the real
Pixhawk validates the arming/handover part: the simulator's hold acts as the companion's would.

## Airfoil polars (wing model "polar")

Set a wing's `aero.model` to `polar` and name its sections (`aero.airfoil_root`, `aero.airfoil_tip`). Names are
NACA 4/5-digit (`naca2412`, `naca23006`, generated analytically), a local `airfoils/<name>.dat` (Selig or
Lednicer format, e.g. copied from the UIUC database), or a UIUC database name that is downloaded on first use
(`e423`, `sd7037`, `clarky`, ...). The polar (CL, CD, CM over -24..28 degrees at 7 Reynolds numbers from 5e4 to
5e6) is produced by XFOIL if an `xfoil` binary is on the PATH, otherwise by NeuralFoil, and cached in
`airfoils/polars/<name>.json`; `airframe-designer` builds it the first time the airframe is used (about a second
per section). At run time every strip looks up its section at its own angle of attack and Reynolds number
(chord x speed / nu), root and tip blended along the span, with the wing's induced angle CL/(pi e AR) subtracted
and the induced drag added, so `oswald` still matters. Stall and the post-stall flat plate come from the polar's
valid range. `POST /api/airfoil/polar {"name": "naca23006"}` builds a polar and returns CL max, stall angle, CD min
and L/D max; `GET /api/airfoils` lists what is cached.

## Two physics engines, and comparing them

Every flight can run on the project's Python rigid body (default) or on **JSBSim**, the open-source flight
dynamics model behind FlightGear and ArduPilot SITL (`--physics jsbsim` on `run`/`batch`, `"physics": "jsbsim"` in
a study, the Physics selector on the Flight tab, or per job in the Batch tab). JSBSim gets the same airframe: a
generated aircraft model with the mass and inertia, the feet as ground contacts, one external force per rotor at
its position along its thrust axis (so JSBSim computes the moment arms itself), and the wing/body aerodynamics as
coefficient tables sampled from this project's aero models. Rotor spool-up, thrust curve, reaction torque and
intake ram drag stay the project's own model and are injected each step. A difference between the two engines
therefore points at integration, frames, moment arms, ground contact or gravity, not at the coefficient data.

```bash
.venv/bin/python -m airframe_designer compare --airframe airframes/quad_x.json --scenario hover --out cmp.json
```

prints both runs' per-phase metrics side by side with deltas and the RMS/max differences of the state histories
(altitude, speed, attitude, rates, thrust); the Batch tab's **Compare physics** button does the same for the live
design. Reference numbers for the quad preset in hover: altitude 7 mm RMS, thrust 0.06 N, attitude ~0.2 degrees.
The cross-check already found and fixed one real defect (a rest-damping hack that parked soft-legged aircraft
25 cm above their spring equilibrium).

## CAD masses

`cad.bodies[<name or index>].mass`, `.offset[i]` and `cad.origin[i]` are ordinary parameter paths, so a study can move a
battery or ask "where must the CG be" with the real inertia of the CAD solids. Import a STEP file once in the Geometry
tab (or POST its bytes to `/api/cad/import?filename=x.step`); `GET /api/cad/totals` gives the bodies' mass and CG.

## Driving it from ChatGPT or another assistant

Everything above is plain HTTP and CLI, so any assistant that can call a tool can do what Claude Code does here.
Three ways, from most to least capable:

1. **MCP server** (ChatGPT connectors, Claude Desktop, Cursor, Codex, any MCP client). Start it with
   `.venv/bin/python -m airframe_designer.mcp_server` (stdio) or `... --http 8765` (streamable HTTP at
   `http://127.0.0.1:8765/mcp`). It exposes tools: `get_status`, `get_airframe`, `list_paths`, `set_parameters`
   (edit by parameter path, optionally push to PX4 and save), `load_airframe`, `px4_command`, `sim_control`
   (reset/pause/speed/physics/wind), `nose_lift`, `list_scenarios`, `analyse`, `run_scenario`, `compare_physics`,
   `run_study`, plus the guide and schema as resources. The live tools need the app running (URL in `AFD_APP_URL`,
   default `http://127.0.0.1:8081`); the headless ones run their own PX4. ChatGPT reaches a *local* server only
   through a public HTTPS tunnel (e.g. `ngrok http 8765`, then add the tunnel URL + `/mcp` as a connector in
   ChatGPT's settings, developer mode). Claude Desktop / Cursor / Codex take the stdio command directly in their
   MCP configuration, e.g.
   `{"mcpServers": {"airframe-designer": {"command": "/Users/you/AIRFRAME_DESIGNER/.venv/bin/python", "args": ["-m", "airframe_designer.mcp_server"]}}}`.
2. **Custom GPT with Actions.** The app publishes its OpenAPI description at `http://127.0.0.1:8081/openapi.json`.
   Put a tunnel in front of port 8081, import that schema as the GPT's Action, and the GPT can call the same
   endpoints (`POST /api/airframe/apply_variables`, `POST /api/px4/push`, `POST /api/batch/run`, ...).
3. **A shell agent** (OpenAI Codex CLI, Claude Code, aider): point it at this repository; `AGENTS.md` / `CLAUDE.md`
   tell it how to run headless flights and studies from the command line.

The app has no authentication: only expose it through a tunnel while you use it, and never on a public URL you
leave running.

### Visible SITL scenarios

`POST /api/sim/scenario/start` takes a scenario object and runs it in the interactive simulator at 1× speed.
It requires disarmed SITL and rejects overlapping runs. Push and verify parameters beforehand; the scenario
object must omit `params`. `GET /api/sim/scenario` returns phase and metrics;
`GET /api/sim/scenario/timeseries` returns recorded samples. The viewport shows the phase and a Stop test
button. `POST /api/sim/scenario/stop` freezes the simulation. A failed run also freezes it for inspection.
Reset/restart the simulated vehicle before another flight; do not use this endpoint for HITL.


### Experimental near-level wing flight (SITL only)

`wing_velocity` is an experimental scenario phase, not the stock PX4 position controller. It sends MAVLink2
`SET_ATTITUDE_TARGET` with the `thrust_body` extension: PX4 still closes attitude/rate loops and allocates motors.
The harness uses ideal simulator position/velocity and an offline `trim_table` of `[speed_m_s, Fx_N, Fy_N, Fz_N]`
rows; it does not validate sensors or constitute an onboard controller. The installed older pymavlink lacks this
extension, so `sim/wing_controller.py` packs the three extension floats explicitly, with wire-layout/CRC tests.

Phases accept `speed`, `duration`, `ramp` (m/s²), and `trim_table`. The current implementation requires a zero-degree
hover frame and CT proportional to physical effective thrust. It independently commands forward/vertical thrust,
uses bank limited to ±8° for lateral tracking, and commands zero body pitch and yaw. A controller instance persists
across these phases to maintain altitude and ramp continuity. Use a fresh scenario for each aircraft. HITL is rejected.
Treat phase completion separately from performance: examine the settled speed, height and attitude time histories.
