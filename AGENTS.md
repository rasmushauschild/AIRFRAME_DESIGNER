# AIRFRAME_DESIGNER — notes for AI agents (Codex, ChatGPT, Claude, Cursor) working in this repo

A PX4-in-the-loop aircraft design simulator. Python 3.12, venv at `.venv` (`.venv/bin/python`). PX4 SITL is built at
`~/PX4-Autopilot/build/px4_sitl_default` (v1.18). No build step for the UI (vanilla JS + vendored three.js).

## Run things
* Interactive app (3D editor, PX4 SITL/HITL, USB remote): `.venv/bin/python -m airframe_designer ui`
  → http://127.0.0.1:8080. **Instance 0 / port 8080 may already be in use by a running session: never kill it.**
* Headless: `.venv/bin/python -m airframe_designer run --airframe airframes/atlas_09.json --scenario hover --out r.json`
* Parallel / studies: `batch --tasks t.json --workers 6`, `study --spec studies/<name>.json`. See docs/AI_GUIDE.md.
* Tests: `.venv/bin/python -m pytest -q` (the `px4` marked test boots a real PX4 on instance 8, ~15 s).
* Static analysis: `.venv/bin/python -m airframe_designer analyse --airframe X`.
* MCP server for other assistants: `.venv/bin/python -m airframe_designer mcp` (stdio) / `mcp --http 8765`; tools in
  `airframe_designer/mcp_server.py`, setup in docs/AI_GUIDE.md.

## Layout (each segment is independent)
```
airframe_designer/
  geometry/   mass (CG, inertia), propulsion (rotors: every one a ducted jet), wings, gear (legs), body,
              cad (STEP solids -> masses via OCP; OBJ/STL meshes to look at, no mass),
              airframe (composition, schema migration, PX4 export, hover check), paths (parameter-path addressing)
  aero/       strip-theory wings, rotor thrust/torque/ram drag, body drag, fastmath
              (a wing is a flat panel: one sweep, one dihedral, chord straight from root to tip.
               ui/scene_math.js holds the same geometry for the 3D view and tests/test_wing_curve.mjs
               compares the two, so what is drawn is what flies)
  dynamics/   quaternion, leg geometry, jsbsim_backend (THE physics engine; there is no other)
  sensors/    IMU/mag/baro/GPS -> HIL messages
  px4/        link (MAVLink SITL/HITL), sitl (process, BSON param seeding, instances), connection (runtime SITL<->HITL),
              events, param_meta
  sim/        simulator loop (hooks), scenario runner, metrics, recorder (every run, 20 Hz, to results/runs)
  analysis/   static hover/cruise model + geometric optimiser (no PX4)
  batch/      worker (run_once), runner (parallel), objective (expressions), optimizers, study
  server/     FastAPI + websocket;  app.py = interactive entry;  cli.py = subcommands
ui/           index.html app.js scene.js style.css vendor/three
airframes/    schema-2 JSON (docs/SCHEMA.md);  scenarios/  studies/  results/ (generated)
```

## Conventions
* Structural frame FRD, origin = reference point, CG in `mass.cg`; all force models take positions relative to the CG.
* PX4 export: rotor positions relative to the CG, rotated into the hover frame (`hover_pitch_deg`), `km` sign = spin.
* Batch runs seed PX4 parameters through `fs/parameters.bson` before boot; `SYS_AUTOSTART` must match the model
  (10016 for none_iris) or PX4 resets everything (handled in `px4/sitl.py`).
* The simulation loop must never block on MAVLink replies in lockstep (PX4 only advances when we send sensors);
  scenario phases are non-blocking state machines.
* Keep `run_once` results JSON-serialisable; metrics are the contract for studies.
* Mass resolution goes through `Airframe.resolve_mass()` (items + CAD bodies), never `mass.resolve()` alone.
* Ground contact damping is clamped per step to the integrator's stable range (dynamics/contact.py); without it a
  light, low-inertia airframe jitters on its legs and the fake gyro noise makes PX4's estimator refuse to arm.
* `sim/nose_lift.py` is the pre-arm ground sequence (motor floors under PX4's commands); it is a simulator hook,
  not a PX4 feature.
* The app's tabs are Geometry, Params, Firmware, Batch and Controller. Every section on every tab is a
  `<details class="sec">`: the white box, its title the handle, its contents directly on it, nothing inside
  it that is another white box. Which sections are open is the user's and is remembered.
* Every run is recorded (`sim/recorder.py`) and kept at the next reset if it armed or left the ground, with the
  airframe and the exported parameters inside the same file. The log window plots it and scrubs the 3D view
  through it; `/api/runs/current` is the recording in progress. Adding a column means appending to `COLUMNS`
  and to `LIVE_COLS` in `ui/app.js` — never reordering, or every saved run reads back wrong.

## The physics, and what you may not do to it
* **JSBSim is the only engine.** It owns integration, gravity, the moment arms and the ground contact. The
  project supplies the rotor model (spool, thrust curve, reaction torque, ram drag), the wing and body aero as
  coefficient tables, and the sensors. There is no switch, no second engine and no comparison mode; a result
  whose engine you have to ask about is not one you can trust.
* **Gravity is JSBSim's.** Nothing in this repo adds a weight force. `constants.G` exists for analysis and for
  converting the accelerometer reading, and `G` is defined in exactly one file.
* **The accelerometer reads specific force**, which is the motion minus gravity (`dv[2] -= G`). In free fall it
  reads zero. That is not a missing gravity term.
* **Every rotor is a ducted jet.** No kinds, no jetfoil, no ram-drag switch. A duct always swallows a mass flow,
  so momentum drag is always modelled. Known limitation: the ram-drag model holds the mass flow at its hover
  value, which over-predicts drag in fast forward flight, where the captured streamtube actually narrows.
* Run `tests/` after any change. `test_jsbsim_backend.py` and `test_dynamics.py` check the engine against
  analytic free fall, Hooke's law on the legs, the hover solve and conservation, not against a previous run.
