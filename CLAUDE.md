# AIRFRAME_DESIGNER — notes for AI agents working in this repo

A PX4-in-the-loop aircraft design simulator. Python 3.12, venv at `.venv` (`.venv/bin/python`). PX4 SITL is built at
`~/PX4-Autopilot/build/px4_sitl_default` (v1.18). No build step for the UI (vanilla JS + vendored three.js).

## Run things
* Interactive app (3D editor, PX4 SITL/HITL, USB remote): `.venv/bin/python -m airframe_designer ui`
  → http://127.0.0.1:8080. **Instance 0 / port 8080 may already be in use by a running session: never kill it.**
* Headless: `.venv/bin/python -m airframe_designer run --airframe airframes/atlas_08.json --scenario hover --out r.json`
  (`--physics jsbsim` runs the same flight on JSBSim; `compare` runs both and diffs them)
* Parallel / studies: `batch --tasks t.json --workers 6`, `study --spec studies/<name>.json`. See docs/AI_GUIDE.md.
* Tests: `.venv/bin/python -m pytest -q` (the `px4` marked test boots a real PX4 on instance 8, ~15 s).
* Static analysis: `.venv/bin/python -m airframe_designer analyse --airframe X`.
* Native PX4 firmware (ATLAS nose lift) builds into `firmware/atlas/build`: `make firmware`, or the app's Update PX4
  rebuilds + relaunches when sources changed (`px4/firmware.py`, `ConnectionManager.update_firmware`).
* MCP server for other assistants: `.venv/bin/python -m airframe_designer mcp` (stdio) / `mcp --http 8765`; tools in
  `airframe_designer/mcp_server.py`, setup in docs/AI_GUIDE.md.

## Layout (each segment is independent)
```
airframe_designer/
  geometry/   mass (CG, inertia), propulsion (rotors), wings, gear (legs), body, cad (STEP solids -> masses, via OCP),
              airframe (composition, schema migration, PX4 export, hover check), paths (parameter-path addressing)
  aero/       strip-theory wings, rotor thrust/torque/ram drag, body drag, fastmath
  dynamics/   quaternion, per-leg contact, rigid body about the CG, jsbsim_backend (JSBSim as an alternative engine)
  sensors/    IMU/mag/baro/GPS -> HIL messages
  px4/        link (MAVLink SITL/HITL), sitl (process, BSON param seeding, instances), connection (runtime SITL<->HITL),
              events, param_meta
  sim/        simulator loop (hooks), scenario runner, metrics
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

## When changing physics
Run `tests/` and the equivalence check idea in git history (forces on random states before/after). Physics cost
matters: ~85 µs per sub-step for a 10-rotor + wing airframe; avoid `np.cross` on small arrays (use `aero/fastmath`).
