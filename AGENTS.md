# AIRFRAME_DESIGNER — notes for AI agents (Codex, ChatGPT, Claude, Cursor) working in this repo

A PX4-in-the-loop aircraft design simulator. Python 3.12, venv at `.venv` (`.venv/bin/python`). PX4 SITL is built at
`~/PX4-Autopilot/build/px4_sitl_default` (v1.18). No build step for the UI (vanilla JS + vendored three.js).

## Run things
* Interactive app (3D editor, PX4 SITL/HITL, USB remote): `.venv/bin/python -m airframe_designer ui`
  → http://127.0.0.1:8080. **Instance 0 / port 8080 may already be in use by a running session: never kill it.**
* Headless: `.venv/bin/python -m airframe_designer run --airframe airframes/atlas_09b.json --scenario hover --out r.json`
* Parallel / studies: `batch --tasks t.json --workers 6`, `study --spec studies/<name>.json`. See docs/AI_GUIDE.md.
* Tests: `.venv/bin/python -m pytest -q` (the `px4` marked test boots a real PX4 on instance 8, ~15 s).
  The suite flies its own airframes from `tests/data/`, never the designs in `airframes/`: those are the
  user's, to rename or delete as they please. `test_airframe_load_examples` is the one that reads them, and
  it only asks that each still loads.
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
  sensors/    IMU/mag/baro/GPS and the H-flow (optical flow + rangefinder) -> HIL messages
  px4/        link (MAVLink SITL/HITL), sitl (process, BSON param seeding, instances), connection (runtime SITL<->HITL),
              events, param_meta
  sim/        simulator loop (hooks), scenario runner, metrics, wind (steady or gusting),
              recorder (every run, 20 Hz, to results/runs)
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
* A rotor or a leg may carry a `pair` token: two items sharing one are a left/right couple and the app keeps
  their geometry mirrored about y as either is edited. They stay two separate things — two PX4 outputs, two
  names, two spin directions, two contact points — so nothing downstream has to know about the link.
  `Airframe.validate()` reports a token that is not on exactly two items.
* Ground contact damping is clamped per step to the integrator's stable range (dynamics/contact.py); without it a
  light, low-inertia airframe jitters on its legs and the fake gyro noise makes PX4's estimator refuse to arm.
* `sim/nose_lift.py` is the pre-arm ground sequence (motor floors under PX4's commands); it is a simulator hook,
  not a PX4 feature.
* Two sensors have a place on the aircraft, both in the structural frame and both with a mounting rotation
  (roll, pitch, yaw degrees, 0 = square with the airframe): the flight controller
  (`design.pixhawk_position` / `pixhawk_rotation`) and the H-flow (`design.flow_sensor`, off unless
  `enabled`). `geometry/airframe.py` holds `mount_rotation()` / `rotation_euler_deg()`, the same yaw-pitch-roll
  composition PX4 builds SENS_BOARD_*_OFF from.
  - The IMU reads in the board's own axes (`sensors/models.py` turns the true vectors by the mount) and the
    export writes the hover rotation *composed with* the mount as SENS_BOARD_X/Y/Z_OFF, so a board declared
    the way it is bolted in is invisible and a board declared wrong is wrong in flight, as on the aircraft.
  - The H-flow sends HIL_OPTICAL_FLOW and DISTANCE_SENSOR at 20 Hz and exports EKF2_OF_*, EKF2_RNG_* and
    SENS_FLOW_ROT (fusion is switched off explicitly when no sensor is fitted). PX4 only knows a flow
    sensor's yaw, in 45 degree steps; the rest of a mount — including the hover pitch, which tilts a
    square-mounted lens away from the ground — stays in the measurement, and PX4 cannot fly on it. The
    Sensor positions section says how far off vertical the beam ends up in hover; ATLAS at 23 degrees of
    hover pitch does not reach altitude with a square-mounted H-flow, and does with the lens tilted to match.
  - The flow signs are PX4's, read out of v1.18 and checked in `tests/test_flow_sensor.py` by running PX4's
    own arithmetic over the message: pixel flow is the negated line-of-sight rate integrated over the frame,
    in the sensor's axes, and the integrated gyro goes out in PX4's body axes because PX4 turns the pixel
    flow by SENS_FLOW_ROT and leaves the delta angle alone.
* The app's tabs are Geometry, PX4, Sequences and Live control — the aircraft, the flight controller, what
  it is told to fly, and flying it by hand.
  - **PX4** is everything about the autopilot in one place: which one is connected (SITL, HITL or the real
    board), its parameters, and the firmware. **Parameters** is one table of what this design changes —
    parameter, set value, PX4's default, what the vehicle has right now, what it is — with the geometry's
    own parameters below it, read-only. There is no separate parameter browser: *+ Add parameter* searches
    every parameter on the vehicle and writes the one you pick as an override at its current value, which
    is then a row you edit. Update PX4 writes those parameters to the vehicle *and* saves them to its flash
    (`/api/px4/push` ends in `preflight_storage`), so the section has no Save-to-flash button; Fetch
    descriptions appears only when this computer has no metadata for the firmware that is running.
    **Firmware modules** holds Build and Flash beside the modules they build:
    modules come from `/api/firmware/modules` (a directory whose CMakeLists.txt calls px4_add_module) and
    their sources are edited in the tab or opened in VS Code (`/api/firmware/open`).
  - **Sequences** holds who does the takeoff and the landing — `design.flight.takeoff` / `.land`, either
    `px4` or a module's directory, asked over MAV_CMD_USER_1 — and `design.missions`: named lists of plain
    actions (take off, hover, move, cruise, set mode, set wind, parameter, motor out, wait, land) turned
    into scenario phases and flown by the same runner the scripted tests use. A sequence has no settings of
    its own beyond its time limit: wind is a step, and a module's own parameters belong to the module, not
    to a panel beside it. Sequences are design-independent: an action says what to do, never which motor
    does it. While one flies, the step it is on is marked in the list itself (`/api/sim/scenario` reports
    the phase index; phase 0 is the wait for PX4, so step i is phase i + 1) and nothing is drawn over the
    3D view. Stop stops the sequence, not the simulation - the aircraft keeps flying under PX4, and a
    sequence that fails no longer freezes the loop, because the graph already holds what happened.
  - **Live control** is the aircraft right now: flight mode, the remote, the wind, the motor sliders.
  Every section on every tab is a `<details class="sec">`: the white box, its title the handle, its contents
  directly on it, nothing inside it that is another white box. Which sections are open is the user's and is
  remembered.
* The wind is a model, not a vector: `sim/wind.py` holds a mean speed and direction (where it comes from,
  as a weather report gives it) and, when `gusty`, a swing about both. The loop asks it for the wind at the
  simulated time on every step, so it is the one owner of `sim.wind_ned` — a scenario phase and the Wind
  section both go through `Simulator.set_wind*`, and a plain vector means a steady wind with the gusts off.
  A gust is a sum of three sines, not noise: smooth enough to fly through, and the same in every run of the
  same scenario, which is what makes two gusty runs comparable. `/api/sim/wind/preview` returns what a model
  would blow over the next minute, and the Wind section draws exactly that, so the picture and the air come
  from one piece of arithmetic. The arrow beside the coordinate indicator in the 3D view is the wind blowing
  now - its length and its colour are the speed (pale grey to deep blue, full at 12 m/s) and there is no text
  in it, because the overlay already carries the number. It is hidden while you are inside a recording: the
  recorder keeps the wind's strength, not its direction, so there would be nothing honest to point.
* The simulation runs from the moment the app opens and keeps running until someone pauses it; the
  recorder collects the whole time.
* One switch over the 3D view decides what the whole thing is: **SITL** (PX4 here, flying the simulation),
  **HITL** (the board, flying the simulation) and **FLIGHT** (`/api/connection/flight`) — the aircraft on
  its own sensors with the app only listening. Pressing one is a decision, not a request: the switch moves
  there whether or not anything is plugged in, and the light beside it is green only while that mode is
  actually doing what it says (SITL linked; HITL answering with SYS_HITL = 1; FLIGHT with position and
  attitude arriving — `telemetry_age` in the status). Red, with the reason on hover, otherwise.
  FLIGHT works over a telemetry radio: a serial port that is not a Pixhawk is opened at 57600 unless the
  Baud box says otherwise, and Update PX4 writes the design's parameters over that same link while
  skipping every build and flash step, which need the board on a cable. In FLIGHT nothing is injected (`PX4Link.listening` is the
  one test, and it blocks every HIL send), the loop stops integrating and `_follow_telemetry` puts the
  aircraft where LOCAL_POSITION_NED and the attitude say it is, on the wall clock, so the 3D view and the
  graph are the real flight. `request_flight_telemetry` asks the vehicle for those streams. `native.py`'s reconnect relaunches PX4 under the loop without changing
  whether it is running.
* The recording is the session, not the flight: **Reset puts the aircraft back on the ground and leaves the
  graph alone** (`Simulator.reset` does not touch the recorder), and **Save run** is the only thing that
  ends one — it writes the recording, with the airframe and the exported parameters inside the same file,
  and the graph starts again from zero while the simulation carries on (`/api/runs/save` with `clear`).
  Two things follow from that, both handled in `sim/recorder.py`: a reset restarts the simulation clock, so
  a sample whose time went backwards rebases the recording instead of stopping it, and a recording that
  fills up (120k samples, about 100 minutes at 20 Hz) drops its oldest quarter rather than quietly ending.
  The log window plots it and scrubs the 3D view
  through it: drag to scrub, Play to run the recording forward from the cursor (at the speed beside it),
  pinch (or ctrl-wheel) to zoom the time axis and two fingers to pan, double-click or Fit for the whole
  run. The simulation stays paused for all of it — the 3D view is either the recording or the live
  aircraft, never half of each. Save run keeps it and starts the next; Reset throws it away and starts the
  next (`/api/runs/clear`); `/api/runs/current` is the recording in progress. The terminal beside the graph
  is the flight controller's and only its: `logLine` keeps the `[px4]` lines (PX4's console and the events
  it sends) and sends everything the app says to the browser console instead. A saved run carries that log
  for the span it covers, and selecting the run shows it.
* What is recorded is everything worth plotting: the fixed `COLUMNS` (pose, speeds, forces, the IMU as it
  reads, the baro, the H-flow, and PX4's flight mode as a number — main mode with Auto's sub-mode behind
  the decimal point, so one column plots as a staircase and reads back as "Hold") and then three per
  rotor — commanded throttle, spooled throttle, thrust —
  appended by `columns_for(n)` when the first sample is taken. A recording carries its own column list, so
  a run saved with four motors still reads back beside one with ten, and the UI builds its catalogue from
  that list: each column says which drawer it belongs in (Flight, Attitude, Motors, Sensors, H-flow,
  Power, State) and both pickers — the graph's traces and the readouts chosen onto the 3D view — are the
  same catalogue. Appending a column is safe; reordering `COLUMNS` is not. Nothing on the 3D view is
  fixed any more: every readout there is one of those columns, chosen with the ＋ button, so the overlay
  and the graph show the same quantities by the same names.

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
