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
* **The Rotors table's first column is the board, not a row number.** *Port* is which output the ESC is
  plugged into - MAIN 1-8 then AUX 1-2, named as they are printed beside the connector - and changing it
  re-maps the motor (`PWM_MAIN_FUNCn` / `PWM_AUX_FUNCn` = 100 + motor number, held as an override).
  Two motors swap by moving one to a free port first, which the table enforces rather than silently
  double-assigning a port. The list is MAIN 1-8 then AUX 1-8, the same sixteen `real_flight.output_keys()`
  wires a design to; the UI stopped at AUX 2 for no reason the hardware shares, so a board with more
  outputs could not be told about them (`MAIN_PORTS` / `PORT_COUNT` in ui/app.js are the one place). It was headed `#` for a long time and read as an index, so nobody could see
  that the ninth motor goes to AUX 1 rather than to a ninth MAIN pin that does not exist.
* A rotor or a leg may carry a `pair` token: two items sharing one are a left/right couple and the app keeps
  their geometry mirrored about y as either is edited. They stay two separate things — two PX4 outputs, two
  names, two spin directions, two contact points — so nothing downstream has to know about the link.
  `Airframe.validate()` reports a token that is not on exactly two items.
* A rotor can be mounted crooked on purpose. `axis` is the design - the Rotors table, the optimiser, the
  analysis and, above all, the PX4 export all keep it - and `tilt_error_deg` / `cant_error_deg` are how far
  that rotor really sits from it. Only the forces read the sum (`Rotor.built_axis`, used by
  `aero/rotor_aero.py` and by the per-rotor force directions in the JSBSim model), so the controller flies
  an aircraft it believes is perfect and the simulation flies the one you built. It is for exactly that
  experiment and nothing else: `validate()` says so out loud while any error is set, the 3D view turns the
  disc to where the rotor really pushes and draws a dashed amber stick along the axis PX4 was given, and
  `ui/scene_math.js` holds the same arithmetic as `geometry/propulsion.py` (pinned to each other by
  tests/data/rotor_axes.json). The Mounting errors section on the Geometry tab also says how far each rotor
  *could* be out: `Airframe.mount_margin()` winds one rotor's error up on a copy until
  `hover_trim_as_built()` gives out - first with nothing left to spare (a motor over 85%), then at all (a
  motor pulling downwards, past its maximum, or the aircraft having to hover leaning more than 15°). Both
  are static limits and generous ones: PX4's own trim runs out first, because it does not know.
* Ground contact damping is clamped per step to the integrator's stable range (dynamics/contact.py); without it a
  light, low-inertia airframe jitters on its legs and the fake gyro noise makes PX4's estimator refuse to arm.
* **Nothing in the simulator may touch what PX4 commands the motors to do.** There was once a pre-arm
  ground sequence here (`sim/nose_lift.py`, motor floors under PX4's commands) and a matching landing
  (`NoseLower`, which drove the motors outright). Both are gone, with their scenario phases
  (`nose_lift`, `nose_land`), their endpoint, their motor selection and their UI. A ground sequence that
  only exists in the simulator makes SITL fly an aircraft nobody built, and the tuning it produces is
  worthless on the bench. The nose lift lives in the firmware (`firmware/atlas/src/atlas_nose_lift`),
  where SITL and the real board run the same code, and a scenario reaches it with the `module` phase
  over MAV_CMD_USER_1. `Simulator.motor_override` remains, because it is the user's own hand on the
  motor sliders and it says so on screen.
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
    **Flash firmware** is the same button as Update PX4 and does the board's image first when it is
    stale (`board_firmware_status`): the firmware belongs to the project, not to the design loaded at
    the time, so a design that never touches the nose-lift module still flashes a board whose image is
    out of date. It was once gated on the design exporting `NLF_ENABLE = 1`, which meant the button said
    one thing and did another. **A board in its bootloader always needs an image** (`detected_board()`
    reads the "BL" in the USB name): it has no firmware to be newer or older than, so the last-flash
    marker says nothing, and a board stranded in its loader must never be refused by the one button that
    rescues it. Its loader does enumerate and the link reports "up" on sysid 0, which is not a vehicle -
    the push says so and stops rather than failing 123 parameter writes one at a time.
    **Firmware modules** has a switch per module - *In this design* - because a module this aircraft does
    not fly with has no business configuring it: switched off, every parameter in its own `params.yaml`
    leaves the export and the Parameters table, the ground-sequence block stops running with them, and
    the values stay in the design so switching it back on brings them back as they were
    (`design.modules_off`, `Airframe.modules_off_params`, `/api/firmware/modules/enable`). Two things add
    a module's parameters by name *after* `px4_params` has filtered them, and both ask as well: the
    real-flight interlock (`NLF_HW_OK` in `real_flight.flight_params`, which a firmware built without the
    module has not got) and the half of Update PX4 that copies everything else from the SITL reference
    (`sitl_diff`) - a SITL built with the module must not put it back on an aircraft flying without it.
    It also holds Build and Flash beside the modules they build:
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
  **Nothing asks with `window.confirm()`**: this app's web view answers it `false` instantly, without
  ever showing a dialog, so a control guarded by it can never be switched on - the motor-slider checkbox
  unticked itself the moment it was ticked, for exactly that reason. A dangerous control asks on itself
  (`confirmTwice`): the first press turns it into "Really?" for 8 seconds, the second does the thing, and
  a click anywhere else or Escape takes the question back. tests/test_ui_js.py fails the build if
  `confirm(` reappears in ui/.
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
* **Every parameter is yours to set, and every one can be put back.** The export applies hand-set
  overrides *last*, so what you type wins - over the allocator too, because it is your aircraft. What the
  app owes you in return is the truth about it: the row says `geometry says X` when the two have drifted
  apart, `validate()` names any override that goes against what the nose-lift sequence needs on its legs
  (it warns, it does not overrule), and one click on the row's ↺ forgets the override and lets the design
  set it again. `px4_params(overrides=False)` is what the geometry alone would set.
* **"Geometry dependent" is not a curated list.** It is exactly the set `px4_params(overrides=False)`
  returns, which is exactly the set that moves by itself when the design changes - the allocator, the
  sensor offsets and rotations, the H-flow's fusion switches, the output functions. The Parameters table
  has two sections, *Added parameters* and *Geometry dependent parameters*, and editing a geometry row
  leaves it where it is rather than promoting it: the section says where a value comes from, not whether
  it has been touched.
* **Every row says what a real flight gets instead** (`In flight`): the same value, a different one, or
  `not sent` with the reason on hover (`real_flight.carries_over` / `flight_note`). Only the aircraft and
  its controller cross over - `CA_`, `NLF_`, `MC_`, `MPC_`, the IMU and H-flow geometry, `EKF2_HGT_REF`
  and `EKF2_GPS_CTRL`. `SYS_HITL` flips to 0, the outputs are re-derived from the wiring, and the
  simulator's arming bypasses, RC, battery, GPS and failsafe settings stay behind, because a real
  aircraft's own are the ones that stop it hurting someone.
* **Indoors is a property of the design** (`design.indoor`, the checkbox under the H-flow). There is no GPS
  indoors and there never will be, so the export stops the estimator waiting for one: `EKF2_GPS_CTRL = 0`,
  the rangefinder becomes the height reference (`EKF2_HGT_REF = 2`) and range fusion goes from conditional
  to always (`EKF2_RNG_CTRL = 2`), with the barometer and the IMU fused underneath as PX4 does anyway.
  Both cross over to a real flight, because they are facts about the aircraft rather than simulator
  conveniences. Without them a flow sensor can be tracking perfectly and the aircraft still has no
  position: `vehicle_local_position` never moves, and neither does anything drawn from it.
* **The parameters go out as a stream, not one at a time.** PX4 echoes every accepted PARAM_SET, so
  `PX4Link.set_params` sends them all at a pace the link can take and collects the echoes as they arrive;
  whatever has not echoed is sent again. A hundred parameters over a radio went from a minute of silence
  to about six seconds, and `GET /api/px4/push/progress` counts them while it happens, because a button
  that says "Pushing…" for a minute is indistinguishable from one that has hung. The retry round is also
  where a type is learned: PX4 refuses a PARAM_SET whose type does not match the parameter (12 is an int
  in Python and a float on the board, and the write is dropped in silence), so anything that did not echo
  is asked about and sent again with the type the board gave. What is left after that is reported by name
  with the reason - "this firmware has no parameter of that name" is the honest answer for `NLF_*` on a
  board without the ATLAS module.
* **A wiring is the whole wiring.** When the export carries motor outputs, the push also clears every
  other `PWM_MAIN_FUNC*` / `PWM_AUX_FUNC*` still pointing at one of this design's motors. Without that,
  each re-map leaves a ghost - a board that had motor 9 on AUX 3 and is told it is now on AUX 1 keeps
  both, PX4 drives both pins, and one motor test spins two fans (a real board here ended up with nine
  motors on thirteen outputs, motor 9 on three of them). Only motor numbers this design owns are taken
  back; a servo or anything else on the board is left alone. `real_flight.apply_verified` has always done
  this on the way to a real flight - the ordinary push did not, which is where the asymmetry came from.
* **Update PX4 leaves the board matching the simulator, not merely matching the design.** It writes the
  design's own parameters first, and then every other parameter that differs from the SITL reference - the
  snapshot of a full SITL parameter set taken whenever the app runs one - so a value this design never
  mentions still ends up where SITL has it. What is never copied from a laptop to an aircraft is the
  aircraft's own truth: calibration, RC, serial links, battery, safety checks, failsafes and the mode slots
  (`ConnectionManager.SYNC_EXCLUDE`). The second half needs the board's list read back, which the push does
  itself on a cable and leaves to Refresh over a radio, saying so in the result rather than quietly skipping
  it. There is no separate Board-vs-SITL section any more: one button, one promise.
* Real flight on a board (`px4/real_flight.py`) is the one conversion that writes to an aircraft that can
  hurt someone, so it is explicit and checked: only the aircraft's own settings cross over (CA_, NLF_, MC_,
  MPC_, the IMU and H-flow geometry), never the simulator's arming, RC, battery, GPS or failsafe
  overrides; `SYS_HITL` goes to 0; and every motor must land on exactly one port or nothing is written at
  all. `output_keys(n)` is only the *default* wiring - the first eight on MAIN, the rest on AUX. What a
  design actually uses is read from `all_output_keys()`, all sixteen, because the Rotors table lets a
  motor sit on any of them: reading only the first n made a motor on AUX 3 invisible and raised "assign
  each motor to exactly one port" at a design that had, which the push then served as a 500.
  **An ESC on an AUX output is driven at 400 Hz**: a design with any motor on AUX sets every
  `PWM_AUX_TIM*` to 400, because the rate belongs to the *timer group* and one timer drives several pins -
  which pin sits on which group is board wiring this cannot know offline, so all of them are set rather
  than one guessed. `PWM_MAIN_TIM*` is never written: MAIN carries the first eight motors on every
  design, and a wrong rate there is a board-level mistake to see rather than one to paper over. The count is the design's, not ATLAS's ten.
* Nothing the browser polls may answer with a 500. A design the real-flight set cannot be built for is a
  409 with a sentence in `error`, and every response - REST and websocket alike - goes out through
  `SafeJSON` / `json_safe`, because one NaN makes the encoder refuse the whole reply and all the user sees
  is "Internal Server Error". Telemetry is cleaned before that, not after: `Simulator._reported` drops a
  non-finite vector instead of copying it into the vehicle, since PX4 streams its local position whether
  or not the estimate is valid and an invalid one is NaN.
* The simulation runs from the moment the app opens and keeps running until someone pauses it; the
  recorder collects the whole time.
* One switch over the 3D view decides what the whole thing is: **SITL** (PX4 here, flying the simulation),
  **HITL** (the board, flying the simulation) and **FLIGHT** — the aircraft on its own sensors with the app
  only listening. Pressing one is a statement about what the aircraft *is*, and `ConnectionManager.set_mode`
  makes it true however the board was left: it opens the link, waits for a real heartbeat (`hil_enabled` is
  read off the heartbeat and is false on a fresh link, so asking before one arrives says "already on its own
  sensors" about a board sitting in HITL), and if the board is the other way round it writes SYS_HITL, saves
  it, reboots and waits for it to come back the right way up - then asks for the flight streams, or trims
  them for HITL. Nobody has to click "Set up for real flight" first. It runs in a background thread behind
  `POST/GET /api/connection/mode`, so the click never waits on a reboot; every wait is bounded and says what
  it was waiting for; pressing the same mode twice costs a heartbeat, and a newer press supersedes an older
  one by generation rather than queueing behind it. The switch shows the mode you asked for, the light shows
  the step while it works, and afterwards it is green only while that mode is actually doing what it says
  (SITL linked; HITL answering with SYS_HITL = 1; FLIGHT with position and attitude arriving —
  `telemetry_age` in the status). Red, with the reason on hover, otherwise.
  FLIGHT works over a telemetry radio, with nothing to set up first: pressing it listens on a link that is
  already carrying the aircraft, and otherwise rescans and opens the port itself, so a radio on USB needs
  no visit to the ports list. How fast it opens is the port's business, not a box's
  (`ConnectionManager.port_baud`): a Pixhawk takes `--baud`, anything else is a telemetry radio and gets
  57600, because 921600 at a radio returns not one byte and reads as "no aircraft". The Baud box is only an
  instruction once you have touched it, and otherwise shows the speed the link really runs at. A SiK radio
  also talks about *itself* over that link, and RADIO_STATUS is never taken for the aircraft
  (`PX4Link._own_radio`): it would light the link green over an empty sky and address every command to the
  radio's own system id. Its numbers are kept (`link.radio`: rssi, remrssi, noise, txbuf) and the switch
  says "the radio is up, the aircraft is not answering it" when that is what is happening. The streams it asks for are worked out from the
  link itself, because asking a 57600 radio for the cable's rates does not make them arrive, it makes them
  arrive late, which looks exactly like an aircraft that has stopped answering. On a cable it is the full
  `FLIGHT_RATES_HZ` (~7 kB/s). On anything narrower, `flight_rates()` divides a budget - `share` of
  light/balanced/full against baud/10 - by giving every stream a floor and spending what is left in
  `SPEND_ORDER`, position and attitude first, so 57600 carries a pose at 10 Hz (balanced) or 20 Hz (full)
  and 115200 carries 25 Hz. Three things make that work, and all three were once wrong:
  `SET_MESSAGE_INTERVAL` is acknowledged by PX4 and so is sent one at a time and repeated until it is
  (`set_message_interval`) - a burst of twenty into a radio is mostly dropped, and a stream that never
  heard keeps PX4's boot rate; the streams PX4 sends unasked and this app never reads are switched off by
  name, but only the ones actually arriving (`SILENCE_ON_RADIO`), which was a third of the traffic; and
  `trim_telemetry`, the HITL throttle, must never run on a listening link, because it switches off the
  quaternion and the local position - the two the 3D view and the graph are made of. The whole parameter
  list (~50 kB) is not downloaded automatically over a radio either: Refresh under Parameters asks for it.
  `link.status()["telemetry"]` reports asked against measured so the gap is visible, and the radio's own
  RADIO_STATUS (rssi, remrssi, txbuf) sits beside it. The aircraft has a cap of its own - PX4 sends no more
  than `MAV_<n>_RATE` on a port and drops the big messages first, so a board capped at 1200 B/s answers
  with the cheap streams and never sends the pose. `board_send_cap()` asks which instance can hear us
  (`mavlink status`, the block with "GCS heartbeat valid" - guessing MAV_0 reads another port's cap) and
  lowers the budget to it. One painter owns the mode pill (`modeShown`): when the status poll and the
  connection poll each decided for themselves, one knew the link was only listening and the other did not,
  and the pill flickered between HITL and FLIGHT twice a second.
* **Whether the app injects sensors is the board's answer, not the parameter cache's.** `PX4Link.listening`
  is the one test that blocks every HIL send, and it reads MAV_MODE_FLAG_HIL_ENABLED out of the heartbeat
  while one is fresh, falling back to `SYS_HITL` only when no heartbeat has arrived. It used to read the
  parameter first, and a parameter nobody has downloaded yet is not a "no": a board freshly connected in
  HITL was treated as a real flight, every HIL_SENSOR was dropped, and the board sat reporting no
  accelerometer, no gyro and no attitude - the simulator looking exactly like broken hardware.
  `block_hil` still overrules both, because FLIGHT and a board about to fly for real are decisions.
* A board still set to `SYS_HITL = 1` cannot be flown for real and cannot be watched in FLIGHT: PX4 shuts
  its own IMU drivers down in HITL and waits for HIL_SENSOR, which FLIGHT deliberately never sends. The
  result is a perfectly good link with no attitude and no position on it - `uorb top` shows sensor_accel
  and sensor_gyro at 0 Hz while a DroneCAN sensor like the H-flow keeps publishing - so the switch says
  exactly that instead of "not saying where it is". What a real flight records is what the aircraft's own
  sensors say, not a blank: OPTICAL_FLOW_RAD, DISTANCE_SENSOR and ALTITUDE are asked for like anything else
  and `Simulator._flight_extra` turns them into the same `flow_*` and `alt_baro` columns a simulated run
  fills, by the same arithmetic as sensors/models.py - so a column means one thing whichever way the flight
  happened. A reading older than two seconds is left out rather than repeated, and ten seconds of no flow
  at all says so in the log instead of plotting zeroes. The graph redraws at 20 Hz, which is the rate the
  recorder keeps; it used to redraw four times a second and turn every trace into a staircase. Update PX4 writes the design's parameters over that same link while
  skipping every build and flash step, which need the board on a cable. In FLIGHT nothing is injected (`PX4Link.listening` is the
  one test, and it blocks every HIL send), the loop stops integrating and `_follow_telemetry` puts the
  aircraft where LOCAL_POSITION_NED and the attitude say it is, on the wall clock, so the 3D view and the
  graph are the real flight. `request_flight_telemetry` asks the vehicle for those streams. `native.py`'s reconnect relaunches PX4 under the loop without changing
  whether it is running.
* The aircraft keeps its own record, and it is the better one: PX4 writes a .ulg to its card for every
  flight (`SDLOG_MODE` = 1 by default, so arm to disarm; 0 logs from boot, which is what catches a ground
  sequence) at a hundred to two hundred samples a second, against the ten or twenty a radio carries. The
  **Flight logs** section on the PX4 tab lists what is on the card with its date and size, downloads one
  over the same link (`px4/logs.py`: LOG_REQUEST_LIST / LOG_REQUEST_DATA, a window at a time, holes
  re-asked for rather than the transfer restarted, resumable, cancellable, telemetry paused for the
  duration, progress on the log's own line). How long it will take is measured, not assumed: the first
  download sets `measured_bps` and every estimate after re-bases on it, and before there is one the
  figure comes from what the telemetry is demonstrably carrying. On a SiK at 64 kbps air with ECC on that
  is about 2 kB/s, whatever the serial side says - half for ECC, half again for a half-duplex link and reads it into a run (`sim/ulog_import.py`): the recorder's own `COLUMNS`, resampled onto a
  50 Hz grid with the same zero-order hold the live recorder uses, so it joins the run list and the graph
  and the 3D view scrubs through it exactly like a flight watched live. A .ulg from the card is opened the
  same way without the wait. Two columns are honestly empty in an imported run - a real aircraft reports
  motor *outputs*, not the thrust they made, and its log knows nothing about the wind it flew through.
  Nothing invents a number that is not in the file.
* How fast the live view can be is the link's business, so it is asked of the link: on a cable the pose
  streams run at 50 Hz and the recorder keeps 50 Hz with them; over a radio both stay at 20, because
  sampling faster than the aircraft speaks writes the same number down five times. The cost of the fast
  one is the session buffer - 120k samples is 40 minutes at 50 Hz where it was 100 at 20.
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
  run. A zoomed window whose right edge is against the newest sample travels with the recording, keeping
  its width; scroll back off the end and it holds still until you scroll to the end again (`followTail`).
  A pan stops against either end rather than squashing the window, so scrolling towards the live end
  actually arrives there. The simulation stays paused for all of it — the 3D view is either the recording or the live
  aircraft, never half of each. Save run keeps it and starts the next; Reset throws it away and starts the
  next (`/api/runs/clear`); `/api/runs/current` is the recording in progress. The terminal beside the graph
  is the flight controller's and only its: `logLine` keeps the `[px4]` lines (PX4's console and the events
  it sends) and sends everything the app says to the browser console instead. A saved run carries that log
  for the span it covers, and selecting the run shows it.
* **A recorded rotation is PX4's, never the airframe's.** `roll`/`pitch`/`yaw`, the body rates and the
  accelerometer go into the recording in PX4's body frame - the structural frame pitched nose-up by
  `hover_pitch_deg` - in SITL, in HITL, in FLIGHT and in an imported .ulg alike, because in a real flight
  PX4's frame is the only one there is and a column has to mean one thing. An aircraft sitting at its
  design hover attitude therefore reads level, as PX4 reads it. The `q*` columns are the exception and
  stay structural: the 3D view draws the aircraft, and the aircraft is a physical object. FLIGHT turns the
  attitude the other way on the way in (`_follow_telemetry`), so what the aircraft reported is what the
  recording says and the model is still drawn where it really is.
* **The pilot's hands are in the recording.** `rc1`..`rc8` are the receiver's channels in microseconds
  (0 = no such channel), from the same RC_CHANNELS whoever is holding the sticks - the USB remote in
  SITL, the receiver on the aircraft in flight - so a replay shows the input beside what the aircraft did
  with it. Eight, because that is sticks plus the switches that matter. They are asked for on a radio
  too (`FLIGHT_RATES_HZ[65]`, floor 2 Hz, ~270 B/s of a 2.6 kB/s budget) and given up before the pose
  when the link is tighter than that; RC_CHANNELS used to be in `SILENCE_ON_RADIO` as traffic nothing
  reads, which left the Remote section saying "not detected - 0 channels" during the one flight you would
  want to watch the sticks in.
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
