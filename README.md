# AIRFRAME_DESIGNER

The current ATLAS simulator session (including the ATLAS_09 model, native PX4 takeoff/landing, and automatic landed pitch) is packaged in [experiments/atlas_nose_lift](experiments/atlas_nose_lift/README.md). Follow its build and launch instructions to run this version on port 8081.

A real-time, physics-based aircraft simulator with **PX4 in the loop**, for designing unconventional airframes:
wings and fins with their own dimensions, angles and coefficients; propellers, ducted fans and jetfoils with
arbitrary thrust vectors; an explicit centre of gravity; individual landing legs. Fly it live with a USB remote
against PX4 SITL or a real Pixhawk (HITL), or run it **headless and unthrottled** so an optimiser (or an AI agent)
can run many simulations per minute and search any set of parameters for a desired outcome.

It grew out of [AIRFRAME_SIMULATOR](https://github.com/rasmushauschild/AIRFRAME_SIMULATOR) and keeps its core
idea: rotor positions and thrust axes are first-class, editable in 3D, and exported 1:1 to PX4's control
allocation (`CA_ROTORn_*`), so PX4 flies exactly the geometry you drew.

* **CAD masses**: import a STEP file in the Geometry tab; every solid becomes a body you give a mass, the CG and
  inertia follow (OpenCascade measures volume, centroid and inertia), bodies can be dragged along their axes in 3D.

## What is new compared to AIRFRAME_SIMULATOR

| area | AIRFRAME_SIMULATOR | AIRFRAME_DESIGNER |
|---|---|---|
| wings | one delta (area, span) | any number of surfaces: span, root/tip chord, sweep, dihedral, incidence, twist, single or mirrored (fins, tailplanes), linear or delta (Polhamus) coefficients; **strip theory** so dihedral effect, roll/pitch damping and weathercock stability emerge |
| CG | origin | explicit `mass.cg`, or computed from component masses; PX4 export and physics follow it |
| legs | 4 generated feet | individual legs: attachment, length, tilt, cant, foot radius, stiffness, damping, friction |
| rotors | props, ducted fans, jetfoils | same, plus enable/disable and health scaling (motor-out tests) |
| running | interactive only | interactive **and** headless: scripted scenarios, metrics, parallel batches, optimisation studies, CLI + Python API |
| PX4 parameters | pushed over MAVLink | seeded into PX4's parameter file before boot (batch), pushed live (interactive) |
| code | one package | segments: geometry, aero, dynamics, sensors, px4, sim, analysis, batch, server |

Speed: about **5-7x real time per worker** for a 10-rotor ducted-fan airframe with a wing (M2 Max, PX4 lockstep at
250 Hz). A 21-second hover test takes about 4.5 s of wall time including PX4 boot; four workers finished eight such
runs in 13 s (about 40 runs per minute), six workers give roughly 55.

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"
```

PX4 SITL build (once), default location `~/PX4-Autopilot`:

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot && make px4_sitl_default     # cmake, ninja, ccache from Homebrew are enough on macOS
```

## Interactive: design and fly

```bash
.venv/bin/python -m airframe_designer ui                       # PX4 SITL started for you, UI on http://127.0.0.1:8080
.venv/bin/python -m airframe_designer ui --mode auto           # Pixhawk over USB if plugged in (HITL), else SITL
.venv/bin/python -m airframe_designer ui --airframe airframes/plane_quad.json --speed 0
.venv/bin/python -m airframe_designer mcp                      # MCP server (stdio) so ChatGPT/Claude/Cursor/Codex can edit + simulate
.venv/bin/python -m airframe_designer mcp --http 8765          # same over HTTP at /mcp (tunnel it for ChatGPT); see docs/AI_GUIDE.md
```

* **Geometry** tab: mass & CG, rotors (drag in 3D: `W` move, `E` rotate the thrust axis), motors, wings table,
  legs table (or generate four from height/spread/landed pitch), body/drag. Every edit is pushed to the physics
  immediately; **Update PX4** writes the geometry to the flight controller.
* **PX4** tab: edited parameters (saved with the airframe), the full parameter list with descriptions, `.params` export.
* **Optimize** tab: static hover/cruise analysis and the fast geometric optimiser (jet angles, hover pitch).
* **Flight** tab: modes, takeoff/land, sim speed, wind, home, sensor noise, per-motor override, and the **nose lift**
  ground sequence for aircraft that park nose-down but hover nose-up: the chosen (front) motors raise the nose to
  the hover pitch while PX4 is still disarmed, then keep holding it while PX4 arms and spools up. Enable "Use for
  Takeoff" and the Takeoff button runs it first. PX4 needs no changes.
* **Connect** tab: SITL/HITL switching, the HITL checklist, and the **USB remote** (RadioMaster over Web Serial,
  EdgeTX HID joystick, or any gamepad; Chrome/Edge) streaming `MANUAL_CONTROL` to PX4 at 50 Hz.
* **Batch** tab: run any scenario headless on the current design in a private PX4 instance while you keep flying;
  run a study and apply its best design.

HITL notes (real Pixhawk, `pwm_out_sim` firmware, `SYS_HITL`, estimator resets, telemetry throttling) are unchanged
from AIRFRAME_SIMULATOR; see its README for the board-side details.

## Headless: simulate, batch, optimise

```bash
.venv/bin/python -m airframe_designer scenarios                                   # bundled flight scripts
.venv/bin/python -m airframe_designer run --airframe airframes/atlas_08.json --scenario hover --out r.json
.venv/bin/python -m airframe_designer run --airframe airframes/atlas_08.json --scenario cruise \
        --set "rotors[0:8].tilt_deg=30" --set "mass.cg[0]=0.03" --out r.json
.venv/bin/python -m airframe_designer batch --tasks tasks.json --workers 6 --out results.jsonl
.venv/bin/python -m airframe_designer study --spec studies/atlas08_hover_tilt.json --workers 6
.venv/bin/python -m airframe_designer analyse --airframe airframes/atlas_08.json --speed-kmh 50   # static, no PX4
.venv/bin/python -m airframe_designer paths --airframe airframes/atlas_08.json                    # every variable
```

A **scenario** is a JSON list of phases (`wait_ready`, `takeoff`, `hold`, `offboard_velocity`, `offboard_position`,
`manual`, `wind`, `motor_failure`, `param`, `land`, ...) that drives PX4 over MAVLink in simulation time. The
result is JSON with per-phase **metrics** (altitude/position/attitude statistics, rates, tilt, motor utilisation
and saturation, momentum-theory power and energy, wing lift share, tracking errors, crash and touchdown
detection). A **study** names an airframe, a scenario, variable paths with ranges, an objective expression over
the metrics, constraints and an algorithm (random / grid / CMA-ES / Nelder-Mead); it writes every trial to
`results/<name>/trials.jsonl` and the best design to `best_airframe.json`.

[docs/AI_GUIDE.md](docs/AI_GUIDE.md) is the full contract for scripts and AI agents;
[docs/SCHEMA.md](docs/SCHEMA.md) documents the airframe file and the parameter-path syntax.

## Two physics engines

The default is the project's own rigid body. Any run can instead use **JSBSim** (`--physics jsbsim`, or the Physics
selector on the Flight tab): the same airframe is exported as a JSBSim aircraft (mass, feet, one force per rotor,
wing/body coefficient tables) so the two engines can be compared on the same flight with
`airframe-designer compare` or the Batch tab's Compare physics button. See docs/AI_GUIDE.md.

## Physics

* 6-DOF rigid body about the CG (inertia tensor with products), semi-implicit Euler at 500-1000 Hz, quaternion attitude.
* Rotors: first-order spool-up, thrust ∝ ω^n along the axis, PX4-consistent reaction torque (`-km·T·axis`), jetfoil
  turning loss, momentum (ram) drag of ducted inlets. Momentum-theory power for energy metrics.
* Wings: strip theory; per strip the local relative wind (including rotation) gives an angle of attack, section
  CL/CD from a linear+induced-drag model, the Polhamus delta model, or **real airfoil polars** (model `polar`):
  NACA 4/5-digit sections generated analytically, any UIUC-database airfoil downloaded on demand, or your own
  `airfoils/<name>.dat`; the polar (CL, CD, CM over angle of attack and Reynolds number) is computed by XFOIL when
  installed, otherwise by NeuralFoil (a neural surrogate trained on XFOIL), cached in `airfoils/polars/`, and looked
  up in real time at each strip's angle of attack and Reynolds number, root and tip sections blended along the
  span, with a finite-wing induced-angle correction. Flat-plate blend past the stall, section pitching moment.
* Body: quadratic drag per axis at a drag centre, quadratic rotational damping.
* Legs: spring-damper-friction contact at each foot, per-leg constants; the vehicle rests, tips and bounces
  according to its actual gear.
* Sensors: IMU (specific force from the actual motion), magnetometer (dipole field), barometer (ISA), GPS with
  reported accuracy; lockstep with PX4's `simulator_mavlink` (HIL_SENSOR / HIL_GPS / HIL_ACTUATOR_CONTROLS).

The model is deliberately the one PX4's allocator assumes (no rotor inflow, no blade flapping, no ground effect),
so what you learn is about the geometry, the CG and the controller, not about modelling artefacts.

## Layout

```
airframe_designer/   geometry · aero · dynamics · sensors · px4 · sim · analysis · batch · server  (+ cli.py, app.py)
ui/                  three.js editor (vanilla JS, no build)
airframes/           schema-2 airframes (ATLAS series, quad_x, hex_x, plane_quad); schema-1 files load and migrate
scenarios/  studies/ flight scripts and optimisation specs
tests/               pytest (fast unit tests + one PX4 integration test)
docs/                SCHEMA.md, AI_GUIDE.md
```

## Conventions (identical to PX4)

Body frame FRD; a rotor pointing up has axis `(0, 0, -1)`; rotor *i* is PX4 Motor *i+1* / `CA_ROTOR{i}_*`;
`km > 0` spins CCW seen from above and torque on the body is `-km·T·axis`. Airframes that hover nose-up set
`hover_pitch_deg`: the geometry is exported rotated into that frame and `SENS_BOARD_Y_OFF` tells PX4 its IMU sits
in the structural frame.

## Experimental ATLAS PX4 ground sequences

See [ATLAS nose-lift takeoff and landing](experiments/atlas_nose_lift/README.md) for the SITL-only native PX4 module, simulator controls, build instructions, and validation summaries.
