# ATLAS_07D native PX4 takeoff and landing — simulation prototype

The module lifts the nose to the existing 25° hover frame while ground contact remains, then transfers control to PX4 position control and climbs 1 m above the starting position, holding heading. It now also provides rear-leg landing, controlled nose lowering and automatic motor shutdown; see [LANDING.md](LANDING.md). Geometry, mass, inertia, fan properties and landing feet are unchanged. This build is deliberately restricted to SITL; it is not a hardware-ready flight controller.

## Standard application

This is the native firmware source used by the main app. There is no separate demo UI or launcher.
After building below, run `python -m airframe_designer ui` from the repository root.
The default model is `airframes/atlas_09b.json`; use `--airframe` to select another model.
The app refuses to silently fall back to stock firmware when this build is missing.

## Use the running simulator

Open http://127.0.0.1:8081/, select **Flight**, then click **Takeoff** in the bottom bar. Start disarmed; the module performs normal arming itself. The old simulator-only nose-lift inputs are disabled in this session. The module holds position and heading after takeoff. From the native hover, choose **Land** to descend onto the rear legs, lower the nose and disarm. **Reset** remains available to restart the software demonstration. See [LANDING.md](LANDING.md) for landing parameters and validation.

`NLF_ALT` controls the requested climb above the initial estimated position; it is not `MIS_TAKEOFF_ALT`. At rest the model's CG is already about 0.21 m above the floor, so a 1 m climb results in approximately 1.2 m displayed altitude.

The app uses the standard repository UI and the native PX4 build in `firmware/atlas`. The original model file hash is recorded in `model_fingerprint.json`; `atlas_07d.native.json` differs in PX4 parameter overrides and the configured landed pitch.

To restart this session after closing it, first stop the existing port-8081 session, then run:

```sh
python -m airframe_designer ui
```

Run this command from the repository root. Select the port and instance with `--http` and `--px4-instance`. Do not launch a second copy on the same instance. Reset reapplies the saved demo parameters; persist intentional changes in `atlas_07d.native.json` before restarting.

## What was built

- `src/atlas_nose_lift/AtlasNoseLift.cpp`: native PX4 scheduled module using estimated attitude, angular velocity, local position and land detection. It does not receive simulator truth.
- `src/atlas_nose_lift/Model.hpp`: validates the fixed rotor positions, axes, reaction-torque coefficients, board offset and thrust mapping before activation.
- `src/atlas_nose_lift/params.yaml`: module parameters, disabled by default.
- `allocator_overlay/`: a small allocator addition preserving nose support during handover, without changing the original PX4 source tree. It uses pre-floor actuator feedback, accepts the floor only when armed in Offboard, expires it after 100 ms, and preserves NaN motor stops.
- `msg/`: two private uORB topics for support and raw allocation feedback.
- `atlas_07d-nose-lift.sitl.params`: reproducible SITL parameter set, including simulator defaults. Parameters alone cannot install the module.
- `test_sitl.py`, `analyze_results.py`, `validation.json`, `takeoff-validation.png`: repeatable tests and recorded results.

Sequence: prime Offboard heartbeat → normal arm → front motors 9/10 rotate the aircraft about the rear feet → settle within 2° for 1 s with ground detection → engage position control while maintaining adaptive front-motor support → fade support over 2 s once raw PX4 allocation supports the nose → climb and hold heading. The floor can decrease immediately when pitch braking is needed. A frozen floor was unstable and is not used.

The ground controller accounts for the fixed rear-pivot gravity moment, quadratic fan command/thrust mapping and asymmetric front-fan yaw moments. `NLF_RATE=5` is the desired ground pitch-rate limit, not a guarantee on actual angular rate; observed peaks are about 9°/s. Normal flight rate limits remain unchanged.

Module commands in the PX4 shell:

```text
atlas_nose_lift start
atlas_nose_lift takeoff
atlas_nose_lift status
atlas_nose_lift land
```

Ground timeout initiates a two-second motor ramp-down and normal disarm, then exits direct control. Mode changes relinquish control. Airborne faults request generic PX4 Land; that fallback is distinct from the new commanded native landing sequence and has not been validated as safe for this airframe. Do not treat it as a proven recovery mechanism.

## Configuration

| Parameter | Value | Purpose |
|---|---:|---|
| CA_METHOD | 0 | Pseudo-inverse allocation, used in the passing configuration |
| NLF_ENABLE | 1 | Explicitly enable this SITL prototype |
| NLF_RATE | 5 | Desired nose-lift rate, °/s |
| NLF_RAMP | 1.5 | Ground thrust ramp, s |
| NLF_DWELL | 1 | Settled duration before handover, s |
| NLF_ALT | 1 | Climb above initial position, m |
| NLF_TIMEOUT | 25 | Ground lift timeout, s |
| COM_DISARM_PRFLT | 40 | Allow time for the ground sequence |
| COM_DISARM_LAND | 60 | Backup timer leaves time for controlled nose lowering; module explicitly disarms after nose contact |
| NLF_LAND_V | 0.10 | Native landing descent speed, m/s |
| NLF_DOWN_R | 4 | Native nose-lowering rate, °/s |

The model's existing flight gains, 25° board offset, rotor geometry, CT=6.5 and KM=0.01 are retained. Flight rate limits are 220/220/200°/s. This is an experimental configuration validated as a whole, not evidence that each parameter is independently optimal.

## Validation and limits

Four recorded successful runs: JSBSim seeds 1–3 and Python physics seed 1, with sensor noise enabled and still air. The first three preceded a narrowly scoped abort-cleanup fix; seed 3 exercises the final build. All reached hover without a nose-lift abort. Ground contact was preserved throughout nose rotation, ending at 24.18–24.35°.

| Measurement | Observed range |
|---|---:|
| Nose-lift duration after arming | 9.02–9.09 s |
| Peak ground pitch rate | 8.54–9.28°/s |
| Peak takeoff pitch | 31.85–32.82° |
| Minimum pitch during handover/climb | 12.92–13.97° |
| Peak takeoff roll magnitude | 0.87–0.98° |
| Horizontal excursion from handover position | 1.19–1.36 m |
| Horizontal excursion from initial resting position | 1.42–1.56 m |
| Final 10 s height standard deviation | 1.0–1.9 cm |
| Final 10 s heading change magnitude | 0.07–0.47° |
| Final 10 s horizontal drift | 3.5–5.1 cm |

The test assertions require preserved contact during lift, handover within 2° of 25°, peak roll below 10°, peak pitch below 40°, heading drift below 5°, height standard deviation below 0.15 m and hover drift below 1 m. These are prototype regression thresholds, not an aircraft safety standard. The handover pitch swing and horizontal travel remain material limitations; this is not a vertical takeoff in a tightly constrained space.

The final timeout test (`NLF_RATE=1`, `NLF_TIMEOUT=10`) aborts at 23.032 s, disarms at 25.540 s, and continues to 51 s on the ground. Disabled-module and wrong-board-offset tests refuse activation and never arm. Their initial simulator settling drop is excluded from ground-contact evaluation. The Flight-tab button was also exercised end to end in the visible instance. A transient simulator-link reconnect occurred during that UI demonstration; PX4 recovered and reached hover. This does not substitute for transport-fault testing.

**Not validated:** hardware, wind/disturbances, actuator failure, estimator failure, physical fan/ESC dynamics, repeated cycles without Reset, changed terrain, or airborne abort recovery. An earlier generic-PX4 landing trial produced ground skidding. The new native landing sequence is tested separately; its results and limits are in LANDING.md. Hardware build guards intentionally prevent accidental use on a flight controller.

## Build and reproduce

PX4 base commit: `2a0e9109238bd0274608e3c25201be7c40becd0c` (local PX4 v1.18 beta tree). AIRFRAME_DESIGNER base: `01d33154e909822ab6666b40510b8495d8cba1fa` with pre-existing local modifications. The allocator overlay checks original source hashes at configure time and refuses an unreviewed upstream change.

**Update PX4** in the app rebuilds this firmware when any file under `src/`, `msg/` or `allocator_overlay/` is newer than the binary, relaunches PX4 SITL on it and then pushes the parameters (the button turns amber when a rebuild is pending; `GET /api/px4/firmware` reports it, `POST /api/px4/firmware/update` does it without pushing). The build lives in `firmware/atlas/build` inside this repository (git-ignored); `make firmware` from the project root
runs the two commands below with the project's `.venv` Python (which needs PX4's build packages:
`.venv/bin/pip install -r ~/PX4-Autopilot/Tools/setup/requirements.txt` once). Rebuild after editing the module,
then Reset (or restart the app) so the fresh binary is launched.

```sh
cmake -S "$PX4_SOURCE_DIR" -B build/px4_sitl_default -G Ninja \
  -DCONFIG=px4_sitl_default -DEXTERNAL_MODULES_LOCATION="$PWD" \
  "-DPYTHON_EXECUTABLE=$(command -v python)"
CCACHE_DIR=/tmp/atlas-nl-ccache cmake --build build/px4_sitl_default -j 6
NLF_TEST_INSTANCE=7 python \
  test_sitl.py repeat '{"CA_METHOD":0}' jsbsim 1
python analyze_results.py results/repeat.json
```

Use an unused SITL instance for tests. The implementation follows PX4's Offboard direct-actuator and position interfaces: https://docs.px4.io/main/en/flight_modes/offboard.html. The custom allocator floor is specific to this prototype and is not a stock PX4 facility.

## SITL connection recovery

The standard launcher restarts its owned PX4 process when reconnecting a fixed SITL instance, and Reset restarts both PX4 and simulator state. Restarting only EKF2 can leave the native module or a closed simulator socket alive. Parameter seeding uses the currently loaded model, so reconnecting does not revert model edits. Use `--airframe path/to/model.json` to select a saved model at startup. These changes apply to the app’s owned SITL instance, not hardware connections or other simulator instances.

### Editable nose lift target

`NLF_TARGET` sets the structural pitch for the ground nose-lift sequence, in degrees (default 25, no configured angle range). Change Takeoff pitch° in the Geometry tab, next to Hover pitch° and Landed pitch°, while connected and disarmed, then leave the field to save. The event log confirms PX4 acknowledgement. Save the airframe to retain its parameter override on disk. The firmware reads this setting when takeoff starts; changes do not retarget an active sequence.

The 25° board/hover-frame transformation remains fixed. This parameter changes the ground target and its settled-angle check, not the airborne hover attitude or landing rest angle. Other legacy simulator controls remain disabled. ATLAS_09 has different landing legs and is not covered by the ATLAS_07D simulation validation.

Editable-target regression: JSBSim seed 1 takeoff plus complete landing passed at 20°, 25°, and 30° on unchanged ATLAS_07D. See `target-validation.json`. This is a limited simulation check, not validation of every intermediate value or other aircraft.

The takeoff target has no minimum or maximum in the UI, parameter metadata, or firmware target validation. Finite-value validation, disarmed editing, existing attitude abort checks, and sequence timeouts remain. An accepted target is not a guarantee the sequence can reach it. Hover, landed, and takeoff pitch share one Geometry row, separate from inertia.

After removing the range restriction, a 35° target passed the existing takeoff and landing checks on ATLAS_07D with JSBSim seed 1 (`target-unlimited-validation.json`).

When SITL is connected and disarmed but arming readiness is missing or blocked, the UI requests `commander check` at most once every ten seconds until PX4 reports readiness. This reruns PX4 checks; readiness still comes from PX4's arming summary.

Reset recovery: owned SITL reconnects now stop and join the sensor loop, close the old link/process, reset the simulation clock and pacing state, then start fresh PX4 and resume sensors. Reset requests have an eight-second startup gate. This avoids restarting PX4 against an old multi-minute simulation timestamp. Model geometry and parameter overrides are preserved.

### Takeoff handover and landing shutdown repair

The ground-to-position transition now keeps publishing the last front-motor commands until the allocator provides its first output in the new mode. Previously the missing output stalled lockstep long enough to trigger the link's five-second silence warning.

The observed prolonged landing was a nose-lowering timeout (45 seconds), not the intended shutdown delay. Contact confirmation now uses height relative to rear-leg touchdown rather than the preflight height, permits five degrees of resting-pitch difference, and retains stopped pitch under downward demand, low vertical speed, and a one-second dwell. Saved position references follow EKF coordinate resets. Once contact is confirmed, motor commands ramp to zero over two seconds and normal disarm is requested at 2.5 seconds. Diagnostic lines report pitch, rest angle, rate, demand, velocity, height relative to takeoff, and contact condition during lowering.

The original live timeout did not include per-condition diagnostics, so its exact failing condition was not established. These changes address identified weaknesses; the added diagnostics distinguish any recurrence.

### Takeoff, hover, and automatic landed pitch

Takeoff pitch uses `NLF_TARGET`. Hover pitch sets `SENS_BOARD_Y_OFF` and the corresponding rotated allocator geometry; the native controller now transforms attitudes and rates using that configured frame. After changing hover pitch, apply Update PX4 and Reset before flying. Landed pitch uses `NLF_LAND_ANG` and sets the lowering trajectory endpoint. All three are fixed for each running sequence. Takeoff and hover are edited while disarmed. The demo calculates Landed pitch automatically from enabled leg feet whenever a model loads or geometry changes, and synchronizes it to PX4 before Takeoff. The Geometry field is read-only.

Ground contact takes priority over a requested angle below the ground. A requested landed pitch more than 3° above the measured resting posture is rejected before arming, with both angles in the error message, because it would otherwise stop lowering before the nose reaches the ground. The original ATLAS_07D legs settle around −12.75°; earlier regression runs used −13°. The demo now derives the unloaded support-plane angle from leg positions, tilt, cant, lengths, and foot radii; spring compression can cause a small difference in the actual resting posture. At least three non-collinear enabled feet sharing a plane within 1 cm are required. Actual contact detection still governs shutdown. These settings do not make arbitrary rotor layouts compatible with the controller.

Three-angle regression: ATLAS_07D passed at takeoff/hover/landed settings 20/24/−13 and 30/27/−15 degrees. The captured ATLAS_09 completed both sequences and shut down after nose contact, with no link loss, but briefly pitched upward at 8.41°/s during landing handover, above the existing 8°/s regression threshold. This current-model run is not reported as passing the full landing checks.

## Current-model ground control and automatic updates

The standard app exports `NLF_MASS`, `NLF_GX`, `NLF_GZ`, `NLF_MOM`, and `NLF_W9` from the current mass, CG, rear-foot pivot, and front-fan forces. `NLF_CFG_OK` rejects unsupported layouts. The controller supports the ten-motor, two-front-fan, three-leg arrangement, not arbitrary aircraft. Native takeoff uses a minimum three-second thrust ramp.

While disarmed, model edits affecting exported parameters are debounced for one second, then the owned SITL process and simulation reset with freshly seeded parameters. Edits while armed are deferred until disarm. No firmware rebuild is needed for these model parameters.

Validation: the captured current model completed takeoff, hover, and landing with motors off using the three-second ramp; the earlier 0.2-second ramp aborted on the attitude envelope. The successful run still exceeded the strict landing drift threshold (7.1 cm), so it is not a full flight-regression pass. The original model also completed both sequences.


## Flashing the same firmware to the Pixhawk

The Connect tab builds this firmware for the plugged-in board (`px4_fmu-v6x`, `multicopter` variant with the HIL
output driver) from the same PX4 tree and the same module sources as the SITL, and flashes it over USB
(`ATLAS_MODULES=$PWD/firmware/atlas scripts/build_hitl_firmware.sh px4_fmu-v6x upload` does the same by hand). On a
flight controller the module refuses to start until `NLF_HW_OK = 1`, a deliberate interlock set from the Connect
tab ("Allow on hardware"). `std::atomic` is not available on NuttX, so the module uses `px4::atomic`.

Every flash (and every "Snapshot board now") is archived: image, all board parameters (QGC `.params`), the
exported parameters, the airframe and these module sources go to https://github.com/rasmushauschild/PX4_FIRMWARE
(clone in `~/.airframe_designer/px4_firmware_archive`, override with `AFD_FIRMWARE_ARCHIVE`). The Versions tab
lists them and can flash any version back and restore its parameters (`airframe_designer/px4/archive.py`).


### Staged physical-RC takeoff (opt-in, awaiting physical validation)

`NLF_RC_THR=1` selects a two-press sequence. The Flight-tab checkbox saves this choice;
Update PX4 builds/flashes and applies it separately. Start with the calibrated physical
RC throttle at minimum. The first Takeoff press arms normally and enters nose lift.
Raising throttle permits more front-fan command (upward cap slew 20%/s); the pitch
controller still stops at the target angle. After the log reports `nose ready`, a
fresh second Takeoff press authorizes the automatic climb to 1 m above the original
takeoff position. Early presses are not queued. The ground stage times out after
55 seconds and requests controlled lowering without exceeding either fan’s command at abort entry. The exporter sets COM_DISARM_LAND to
at least 120 seconds so normal ground auto-disarm does not interrupt that lowering.

Holding throttle below 5% for 0.4 seconds after raising it, or losing fresh physical
RC for 300 ms requests controlled
nose lowering (ground stage) or landing (after liftoff authorization). Throttle is
not a direct motor-power control in the airborne stage. A separate kill switch still
stops the motors. Attitude-stick takeover and PX4 failsafes retain authority. A narrow,
freshness-limited manual-control overlay prevents throttle motion alone from triggering
Position-mode takeover while this staged sequence owns throttle.

The target is an estimated relative height, not a guaranteed one-metre physical ceiling.
This change must be validated on a restrained test setup before free flight. Building
it does not enable it on the connected board, and no physical validation is implied.


### GPS-free RC development prototype (SITL and isolated HITL)

`NLF_MANUAL=1` selects a separate controller when `atlas_nose_lift` starts.
Default is zero. Hardware runs this mode only with `SYS_HITL=1` and a fresh active HIL state, checked continuously. `NLF_HW_OK` cannot bypass this guard.
HITL requires all motor power physically disconnected. Real flight is unsupported. No geometry changes are required.

In an isolated SITL instance, stop the module, set `NLF_MANUAL=1`, and restart
it. Supply calibrated RC_CHANNELS/input_rc with throttle, roll, pitch and yaw
mapping; the same shell/RC takeoff triggers are used. Do not use the connected
hardware session for development. Normal PX4 arming checks remain active.

The first request requires disarmed, landed, fresh finite attitude/rates, valid vertical estimates and
fresh RC with throttle below 5%. The module supplies a zero-power stream,
requests Offboard and normal arming, and lifts the nose under an RC power cap.
A fresh second request after one second of settled pitch switches to attitude
setpoints. Earlier requests are discarded, not queued. Thrust increases are
slew limited; reductions follow the pilot immediately. Roll/pitch command up
to 10 degrees in the configured hover frame, yaw up to 30 degrees/second.
In this simulation-only manual path, allocated thrust fractions are converted
into speed fractions before publication, matching the quadratic ATLAS motor model.
Direct nose-lift commands and support floors are already speed fractions and are
not converted again. `THR_MDL_FAC` remains zero. This does not provide position hold.
The controller consumes attitude sticks itself in this mode; an explicit RC
mode switch still releases control. Horizontal position is not read by this path. The vertical estimate uses the configured EKF height sources.

Stage two now uses PX4 PositionControl for a 2 m climb relative to the initial
resting height, ramped at 0.35 m/s with a 0.4 m/s climb-speed limit. Only estimated
vertical position, velocity and acceleration are used. The RC throttle remains
a power ceiling, including immediate reductions; lowering it can prevent the
climb or override height hold. `NLF_M_HOV` supplies hover-thrust feedforward
(default 0.35 for the current model). Fresh valid vertical estimates are required;
invalid estimates release the offboard stream to PX4's configured failsafe.
Height resets shift the reference, and velocity resets clear the integral.
Barometer/IMU height is relative and can drift; it is not a floor-range measurement.
There is no horizontal position hold, automatic contact detection, or
automatic landing. The Land command reports this limitation rather than
invoking the GPS-dependent landing controller. Landing and nose lowering
must be flown manually. RC/attitude loss, kill, disarm, mode change or module
stop release the stream and leave the configured PX4 failsafe in charge;
this is not a validated GPS-free recovery procedure. Verify those failsafes
in simulation before considering a hardware design.

Validation includes state/reference tests, SITL compilation and a focused native PositionControl vertical-model test. The altitude-hold change has not yet had a full aircraft HITL flight test. The
nose-to-flight allocation transition, landing, RC-loss recovery, estimator
resets and sustained drift have NOT received flight-simulation or physical
validation. The prototype is unavailable in real-flight mode. HITL is not physical-flight validation.

Vertical-controller test after `make firmware`: `.venv/bin/python tests/firmware/run_vertical_test.py`.

State tests: `c++ -std=c++17 tests/firmware/test_manual_gate.cpp -o /tmp/test_manual_gate && /tmp/test_manual_gate`


### Pilot handover: nose lift, then the pilot lifts off (no GPS)

`NLF_PILOT` (Flight tab, "Handover after nose lift") selects who flies after the nose is up: 0 = PX4 position hold as
above, 1 = the pilot in Stabilized, 2 = the pilot in Altitude mode. Nothing in the pilot path reads a horizontal
position, so it works indoors with the GPS switched off (`EKF2_GPS_CTRL 0`, `SYS_HAS_GPS 0`, `EKF2_HGT_REF 0`; the
Flight tab's "No GPS (indoor)" box sets these). The module reads the pilot's throttle from `manual_control_setpoint`,
so an RC receiver and a MAVLink joystick (the designer's USB remote in SITL) behave the same.

The rule behind the design: PX4's attitude loop never runs while the aircraft is on its legs. On the legs, 35 deg
below PX4's level at low collective, the allocator saturates a rear motor to full thrust and the simulated aircraft
tipped over backwards, both at takeoff and at touchdown. So the module flies every ground phase itself through
Offboard direct actuators, and PX4 only ever gets the aircraft in the air.

Sequence. Throttle at minimum, Takeoff: PX4 arms in Offboard, the module lifts the nose all the way to PX4's level
(the hover pitch, not `NLF_TARGET`) with the front fans and reports `nose ready`. Second Takeoff press (throttle still
at minimum): the pilot now has the throttle. The stick maps to a lift fraction (bottom 0, centre 1 = hover, top
`NLF_LMAX`, default 1.4), the eight rear motors follow the model's zero-torque hover pattern (`NLF_U1..8`, exported
from the hover check) scaled by it, and the front fans keep the nose at level with the ground model blended from the
ground hold to their hover share (`NLF_UF`) as the rear motors take the weight. Raise the throttle slowly and the
feet unload evenly. Once the aircraft has risen 0.15 m for 0.2 s the module asks PX4 for the pilot's mode and hands
over; PX4's Stabilized curve also puts hover at the centre of the stick (`MPC_THR_HOVER`, preset by the exporter), so
the throttle keeps its meaning. From then on PX4 flies and the module only watches.

Landing: hover low and press Land, or simply descend: once the aircraft has been above 0.6 m, being below 0.4 m of
the takeoff spot while not climbing with the throttle at or below the centre makes the module take the motors back
(Offboard direct actuators, same throttle mapping, continuous). Lower the throttle further and the rear feet touch; a
stalled vertical motion (|vz| < 0.15 m/s for 0.4 s) with the throttle below 35 % is the contact, and the usual nose
lowering, motor ramp-down and disarm follow. Climbing away above 0.6 m during a landing hands the aircraft back to
PX4. Land before liftoff, a lost remote, leaving Offboard on the ground, or no liftoff within 45 s all lower the nose
again. A failure while off the ground under the module's commands hands PX4 the aircraft; the module never requests
an automatic landing here.

The exporter sets `COM_DISARM_LAND` and `COM_DISARM_PRFLT` to at least 120 s (PX4 reads the wait as "landed, not
taking off"), forces `MC_AIRMODE 0` and presets `MPC_THR_HOVER`. `NLF_RC_THR` is ignored in this mode. While the
module owns the throttle, throttle motion is excluded from PX4's stick override; keep the attitude sticks still on
the ground. Unit test: `tests/firmware/test_pilot_handover.cpp`; SITL run with a scripted pilot on a joystick:
`NLF_PILOT_TEST=1 ... test_sitl.py pilot '{"NLF_PILOT":1,"EKF2_GPS_CTRL":0,"SYS_HAS_GPS":0,"EKF2_HGT_REF":0}'`.
