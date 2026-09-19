# ATLAS_07D native PX4 takeoff and landing — simulation prototype

The module lifts the nose to the existing 25° hover frame while ground contact remains, then transfers control to PX4 position control and climbs 1 m above the starting position, holding heading. It now also provides rear-leg landing, controlled nose lowering and automatic motor shutdown; see [LANDING.md](LANDING.md). Geometry, mass, inertia, fan properties and landing feet are unchanged. This build is deliberately restricted to SITL; it is not a hardware-ready flight controller.

## Standard application

This is the native firmware source used by the main app. There is no separate demo UI or launcher.
After building below, run `python -m airframe_designer ui` from the repository root.
The default model is `airframes/atlas_09.json`; use `--airframe` to select another model.
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
