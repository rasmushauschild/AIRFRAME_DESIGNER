# ATLAS_07D native PX4 landing sequence

This extends `atlas_nose_lift` with a `land` command. The same native PX4 module owns the entire takeoff, hover, rear-leg touchdown, nose-lowering and disarm sequence. It uses PX4 estimates, not simulator truth. The aircraft geometry, mass, inertia, fan model and landing feet are unchanged.

## Use

In the simulator at http://127.0.0.1:8081/, select **Flight**. Use **Takeoff**, wait for `hover achieved with heading held`, then choose **Land**. The aircraft descends, settles onto its rear feet, lowers its nose and switches off all ten motors. It finishes disarmed. The simulator's Reset button is still available.

The equivalent PX4 shell command is:

```text
atlas_nose_lift land
```

Landing requires the module's active hover from a preceding native takeoff. This gives it a recorded ground height and resting pitch. A standalone module start cannot land an aircraft whose ground reference it has never established. Requests in other phases are rejected. Switching flight modes relinquishes control.

## Sequence

1. Hold horizontal position and heading while lowering the height target at 0.10 m/s.
2. Infer rear-foot support from stopped vertical motion, continuing descent demand, proximity to the recorded rear-contact plane, low roll and a 0.20 s dwell. This is estimator-based contact inference, not a physical contact sensor.
3. Switch to direct front-motor control, stop the eight rear motors, and settle on the rear feet for one second. An allocator ownership check prevents a delayed flight-controller sample from overwriting direct ground control during the switch.
4. Lower the pitch target at 4°/s. Motors 9 and 10 provide the support needed to lower the nose gradually against gravity. The target extends slightly beyond the recorded resting pitch so nose-foot support can be detected from stopped rotation despite a continuing nose-down request.
5. Require a one-second contact dwell near the recorded ground posture. Then ramp the remaining motor commands to zero over two seconds, request normal disarm, and leave direct control. No forced disarm is used by this sequence.

The natural CG movement while pitching about the rear legs is distinct from the feet sliding. The validation report measures both.

## Parameters

| Parameter | Demo value | Meaning |
|---|---:|---|
| NLF_LAND_V | 0.10 m/s | Requested descent speed; actual touchdown speed is measured in the tests |
| NLF_DOWN_R | 4°/s | Requested nose-lowering rate; short handover transients can exceed this |
| COM_DISARM_LAND | 60 s | Backup landed auto-disarm timeout |

The longer backup timeout prevents PX4's ordinary landed timer from cutting the front motors while they are still supporting the raised nose. The module disarms explicitly after nose contact; it does not normally wait 60 seconds. It rejects landing if the configured backup delay is shorter than the pitch-down sequence needs. Auto-disarm is retained, not disabled. These values are included in the updated SITL parameter file and demo model.

## Scope

This remains a SITL-only prototype for the fixed ATLAS_07D on the same flat ground plane used at takeoff, in still air. The firmware build guard remains in place. Hardware landing, changed terrain elevation or slope, contact-sensor integration, wind, actuator faults, estimator failures and airborne fault recovery are not validated. The initial takeoff's pitch and horizontal transient remain as documented in README.md.

The existing generic PX4 Land behavior is separate from this sequence. Earlier failed generic-landing and development runs are not evidence for the final native landing behavior. Use the final validation files listed below.

## Final validation

All four final-build runs passed the landing checks and the existing takeoff regression checks: JSBSim sensor-noise seeds 1, 2 and 3, plus Python physics seed 1. Sensor noise was enabled, with still air and unchanged geometry.

| Measurement | JSBSim, 3 runs | Python, 1 run |
|---|---:|---:|
| Rear-leg touchdown descent speed | 0.107–0.130 m/s | 0.083 m/s |
| Nose clearance at rear touchdown | 0.528–0.533 m | 0.530 m |
| Peak pitch rate during ground lowering, including handover | 4.97–5.71°/s | 4.90°/s |
| Rear-support movement after touchdown | 0.8–0.9 cm | 17.7 cm |
| Peak roll magnitude during landing | below 0.5° | below 0.5° |
| Landing request to normal disarm | 22.68–22.92 s | 23.60 s |
| Motors after completion | all ten at zero | all ten at zero |

In every run, both rear feet stayed in contact throughout nose lowering, and the nose foot contacted the ground before the motor ramp-down started. The final resting pitch was approximately −12.75°, matching the fixed model's loaded landing-gear posture. The aircraft ended disarmed, with no landing abort. The Python contact model still permits noticeably more rear-foot creep than JSBSim; this is a known limitation, not a stationary-foot result.

The explicit regression limits include rear touchdown below 0.15 m/s, peak lowering rate below 8°/s, roll below 3°, rear-support movement below 0.20 m, continuous rear support, nose contact before shutdown/disarm, and all motors off afterward. These are simulation regression bounds, not hardware safety certification.

Guard tests also passed: a disarmed landing request never armed any motors, and a five-second backup auto-disarm setting was rejected while the aircraft remained in hover. The visible Flight-tab flow was exercised through touchdown and normal disarm. Afterward the simulator was reset to load the final build and clear ground-contact estimator warnings. Earlier development runs are retained locally but excluded from the final validation bundle.

Evidence: `landing-validation.json`, `landing-takeoff-regression.json`, `landing-validation.png`, and `results/landing_contact*.json`. `analyze_landing.py` checks recorded simulation truth only; none of these truth measurements are fed into the controller. The updated ZIP includes the source, model and parameter files, validation scripts, final runs and guard-test records.

Reproduce one full sequence from this directory, using an unused SITL instance:

```sh
NLF_TEST_LANDING=1 NLF_TEST_INSTANCE=7 \
  python test_sitl.py landing_repeat \
  '{"CA_METHOD":0,"COM_DISARM_LAND":60,"NLF_DOWN_R":4}' jsbsim 1
python analyze_landing.py results/landing_repeat.json
```

Latest contact/handover revision: see README and `handover-landing-recovery.json`. Contact height is now relative to rear touchdown, saved coordinates follow EKF resets, and resting-pitch tolerance is 5°. Three regression runs (ATLAS_09 seeds 1/3 and ATLAS_07D seed 1) had no simulator-link loss or timeout; motors reached zero 3.02 seconds after nose contact. Earlier evidence describes the previous controller revision.
