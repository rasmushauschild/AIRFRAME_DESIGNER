# V4 flight test, 22 October 2026: the yaw spin

Run: `results/runs/20261022-192443.json` ("ATLAS_V4_TUNE_T0b_alt25", 175 s), design
`airframes/atlas_v4_tune_t0b_alt25.json`, flown in FLIGHT over the radio. Armed in Takeoff mode
(PX4 auto takeoff to 2.5 m) at 139.1 s, lifted off at 140.5 s, killed at 150.5 s at about 0.9 m.
The card's .ulg was not downloaded (the board was off the link when this was committed).

## What happened

The yaw rate went from 12 to about 300 deg/s nose-right from liftoff on. PX4 answered the right way
on its card the whole time: M8 near full, M7 on its 10 % floor, the right-side rear fans above the
left. The spin followed the front pair's command with the wrong sign, about half a second later:

| t | M8 - M7 (card: nose-left) | yaw rate 0.5 s later |
|---|---|---|
| 144.0 s | 0.74 | 284 deg/s |
| 145.0 s | 0.28 | 157 deg/s |
| 146.0 s | 0.71 | 276 deg/s |
| 147.0 s | 0.29 | 154 deg/s |
| 149.5 s | ~0 | 51 deg/s |

A least-squares fit of yaw acceleration against the front and rear differentials gives a positive
front coefficient at every lag from 0 to 0.7 s (best at 0.4 s, R2 0.36) and a negative (correct)
rear one: the front pair yawed the aircraft opposite to the card, so PX4's correction fed the spin.

## Cause

M7 and M8 were swapped on the outputs after the signal leads were moved off the ground pins. The
right front ESC is on AUX 4, which carried motor 7 (`PWM_AUX_FUNC4 = 107`); the left front is on
MAIN 8, which carried motor 8.

## Fix (written to the board and saved, evening of 22 October 2026)

`PWM_MAIN_FUNC7 = 0`, `PWM_MAIN_FUNC8 = 107` (left front), `PWM_AUX_FUNC4 = 108` (right front),
held as overrides in the design so Update PX4 keeps them. AUX 4 already ran at 400 Hz with
1100-1900 / 1000 us. `PWM_AUX_FUNC1/2 = 109` remain (no motor 9 in this design). Take the wiring
overrides out before HITL.

To confirm before flying (props off): M7 spins the left front fan, M8 the right, seen from behind.

The fans' own reaction torque (all one direction; bench km 0.0134 against 0.006 on the card) grows
with thrust but is far smaller than the authority PX4 had, and does not explain 300 deg/s.
