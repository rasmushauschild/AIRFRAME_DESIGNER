# V4 hover pitch and park angle, CA_ROTOR card fixed (2026-10-22)

Firmware: native ATLAS SITL at PX4 2a0e9109 (the V3 Oct 19 build). Parameters: V3's flown set, the Oct 21 hover-20
CA_ROTOR card (never changed), MC_AIRMODE 0 (PX4 default). Airframe: airframe_V4_fusion_v8_full.json (Fusion
SMALL_SCALE_V4.2 v8 skeleton, 13.4 kg weighed set). One parameter at a time.

## Static model (analysis/authority_sweep.py)

* Natural hover pitch with this card and mass: 18.05 deg (all motors inside 0..1; rear 0.61-0.64, front 0.43-0.48).
* Hover pitch (SENS_BOARD_Y_OFF) with the card fixed only sets PX4's lean (natural - hover) and turns PX4's roll/yaw
  axes against the card's. Pitch authority 19.5 / 21.5 rad/s^2 at any hover pitch; roll 9.4 -> 8.3 and yaw 3.1 -> 3.6
  rad/s^2 from 10 to 30 deg. Yaw is the weakest axis everywhere. Best choice: 18 deg (zero lean).
* Park: gravity about the rear feet against the most nose-up moment PX4 can allocate (49.8 N m):
  net 1.5 / 2.5 / 3.5 / 5.7 / 8.2 rad/s^2 at park -20 / -15 / -10 / 0 / +10 deg.

## SITL (analysis/fly_takeoff.py, hover_stats.py; Takeoff mode, arm, 40 s)

| Hover | Park | Front leg | Result | Lift-off | Hover (last 20 s) |
|---|---|---|---|---|---|
| 18 | -20 | 0.068 m | stays on legs (44 N peak) | - | - |
| 18 | -15 | 0.161 m | stays on legs (67 N peak) | - | - |
| 18 | -10 | 0.249 m | flies | 4.0 s | pitch 18.07 +-0.33, roll 1.21 +-0.23, drift 0.14 m |
| 18 | 0 | 0.420 m | flies | 3.9 s | pitch 18.04 +-0.33, roll 1.20 +-0.27, drift 0.11 m |
| 18 | +10 | 0.591 m | flies | 4.0 s | pitch 18.05 +-0.30, roll 1.22 +-0.23, drift 0.18 m |

Recommendation: hover pitch 18 deg; park -10 deg or higher (threshold between -15 and -10 in this SITL), -5..0 deg
for margin. Above -10 the take-off does not get faster; it only needs a longer front leg.
Not covered: per-axis authority was not flown as doublets (static model only); one flight per point, nominal mass
and thrust (no Monte Carlo spread); airmode stays off.

## Sensor positions corrected (2026-10-22, later)

The PX4 and H-flow had been placed at V3's offsets from the CG (the as-built file had neither). They are now where
SMALL_SCALE_V4.2 v8 has them: PX4 board ('PX4 - beside relocated batteries') at FRD (0.0154, 0, -0.5708), flat;
H-flow ('HFlow sensor - follows HFlowAngle', HFlowAngle = 20 deg) at (0.003, 0, -0.5217), pitch -20 deg (beam 2 deg
off vertical at hover 18). EKF2_IMU_POS_*, EKF2_OF_POS_*, EKF2_RNG_POS_* are no longer pinned to V3's values and come
from the geometry (IMU 0.161 / -0.001 / -0.199, flow and range 0.165 / -0.001 / -0.148). CA_ROTOR unchanged.
Re-flown at hover 18, park 0: lift-off 3.6 s, hover pitch 18.05 +-0.37, roll 1.23 +-0.35, drift 0.11 m.

## Yaw spin on take-off (analysis/yaw_logs.py, yaw_takeoff.py)

V3 board logs (Oct 20): no yaw on the ground; during the climb a one-way yaw of up to -50 deg/s mean, 75-108 deg/s
peak, with motors at 100 %; PX4's yaw torque demand rising slowly (+0.2 climb -> +0.34 hover) and reported as
allocated. Board yaw loop MC_YAWRATE_P 0.07 / I 0.035.

V4 SITL, hover 18, park 0, one change at a time (2.7 s climb to ~1.9 m):

| Scenario | Fan torque km | Yaw gains P / I | Yaw in climb (mean / peak) | Turned in climb | Hover peak |
|---|---|---|---|---|---|
| A base | 0.006 | 0.2 / 0.1 | -0.2 / 7 deg/s | -0.8 deg | 1.8 deg/s |
| B board gains | 0.006 | 0.07 / 0.035 | -0.1 / 6 | -0.6 | 1.1 |
| C bench km | 0.0134 | 0.2 / 0.1 | 13 / 22 | 37 | 7.9 |
| D bench km + board gains | 0.0134 | 0.07 / 0.035 | 28 / 37 | 78 | 28.9 |

Cause: all nine fans spin the same way, so their reaction torque twists the airframe in one direction, growing with
thrust; it jumps at lift-off. The CA_ROTOR card's KM values were computed for km 0.006, half the bench value, so PX4
does not pre-compensate the real twist and the yaw integrator has to find it; with the weak board gains that takes
the whole climb.

## Yaw oscillation after take-off (analysis/osc.py)

With the bench fan twist (km 0.0134), hover 18, park 0. Half-amplitude of the yaw-rate swing:

| Yaw rate gains P / I | Period | 8-30 s | 30-60 s | 60-90 s | Turned in climb |
|---|---|---|---|---|---|
| 0.07 / 0.035 (board, D) | 5.5 s | 26 deg/s | 17 | 11 | 78 deg |
| 0.2 / 0.1 (PX4 default, E) | 3.6 s | 5.5 | 1.0 | 1.0 | 37 deg |
| 0.4 / 0.1 (F) | 1.6 s | 3.0 | 0.9 | 1.0 | 22 deg |

The swing is the yaw loop (front canted pair M7/M8 moving with it); the board's yaw rate P is too low.
