#pragma once
#include <cmath>
#include <cstdint>
// Pilot handover (NLF_PILOT = 1 Stabilized, 2 Altitude): after the nose lift the module hands PX4 to a manual flight
// mode and the pilot's throttle lifts the aircraft. No position estimate is needed anywhere in this path (indoor,
// GPS off): the nose lift is attitude-only, the flight is the pilot's, and the landing is inferred from a stalled
// descent. Pure decisions, unit-tested without PX4 (tests/firmware/test_pilot_handover.cpp).
struct AtlasPilotHandover {
 // stick throttle in [0,1] (manual_control_setpoint.throttle is [-1,1]: RC or a MAVLink joystick)
 static constexpr float kHandoverThrottle = 0.05f;   // PX4 only gets the sticks with the throttle at minimum
 static constexpr float kGuardThrottle = 0.10f;      // below this the front-fan floor stays: a throttle cut on the feet must not drop the nose
 static constexpr float kLandingThrottle = 0.35f;    // below this a stalled vertical motion means the rear feet carry the aircraft
 static constexpr float kStalledVz = 0.15f;          // m/s
 static constexpr float kAirborneRise = 0.30f;       // m above the settled nose-up height: clearly off the ground, pivoting alone gives less
 static constexpr uint64_t kAirborneDwell = 300000;  // us
 static constexpr uint64_t kContactDwell = 400000;   // us: shorter than PX4's own landed detection, so the floor is back before PX4 cuts thrust
 static constexpr uint64_t kGroundWait = 45000000;   // us: the pilot has this long to lift off before the nose is lowered again (keep below COM_DISARM_LAND)
 static constexpr float kLiftoffRise = 0.15f;        // m above the settled height: the feet are off the ground, PX4 gets the aircraft
 static constexpr uint64_t kLiftoffDwell = 200000;   // us
 static constexpr float kTakeoverHeight = 0.40f;     // m above the takeoff spot: below this, descending, the module takes the landing back
 static constexpr float kHighLatch = 0.60f;          // m: the automatic takeover arms only once the aircraft has been this high
 static constexpr float kHandBackRise = 0.60f;       // m: a landing abandoned by climbing away goes back to PX4 here
 // Stick [0,1] -> lift fraction (thrust over weight, quadratic motors): 0 -> 0, centre -> 1 (hover), full -> lmax.
 // PX4's own Stabilized curve puts hover at the centre too, so the throttle means the same before and after handover.
 static float lift_fraction(float stick, float lmax) {
  if (!std::isfinite(stick) || !std::isfinite(lmax)) { return 0.f; }
  const float s=stick<0.f ? 0.f : (stick>1.f ? 1.f : stick);
  return s<=.5f ? 2.f*s : 1.f+(s-.5f)*2.f*(lmax-1.f);
 }
 static bool liftoff(float rise_m, float lift) { return std::isfinite(rise_m) && rise_m>kLiftoffRise && lift>0.8f; }
 static bool takeover(bool was_high, bool stick_valid, float stick, float rise_m, float vz_ned) {
  return was_high && stick_valid && std::isfinite(rise_m) && std::isfinite(vz_ned) && rise_m<kTakeoverHeight && vz_ned>-0.05f && stick<=0.55f;
 }

 static float throttle_from_setpoint(float sp) {
  if (!std::isfinite(sp)) { return -1.f; }
  const float t=(sp+1.f)*.5f;
  return t<0.f ? 0.f : (t>1.f ? 1.f : t);
 }
 static bool may_handover(bool settled, bool second_press, bool stick_valid, float throttle) {
  return settled && second_press && stick_valid && throttle>=0.f && throttle<=kHandoverThrottle;
 }
 static bool airborne(float rise_m) { return std::isfinite(rise_m) && rise_m>kAirborneRise; }
 // The floor holds the nose at PX4's level while the collective is too low for PX4 to carry it. It must never pin the
 // nose against PX4: above level the allocator wants the front fans down, and a floor there tips the aircraft over
 // backwards. Full at level, gone six degrees above it.
 static constexpr float kReleaseBand = 6.f;   // deg above level over which the floor fades to zero
 static float band_scale(float theta_deg, float level_deg) {
  if (!std::isfinite(theta_deg) || !std::isfinite(level_deg)) { return 0.f; }
  const float s=(level_deg+kReleaseBand-theta_deg)/kReleaseBand;
  return s<0.f ? 0.f : (s>1.f ? 1.f : s);
 }
 // PX4's own allocation already carries the nose: the floor is redundant and is faded out for good.
 static bool supported(bool feedback_active, bool feedback_fresh, float out9, float out10, float c9, float c10) {
  return feedback_active && feedback_fresh && std::isfinite(out9) && std::isfinite(out10) && out9>=0.95f*c9 && out10>=0.95f*c10;
 }
 // A throttle cut brings the floor back: on the feet it keeps the nose from dropping onto the front legs.
 static bool guard(bool stick_valid, float throttle) { return stick_valid && throttle<kGuardThrottle; }
 // One sample of "the rear feet are down": the pilot commands well below hover yet the aircraft does not move
 // vertically. In Stabilized that throttle would accelerate a free aircraft downward at more than 0.2 g; in Altitude
 // mode it is a descent command that is not being followed. The caller dwells kContactDwell on this.
 static bool contact_sample(bool stick_valid, float throttle, float vz) {
  return stick_valid && std::isfinite(vz) && throttle<kLandingThrottle && std::fabs(vz)<kStalledVz;
 }
 static bool contact(bool airborne_latched, bool sample_dwelled, bool px4_contact) {
  return airborne_latched && (sample_dwelled || px4_contact);
 }
};
