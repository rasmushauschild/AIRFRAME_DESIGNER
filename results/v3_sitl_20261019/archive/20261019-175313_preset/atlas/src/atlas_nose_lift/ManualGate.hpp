#pragma once
#include <cmath>
#include <cstdint>
// No position input: this mode must not silently depend on GNSS or a local origin.
struct AtlasManualGate {
 static bool simulation_allowed(bool sitl, int sys_hitl, bool hil_active, bool status_fresh) {
  return sitl || (sys_hitl==1 && hil_active && status_fresh);
 }
 enum State { Idle, Prime, Nose, Flight, Failed };
 State state{Idle};
 uint64_t stable_since{};
 float thrust{};
 bool start(bool disarmed, bool landed, bool fresh, float throttle) {
  if ((state!=Idle && state!=Failed) || !disarmed || !landed || !fresh || !std::isfinite(throttle) || throttle<0 || throttle>.05f) { return false; }
  state=Prime; stable_since=0; thrust=0; return true;
 }
 bool confirm(bool press, bool stable, uint64_t now) {
  if (state!=Nose) { return false; }
  if (!stable) { stable_since=0; return false; }
  if (!stable_since) { stable_since=now; }
  if (press && now-stable_since>=1000000) { state=Flight; thrust=0; return true; }
  return false;
 }
 static float support_floor(float entry, float elapsed, float pilot, bool nose_high) {
  // Never force front thrust against a nose-down correction. A timed release
  // cannot deadlock waiting for the controller to request more front thrust.
  if (nose_high) { return 0.f; }
  return fminf(pilot,entry*fmaxf(0.f,1.f-elapsed/.5f));
 }
 float slew(float target, float dt, float rise_rate=.2f) {
  if (!std::isfinite(target) || !std::isfinite(dt) || !std::isfinite(rise_rate) || rise_rate<=0 || dt<=0) { return thrust; }
  target=fmaxf(0.f,fminf(1.f,target));
  dt=fminf(dt,.02f);
  // Pilot reductions are immediate; the caller selects the upward slew rate.
  thrust=target<thrust ? target : fminf(target,thrust+rise_rate*dt);
  return thrust;
 }
};
