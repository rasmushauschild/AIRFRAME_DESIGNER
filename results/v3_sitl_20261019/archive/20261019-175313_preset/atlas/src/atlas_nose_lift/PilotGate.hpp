#pragma once
#include <cmath>
#include <cstdint>
// Kept independent of PX4 so loss, low-start and reduction behaviour can be tested without motors.
struct AtlasPilotGate {
 static bool rc_fresh(uint64_t now, uint64_t last_signal, bool lost) {
  return !lost && last_signal != 0 && now >= last_signal && now-last_signal <= 300000;
 }
 static float abort_limit(float requested, float entry_command) {
  return requested < entry_command ? requested : entry_command;
 }
 static bool owns_throttle(bool enabled, bool active, bool armed, bool offboard_or_landing) {
  return enabled && active && armed && offboard_or_landing;
 }
 float peak{0.f};
 bool raised{false};
 uint64_t low_since{0};
 void reset() { peak=0.f; raised=false; low_since=0; }
 static bool can_start(bool valid, float throttle) {
  return valid && std::isfinite(throttle) && throttle >= 0.f && throttle <= .05f;
 }
 static bool may_liftoff(bool stable, bool second_press, bool valid) { return stable && second_press && valid; }
 bool update(bool valid, float throttle, uint64_t now=1) {
  if (!valid || !std::isfinite(throttle) || throttle < 0.f || throttle > 1.f) { return false; }
  if (throttle > peak) { peak=throttle; }
  if (peak > .10f) { raised=true; }
  // Normal corrections are not a latched landing command. Deliberate low stick
  // must persist; invalid/lost RC still aborts immediately above.
  if (raised && throttle < .05f) {
   if (!low_since) { low_since=now; }
   return now-low_since < 400000;
  }
  low_since=0;
  return true;
 }
};
