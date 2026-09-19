#pragma once
#include <cmath>
#include <cstdint>
struct AtlasManualAltitude {
 float ground{}, reference{};
 uint8_t resets{};
 static bool valid(uint64_t now, uint64_t stamp, bool z_valid, bool vz_valid, float z, float vz, float az) {
  return stamp && now>=stamp && now-stamp<200000 && z_valid && vz_valid
      && std::isfinite(z) && std::isfinite(vz) && std::isfinite(az);
 }
 void start(float z, uint8_t counter) { ground=z; reference=z; resets=counter; }
 bool reset(uint8_t counter, float delta) {
  if(counter==resets) { return true; }
  if(uint8_t(counter-resets)!=1 || !std::isfinite(delta)) { return false; }
  ground+=delta; reference+=delta; resets=counter; return true;
 }
 void begin_flight(float z) { reference=z; }
 float step(float dt) {
  const float target=ground-2.f; // relative height, not distance to the floor
  const float change=fmaxf(-.35f*dt,fminf(.2f*dt,target-reference));
  reference+=change; return reference;
 }
};
