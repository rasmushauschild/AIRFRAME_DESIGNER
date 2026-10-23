#pragma once
#include <cmath>
#include <cstdint>
namespace atlas_manual_thrust {
inline bool enabled(bool simulation, bool armed, bool offboard, bool attitude,
                    bool owner, uint64_t now, uint64_t stamp, float thrust_factor) {
 return simulation && armed && offboard && attitude && owner && stamp && now>=stamp
     && now-stamp<100000 && std::isfinite(thrust_factor) && std::fabs(thrust_factor)<.0001f;
}
// The allocator solves thrust fractions. The ATLAS quadratic motor model needs
// speed fractions; THR_MDL_FAC stays zero for the direct nose-lift speed commands.
inline float speed(float thrust) {
 return std::isfinite(thrust) && thrust>=0.f && thrust<=1.f ? std::sqrt(thrust) : NAN;
}
}
