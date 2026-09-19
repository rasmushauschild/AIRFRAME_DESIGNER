#include <cassert>
#include <cmath>
#include "../../firmware/atlas/src/atlas_nose_lift/PilotHandover.hpp"
int main(){
 using H=AtlasPilotHandover;
 // manual_control_setpoint throttle [-1,1] -> [0,1]; invalid -> -1 (never "at minimum")
 assert(std::fabs(H::throttle_from_setpoint(-1.f))<1e-6f);
 assert(std::fabs(H::throttle_from_setpoint(0.f)-.5f)<1e-6f);
 assert(std::fabs(H::throttle_from_setpoint(1.f)-1.f)<1e-6f);
 assert(H::throttle_from_setpoint(NAN)<0.f);
 // PX4 gets the sticks only on a deliberate second press with the throttle at minimum and a live remote
 assert(H::may_handover(true,true,true,0.f));
 assert(H::may_handover(true,true,true,.05f));
 assert(!H::may_handover(true,true,true,.06f));
 assert(!H::may_handover(true,true,false,0.f));
 assert(!H::may_handover(true,false,true,0.f));
 assert(!H::may_handover(false,true,true,0.f));
 assert(!H::may_handover(true,true,true,-1.f));
 // pivoting the nose alone lifts the IMU by less than the airborne threshold
 assert(!H::airborne(.10f)); assert(!H::airborne(.30f)); assert(H::airborne(.31f)); assert(!H::airborne(NAN));
 // floor: full at and below level, gone above the release band, never with an invalid attitude
 assert(H::band_scale(23.f,23.f)==1.f);
 assert(H::band_scale(10.f,23.f)==1.f);
 assert(std::fabs(H::band_scale(26.f,23.f)-.5f)<1e-5f);
 assert(H::band_scale(29.f,23.f)==0.f);
 assert(H::band_scale(NAN,23.f)==0.f);
 // supported: PX4's unmodified allocation at or above the floor on both front fans, from a live feedback
 assert(H::supported(true,true,.75f,.70f,.74f,.69f));
 assert(!H::supported(true,true,.60f,.70f,.74f,.69f));
 assert(!H::supported(false,true,.75f,.70f,.74f,.69f));
 assert(!H::supported(true,false,.75f,.70f,.74f,.69f));
 assert(!H::supported(true,true,NAN,.70f,.74f,.69f));
 // guard: a throttle cut with a live remote
 assert(H::guard(true,.05f));
 assert(!H::guard(true,.5f));
 assert(!H::guard(false,.05f));      // no remote: PX4's failsafe flies, the floor must not pitch it up
 // contact sample: low throttle and no vertical motion; a free aircraft at that throttle would be moving
 assert(H::contact_sample(true,.2f,.05f));
 assert(!H::contact_sample(true,.5f,.05f));
 assert(!H::contact_sample(true,.2f,.4f));
 assert(!H::contact_sample(false,.2f,.05f));
 assert(!H::contact_sample(true,.2f,NAN));
 // contact only counts once the aircraft has been airborne; PX4's land detector is an alternative source
 assert(!H::contact(false,true,true));
 assert(H::contact(true,true,false));
 assert(H::contact(true,false,true));
 assert(!H::contact(true,false,false));
 // throttle -> lift fraction: off at the bottom, hover at the centre, lmax at the top, clamped
 assert(H::lift_fraction(0.f,1.4f)==0.f);
 assert(std::fabs(H::lift_fraction(.25f,1.4f)-.5f)<1e-6f);
 assert(std::fabs(H::lift_fraction(.5f,1.4f)-1.f)<1e-6f);
 assert(std::fabs(H::lift_fraction(1.f,1.4f)-1.4f)<1e-6f);
 assert(std::fabs(H::lift_fraction(2.f,1.4f)-1.4f)<1e-6f);
 assert(H::lift_fraction(NAN,1.4f)==0.f);
 // liftoff: clearly risen with the throttle near hover; pivoting the nose alone does not count
 assert(H::liftoff(.2f,1.f)); assert(!H::liftoff(.1f,1.f)); assert(!H::liftoff(.2f,.5f)); assert(!H::liftoff(NAN,1.f));
 // automatic landing takeover: low, not climbing, throttle at or below the centre, only after having been high
 assert(H::takeover(true,true,.5f,.3f,.1f));
 assert(!H::takeover(false,true,.5f,.3f,.1f));
 assert(!H::takeover(true,false,.5f,.3f,.1f));
 assert(!H::takeover(true,true,.7f,.3f,.1f));
 assert(!H::takeover(true,true,.5f,.5f,.1f));
 assert(!H::takeover(true,true,.5f,.3f,-.3f));
 assert(H::kContactDwell<900000);   // must beat PX4's landed detection (ground contact + maybe landed + landed)
 assert(H::kGroundWait<60000000);   // must beat the exporter's COM_DISARM_LAND floor of 120 s with margin
 return 0;
}
