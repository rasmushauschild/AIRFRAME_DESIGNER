#include <cassert>
#include <limits>
#include "../../firmware/atlas/src/atlas_nose_lift/PilotGate.hpp"
int main(){
 // Low-power abort must never jump to model feedforward support.
 assert(AtlasPilotGate::abort_limit(.85f,.12f)==.12f);
 assert(AtlasPilotGate::abort_limit(.04f,.12f)==.04f);
 assert(AtlasPilotGate::abort_limit(.85f,0.f)==0.f);
 // Shutdown is active until disarmed, preserving throttle ownership.
 assert(AtlasPilotGate::owns_throttle(true,true,true,true));
 assert(!AtlasPilotGate::owns_throttle(true,true,false,true));
 assert(!AtlasPilotGate::owns_throttle(true,true,true,false));
 assert(!AtlasPilotGate::owns_throttle(false,true,true,true));

 assert(AtlasPilotGate::rc_fresh(1000000,999000,false));
 assert(!AtlasPilotGate::rc_fresh(1000000,600000,false));
 assert(!AtlasPilotGate::rc_fresh(1000000,999000,true));
 assert(!AtlasPilotGate::rc_fresh(1000000,0,false));
 assert(!AtlasPilotGate::rc_fresh(1000000,1000001,false));
 assert(!AtlasPilotGate::may_liftoff(false,true,true));
 assert(!AtlasPilotGate::may_liftoff(true,false,true));
 assert(!AtlasPilotGate::may_liftoff(true,true,false));
 assert(AtlasPilotGate::may_liftoff(true,true,true));
 assert(AtlasPilotGate::can_start(true,0));
 assert(!AtlasPilotGate::can_start(true,.4f));
 assert(!AtlasPilotGate::can_start(false,0));
 AtlasPilotGate gate;
 assert(gate.update(true,0)); assert(gate.update(true,.4f));
 assert(gate.update(true,.38f)); assert(gate.update(true,.34f));
 gate.reset(); assert(gate.update(true,.2f));
 assert(gate.update(true,0,1000000));
 assert(gate.update(true,0,1399999));
 assert(!gate.update(true,0,1400000));
 gate.reset(); assert(gate.update(true,.5f));
 assert(gate.update(true,0,1000000)); assert(gate.update(true,.2f,1200000));
 assert(gate.update(true,0,1300000)); assert(gate.update(true,0,1500000));
 assert(!gate.update(false,0,1500001));
 gate.reset(); assert(!gate.update(false,.4f));
 assert(!gate.update(true,std::numeric_limits<float>::quiet_NaN()));
 assert(!gate.update(true,1.2f));
}
