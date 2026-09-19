#include "../../firmware/atlas/src/atlas_nose_lift/ManualGate.hpp"
#include <cassert>
#include <limits>
int main() {
 assert(AtlasManualGate::simulation_allowed(true,0,false,false));
 assert(AtlasManualGate::simulation_allowed(false,1,true,true));
 assert(!AtlasManualGate::simulation_allowed(false,0,true,true));
 assert(!AtlasManualGate::simulation_allowed(false,1,false,true));
 assert(!AtlasManualGate::simulation_allowed(false,1,true,false));
 assert(!AtlasManualGate::simulation_allowed(false,2,true,true));
 assert(AtlasManualGate::support_floor(.6f,0,.4f,false)<=.4f);
 assert(AtlasManualGate::support_floor(.6f,.5f,.8f,false)==0);
 assert(AtlasManualGate::support_floor(.6f,0,.8f,true)==0);
 assert(AtlasManualGate::support_floor(.6f,.25f,.8f,false)<.31f);
 AtlasManualGate g;
 assert(!g.start(false,true,true,0));
 assert(!g.start(true,false,true,0));
 assert(!g.start(true,true,false,0));
 assert(!g.start(true,true,true,.2f));
 assert(!g.start(true,true,true,std::numeric_limits<float>::quiet_NaN()));
 assert(g.start(true,true,true,0));
 assert(!g.confirm(true,true,2000000));
 g.state=AtlasManualGate::Nose;
 assert(!g.confirm(true,true,2000000));
 assert(!g.confirm(false,true,4000000));
 assert(!g.confirm(true,false,4100000));
 assert(!g.confirm(true,true,4200000));
 assert(g.confirm(true,true,5300000));
 assert(g.state==AtlasManualGate::Flight);
 assert(!g.start(true,true,true,0));
 const float first=g.slew(1,.004f);
 assert(first>0 && first<.001f);
 assert(g.slew(0,.004f)==0);
 assert(g.slew(1,10)<.005f);
 const float before=g.thrust;
 assert(g.slew(std::numeric_limits<float>::quiet_NaN(),.004f)==before);
 g.state=AtlasManualGate::Failed;
 assert(g.start(true,true,true,0));
 assert(g.thrust==0);
 assert(g.slew(.6f,.02f,1.f)>.019f);
 for(int i=0;i<30;i++) { g.slew(.6f,.02f,1.f); }
 assert(g.thrust<=.6f && g.thrust>.599f);
 assert(g.slew(.1f,.004f,1.f)==.1f);

}
