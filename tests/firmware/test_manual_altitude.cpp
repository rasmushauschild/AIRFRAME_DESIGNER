#include "../../firmware/atlas/src/atlas_nose_lift/ManualAltitude.hpp"
#include <cassert>
#include <cmath>
int main() {
 using H=AtlasManualAltitude;
 assert(H::valid(200000,100000,true,true,0,0,0));
 assert(!H::valid(300000,100000,true,true,0,0,0));
 assert(!H::valid(200000,210000,true,true,0,0,0));
 assert(!H::valid(200000,100000,false,true,0,0,0));
 assert(!H::valid(200000,100000,true,false,0,0,0));
 assert(!H::valid(200000,100000,true,true,NAN,0,0));
 H h; h.start(4,0); h.begin_flight(3.9f);
 for(int i=0;i<2000;i++) { h.step(.004f); }
 assert(std::fabs(h.reference-2)<1e-5f);
 assert(h.reset(1,20)); assert(std::fabs(h.reference-22)<1e-5f);
 assert(std::fabs(h.ground-h.reference-2)<1e-5f);
 assert(h.reset(1,20)); assert(h.ground==24);
 assert(!h.reset(3,2)); assert(!h.reset(2,NAN));
 h.start(0,255); assert(h.reset(0,2));
}
