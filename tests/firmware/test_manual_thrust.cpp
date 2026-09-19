#include "../../firmware/atlas/allocator_overlay/ManualThrust.hpp"
#include <cassert>
#include <cmath>
int main() {
 using namespace atlas_manual_thrust;
 assert(enabled(true,true,true,true,true,200000,150000,0));
 assert(!enabled(false,true,true,true,true,200000,150000,0));
 assert(!enabled(true,false,true,true,true,200000,150000,0));
 assert(!enabled(true,true,false,true,true,200000,150000,0));
 assert(!enabled(true,true,true,false,true,200000,150000,0));
 assert(!enabled(true,true,true,true,false,200000,150000,0));
 assert(!enabled(true,true,true,true,true,200000,100000,0));
 assert(!enabled(true,true,true,true,true,200000,210000,0));
 assert(!enabled(true,true,true,true,true,200000,150000,1));
 assert(std::isnan(speed(NAN)) && std::isnan(speed(-.1f)));
 assert(speed(0)==0 && speed(1)==1);
 // An asymmetric pair balanced in thrust must stay balanced after the
 // quadratic motor model. Passing allocator outputs through unchanged fails.
 const float front=.5f, rear=.25f;
 const float moment=speed(front)*speed(front)-2*speed(rear)*speed(rear);
 assert(std::fabs(moment)<1e-6f);
 assert(std::fabs(front*front-2*rear*rear)>.1f);
 for(int i=0;i<=100;i++) {
  const float thrust=i/100.f;
  assert(std::fabs(speed(thrust)*speed(thrust)-thrust)<1e-6f);
 }
}
