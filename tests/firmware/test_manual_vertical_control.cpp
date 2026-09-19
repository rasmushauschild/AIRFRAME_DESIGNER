#include <PositionControl.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>
#include "../../firmware/atlas/src/atlas_nose_lift/ManualAltitude.hpp"
int main() {
 PositionControl c;
 c.setPositionGains(matrix::Vector3f(1.f,1.f,1.f));
 c.setVelocityGains(matrix::Vector3f(2.f,2.f,3.f),matrix::Vector3f(0.f,0.f,1.f),matrix::Vector3f(0.f,0.f,.1f));
 c.setVelocityLimits(1.f,.4f,.25f); c.setHorizontalThrustMargin(0); c.setTiltLimit(.35f); c.setHoverThrust(.35f);
 AtlasManualAltitude h; h.start(0,0);
 float z=0,v=0,a=0,thrust=.08f,max_h=0;
 for(int i=0;i<5000;i++) {
  const float dt=.004f;
  const float cap=fminf(.8f,thrust+dt);
  c.setThrustLimits(0,fmaxf(.001f,cap));
  PositionControlStates st{};st.position=matrix::Vector3f(NAN,NAN,z);st.velocity=matrix::Vector3f(NAN,NAN,v);st.acceleration=matrix::Vector3f(NAN,NAN,a);st.yaw=0;c.setState(st);
  auto sp=PositionControl::empty_trajectory_setpoint;sp.position[2]=h.step(dt);sp.acceleration[0]=sp.acceleration[1]=0;c.setInputSetpoint(sp);
  assert(c.update(dt));vehicle_local_position_setpoint_s out{};c.getLocalPositionSetpoint(out);
  thrust=fmaxf(0,fminf(cap,-out.thrust[2]));
  a=9.80665f*(1-thrust/.35f); v+=a*dt;z+=v*dt;
  if(z>0){z=0;v=0;a=0;}
  max_h=fmaxf(max_h,-z);
 }
 printf("vertical-controller unit model: final height %.3f m, peak %.3f m, vertical speed %.3f m/s\n",double(-z),double(max_h),double(v));
 assert(fabsf(z+2)<.05f && fabsf(v)<.05f && max_h<2.2f);
 // Pilot ceiling is applied inside the controller, including zero-throttle.
 c.setThrustLimits(0,.001f);assert(c.update(.004f));vehicle_local_position_setpoint_s out{};c.getLocalPositionSetpoint(out);assert(-out.thrust[2]<=.00101f);
}
