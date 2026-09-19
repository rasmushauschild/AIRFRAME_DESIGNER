// Included inside AtlasNoseLift. Hardware execution requires the active HITL guard in Run.
 bool _manual_selected{false}, _manual_latched{false};
 AtlasManualGate _manual{};
 AtlasManualAltitude _manual_height{};
 PositionControl _manual_z_control{};
 uint8_t _manual_vz_reset{};
 hrt_abstime _manual_time{}, _manual_start{}, _manual_last_command{}, _manual_diagnostic{};
 uint8_t _manual_att_reset{};
 float _manual_yaw{}, _manual_pitch{}, _manual_front9{}, _manual_front10{}, _manual_integral{};
 uORB::Publication<vehicle_attitude_setpoint_s> _manual_att_pub{ORB_ID(vehicle_attitude_setpoint)};
 __attribute__((noinline)) void run_manual() {
  poll_commands(); poll_rc_switch();
  const auto now=hrt_absolute_time();
  const float dt=_manual_time ? math::constrain((now-_manual_time)*1e-6f,0.f,.02f) : .004f;
  _manual_time=now;
  vehicle_status_s status{}; _status_sub.copy(&status); poll_kill(status);
  vehicle_attitude_s att{}; _att_sub.copy(&att);
  vehicle_angular_velocity_s rates{}; _rates_sub.copy(&rates);
  vehicle_local_position_s pos{}; _pos_sub.copy(&pos);
  const bool height_valid=AtlasManualAltitude::valid(now,pos.timestamp,pos.z_valid,pos.v_z_valid,pos.z,pos.vz,pos.az);
  vehicle_land_detected_s land{}; _land_sub.copy(&land);
  actuator_armed_s aa{}; _armed_sub.copy(&aa);
  rc_channels_s rc{}; _rc_sub.copy(&rc);
  const bool armed=status.arming_state==vehicle_status_s::ARMING_STATE_ARMED;
  const bool offboard=status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
  bool fresh=AtlasPilotGate::rc_fresh(now,rc.timestamp,rc.signal_lost);
  float sticks[4]{};
  const int functions[]={rc_channels_s::FUNCTION_THROTTLE,rc_channels_s::FUNCTION_ROLL,rc_channels_s::FUNCTION_PITCH,rc_channels_s::FUNCTION_YAW};
  for(int i=0;i<4;i++) {
   const int ch=rc.function[functions[i]];
   if(ch<0 || ch>=rc.channel_count || ch>=18 || !PX4_ISFINITE(rc.channels[ch]) || fabsf(rc.channels[ch])>1.001f) { fresh=false; }
   else { sticks[i]=rc.channels[ch]; }
  }
  const float throttle=(sticks[0]+1.f)*.5f;
  bool valid=att.timestamp && now>=att.timestamp && now-att.timestamp<200_ms && rates.timestamp && now>=rates.timestamp && now-rates.timestamp<200_ms;
  float norm=0;
  for(int i=0;i<4;i++) { valid=valid && PX4_ISFINITE(att.q[i]); norm+=att.q[i]*att.q[i]; }
  for(int i=0;i<3;i++) { valid=valid && PX4_ISFINITE(rates.xyz[i]); }
  valid=valid && fabsf(norm-1.f)<.02f;
  const bool request=take(_request), stop=take(_land_request);
  if (should_exit()) { clear_floor(); ScheduleClear(); exit_and_cleanup(desc); return; }
  const matrix::Eulerf e{matrix::Quatf(att.q)};
  if(request && (_manual.state==AtlasManualGate::Idle || _manual.state==AtlasManualGate::Failed)) {
   updateParams();
   AtlasGroundModel candidate{};
   const float hover=_hover_param.get();
   const bool geometry=atlas_model_matches(hover,candidate);
   const bool settings=PX4_ISFINITE(_lift_target.get()) && PX4_ISFINITE(_rate.get()) && _rate.get()>=1 && _rate.get()<=15 &&
       PX4_ISFINITE(_ramp.get()) && _ramp.get()>=1 && _ramp.get()<=10;
   if(!_enable.get() || !geometry || !settings || aa.kill || aa.termination || !valid || !height_valid ||
      !PX4_ISFINITE(_manual_hover.get()) || _manual_hover.get()<.1f || _manual_hover.get()>.7f ||
      !_manual.start(!armed,land.timestamp && now-land.timestamp<500_ms && land.landed,fresh,throttle)) {
    PX4_WARN("manual start rejected: require disarmed/landed, valid attitude/height and fresh RC at low throttle, kill released"); return;
   }
   _ground=candidate; _hover_angle=hover; _manual_yaw=e.psi(); _manual_pitch=e.theta();
   _manual_start=now; _manual_last_command=0; _manual_integral=0; _manual_front9=0; _manual_front10=0; clear_floor();
   _manual_height.start(pos.z,pos.z_reset_counter); _manual_vz_reset=pos.vz_reset_counter;
   _manual_z_control.resetIntegral();
   _manual_z_control.setPositionGains(matrix::Vector3f(1.f,1.f,1.f));
   _manual_z_control.setVelocityGains(matrix::Vector3f(2.f,2.f,3.f),matrix::Vector3f(0.f,0.f,1.f),matrix::Vector3f(0.f,0.f,.1f));
   _manual_z_control.setVelocityLimits(1.f,.4f,.25f);
   _manual_z_control.setHorizontalThrustMargin(0.f); _manual_z_control.setTiltLimit(math::radians(20.f));
   _manual_z_control.setHoverThrust(_manual_hover.get());
   PX4_WARN("HITL: second press climbs to 2 m; RC throttle limits power; no XY hold");
  }
  if(_manual.state==AtlasManualGate::Idle || _manual.state==AtlasManualGate::Failed) { return; }
  if(!valid || !height_valid || !_manual_height.reset(pos.z_reset_counter,pos.delta_z) || !fresh || aa.kill || aa.termination ||
     (_manual.state!=AtlasManualGate::Prime && (!armed || !offboard))) {
   clear_floor(); _manual.state=AtlasManualGate::Failed;
   // Stop offboard publications: PX4's configured loss/mode/kill handling owns the vehicle.
   // Do not invent a GPS-dependent landing or force-disarm an airborne aircraft.
   PX4_WARN("manual released: RC/attitude/height loss, kill, disarm, or mode change"); return;
  }
  if(stop) {
   if(_manual.state==AtlasManualGate::Prime && !armed) { _manual.state=AtlasManualGate::Idle; return; }
   // No position/contact sensor means there is no honest automatic land command.
   PX4_WARN("manual mode: lower RC throttle to land; no automatic landing available");
  }
  if(_manual.state==AtlasManualGate::Prime) {
   if(now-_manual_start>15_s) { _manual.state=AtlasManualGate::Failed; PX4_WARN("manual arming timeout"); return; }
   if(now-_manual_start>2_s && now-_manual_last_command>1_s) {
    if(!offboard) { command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,6.f,status); }
    else if(!armed) { command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,1.f,0.f,status); }
    _manual_last_command=now;
   }
   if(armed && offboard) { _manual.state=AtlasManualGate::Nose; _manual_start=now; }
  }
  if(_manual.state==AtlasManualGate::Nose) {
   const float theta=matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta();
   const matrix::Vector3f omega=matrix::Dcmf(matrix::Eulerf(0.f,math::radians(-_hover_angle),0.f))*matrix::Vector3f(rates.xyz);
   const float cap=_manual.slew(throttle,dt);
   const float ease=math::constrain((now-_manual_start)*1e-6f/_ramp.get(),0.f,1.f);
   const float error=math::constrain(_lift_target.get()-math::degrees(theta),-_rate.get(),_rate.get()*ease)-math::degrees(omega(1));
   if(math::max(_manual_front9,_manual_front10)<cap-.001f || error<0) { _manual_integral=math::constrain(_manual_integral+error*dt,-40.f,40.f); }
   const float ff=_ground.mass*9.80665f*(_ground.gx*cosf(theta)+_ground.gz*sinf(theta))/_ground.moment;
   const float frac=math::constrain(ease*ff+.02f*error+.012f*_manual_integral,0.f,1.f);
   const float delta=math::constrain(-3.f*omega(2),-.25f,.25f);
   _manual_front9=sqrtf(math::constrain(frac*(_ground.w9+delta),0.f,1.f));
   _manual_front10=sqrtf(math::constrain(frac*(2.f-_ground.w9-delta),0.f,1.f));
   const float largest=math::max(_manual_front9,_manual_front10);
   if(largest>cap && largest>0) { _manual_front9*=cap/largest; _manual_front10*=cap/largest; }
   const bool stable=fabsf(math::degrees(theta)-_lift_target.get())<2 && fabsf(e.phi())<math::radians(3.f) &&
       fabsf(rates.xyz[1])<math::radians(3.f) && fabsf(rates.xyz[2])<math::radians(3.f);
   if(_manual.confirm(request,stable,now)) {
    _manual_yaw=e.psi(); _manual_att_reset=att.quat_reset_counter; _manual_pitch=e.theta(); _manual_start=now;
    // Start at the same collective proxy published by the nose stage, not zero
    // or the RC cap (which is a per-fan command, not aircraft collective thrust).
    _manual.thrust=math::min(throttle,.08f*(_manual_front9*_manual_front9+_manual_front10*_manual_front10));
    _manual_height.begin_flight(pos.z); _manual_z_control.resetIntegral();
    _manual_vz_reset=pos.vz_reset_counter;
    PX4_WARN("manual flight: climb/hold 2 m; RC throttle is power ceiling");
   }
  }
  atlas_nose_lift_pilot_s ownership{}; ownership.timestamp=now; ownership.active=armed && offboard;
  ownership.manual_attitude=ownership.active; _pilot_pub.publish(ownership);
  offboard_control_mode_s mode{}; mode.timestamp=now;
  if(_manual.state==AtlasManualGate::Flight) {
   mode.attitude=true; _mode_pub.publish(mode);
   // Bound the controller itself by the pilot ceiling and motor ramp, so
   // its integrator cannot wind up behind either limit. No horizontal states.
   if(pos.vz_reset_counter!=_manual_vz_reset) {
    _manual_z_control.resetIntegral(); _manual_vz_reset=pos.vz_reset_counter;
   }
   const float vertical=math::constrain(cosf(e.phi())*cosf(e.theta()),.5f,1.f);
   const float cap=math::min(throttle,math::min(.8f,_manual.thrust+dt));
   _manual_z_control.setThrustLimits(0.f,math::max(.001f,cap*vertical));
   PositionControlStates height_state{};
   height_state.position=matrix::Vector3f(NAN,NAN,pos.z);
   height_state.velocity=matrix::Vector3f(NAN,NAN,pos.vz);
   height_state.acceleration=matrix::Vector3f(NAN,NAN,pos.az); height_state.yaw=e.psi();
   _manual_z_control.setState(height_state);
   trajectory_setpoint_s height_sp=PositionControl::empty_trajectory_setpoint;
   height_sp.position[2]=_manual_height.step(dt);
   height_sp.acceleration[0]=0.f; height_sp.acceleration[1]=0.f;
   _manual_z_control.setInputSetpoint(height_sp);
   if(!_manual_z_control.update(dt)) {
    clear_floor(); _manual.state=AtlasManualGate::Failed; PX4_WARN("manual altitude controller invalid"); return;
   }
   vehicle_local_position_setpoint_s height_output{};
   _manual_z_control.getLocalPositionSetpoint(height_output);
   const float thrust=math::constrain(-height_output.thrust[2]/vertical,0.f,cap);
   _manual.thrust=thrust;
   if (att.quat_reset_counter!=_manual_att_reset) {
    _manual_yaw=matrix::wrap_pi(_manual_yaw+matrix::Eulerf(matrix::Quatf(att.delta_q_reset)).psi());
    _manual_att_reset=att.quat_reset_counter;
   }
   _manual_yaw=matrix::wrap_pi(_manual_yaw+sticks[3]*math::radians(30.f)*dt);
   const float pitch_target=-sticks[2]*math::radians(10.f);
   _manual_pitch+=math::constrain(pitch_target-_manual_pitch,-math::radians(5.f)*dt,math::radians(5.f)*dt);
   vehicle_attitude_setpoint_s sp{}; sp.timestamp=now;
   matrix::Quatf q{matrix::Eulerf(sticks[1]*math::radians(10.f),_manual_pitch,_manual_yaw)}; q.copyTo(sp.q_d);
   sp.yaw_sp_move_rate=sticks[3]*math::radians(30.f); sp.thrust_body[2]=-thrust; _manual_att_pub.publish(sp);
   // Short support bridge only; release immediately if nose-high or rotating
   // backward. The attitude controller must be able to reduce front thrust.
   const float elapsed=(now-_manual_start)*1e-6f;
   const bool nose_high=e.theta()>_manual_pitch+math::radians(1.f) || rates.xyz[1]>math::radians(3.f);
   atlas_nose_lift_floor_s floor{}; floor.timestamp=now; floor.timestamp_sample=att.timestamp; floor.active=elapsed<.5f && !nose_high;
   for(int i=0;i<12;i++) { floor.control[i]=NAN; }
   floor.control[8]=AtlasManualGate::support_floor(_manual_front9,elapsed,throttle,nose_high);
   floor.control[9]=AtlasManualGate::support_floor(_manual_front10,elapsed,throttle,nose_high);
   _floor_pub.publish(floor);
   if(now-_manual_diagnostic>200_ms) {
    _manual_diagnostic=now;
    PX4_INFO("altitude: h=%.2f ref=%.2f vz=%.2f thrust=%.3f cap=%.3f",
      double(_manual_height.ground-pos.z),double(_manual_height.ground-_manual_height.reference),
      double(pos.vz),double(thrust),double(throttle));
    atlas_nose_lift_floor_s feedback{}; _motors_sub.copy(&feedback);
    PX4_INFO("manual transition: t=%.2f pitch=%.2f target=%.2f q=%.2f RC=%.2f thrust=%.2f front=%.2f/%.2f rear=%.2f floor=%.2f/%.2f",
      double(elapsed),double(math::degrees(e.theta())),double(math::degrees(_manual_pitch)),double(math::degrees(rates.xyz[1])),
      double(throttle),double(thrust),double(feedback.control[8]),double(feedback.control[9]),double(feedback.control[0]),
      double(floor.control[8]),double(floor.control[9]));
   }
  } else {
   mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for(int i=0;i<12;i++) { motors.control[i]=NAN; }
   motors.control[8]=_manual_front9; motors.control[9]=_manual_front10; _motors_pub.publish(motors);
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=-.08f*(_manual_front9*_manual_front9+_manual_front10*_manual_front10); _thrust_pub.publish(thrust);
  }
 }
