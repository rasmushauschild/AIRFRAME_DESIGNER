// Experimental ATLAS ground-attitude sequencer. Free on SITL; on a flight controller only with NLF_HW_OK = 1.
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <px4_platform_common/atomic.h>   // NuttX has no std::atomic; px4::atomic works on SITL and boards
#include "Model.hpp"
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/atlas_nose_lift_floor.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/rc_channels.h>
#include <systemlib/mavlink_log.h>
using namespace time_literals;

class AtlasNoseLift : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem {
public:
 static Descriptor desc;
 AtlasNoseLift(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}
 static int task_spawn(int, char**) {
  auto *p = new AtlasNoseLift();
  if (!p) { return PX4_ERROR; }
  desc.object.store(p); desc.task_id = task_id_is_work_queue;
  p->ScheduleOnInterval(4_ms); return PX4_OK;
 }
 static int custom_command(int argc, char **argv) {
  if (argc && !strcmp(argv[0], "takeoff") && desc.object.load()) {
   static_cast<AtlasNoseLift*>(desc.object.load())->_request.store(true); return PX4_OK;
  }
  if (argc && !strcmp(argv[0], "land") && desc.object.load()) {
   static_cast<AtlasNoseLift*>(desc.object.load())->_land_request.store(true); return PX4_OK;
  }
  return print_usage("start the module, then request takeoff or land");
 }
 static int print_usage(const char *reason = nullptr) {
  if (reason) { PX4_WARN("%s", reason); }
  PRINT_MODULE_USAGE_NAME("atlas_nose_lift", "controller");
  PRINT_MODULE_USAGE_COMMAND("start"); PRINT_MODULE_USAGE_COMMAND("takeoff"); PRINT_MODULE_USAGE_COMMAND("land");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); return 0;
 }
 int print_status() override { PX4_INFO("phase=%d pitch=%.2f front9=%.3f", int(_phase), double(_pitch), double(_cmd9)); return 0; }
private:
 enum Phase { Idle, Prime, Lift, Spool, Climb, Hover, Descend, LowerNose, Shutdown, Aborting, Failed };
 Phase _phase{Idle};
 px4::atomic<bool> _request{false}, _land_request{false};
 static bool take(px4::atomic<bool> &flag) { bool expected = true; return flag.compare_exchange(&expected, false); }
 AtlasGroundModel _ground{};
 float _rest_pitch{}, _lower_target{}, _hover_angle{}, _landing_angle{};
 hrt_abstime _contact_dwell{};
 float _touch_z{};
 hrt_abstime _start{}, _phase_start{}, _dwell{}, _last_command{};
 hrt_abstime _fade_start{};
 hrt_abstime _support_since{};   // Spool: when PX4's own allocation first matched the floor (must persist before fading)
 float _fade9{}, _fade10{}, _abort9{}, _abort10{};
 float _integral{}, _cmd9{}, _cmd10{};
 hrt_abstime _diagnostic_time{};
 uint8_t _xy_reset{}, _z_reset{}, _heading_reset{};
 float _pitch{}, _yaw{}, _x{}, _y{}, _z{}, _target_z{};
 bool _was_armed{false};
 bool _released{false};
 orb_advert_t _mavlink_log_pub{nullptr};
#define NL_INFO(...) do { PX4_INFO(__VA_ARGS__); mavlink_log_info(&_mavlink_log_pub, __VA_ARGS__); } while (0)
#define NL_WARN(...) do { PX4_WARN(__VA_ARGS__); mavlink_log_warning(&_mavlink_log_pub, __VA_ARGS__); } while (0)
#define NL_ERR(...) do { PX4_ERR(__VA_ARGS__); mavlink_log_critical(&_mavlink_log_pub, __VA_ARGS__); } while (0)   // Aborting phase entered only to keep lockstep alive after an external disarm
 uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)}, _rates_sub{ORB_ID(vehicle_angular_velocity)},
  _pos_sub{ORB_ID(vehicle_local_position)}, _status_sub{ORB_ID(vehicle_status)}, _land_sub{ORB_ID(vehicle_land_detected)};
 uORB::Subscription _motors_sub{ORB_ID(atlas_nose_lift_feedback)};
 uORB::Publication<atlas_nose_lift_floor_s> _floor_pub{ORB_ID(atlas_nose_lift_floor)};
 uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
 uORB::Publication<actuator_motors_s> _motors_pub{ORB_ID(actuator_motors)};
 uORB::Publication<offboard_control_mode_s> _mode_pub{ORB_ID(offboard_control_mode)};

 uORB::Publication<trajectory_setpoint_s> _traj_pub{ORB_ID(trajectory_setpoint)};
 uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
 // External trigger, the same on SITL, HITL and the real aircraft: MAV_CMD_USER_1 (31010) with param1 = 1 (takeoff)
 // or 2 (land), sent by any ground station or the designer app. The shell commands remain for the console.
 static constexpr uint32_t CMD_ATLAS = 31010;
 // Remote-control triggers: two momentary buttons. NLF_RC_CH requests takeoff on its rising edge, NLF_RC_LAND
 // requests landing on its rising edge; nothing happens for buttons already pressed at boot.
 uORB::Subscription _rc_sub{ORB_ID(rc_channels)};
 bool _btn_to{false}, _btn_land{false}, _rc_seen{false};
 void poll_rc_switch() {
  const int ct = _rc_ch.get(), cl = _rc_land.get();
  if (ct < 1 && cl < 1) { _rc_seen = false; return; }
  rc_channels_s rc{};
  if (!_rc_sub.update(&rc)) { return; }
  if (rc.signal_lost) { return; }
  const bool to = ct >= 1 && ct <= rc.channel_count && rc.channels[ct - 1] > 0.5f;
  const bool land = cl >= 1 && cl <= rc.channel_count && rc.channels[cl - 1] > 0.5f;
  if (!_rc_seen) { _rc_seen = true; _btn_to = to; _btn_land = land; return; }
  if (to && !_btn_to) { _request.store(true); }
  if (land && !_btn_land) { _land_request.store(true); }
  _btn_to = to; _btn_land = land;
 }
 uORB::Subscription _cmd_sub{ORB_ID(vehicle_command)};
 uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};
 void poll_commands() {
  vehicle_command_s cmd{};
  while (_cmd_sub.update(&cmd)) {
   if (cmd.command != CMD_ATLAS) { continue; }
   const int what = (int)(cmd.param1 + 0.5f);
   if (what == 1) { _request.store(true); } else if (what == 2) { _land_request.store(true); }
   vehicle_command_ack_s ack{}; ack.timestamp=hrt_absolute_time(); ack.command=cmd.command;
   ack.result=(what == 1 || what == 2) ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED : vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
   ack.target_system=cmd.source_system; ack.target_component=cmd.source_component; ack.from_external=false;
   _ack_pub.publish(ack);
  }
 }
 DEFINE_PARAMETERS(
  (ParamInt<px4::params::NLF_ENABLE>) _enable,
  (ParamFloat<px4::params::NLF_RATE>) _rate,
  (ParamFloat<px4::params::NLF_TARGET>) _lift_target,
 (ParamInt<px4::params::NLF_HW_OK>) _hw_ok,
 (ParamInt<px4::params::NLF_RC_CH>) _rc_ch,
 (ParamInt<px4::params::NLF_RC_LAND>) _rc_land,
  (ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _hover_param,
  (ParamFloat<px4::params::NLF_LAND_ANG>) _land_angle_param,

  (ParamFloat<px4::params::NLF_RAMP>) _ramp,
  (ParamFloat<px4::params::NLF_DWELL>) _hold,
  (ParamFloat<px4::params::NLF_ALT>) _alt,
  (ParamFloat<px4::params::NLF_TIMEOUT>) _timeout,
  (ParamFloat<px4::params::NLF_LAND_V>) _land_speed,
  (ParamFloat<px4::params::NLF_DOWN_R>) _down_rate,
  (ParamFloat<px4::params::COM_DISARM_LAND>) _disarm_delay
 )
 void command(uint32_t cmd, float p1, float p2, const vehicle_status_s &status, float p3=0.f) {
  vehicle_command_s c{}; c.timestamp=hrt_absolute_time(); c.command=cmd; c.param1=p1; c.param2=p2; c.param3=p3;
  c.target_system=status.system_id; c.target_component=status.component_id;
  c.source_system=status.system_id; c.source_component=status.component_id; c.from_external=false;
  _command_pub.publish(c);
 }
 void clear_floor() {
  atlas_nose_lift_floor_s floor{}; floor.timestamp=hrt_absolute_time(); floor.active=false;
  _floor_pub.publish(floor);
 }
 void fail(const char *reason, const vehicle_status_s &status, bool airborne) {
  NL_ERR("abort: %s", reason); clear_floor();
  if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) { _phase=Failed; return; }
  if (!airborne && status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
   _phase=Aborting; _phase_start=hrt_absolute_time(); _abort9=_cmd9; _abort10=_cmd10;
   return;
  }
  _phase=Failed;
  command(vehicle_command_s::VEHICLE_CMD_NAV_LAND,0.f,0.f,status);
 }
 void Run() override {
  poll_commands();
  poll_rc_switch();
  vehicle_status_s status{}; _status_sub.copy(&status);
  vehicle_attitude_s att{}; _att_sub.copy(&att);
  vehicle_angular_velocity_s rates{}; _rates_sub.copy(&rates);
  vehicle_local_position_s pos{}; _pos_sub.copy(&pos);
  vehicle_land_detected_s land{}; _land_sub.copy(&land);
  const auto now=hrt_absolute_time();
  const bool armed=status.arming_state==vehicle_status_s::ARMING_STATE_ARMED;
  if (should_exit()) {
   if (_phase>=Prime && _phase<=LowerNose) { fail("module stopped",status,_phase!=Prime && _phase!=Lift); }
   if (_phase!=Aborting && _phase!=Shutdown) { clear_floor(); ScheduleClear(); exit_and_cleanup(desc); return; }
  }
  if (_phase==Aborting || _phase==Shutdown) {
   if (status.nav_state!=vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
    if (_phase==Shutdown) { if (!armed) { NL_INFO("landing complete: all motors off and disarmed"); } else { NL_WARN("landing interrupted by mode change"); } _phase=Idle; }
    else { _phase=_released ? Idle : Failed; if (_released) { NL_INFO("control handed back to PX4"); } _released=false; }
    return;
   }
   // Keep zero actuator samples flowing until the commander leaves direct control.
   // Otherwise lockstep SITL can stop advancing after normal ground disarm.
   if (!armed && now-_last_command>1_s) {
    command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,4.f,status,3.f); _last_command=now;
   }
   const float scale=math::constrain(1.f-(now-_phase_start)*1e-6f/2.f,0.f,1.f);
   offboard_control_mode_s mode{}; mode.timestamp=now; mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for(int i=0;i<12;i++) { motors.control[i]=NAN; }
   motors.control[8]=_abort9*scale; motors.control[9]=_abort10*scale; _motors_pub.publish(motors);
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=-0.08f*(motors.control[8]*motors.control[8]+motors.control[9]*motors.control[9]);
   _thrust_pub.publish(thrust);
   if (armed && now-_phase_start>2500_ms && now-_last_command>1_s) {
    command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.f,0.f,status); _last_command=now;
   }
   return;
  }
  const matrix::Eulerf e(matrix::Quatf(att.q)); _pitch=e.theta();
  if (take(_request)) {
   updateParams();
   if (_phase!=Idle && _phase!=Failed) { NL_WARN("sequence already active"); return; }
#if !defined(CONFIG_ARCH_BOARD_PX4_SITL)
   // Hardware interlock: on a flight controller the experimental sequence runs only after NLF_HW_OK was set by hand.
   if (_hw_ok.get() != 1) { NL_ERR("experimental module on hardware: set NLF_HW_OK = 1 to allow it"); return; }
#endif
   _hover_angle=_hover_param.get();
   if (!atlas_model_matches(_hover_angle, _ground)) { NL_ERR("ground model configuration invalid; Update PX4"); return; }
   if (!_enable.get() || armed || !land.landed || !pos.xy_valid || !pos.z_valid || now-att.timestamp>200_ms
       || !PX4_ISFINITE(_pitch) || !PX4_ISFINITE(e.psi()) || !PX4_ISFINITE(pos.z)) {
    NL_ERR("requires enabled, disarmed, landed, fresh valid attitude and position"); return;
   }
   if (!PX4_ISFINITE(_land_angle_param.get()) || !PX4_ISFINITE(_hover_angle) || !PX4_ISFINITE(_lift_target.get()) ||
       !PX4_ISFINITE(_rate.get()) || _rate.get()<1.f || _rate.get()>15.f ||
       !PX4_ISFINITE(_ramp.get()) || _ramp.get()<1.f || _ramp.get()>10.f ||
       !PX4_ISFINITE(_hold.get()) || _hold.get()<0.5f || _hold.get()>5.f ||
       !PX4_ISFINITE(_alt.get()) || _alt.get()<0.5f || _alt.get()>5.f ||
       !PX4_ISFINITE(_timeout.get()) || _timeout.get()<10.f || _timeout.get()>60.f) {
    NL_ERR("invalid sequence parameters"); return;
   }
   _phase=Prime; _start=now; _phase_start=now; _dwell=0; _fade_start=0; _integral=0.f; _cmd9=0.f; _cmd10=0.f; _last_command=0; _was_armed=false;
   _rest_pitch=math::degrees(matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta());
   _landing_angle=_land_angle_param.get();
   if (_landing_angle > _rest_pitch+3.f) {
    NL_ERR("landed pitch %.1f is above measured ground posture %.1f; adjust landed pitch to the legs", double(_landing_angle), double(_rest_pitch));
    _phase=Idle; return;
   }
   _xy_reset=pos.xy_reset_counter; _z_reset=pos.z_reset_counter; _heading_reset=pos.heading_reset_counter;
   _yaw=e.psi(); _x=pos.x; _y=pos.y; _z=pos.z; _target_z=_z;
   NL_INFO("priming nose lift at pitch %.1f",double(math::degrees(_pitch)));
  }
  if (take(_land_request)) {
   updateParams();
   if (_phase!=Hover || !armed || !_enable.get() || !atlas_model_matches(_hover_angle, _ground)) {
    NL_WARN("landing requires this module's active hover and matching model");
   } else if (!PX4_ISFINITE(_land_speed.get()) || _land_speed.get()<0.05f || _land_speed.get()>0.3f ||
              !PX4_ISFINITE(_down_rate.get()) || _down_rate.get()<1.f || _down_rate.get()>5.f) {
    NL_ERR("invalid landing parameters");
   } else if (!PX4_ISFINITE(_disarm_delay.get()) || _disarm_delay.get()<40.f/_down_rate.get()+8.f) {
    NL_ERR("landing needs COM_DISARM_LAND long enough for controlled nose lowering (use 60 s)");
   } else {
    _phase=Descend; _phase_start=now; _contact_dwell=0; _target_z=pos.z;
    _x=pos.x; _y=pos.y; clear_floor();
    NL_INFO("landing: descending onto rear legs");
   }
  }
  if (_phase==Idle || _phase==Failed) { return; }
  if (now-att.timestamp>200_ms || now-rates.timestamp>200_ms || now-pos.timestamp>500_ms ||
      !pos.xy_valid || !pos.z_valid || !PX4_ISFINITE(_pitch) || !PX4_ISFINITE(e.phi()) ||
      !PX4_ISFINITE(e.psi()) || !PX4_ISFINITE(rates.xyz[0]) || !PX4_ISFINITE(rates.xyz[1]) || !PX4_ISFINITE(rates.xyz[2]) || !PX4_ISFINITE(pos.x) || !PX4_ISFINITE(pos.y) || !PX4_ISFINITE(pos.z) || !PX4_ISFINITE(pos.vx) || !PX4_ISFINITE(pos.vy) || !PX4_ISFINITE(pos.vz)) {
   fail("stale or invalid estimate",status,_phase!=Prime && _phase!=Lift); return;
  }
  // Keep stored ground coordinates and setpoints in the estimator's current frame.
  if (pos.xy_reset_counter != _xy_reset) {
   if (!PX4_ISFINITE(pos.delta_xy[0]) || !PX4_ISFINITE(pos.delta_xy[1])) { fail("invalid XY reset",status,_phase!=Prime && _phase!=Lift); return; }
   _x+=pos.delta_xy[0]; _y+=pos.delta_xy[1]; _xy_reset=pos.xy_reset_counter;
  }
  if (pos.z_reset_counter != _z_reset) {
   if (!PX4_ISFINITE(pos.delta_z)) { fail("invalid height reset",status,_phase!=Prime && _phase!=Lift); return; }
   _z+=pos.delta_z; _target_z+=pos.delta_z; _touch_z+=pos.delta_z; _z_reset=pos.z_reset_counter;
  }
  if (pos.heading_reset_counter != _heading_reset) {
   if (!PX4_ISFINITE(pos.delta_heading)) { fail("invalid heading reset",status,_phase!=Prime && _phase!=Lift); return; }
   _yaw=matrix::wrap_pi(_yaw+pos.delta_heading); _heading_reset=pos.heading_reset_counter;
  }
  if (_was_armed && (!armed || status.nav_state!=vehicle_status_s::NAVIGATION_STATE_OFFBOARD)) {
   clear_floor(); _was_armed=false;
   if (!armed && status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
    // Disarmed while still under direct actuator control (typically the land detector's auto-disarm during a slow
    // nose lift): nobody publishes motor samples any more, the output driver goes quiet and lockstep SITL stops
    // advancing. Keep zero samples flowing and hand the commander back to Hold before going idle.
    NL_INFO("released control on disarm; handing PX4 back to Hold");
    _phase=Aborting; _released=true; _phase_start=now; _abort9=0.f; _abort10=0.f; _last_command=0; return;
   }
   NL_INFO("released control on disarm or mode change"); _phase=Idle; return;
  }
  if (fabsf(e.phi())>math::radians(20.f) || _pitch>math::radians(15.f) || _pitch<math::radians(-55.f)) {
   fail("attitude envelope",status,_phase!=Prime && _phase!=Lift); return;
  }
  if (_phase==Descend) {
   // Contact inference uses PX4 estimates and the saved ground datum, never simulator truth.
   // Require descent demand, stopped vertical motion near the known rear-foot plane, and dwell.
   const float rear_contact_z=_z-0.18f;
   const bool rear_contact=now-_phase_start>2_s && fabsf(pos.z-rear_contact_z)<0.18f &&
       fabsf(pos.vz)<0.055f && _target_z-pos.z>0.065f && fabsf(e.phi())<math::radians(3.f);
   if (rear_contact) {
    if (!_contact_dwell) { _contact_dwell=now; }
    if (now-_contact_dwell>200_ms) {
     _phase=LowerNose; _phase_start=now; _dwell=0; _integral=0.f; _touch_z=pos.z;
     _lower_target=math::degrees(matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta());
     NL_INFO("landing: rear support detected; lowering nose");
    }
   } else { _contact_dwell=0; }
   if (now-_phase_start>60_s) { fail("rear contact timeout",status,true); return; }
  }
  if (_phase==LowerNose) {
   const matrix::Dcmf structural=matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f));
   const float theta=matrix::Eulerf(structural).theta();
   const matrix::Vector3f omega=matrix::Dcmf(matrix::Eulerf(0.f,math::radians(-_hover_angle),0.f))*matrix::Vector3f(rates.xyz);
   // Settle on the rear feet before starting the rate-limited pitch-down trajectory.
   if (now-_phase_start>1_s) { _lower_target=math::max(_landing_angle-3.f,_lower_target-_down_rate.get()*0.004f); }
   const float q_des=math::constrain(2.f*(_lower_target-math::degrees(theta)),-_down_rate.get(),_down_rate.get());
   const float error=q_des-math::degrees(omega(1));
   _integral=math::constrain(_integral+error*0.004f,-40.f,40.f);
   const float ff=_ground.mass*9.80665f*(_ground.gx*cosf(theta)+_ground.gz*sinf(theta))/_ground.moment;
   const float frac=math::constrain(ff+0.02f*error+0.012f*_integral,0.f,1.f);
   const float w9 = _ground.w9;
   const float delta=math::constrain(-3.f*omega(2),-0.25f,0.25f);
   _cmd9=sqrtf(math::constrain(frac*(w9+delta),0.f,1.f));
   _cmd10=sqrtf(math::constrain(frac*(2.f-w9-delta),0.f,1.f));
   offboard_control_mode_s mode{}; mode.timestamp=now; mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for (int i=0;i<12;i++) { motors.control[i]=NAN; }
   motors.control[8]=_cmd9; motors.control[9]=_cmd10; _motors_pub.publish(motors);
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=-0.08f*(_cmd9*_cmd9+_cmd10*_cmd10); _thrust_pub.publish(thrust);
   // A stopped pitch despite a continuing nose-down request indicates nose-foot support.
   // Require the resting posture and a bounded height change since rear contact.
   // The takeoff height can drift during a long hover; use the recent contact datum.
   // XY velocity is not a nose-contact test: EKF ground-contact transients can bias it.
   const bool nose_down=now-_phase_start>3_s && fabsf(math::degrees(theta)-_rest_pitch)<5.f &&
       _lower_target<_rest_pitch+0.5f && q_des<-1.f && fabsf(math::degrees(omega(1)))<1.5f &&
       fabsf(pos.vz)<0.10f && fabsf(pos.z-_touch_z)<0.35f;
   if (now-_diagnostic_time>1_s) {
    _diagnostic_time=now;
    NL_INFO("lower: pitch %.2f rest %.2f q %.2f demand %.2f vz %.3f dz %.3f contact %d", double(math::degrees(theta)), double(_rest_pitch), double(math::degrees(omega(1))), double(q_des), double(pos.vz), double(pos.z-_z), int(nose_down));
   }
   if (nose_down) {
    if (!_dwell) { _dwell=now; }
    if (now-_dwell>1_s) {
     NL_INFO("landing: nose settled; ramping motors off");
     _phase=Shutdown; _phase_start=now; _abort9=_cmd9; _abort10=_cmd10; _last_command=0;
    }
   } else { _dwell=0; }
   if (now-_phase_start>45_s) { fail("nose lowering timeout",status,false); }
   return;
  }
  if (_phase==Prime) {
   if (now-_start>15_s) { fail("arming timeout",status,false); return; }
   if (now-_start>2_s && now-_last_command>1_s) {
    if (status.nav_state!=vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
     command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,6.f,status);
    } else if (!armed) { command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,1.f,0.f,status); }
    _last_command=now;
   }
   if (armed && status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
    _phase=Lift; _phase_start=now; _was_armed=true; NL_INFO("lifting nose");
   }
  }
  if (_phase==Lift || _phase==Spool) {
   const float elapsed=(now-_phase_start)*1e-6f;
   // Structural pitch uses the configured board/hover-frame offset.
   const matrix::Dcmf structural = matrix::Dcmf(matrix::Quatf(att.q)) * matrix::Dcmf(matrix::Eulerf(0.f, math::radians(_hover_angle), 0.f));
   const float theta = matrix::Eulerf(structural).theta();
   const matrix::Vector3f omega = matrix::Dcmf(matrix::Eulerf(0.f, math::radians(-_hover_angle), 0.f)) * matrix::Vector3f(rates.xyz);
   const float q = math::degrees(omega(1));
   const float ease = _phase==Lift ? math::constrain(elapsed / _ramp.get(), 0.f, 1.f) : 1.f;
   const float q_des = math::constrain(_lift_target.get()-math::degrees(theta), -_rate.get(), _rate.get()*ease);
   const float error = q_des-q;
   _integral=math::constrain(_integral+error*0.004f,-40.f,40.f);
   // Feedforward from the exported current mass, rear pivot and front fan moments.
   const float ff = _ground.mass*9.80665f*(_ground.gx*cosf(theta)+_ground.gz*sinf(theta))/_ground.moment;
   const float frac=math::constrain(ease*ff+0.02f*error+0.012f*_integral,0.f,1.f);
   const float w9 = _ground.w9;
   const float delta=math::constrain(-3.f*omega(2),-0.25f,0.25f);
   _cmd9=sqrtf(math::constrain(frac*(w9+delta),0.f,1.f));
   _cmd10=sqrtf(math::constrain(frac*(2.f-w9-delta),0.f,1.f));
   if (_phase==Lift && elapsed>_timeout.get()) { fail("lift timeout",status,_z-pos.z>0.3f); return; }
   if (_phase==Lift && _z-pos.z>0.35f && fabsf(_pitch)>math::radians(3.f)) { fail("early liftoff",status,true); return; }
   if (_phase==Lift && land.landed && fabsf(math::degrees(theta)-_lift_target.get())<2.f && fabsf(rates.xyz[1])<math::radians(3.f)
       && fabsf(e.phi())<math::radians(3.f) && fabsf(rates.xyz[2])<math::radians(3.f)) {
    if (!_dwell) { _dwell=now; }
    if ((now-_dwell)*1e-6f>=_hold.get()) {
     _phase=Spool; _phase_start=now; _x=pos.x; _y=pos.y; _target_z=pos.z; _support_since=0;
     _yaw=e.psi();
     NL_INFO("nose settled and ground confirmed; climbing with heading held");
    }
   } else { _dwell=0; }
  }
  offboard_control_mode_s mode{}; mode.timestamp=now;
  if (_phase==Prime || _phase==Lift) {
   mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for (int i=0;i<actuator_motors_s::NUM_CONTROLS;i++) { motors.control[i]=NAN; }
   if (_phase==Lift) { motors.control[8]=_cmd9; motors.control[9]=_cmd10; }
   _motors_pub.publish(motors);
   // Direct actuator control otherwise leaves PX4's thrust input stale. Supply the
   // requested collective thrust fraction for landing detection (not a landed flag).
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=_phase==Lift ? -0.08f*(_cmd9*_cmd9+_cmd10*_cmd10) : 0.f;
   _thrust_pub.publish(thrust);
  } else {
   if (_phase==Spool) {
    atlas_nose_lift_floor_s output{}; _motors_sub.copy(&output);
    // Keep the output stream alive until the position-control allocator has published.
    // Without this bridge lockstep waits for motors while commander waits for time
    // to advance before activating the new control mode.
    if (!output.active || output.timestamp < _phase_start) {
     actuator_motors_s bridge{}; bridge.timestamp=now; bridge.timestamp_sample=att.timestamp;
     for (int i=0;i<12;i++) { bridge.control[i]=NAN; }
     bridge.control[8]=_cmd9; bridge.control[9]=_cmd10;
     _motors_pub.publish(bridge);
    }
    // Compare unmodified allocation against support, avoiding feedback through the floor. A momentary match is not
    // enough: right after the mode switch the attitude loop alone can put the front fans above the floor while the
    // takeoff ramp still has the rear motors at zero, and fading then drops the nose and skids the aircraft forward.
    // Fade only once PX4 has matched the floor continuously for half a second and its land detector reports liftoff.
    const bool supported = output.active && now-output.timestamp<100_ms && output.control[8]>=0.95f*_cmd9 && output.control[9]>=0.95f*_cmd10;
    if (!supported) { _support_since=0; } else if (!_support_since) { _support_since=now; }
    if (!_fade_start && supported && now-_support_since>=500_ms && !land.landed) {
     _fade_start=now; _fade9=_cmd9; _fade10=_cmd10; NL_INFO("PX4 supports nose and is lifting; fading motor floor");
    }
    if (now-_phase_start>15_s && !_fade_start) { fail("handover timeout",status,!land.landed); return; }
    atlas_nose_lift_floor_s floor{}; floor.timestamp=now; floor.timestamp_sample=att.timestamp; floor.active=true;
    for(int i=0;i<12;i++) { floor.control[i]=NAN; }
    const float fade = _fade_start ? math::constrain(1.f-(now-_fade_start)*1e-6f/2.f,0.f,1.f) : 1.f;
    floor.control[8]=_fade_start ? math::min(_cmd9,_fade9*fade) : _cmd9;
    floor.control[9]=_fade_start ? math::min(_cmd10,_fade10*fade) : _cmd10;
    _floor_pub.publish(floor);
    if (_fade_start && now-_fade_start>2_s) { _phase=Climb; _phase_start=now; _target_z=pos.z; NL_INFO("handover complete"); }
   }
   mode.position=true; _mode_pub.publish(mode);
   if (_phase==Descend) { _target_z=math::min(_z+0.2f,_target_z+_land_speed.get()*0.004f); }
   else { _target_z=math::max(_z-_alt.get(),_target_z-0.002f); }
   trajectory_setpoint_s sp{}; sp.timestamp=now;
   sp.position[0]=_x; sp.position[1]=_y; sp.position[2]=_target_z;
   for(int i=0;i<3;i++) { sp.velocity[i]=NAN; sp.acceleration[i]=NAN; sp.jerk[i]=NAN; }
   sp.yaw=_yaw; sp.yawspeed=0.f; _traj_pub.publish(sp);
   if (_phase==Climb && fabsf(pos.z-(_z-_alt.get()))<0.15f && fabsf(pos.vz)<0.15f) {
    _phase=Hover; NL_INFO("hover achieved with heading held");
   }
   if (_phase==Climb && now-_phase_start>30_s) { fail("climb timeout",status,true); }
  }
 }
};
ModuleBase::Descriptor AtlasNoseLift::desc{task_spawn,custom_command,print_usage};
extern "C" __EXPORT int atlas_nose_lift_main(int argc,char *argv[]) { return ModuleBase::main(AtlasNoseLift::desc,argc,argv); }
