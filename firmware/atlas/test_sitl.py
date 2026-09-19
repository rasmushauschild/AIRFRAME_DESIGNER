from pathlib import Path
import sys, json, time, subprocess, math, os
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from airframe_designer.geometry.airframe import Airframe
from airframe_designer.px4.sitl import launch_px4, stop_px4, PX4Instance, param_types_from_meta
from airframe_designer.px4.link import PX4Link
from airframe_designer.px4 import param_meta
from airframe_designer.sim.simulator import Simulator
from airframe_designer.dynamics.quaternion import q_to_rotmat
from airframe_designer.batch.worker import BATCH_PX4_DEFAULTS
base=Path(__file__).resolve().parent
name=sys.argv[1] if len(sys.argv)>1 else 'trial'
extra=json.loads(sys.argv[2]) if len(sys.argv)>2 else {}
physics=sys.argv[3] if len(sys.argv)>3 else 'jsbsim'
seed=int(sys.argv[4]) if len(sys.argv)>4 else 1
root=str(base)
bin=base/'build/px4_sitl_default/bin'
af=Airframe.load(os.environ.get('NLF_TEST_MODEL',str(Path(__file__).resolve().parent/'atlas_07d.original.json')))
if extra.get('NLF_PILOT'): af.px4_overrides['NLF_PILOT']=extra['NLF_PILOT']   # export the pilot-handover set (disarm timers, hover thrust)
params=dict(BATCH_PX4_DEFAULTS)
params.update(af.px4_params_sitl())
params.update({'NLF_ENABLE':1,'MC_PITCHRATE_MAX':220.0,'MC_ROLLRATE_MAX':220.0,'MC_YAWRATE_MAX':200.0,'COM_DISARM_PRFLT':40.0,'COM_DISARM_LAND':5.0})
if os.environ.get('NLF_HOVER_ANGLE'):
 af.hover_pitch_deg=float(os.environ['NLF_HOVER_ANGLE'])
 params.update(af.px4_params_sitl())
params.update(extra)
meta,_=param_meta.load_local(root,None)
types=param_types_from_meta(meta)
instance=int(os.environ.get('NLF_TEST_INSTANCE','7'))
inst=PX4Instance(instance,f'/tmp/atlas-nl-{name}-{instance}',fresh=True)
lines=[]; landing_complete_t=None
def log(s):
 global landing_complete_t
 if "landing complete: all motors off" in s and "sim" in globals(): landing_complete_t=sim.t
 lines.append((f"[sim_t={sim.t:.3f}] " if "sim" in globals() else "")+s)
 if os.environ.get('NLF_VERBOSE') or 'atlas_nose_lift' in s or 'Armed' in s or 'Disarmed' in s or 'ERROR' in s or 'WARN' in s: print(s,flush=True)
proc=launch_px4(root,'none_iris',log,instance=instance,rootfs=str(inst.workdir),params=params,param_types=types,quiet=False)
link=PX4Link('sitl',inst.tcp_address,ctl_address=inst.ctl_address,log=log)
rows=[]; started=False; requested=False; land_sent=False; started_t=0; inspected=False
try:
 link.open()
 sim=Simulator(af,link,sensor_rate=250,physics_substeps=4,speed=0,lockstep=True,log=log,seed=seed,physics=physics)
 def cli(module,*args):
  r=subprocess.run([str(bin/f'px4-{module}'),'--instance',str(instance),*args],capture_output=True,text=True,timeout=5)
  print(r.stdout,r.stderr,flush=True)
  return r
 # NLF_PILOT_TEST: a scripted pilot on a MAVLink joystick. Throttle at minimum through the nose lift, second Takeoff
 # press after 'nose ready', then a slow throttle ramp until airborne, a hover a metre up held by a simple height
 # loop, a slow descent onto the rear legs and hands off while the module lowers the nose and disarms.
 pilot={'thr':0.0,'stage':'ground','t_ready':None,'ground_d':None,'t_air':None,'last_send':-1.0,'hover_thr':float(params.get('MPC_THR_HOVER',0.7))}
 def seen(sub): return any(sub in l for l in lines)
 def pilot_step(s):
  b=s.sim
  if pilot['ground_d'] is None: pilot['ground_d']=float(b.pos[2])
  alt=pilot['ground_d']-float(b.pos[2]); vz_up=-float(b.vel[2])
  if pilot['stage']=='ground' and seen('nose ready:') and pilot['t_ready'] is None: pilot['t_ready']=s.t
  if pilot['stage']=='ground' and pilot['t_ready'] is not None and s.t>pilot['t_ready']+1.0:
   pilot['stage']='handover'; cli('atlas_nose_lift','takeoff')
  if pilot['stage']=='handover' and seen('you have the throttle'):
   pilot['stage']='ramp'; pilot['t_air']=s.t
  if pilot['stage']=='ramp':
   pilot['thr']=min(0.9, (s.t-pilot['t_air'])/8.0)    # 0 -> 0.9 over 8 s: as slow as a careful thumb
   if alt>0.5: pilot['stage']='hover'; pilot['t_air']=s.t
  elif pilot['stage'] in ('hover','descend'):
   target=1.0 if pilot['stage']=='hover' else -0.5
   if pilot['stage']=='hover' and s.t>pilot['t_air']+6: pilot['stage']='descend'
   pilot['thr']=max(0.0,min(0.95, pilot['hover_thr']+0.25*(target-alt)-0.35*vz_up))
   if pilot['stage']=='descend' and alt<0.25 and abs(vz_up)<0.2:
    pilot['stage']='touchdown'; pilot['t_touch']=s.t; pilot['thr_touch']=pilot['thr']
  elif pilot['stage']=='touchdown':
   pilot['thr']=max(0.0, pilot['thr_touch']*(1.0-(s.t-pilot['t_touch'])/2.0))   # a pilot pulls the throttle down after touchdown
   if seen('lowering nose'): pilot['stage']='landed'; pilot['thr']=0.0
  elif pilot['stage']=='landed': pilot['thr']=0.0
  if s.t-pilot['last_send']>=0.05:
   pilot['last_send']=s.t
   try: link.send_manual_control(0.0,0.0,pilot['thr'],0.0)
   except Exception as exc: print('[pilot] manual control not sent:',exc,flush=True)
 def hook(s):
  global started,started_t,requested,land_sent,inspected
  if os.environ.get('NLF_PILOT_TEST'): pilot_step(s)
  if s.t>10 and not started:
   # The module's own work queue is created on 'start' and needs lockstep time to advance, which this hook blocks:
   # start it without waiting and send the request a little later in simulated time.
   started=True; started_t=s.t
   subprocess.Popen([str(bin/'px4-atlas_nose_lift'),'--instance',str(instance),'start'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  if started and not requested and s.t>started_t+2:
   requested=True; cli('atlas_nose_lift','land' if os.environ.get('NLF_LAND_ONLY') else 'takeoff')
  if s.step_count%5==0:
   b=s.sim
   rows.append([s.t,*b.pos.tolist(),*b.vel.tolist(),*np.degrees(b.euler).tolist(),*np.degrees(b.rates).tolist(),int(b.on_ground),int(link.actuator_armed),*b.cmd.tolist(), *(-(b.legs.r @ q_to_rotmat(b.q)[2] + b.pos[2] + b.legs.radius)).tolist()])
  if s.t>35 and not inspected and os.environ.get('NLF_INSPECT'):
   inspected=True
   for topic in ('vehicle_angular_velocity','vehicle_torque_setpoint','control_allocator_status','vehicle_land_detected'):
    cli('listener',topic,'-n','1')
  if s.t>float(os.environ.get('NLF_LAND_AT','50')) and not land_sent and os.environ.get("NLF_TEST_LANDING"):
   land_sent=True; cli('atlas_nose_lift','status'); cli('atlas_nose_lift','land')
 sim.hooks.append(hook)
 limit=float(os.environ.get('NLF_PILOT_LIMIT','90')) if os.environ.get('NLF_PILOT_TEST') else 20 if os.environ.get('NLF_LAND_ONLY') else (65 if os.environ.get('NLF_GUARD_TEST') else (float(os.environ.get('NLF_LAND_AT','50'))+60 if os.environ.get('NLF_TEST_LANDING') else 51))
 reason=sim.run_sync(until=lambda s:s.t>limit or (landing_complete_t is not None and s.t>landing_complete_t+8),max_wall_time=float(os.environ.get('NLF_WALL','300')))
 print('[harness] run ended:',reason,'sim_t=%.1f'%sim.t,flush=True)
 cli('atlas_nose_lift','status')
 rear_feet = sorted([np.asarray(l.foot()) for l in af.active_legs()], key=lambda p:p[0])[:2]
 result={'rear_pivot':(np.mean(rear_feet,axis=0)-af.cg).tolist(),'name':name,'physics':physics,'seed':seed,'params':params,'reason':reason,'cols':['t','n','e','d','vn','ve','vd','roll','pitch','yaw','p','q','r','ground','armed']+[f'm{i}' for i in range(1,11)]+[f'foot_gap{i}' for i in range(1,4)],'rows':rows,'log':lines}
 (base/'results'/f'{name}.json').write_text(json.dumps(result))
 a=np.asarray(rows)
 flight=a[(a[:,0]>40)&(a[:,0]<50)]
 print(json.dumps({'final':rows[-1][:15], 'hover_mean':flight.mean(axis=0)[:15].tolist() if len(flight) else None},indent=2))
finally:
 link.close(); stop_px4(proc)
