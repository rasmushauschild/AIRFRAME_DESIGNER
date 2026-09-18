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
 if 'atlas_nose_lift' in s or 'Armed' in s or 'Disarmed' in s or 'ERROR' in s or 'WARN' in s: print(s,flush=True)
proc=launch_px4(root,'none_iris',log,instance=instance,rootfs=str(inst.workdir),params=params,param_types=types,quiet=False)
link=PX4Link('sitl',inst.tcp_address,ctl_address=inst.ctl_address,log=log)
rows=[]; started=False; land_sent=False; started_t=0; inspected=False
try:
 link.open()
 sim=Simulator(af,link,sensor_rate=250,physics_substeps=4,speed=0,lockstep=True,log=log,seed=seed,physics=physics)
 def cli(module,*args):
  r=subprocess.run([str(bin/f'px4-{module}'),'--instance',str(instance),*args],capture_output=True,text=True,timeout=5)
  print(r.stdout,r.stderr,flush=True)
  return r
 def hook(s):
  global started,started_t,land_sent,inspected
  if s.t>10 and not started:
   started=True; started_t=s.t
   cli('atlas_nose_lift','start'); cli('atlas_nose_lift','land' if os.environ.get('NLF_LAND_ONLY') else 'takeoff')
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
 limit=20 if os.environ.get('NLF_LAND_ONLY') else (65 if os.environ.get('NLF_GUARD_TEST') else (float(os.environ.get('NLF_LAND_AT','50'))+60 if os.environ.get('NLF_TEST_LANDING') else 51))
 reason=sim.run_sync(until=lambda s:s.t>limit or (landing_complete_t is not None and s.t>landing_complete_t+8),max_wall_time=300)
 cli('atlas_nose_lift','status')
 result={'name':name,'physics':physics,'seed':seed,'params':params,'reason':reason,'cols':['t','n','e','d','vn','ve','vd','roll','pitch','yaw','p','q','r','ground','armed']+[f'm{i}' for i in range(1,11)]+[f'foot_gap{i}' for i in range(1,4)],'rows':rows,'log':lines}
 (base/'results'/f'{name}.json').write_text(json.dumps(result))
 a=np.asarray(rows)
 flight=a[(a[:,0]>40)&(a[:,0]<50)]
 print(json.dumps({'final':rows[-1][:15], 'hover_mean':flight.mean(axis=0)[:15].tolist() if len(flight) else None},indent=2))
finally:
 link.close(); stop_px4(proc)
