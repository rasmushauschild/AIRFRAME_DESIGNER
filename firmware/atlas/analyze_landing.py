"""Evaluate landing against recorded simulator truth, never used by the controller."""
import json,re,sys
from pathlib import Path
import numpy as np

def analyze(path):
 d=json.loads(Path(path).read_text());a=np.asarray(d['rows'])
 def marker(text):
  hits=[float(re.search(r'sim_t=([\d.]+)',s)[1]) for s in d['log'] if text in s and 'sim_t=' in s]
  return hits[0] if hits else None
 start=marker('landing: descending');lower=marker('rear support detected');stop=marker('nose settled; ramping');done=marker('landing complete:')
 result={'run':d['name'],'physics':d['physics'],'seed':d['seed'],'completed':d['reason']=='done','phase_times':{'descend':start,'lower_nose':lower,'shutdown':stop,'complete':done},'aborts':[s for s in d['log'] if '[atlas_nose_lift] abort:' in s]}
 if not start or not lower or not stop or not done:
  result['passes_landing_checks']=False;return result
 land=a[a[:,0]>=start];rear_hits=np.flatnonzero((land[:,26]<=0)&(land[:,27]<=0));nose_hits=np.flatnonzero(land[:,25]<=0)
 if not len(rear_hits) or not len(nose_hits):result['passes_landing_checks']=False;return result
 rear=land[rear_hits[0]];nose=land[nose_hits[0]];before_rear=land[max(0,rear_hits[0]-1)]
 lowering=land[(land[:,0]>=lower)&(land[:,0]<stop)];ground=land[land[:,0]>=rear[0]];tail=land[land[:,0]>done+2]
 disarmed=land[land[:,14]==0];first_disarmed=disarmed[0] if len(disarmed) else land[-1]
 phi,theta,psi=np.radians(ground[:,7]),np.radians(ground[:,8]),np.radians(ground[:,9])
 # Rear support midpoint in world coordinates: distinguishes gear sliding from CG motion during pitching.
 px,py,pz=d.get('rear_pivot',[-.3,0,.3])
 rx=px*np.cos(psi)*np.cos(theta)+py*(np.cos(psi)*np.sin(theta)*np.sin(phi)-np.sin(psi)*np.cos(phi))+pz*(np.cos(psi)*np.sin(theta)*np.cos(phi)+np.sin(psi)*np.sin(phi))
 ry=px*np.sin(psi)*np.cos(theta)+py*(np.sin(psi)*np.sin(theta)*np.sin(phi)+np.cos(psi)*np.cos(phi))+pz*(np.sin(psi)*np.sin(theta)*np.cos(phi)-np.cos(psi)*np.sin(phi))
 rear_xy=ground[:,1:3]+np.stack([rx,ry],axis=1)
 result.update({'rear_touchdown_time_s':float(rear[0]),'rear_touchdown_descent_m_s':round(float(before_rear[6]),4),'rear_touchdown_pitch_deg':round(float(rear[8]),3),'nose_clearance_at_rear_touchdown_m':round(float(rear[25]),3),'nose_touchdown_time_s':float(nose[0]),'nose_touchdown_pitch_deg':round(float(nose[8]),3),'lowering_peak_pitch_rate_deg_s':round(float(np.abs(lowering[:,11]).max()),3),'ground_contact_preserved':bool(np.all(ground[:,13]==1)),'rear_feet_contact_preserved':bool(np.all(ground[:,26:28]<=.002)),'peak_roll_deg':round(float(np.abs(land[:,7]).max()),3),'rear_support_drift_m':round(float(np.linalg.norm(rear_xy-rear_xy[0],axis=1).max()),3),'landing_horizontal_excursion_m':round(float(np.linalg.norm(land[:,1:3]-land[0,1:3],axis=1).max()),3),'disarm_time_s':float(first_disarmed[0]),'nose_grounded_before_disarm':bool(nose[0]<first_disarmed[0] and first_disarmed[25]<=.002),'nose_grounded_before_shutdown':bool(nose[0]<=stop),'final_pitch_deg':round(float(land[-1,8]),3),'final_disarmed':bool(land[-1,14]==0),'all_motors_off':bool(len(tail) and np.all(np.abs(tail[:,15:25])<1e-6))})
 result['passes_landing_checks']=bool(result['completed'] and not result['aborts'] and rear[0]<nose[0] and rear[25]>.3 and result['ground_contact_preserved'] and result['rear_feet_contact_preserved'] and result['rear_touchdown_descent_m_s']<.15 and result['lowering_peak_pitch_rate_deg_s']<8 and result['peak_roll_deg']<3 and result['rear_support_drift_m']<.2 and result['nose_grounded_before_shutdown'] and result['nose_grounded_before_disarm'] and result['final_disarmed'] and result['all_motors_off'])
 return result
if __name__=='__main__':
 for path in sys.argv[1:]:print(json.dumps(analyze(path),indent=2))
