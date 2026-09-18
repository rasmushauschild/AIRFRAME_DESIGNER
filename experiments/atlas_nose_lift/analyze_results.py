"""Evaluate recorded truth only; never supplies truth to the PX4 controller."""
import json, math, re, sys
from pathlib import Path
import numpy as np

def analyze(path):
 d=json.loads(Path(path).read_text()); a=np.asarray(d['rows']); col={n:i for i,n in enumerate(d['cols'])}
 def marker(text):
  hits=[float(re.search(r'sim_t=([\d.]+)',s)[1]) for s in d['log'] if text in s and 'sim_t=' in s]
  return hits[0] if hits else None
 start=marker('lifting nose'); handover=marker('nose settled'); done=marker('handover complete'); hover=marker('hover achieved')
 ground=a[(a[:,0]>=start)&(a[:,0]<handover)] if start and handover else np.empty((0,len(col)))
 flight=a[(a[:,0]>=handover)&(a[:,0]<50)] if handover else np.empty((0,len(col)))
 h=a[(a[:,0]>=40)&(a[:,0]<50)]
 result={'run':d['name'],'physics':d['physics'],'seed':d['seed'],'completed':d['reason']=='done','phase_times':{'lift':start,'climb':handover,'handover_complete':done,'hover':hover},'aborts':[s for s in d['log'] if '[atlas_nose_lift] abort:' in s]}
 if len(ground):
  result['ground']={'duration_s':round(handover-start,3),'last_pitch_deg':round(float(ground[-1,8]),3),'peak_pitch_rate_deg_s':round(float(np.abs(ground[:,11]).max()),3),'contacts_preserved':bool(np.all(ground[:,13]==1)),'peak_roll_deg':round(float(np.abs(ground[:,7]).max()),3),'yaw_change_deg':round(float(np.degrees(np.unwrap(np.radians(ground[:,9])))[-1]-ground[0,9]),3)}
 if len(flight):
  result['takeoff']={'peak_pitch_deg':round(float(flight[:,8].max()),3),'minimum_pitch_deg':round(float(flight[:,8].min()),3),'peak_roll_deg':round(float(np.abs(flight[:,7]).max()),3),'horizontal_excursion_m':round(float(np.linalg.norm(flight[:,1:3]-flight[0,1:3],axis=1).max()),3)}
 if len(h):
  yaw=np.degrees(np.unwrap(np.radians(h[:,9])))
  result['hover']={'height_mean_m':round(float(-h[:,3].mean()),3),'height_std_m':round(float(h[:,3].std()),3),'pitch_mean_deg':round(float(h[:,8].mean()),3),'roll_max_deg':round(float(np.abs(h[:,7]).max()),3),'yaw_drift_deg':round(float(yaw[-1]-yaw[0]),3),'yaw_rate_max_deg_s':round(float(np.abs(h[:,12]).max()),3),'horizontal_drift_m':round(float(np.linalg.norm(h[-1,1:3]-h[0,1:3])),3)}
 result['passes_takeoff_checks']=bool(hover and done and not result['aborts'] and len(ground) and result['ground']['contacts_preserved'] and abs(result['ground']['last_pitch_deg']-d['params'].get('NLF_TARGET',25))<2 and result['takeoff']['peak_roll_deg']<10 and result['takeoff']['peak_pitch_deg']<40 and abs(result['hover']['yaw_drift_deg'])<5 and result['hover']['height_std_m']<.15 and result['hover']['horizontal_drift_m']<1)
 return result
if __name__=='__main__':
 for path in sys.argv[1:]:
  print(json.dumps(analyze(path),indent=2))
