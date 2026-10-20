"""Bounded post-run intervention/execution/time accounting; no cause attribution to GT model."""
from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).resolve().parent
if __name__=='__main__':
 result={}
 for arm in ('b0','candidate'):
  d=H/f'forward_course_geometry_e1_{arm}_run_v1';rows=json.loads((d/'policy_records.json').read_text());states=json.loads((d/'states.json').read_text());physics=dict(np.load(d/'physics.npz'));used=[r for r in rows if r.get('forward_course_geometry',{}).get('used')]
  groups={}
  for i,r in enumerate(rows):
   direction=r.get('forward_course_geometry',{}).get('composition_input',{}).get('active','no_composition')
   g=groups.setdefault(direction,dict(calls=0,axis_displacement_m=0.,travel_m=0.));g['calls']+=1
   delta=np.asarray(states[i+1]['position'][:2])-np.asarray(states[i]['position'][:2]);g['axis_displacement_m']+=float(delta[0]);g['travel_m']+=float(np.linalg.norm(delta))
  p=np.asarray([r['request'][:2] for r in rows]);result[arm]=dict(geometry_used=len(used),used_frames=[r['frame'] for r in used],direction_accounting=groups,protocol_aux_at_30deg_calls=int((np.abs(np.abs(p[:,1])-np.pi/6)<1e-6).sum()))
  if used:
   r=used[0];c=r['current_tracking_chain'];meta=r['forward_course_geometry'];i=r['frame'];step=i*5+1
   result[arm]['first_event']=dict(frame=i,time_s=r['time_s'],parent_curvature=meta['parent_curvature'],selected_curvature=meta['selected_curvature'],course_heading=meta['course_heading_rad'],root_yaw=c['root_heading_rad'],request=r['request'][:2],applied_next_tick=physics['applied_action'][step].tolist(),previous_applied=physics['applied_action'][step-1].tolist())
 result.update(scope='Actual direction/time accounting and first consumer/tick only. Diverged-arm later totals are descriptive, not unique causal failure proof; no rescans or new performance gates.',new_physics=0,no_fit=True,whole_mission_complete=False)
 p=H/'forward_course_geometry_e1_causal_accounting_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
