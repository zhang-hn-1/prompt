"""Offline actual phase/causal-prefix descriptions; no policy or planner calls."""
from pathlib import Path
import json,datetime
import numpy as np
from route_geometry_audit import project
H=Path(__file__).resolve().parent

def phase_rows(folder,ref):
 a=np.load(folder/'episode_arrays.npz');rows=json.loads((folder/'policy_records.json').read_text());xy=a['positions_world_xy'];pr,er,_=project(xy,ref);speed=np.linalg.norm(a['linear_velocity_world_m_s'][:,:2],axis=1);phases=[]
 for i,r in enumerate(rows):
  source=r['request']['request_source'];direction=r['diagnostics']['active_drive_direction'];key=(source,direction)
  if not phases or (phases[-1]['source'],phases[-1]['accepted_direction'])!=key:phases.append(dict(source=source,accepted_direction=direction,start_frame=i,end_frame=i+1))
  else:phases[-1]['end_frame']=i+1
 for r in phases:
  i,j=r['start_frame'],r['end_frame'];count=0;qual=None
  for k in range(i,j):
   count=count+1 if speed[k]<.05 else 0
   if count>=5 and qual is None:qual=k/10.
  r.update(start_time_s=i/10.,end_time_s=j/10.,duration_s=(j-i)/10.,independent_reference_progress_m=float(pr[j]-pr[i]),entry_error_m=float(er[i]),exit_error_m=float(er[j]),peak_error_m=float(er[i:j+1].max()),actual_path_m=float(np.linalg.norm(np.diff(xy[i:j+1],axis=0),axis=1).sum()),xy_speed_peak_m_s=float(speed[i:j+1].max()),five_low_from_phase_start_time_s=qual)
 changes=[]
 for i in range(1,len(rows)):
  prev=rows[i-1]['diagnostics']['active_drive_direction'];now=rows[i]['diagnostics']['active_drive_direction']
  if prev!=now:changes.append(dict(frame=i,time_s=i/10.,previous=prev,accepted=now,main_joint_rad_s=float(a['joint_velocity_rad_s'][i,0]),applied_main_rad_s=float(a['applied_action'][i,0]),actual_xy_speed_m_s=float(speed[i])))
 brake=[r for r in phases if r['source']=='direction_change_braking']
 return dict(phases=phases,accepted_direction_changes=changes,total_braking_s=sum(r['duration_s'] for r in brake),braking_intervals=brake,final_reference_progress_m=float(pr[-1]),final_lateral_error_m=float(er[-1]))

def main():
 assert json.loads((H/'root_direction_status.json').read_text())['exit_code']==0
 ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2]
 d=dict(audited_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),baseline=phase_rows(H/'guarded_reference_run',ref),candidate=phase_rows(H/'root_direction_run',ref),offline_only=True,no_alternate_plan_truth=True)
 (H/'root_direction_phase_audit.json').write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({k:{x:v[x] for x in ['total_braking_s','accepted_direction_changes','braking_intervals']} for k,v in d.items() if k in ['baseline','candidate']},indent=2))
if __name__=='__main__':main()
