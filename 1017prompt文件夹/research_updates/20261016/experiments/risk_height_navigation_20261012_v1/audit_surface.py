"""Offline scoring of saved accepted hazard heights; truth is never online input."""
from pathlib import Path
import json,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];summary=[]
for name,run in [('original','nearfield_rig_navigation_20261012_v1/baseline_e2_run'),('forward','nearfield_rig_navigation_20261012_v1/candidate_e2_run'),('compact','nearfield_compact_navigation_20261012_v1/candidate_e2_run')]:
 R=W/'experiments'/run;counts=dict(accepted=0,truth_valid=0,truth_floor=0,truth_obstacle=0,interval_ambiguous=0,ambiguous_truth_floor=0,ambiguous_truth_obstacle=0);examples=[]
 for p in sorted((R/'partial_observations').glob('*.npz')):
  a=np.load(p);frame=np.load(R/'frames'/p.name);q=a['query_pixel_id'];x=q%640;y=q//640;gt=frame['left_depth_m'][y,x];valid=np.isfinite(gt)&(gt>0);T=frame['left_camera_usd_world'];rays=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))]);world=T[:3,3]+(rays*gt[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
  floor=valid&(abs(world[:,2])<=.03);obstacle=valid&(world[:,2]>.05)&(world[:,2]<=1.5)
  amb=(a['world'][:,2]-a['sigma_m']<=.05)
  for k,v in [('accepted',len(q)),('truth_valid',valid.sum()),('truth_floor',floor.sum()),('truth_obstacle',obstacle.sum()),('interval_ambiguous',amb.sum()),('ambiguous_truth_floor',(amb&floor).sum()),('ambiguous_truth_obstacle',(amb&obstacle).sum())]:counts[k]+=int(v)
  if len(q):examples.append(dict(frame=p.stem,n=len(q),floor=int(floor.sum()),obstacle=int(obstacle.sum()),ambiguous=int(amb.sum())))
 summary.append(dict(name=name,counts=counts,largest_false_floor_frames=sorted(examples,key=lambda x:x['floor'],reverse=True)[:5]))
out=dict(scope='Only saved accepted surface points; renderer same-frame truth for offline qualification; floor bandabsZ<=.03 is scoring tolerance, not an online gate. Sigma is existing1pixel approximation, not calibrated confidence. Multiple points/frames are not independent samples.',runs=summary)
(D/'surface_height_audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
