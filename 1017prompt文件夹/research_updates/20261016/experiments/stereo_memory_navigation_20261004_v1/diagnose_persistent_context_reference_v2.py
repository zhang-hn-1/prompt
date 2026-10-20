"""Current installed remaining-reference relationship, no controller recall or future plan."""
from pathlib import Path
import json,numpy as np
from remaining_reference_v1 import project_remaining
H=Path(__file__).resolve().parent;D=H/'flat_ground_pose_D3_run_v1';O=H/'persistent_context_D3_historical_v1';b=np.load(D/'immutable_possible_set_birth.npz');pts=b['possible_xy'];states=json.loads((D/'states.json').read_text());rows=[]
def distance(points,line):
 if len(line)<2:return float(np.linalg.norm(points-line[0],axis=1).min())
 out=np.full(len(points),np.inf)
 for a,z in zip(line[:-1],line[1:]):
  v=z-a;n=v@v;q=np.clip((points-a)@v/n,0,1) if n>0 else np.zeros(len(points));out=np.minimum(out,np.linalg.norm(points-(a+q[:,None]*v),axis=1))
 return float(out.min())
for f in [20,100,200,300,400]:
 root=np.asarray(states[f]['position'])[:2];p=D/'local_plans'/f'frame_{f:03d}.npz';chord=np.stack([root,np.array([2.2,0.])]);row=dict(frame=f,current_time_s=states[f]['time_s'],current_goal_chord_nearest_MODEL_distance_m=distance(pts,chord),current_plan_available=p.exists(),role='hypothesis distance only; chord not counterfactual executing path')
 if p.exists():
  z=np.load(p);assert float(z['time_s'])<=states[f]['time_s'];q=project_remaining(z['pose'][:,:2],root);line=np.asarray(q['remaining_xy']);row.update(plan_source_time_s=float(z['time_s']),remaining_reference_nearest_MODEL_distance_m=distance(pts,line),projection_distance_m=q['distance_m'],ambiguous_projection=q['ambiguous_progress'])
 rows.append(row)
r=dict(scope='HISTORICAL_REMAINING_REFERENCE_NOT_CONTROL_AB',rows=rows,no_planner_recall_or_future_truth=True,clearance_m=.8707106781186548,v1_missing_frame020_failure_preserved=True,no_future_plan_used_to_fill_missing=True);(O/'reference_MODEL_relationship_v2.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(rows,indent=2))
