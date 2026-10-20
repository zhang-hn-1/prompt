from pathlib import Path
import json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def current_state(row):
 xy=np.asarray(row["position"],float)[:2];v=np.asarray(row["velocity"],float)[:2];q=np.asarray(row["quaternion"],float)
 if xy.shape!=(2,) or v.shape!=(2,) or q.shape!=(4,) or not np.isfinite(np.r_[xy,v,q]).all():raise ValueError("finite real current state")
 return xy,v,q
def project(xy,q):
 a=q[:-1];delta=np.diff(q,axis=0);length=np.linalg.norm(delta,axis=1);assert np.all(length>1e-9)
 u=np.clip(np.sum((xy-a)*delta,axis=1)/length**2,0,1);near=a+u[:,None]*delta;dist=np.linalg.norm(xy-near,axis=1);k=int(np.argmin(dist));arc=np.r_[0.,np.cumsum(length)];return float(arc[k]+u[k]*length[k]),float(dist[k]),float(np.arctan2(delta[k,1],delta[k,0])),int(k)
def main():
 inputs={str(Path(__file__)):sha(Path(__file__)),str(H/'first_reference_shape_attribution_scope_v1.json'):sha(H/'first_reference_shape_attribution_scope_v1.json')};arms={};refs=[]
 for arm in ('b0','candidate'):
  d=H/f'bank_qualified_navigation_e1_{arm}_run_v1';rp=d/'policy_records.json';sp=d/'states.json';rows=json.loads(rp.read_text());states=json.loads(sp.read_text());events=[i for i,r in enumerate(rows) if r.get('planner_act_called',False) and r.get('local_plan_created',False)];start,end=events[:2];lp=d/'local_plans'/f'frame_{start:03d}.json';plan=json.loads(lp.read_text());q=np.asarray([[w['x_m'],w['y_m']] for w in plan['waypoints']],float);refs.append(q)
  inputs.update({str(p):sha(p) for p in (rp,sp,lp)});records=[]
  for f in range(start,end):
   xy,v,quat=current_state(states[f]);arc,dist,tangent,k=project(xy,q);speed=float(np.linalg.norm(v));R=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix();body=float(np.arctan2(R[1,0],R[0,0]));wrap=lambda z:float((z+np.pi)%(2*np.pi)-np.pi)
   records.append(dict(frame=f,time_s=rows[f]['time_s'],actual_xy=xy.tolist(),continuous_reference_progress_m=arc,cross_track_m=dist,nearest_segment=k,tangent_rad=tangent,body_tangent_error_rad=wrap(body-tangent),actual_speed_m_s=speed,course_tangent_error_rad=wrap(np.arctan2(v[1],v[0])-tangent) if speed>.05 else None))
  err=np.asarray([r['cross_track_m'] for r in records]);points=np.asarray([r['actual_xy'] for r in records]);arms[arm]=dict(first_reference_frame=start,next_rebuild_frame=end,sampled_prefix_rows=len(records),reference_polyline_length_m=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),last_projection_progress_m=records[-1]['continuous_reference_progress_m'],maximum_projection_progress_m=max(r['continuous_reference_progress_m'] for r in records),cross_track_rms_m=float(np.sqrt(np.mean(err**2))),cross_track_peak_m=float(err.max()),sampled_actual_travel_m=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum()),records=records)
 assert np.array_equal(*refs)
 out=dict(arms=arms,identical_complete_first_reference_xy=True,reference_was_known_at_start=True,current_actual_state_retrospective_only=True,source_input_sha256=inputs,no_solver_actor_or_physics=True,not_control_or_model_admission=True,whole_mission_complete=False)
 p=H/'first_reference_shape_attribution_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:{n:v for n,v in a.items() if n!='records'} for k,a in arms.items()}))
if __name__=='__main__':main()
