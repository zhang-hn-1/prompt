"""Closed-run factual phase/command audit, not a predictor or alternate rollout."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1'
assert json.loads((H/'minimum_prefix_transition_navigation_status_v1.json').read_text())['status']=='finished'
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s
rows=json.loads((D/'policy_records.json').read_text());req=np.load(D/'requests.npz')['requests'];z=np.load(D/'physics.npz');xy=z['position_world_xy'].astype(float);goal=np.array([3.300000005960465,5.960465e-9]);segments=[];covered=[];types=[];errors=[]
for i,r in enumerate(rows):
 if r['prefix_observer']:label='initial_view_acquisition'
 elif req[i,0]>0:label='normal_forward'
 elif req[i,0]<0:label='normal_reverse'
 elif r['tracker_state_after_act']['pending_direction'] is not None:label='low_speed_direction_confirmation'
 else:label='no_route_or_other_zero'
 types.append(label)
 p=D/'local_plans'/f'frame_{i:03d}.json';plan=json.loads(p.read_text()) if p.exists() else None
 if plan is not None and req[i,0]<0:
  index=r['tracker_state_after_act']['path_index'];w=plan['waypoints'][index]
  covered.append(dict(frame=i,planned_direction=w['drive_direction'],planned_main=w['main_command_rad_s'],planned_aux=w['auxiliary_angle_rad'],actual_request_main=float(req[i,0]),actual_request_aux=float(req[i,1]),reverse_carrier_covered=bool(abs(w['main_command_rad_s']+.5)<1e-6 and abs(w['auxiliary_angle_rad'])<1e-6),requested_response_point_covered=bool(abs(req[i,0]+.3)<1e-6 and abs(req[i,1])<1e-6)))
 if plan is not None:
  q=np.array([[w['x_m'],w['y_m']] for w in plan['waypoints']]);delta=np.diff(q,axis=0);den=np.sum(delta**2,axis=1);u=np.clip(np.divide(np.sum((xy[i*5]-q[:-1])*delta,axis=1),den,out=np.zeros(len(den)),where=den>0),0,1);projection=q[:-1]+u[:,None]*delta;errors.append(float(np.linalg.norm(projection-xy[i*5],axis=1).min()))
start=0
for i in range(1,len(types)+1):
 if i==len(types) or types[i]!=types[start]:
  a=xy[start*5];b=xy[i*5];segments.append(dict(phase=types[start],start_frame=start,end_frame_exclusive=i,duration_s=(i-start)*.1,true_axis_delta_m=float(b[0]-a[0]),goal_enter_m=float(np.linalg.norm(a-goal)),goal_exit_m=float(np.linalg.norm(b-goal)),true_path_m=float(np.linalg.norm(np.diff(xy[start*5:i*5+1],axis=0),axis=1).sum())));start=i
r=dict(decision='CLOSED_FACTUAL_PHASE_AND_REQUEST_DOMAIN_AUDIT',durations_s={k:sum(s['duration_s'] for s in segments if s['phase']==k) for k in sorted(set(types))},segments=segments,reverse_requests=len(covered),planned_reverse_carrier_covered=sum(v['reverse_carrier_covered'] for v in covered),actual_reverse_requests_outside_frozen_straight_response_point=sum(not v['requested_response_point_covered'] for v in covered),reverse_request_evidence=covered,query_attempts=sum(bool(x.get('first_primitive_direction_query')) for x in rows),masked_query_count=sum(bool(x.get('first_primitive_direction_query')) and x['first_primitive_direction_query']['direction']=='forward' for x in rows),moving_to_installed_reference_cross_RMS_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,cross_peak_m=max(errors) if errors else None,error_projection_reference_changes_not_success=True,not_confirmed_control_bug=True,not_new_physics=True,no_model_fit=True,not_navigation_admission=True,whole_research_complete=False)
(H/'minimum_prefix_transition_failure_and_command_domain_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['segments','reverse_request_evidence']},indent=2))
