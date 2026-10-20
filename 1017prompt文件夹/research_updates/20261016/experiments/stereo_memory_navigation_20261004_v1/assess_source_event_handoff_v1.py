"""All-source post-close local A/B score; actual outcomes, never index success."""
from pathlib import Path
import datetime,json
import numpy as np
from route_geometry_audit import hazard_clearance
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent
verified=json.loads((H/'source_event_handoff_causal_verification_v1.json').read_text())
outcomes=[];loaded=[]
def geometry(d,p):
 w=p['world'];haz=(w[:,2]>.05)&(w[:,2]<=1.5);w=w[haz];i=int(p['frame']);gt=np.load(d/'offline_truth'/f'frame_{i:03d}.npz');es=[]
 for eye,key in [('left','query_pixel_id'),('right','partner_pixel_id')]:
  idx=p[key][haz];x,y=idx%640,idx//640;z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;es.append(np.linalg.norm(actual-w,axis=1))
 e=np.asarray(es).T;finite=np.isfinite(e).all(1);ok=finite&(e<=.1).all(1)
 return dict(n=len(w),correct=int(ok.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()) if len(w) else None,p90_finite_max_endpoint_error_m=float(np.quantile(e[finite].max(1),.9)) if finite.any() else None,correct_XY_bins=len(np.unique(np.floor(w[ok,:2]/.1).astype(int),axis=0)))
for a in ('B0','C1'):
 d=H/f'source_event_handoff_{a}_run_v1';c=json.loads((H/f'source_event_handoff_{a}_collection_verification_v1.json').read_text());v=next(x for x in verified['arms'] if x['arm']==a)
 states=json.loads((d/'states.json').read_text());rows=json.loads((d/'policy_records.json').read_text());op=json.loads((d/'source_opportunity_records.json').read_text());identity=json.loads((d/'identity.json').read_text());task=json.loads((d/'task_episode.json').read_text());handoff=v['handoff_time_s'];seed=next(r['frame'] for r in op if r['qualified_seed']);scores=[]
 for f in sorted((d/'qualified_sensor_packets').glob('source_*.npz')):
  p=np.load(f);i=int(p['frame'])
  if i<seed:continue
  cur=geometry(d,p);past=np.load(d/'appearance_old_packets'/f.name);old=geometry(d,past)
  scores.append(dict(frame=i,time_s=float(p['time_s']),current=cur,past_birth_frame=int(past['frame']),past_birth_time_s=float(past['time_s']),old_appearance=old))
 nonempty=[g for r in scores for g in [r['current'],r['old_appearance']] if g['n']]
 sourcepass=bool(nonempty) and all(r['precision']>=.95 for r in nonempty)
 xy=np.asarray([s['position'][:2] for s in states]);t=np.asarray([s['time_s'] for s in states]);start=np.asarray(task['actual_episode']['start_position_world_m'][:2]);goal=np.asarray(task['actual_episode']['goal_position_world_m'][:2]);direction=(goal-start)/np.linalg.norm(goal-start)
 distance=np.linalg.norm(xy-goal,axis=1);progress=(xy-start)@direction;cross=(xy-start)@np.array([-direction[1],direction[0]])
 geometry_rows=[];truth=np.load(d/'offline_occupancy_label.npz');proxy=np.where(truth['labels']>0,1,-1).astype(np.int8)
 for i,r in enumerate(rows):
  if r['local']['time_s']<handoff-1e-6:continue
  m=np.load(d/'maps'/f'frame_{i:03d}.npz');plan=np.load(d/'local_plans'/f'frame_{i:03d}.npz');haz=m['origin']+np.argwhere(m['labels']==1)*float(m['resolution']);actual=hazard_clearance(xy[i:i+2],haz)
  saved=reference_geometry_status(xy[i],plan['pose'][:,:2],m['labels'],m['origin'],float(m['resolution']));gt=reference_geometry_status(xy[i],plan['pose'][:,:2],proxy,truth['origin'],float(truth['resolution']))
  geometry_rows.append(dict(frame=i,time_s=float(t[i]),reference_version=int(plan['version']),reference_birth_time_s=float(plan['time_s']),actual_observed_point_clearance_m=actual,observed_reference=saved,offline_proxy_reference=gt,final_request=r['local']['request'],reference_decision=r['local']['reference_decision']))
 clear=[r['actual_observed_point_clearance_m'] for r in geometry_rows if r['actual_observed_point_clearance_m'] is not None];clearpass=bool(clear) and min(clear)>=.6-1e-8
 passed=c['local_task_pass'] and sourcepass and clearpass
 result=dict(arm=a,decision='PASS_LIMITED_LOCAL_ONLY' if passed else 'REJECT_LOCAL_CONTROL_CANDIDATE' if a=='C1' else 'BASELINE_LOCAL_TASK_FAILED',handoff_time_s=handoff,seed_frame=seed,source_quality_all_nonempty_ge95=sourcepass,all_source_scores=scores,necessary_observed_point_clearance_ge06=clearpass,minimum_observed_point_clearance_m=min(clear) if clear else None,geometry_trace=geometry_rows,termination=c['termination'],elapsed_s=c['elapsed_s'],contact_samples=c['contact_samples'],final_distance_m=float(distance[-1]),minimum_distance_m=float(distance.min()),actual_goal_axis_progress_m=float(progress[-1]),distance_reduction_m=float(distance[0]-distance[-1]),post_handoff_axis_gain_m=float(progress[-1]-progress[int(round(handoff*10))]),cross_RMS_m=float(np.sqrt(np.mean(cross**2))),cross_peak_m=float(np.abs(cross).max()),path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),local_task_pass=c['local_task_pass'],goal_entry_frames=np.flatnonzero(distance<=.2).tolist(),knownpose_privileged=True,not_formal_navigation=True,source_model_global_REJECT_preserved=True)
 outcomes.append(result);loaded.append((d,states,rows,identity,task))
# Independently verify physical/execution/policy/rendered prefix before first changed final request.
d0,s0,r0,id0,task0=loaded[0];d1,s1,r1,id1,task1=loaded[1];N=min(len(r0),len(r1));actions0=np.array([r['local']['request'] for r in r0[:N]]);actions1=np.array([r['local']['request'] for r in r1[:N]]);diff=np.flatnonzero(np.any(actions0!=actions1,axis=1));branch=int(diff[0]) if len(diff) else N
prefix=dict(first_final_request_difference_frame=branch,same_actual_task=task0==task1,same_scene=id0['scene_sha256']==id1['scene_sha256'],same_controller_class=id0['controller_class']==id1['controller_class'],same_runtime_class=id0['runtime_class']==id1['runtime_class'],same_policy_source=id0['policy_act_source_sha256']==id1['policy_act_source_sha256'],same_rig=id0['camera_rig']==id1['camera_rig'],same_light=id0['lighting']==id1['lighting'],same_source_identity=id0['source_sha256']==id1['source_sha256'])
def first_difference(fn,stop):
 for i in range(stop):
  if not fn(i):return i
 return None
prefix['first_state_difference']=first_difference(lambda i:s0[i]==s1[i],branch+1)
e0=json.loads((d0/'execution_states.json').read_text());e1=json.loads((d1/'execution_states.json').read_text());prefix['first_execution_snapshot_difference']=first_difference(lambda i:e0[i]==e1[i],branch+1)
for folder in ['online_inputs','history','known_poses','policy_inputs']:
 def eq(i):
  f=np.load(d0/folder/f'frame_{i:03d}.npz');g=np.load(d1/folder/f'frame_{i:03d}.npz')
  keys=set(f.files)-({'request'} if folder=='policy_inputs' else set())
  return keys==set(g.files)-({'request'} if folder=='policy_inputs' else set()) and all(np.array_equal(f[k],g[k],equal_nan=True) for k in keys)
 prefix['first_'+folder+'_difference']=first_difference(eq,branch+1)
prefix['first_nominal_request_difference']=first_difference(lambda i:r0[i]['local']['nominal_request']==r1[i]['local']['nominal_request'],branch)
prefix['first_tracker_diagnostic_difference']=first_difference(lambda i:r0[i]['tracking']==r1[i]['tracking'] and r0[i]['mapping']==r1[i]['mapping'],branch)
prefix['strict_complete_prefix_certified']=all(v is None for k,v in prefix.items() if k.startswith('first_') and k!='first_final_request_difference_frame')
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=outcomes,prefix_comparison=prefix,original35_rule=True,formal_triggered=False,whole_research_complete=False,GT_postclose_only=True,unknown_floor_not_certified_free=True,source_clearance_necessary_not_coverage_certificate=True,no_threshold_or_scientific_retry=True)
out=H/'source_event_handoff_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(arms=[{k:v for k,v in r.items() if k not in ['all_source_scores','geometry_trace']} for r in outcomes],prefix_comparison=prefix),indent=2))
