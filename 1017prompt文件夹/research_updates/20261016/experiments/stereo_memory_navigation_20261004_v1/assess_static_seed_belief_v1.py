"""Frozen controlled-task scores: postclose labels; no gate changes."""
from pathlib import Path
import datetime,json
import numpy as np
H=Path(__file__).resolve().parent
status=json.loads((H/'static_seed_belief_stage_status_v1.json').read_text())
verified=json.loads((H/'static_seed_belief_causal_verification_v1.json').read_text())
def geometry(d,p):
 w=p['world'];haz=(w[:,2]>.05)&(w[:,2]<=1.5);w=w[haz];i=int(p['frame']);gt=np.load(d/'offline_truth'/f'frame_{i:03d}.npz');es=[]
 for eye,key in [('left','query_pixel_id'),('right','partner_pixel_id')]:
  idx=p[key][haz];x,y=idx%640,idx//640;z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;es.append(np.linalg.norm(actual-w,axis=1))
 e=np.asarray(es).T;finite=np.isfinite(e).all(1);ok=finite&(e<=.1).all(1)
 return dict(n=len(w),correct=int(ok.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()) if len(w) else None,p90_finite_max_endpoint_error_m=float(np.quantile(e[finite].max(1),.9)) if finite.any() else None,correct_XY_bins=len(np.unique(np.floor(w[ok,:2]/.1).astype(int),axis=0)))

outcomes=[];loaded=[]
for e in status['arms']:
 a=e['arm'];assert e['stage']=='ACTUAL_CLOSED'
 d=H/f'static_seed_belief_{a}_run_v1';c=json.loads((H/f'static_seed_belief_{a}_collection_verification_v1.json').read_text())
 states=json.loads((d/'states.json').read_text());rows=json.loads((d/'policy_records.json').read_text());op=json.loads((d/'source_opportunity_records.json').read_text());identity=json.loads((d/'identity.json').read_text());task=json.loads((d/'task_episode.json').read_text());handoff=identity['observation_handoff_time_s'];seed=next((r['frame'] for r in op if r['qualified_seed']),None);scores=[]
 for f in sorted((d/'qualified_sensor_packets').glob('source_*.npz')):
  p=np.load(f);i=int(p['frame'])
  if seed is None or i<seed:continue
  scores.append(dict(frame=i,time_s=float(p['time_s']),current=geometry(d,p)))
 nonempty=[r['current'] for r in scores if r['current']['n']]
 sourcepass=bool(nonempty) and all(r['precision']>=.95 for r in nonempty)
 seedscore=geometry(d,np.load(d/'immutable_belief_seed.npz')) if (d/'immutable_belief_seed.npz').exists() else next((r['current'] for r in scores if r['frame']==seed),None)
 if seedscore is not None:sourcepass=sourcepass and seedscore['precision']>=.95
 xy=np.asarray([s['position'][:2] for s in states]);t=np.asarray([s['time_s'] for s in states]);goal=np.asarray(task['actual_episode']['goal_position_world_m'][:2]);start=np.asarray(task['actual_episode']['start_position_world_m'][:2]);distance=np.linalg.norm(xy-goal,axis=1);progress=xy[:,0]-start[0]
 ph=np.load(d/'physics.npz');pxy=ph['root_position_world_m'][:,:2] if 'root_position_world_m' in ph.files else xy
 # exact closest distance on every physical/evaluation segment to cylinder XY axis
 p0=pxy[:-1];v=pxy[1:]-p0;vv=np.sum(v*v,1);u=np.divide(-np.sum(p0*v,1),vv,out=np.zeros(len(v)),where=vv>0);closest=p0+np.clip(u,0,1)[:,None]*v;clearance=float(np.linalg.norm(closest,axis=1).min()-.75)
 blind=[];active=[]
 for i,r in enumerate(rows):
  m=np.load(d/'maps'/f'frame_{i:03d}.npz');sf=int(r['local']['source_frame']);q=np.load(d/'qualified_sensor_packets'/f'source_{sf:03d}.npz');w=q['world'];n=int(((w[:,2]>.05)&(w[:,2]<=1.5)).sum());old=int(np.isin(m['provenance'],[4,5]).sum());belief=int((m['provenance']==5).sum());delta=float(np.linalg.norm(xy[i+1]-xy[i]));in_crossing=abs(xy[i,0])<=.75
  active.append(dict(frame=i,time_s=float(t[i]),current_hazards=n,history_or_belief_cells=old,static_belief_cells=belief,move_m=delta,obstacle_x_range=in_crossing,final_request=r['local']['request']))
  if n==0 and old>0 and delta>1e-4 and in_crossing:blind.append(i)
 reached=bool(c['local_task_pass']);passed=bool(reached and sourcepass and clearance>=-1e-4 and xy[-1,0]>.75)
 clocks=json.loads((d/'camera_clocks.json').read_text());pred=[r['rgb_source_wall_s'] for r in clocks if r['source_predicted']]
 outcomes.append(dict(arm=a,decision='PASS_LIMITED_CONTROLLED_TASK' if passed else 'REJECT_LOCAL_CANDIDATE' if a=='C1' else 'BASELINE_TASK_FAILED' if handoff is not None else 'BOOTSTRAP_NOT_EXERCISED',handoff_time_s=handoff,seed_frame=seed,seed_score=seedscore,all_consumed_source_precision_ge95=bool(sourcepass),source_scores=scores,termination=c['termination'],elapsed_s=c['elapsed_s'],contacts=c['contact_samples'],actual_goal_reached=reached,final_distance_m=float(distance[-1]),path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),real_goal_axis_progress_m=float(progress[-1]),cross_RMS_m=float(np.sqrt(np.mean(xy[:,1]**2))),cross_peak_m=float(abs(xy[:,1]).max()),true_min_body_obstacle_clearance_m=clearance,actual_obstacle_segment_passed=bool(xy[-1,0]>.75),blind_zone_motion_frames=blind,blind_zone_pass_proven=bool(blind and passed),source_latency_p50_s=float(np.median(pred)),source_latency_p90_s=float(np.quantile(pred,.9)),policy_latency_p90_s=float(np.quantile([r['policy_wall_s'] for r in rows],.9)),memory_usage=active,knownpose_privileged=True,not_original_E1=True,original_formal_35_not_triggered=True))
 loaded.append((d,states,rows,identity,task))
comparison={};increment=False
if len(loaded)==2:
 d0,s0,r0,id0,task0=loaded[0];d1,s1,r1,id1,task1=loaded[1];N=min(len(r0),len(r1));a0=np.array([r['local']['request'] for r in r0[:N]]);a1=np.array([r['local']['request'] for r in r1[:N]]);diff=np.flatnonzero(np.any(a0!=a1,axis=1));branch=int(diff[0]) if len(diff) else N
 def firstdiff(fn):return next((i for i in range(branch+1) if not fn(i)),None)
 comparison=dict(first_executed_request_difference_frame=branch if len(diff) else None,same_actual_task=task0==task1,same_controller_source=id0['policy_act_source_sha256']==id1['policy_act_source_sha256'],same_runtime_class=id0['runtime_class']==id1['runtime_class'],same_rig=id0['camera_rig']==id1['camera_rig'],same_light_implementation=id0['lighting']==id1['lighting'],first_state_difference=firstdiff(lambda i:s0[i]==s1[i]))
 e0=json.loads((d0/'execution_states.json').read_text());e1=json.loads((d1/'execution_states.json').read_text());comparison['first_execution_snapshot_difference']=firstdiff(lambda i:e0[i]==e1[i])
 def rgbeq(i):
  x=np.load(d0/'online_inputs'/f'frame_{i:03d}.npz');y=np.load(d1/'online_inputs'/f'frame_{i:03d}.npz');return all(np.array_equal(x[k],y[k]) for k in x.files)
 comparison['first_RGB_difference']=firstdiff(rgbeq)
 comparison['strict_rendered_prefix_same']=comparison['first_RGB_difference'] is None and comparison['first_state_difference'] is None and comparison['first_execution_snapshot_difference'] is None
 x0=np.array([s['position'][0] for s in s0[:N+1]]);x1=np.array([s['position'][0] for s in s1[:N+1]]);comparison['minimum_shared_time_progress_delta_m']=float((x1-x0).min())
 b,c=outcomes;physicalbetter=not b['actual_goal_reached'] or c['true_min_body_obstacle_clearance_m']>b['true_min_body_obstacle_clearance_m']+.10 or c['contacts']<b['contacts']
 increment=bool(c['decision']=='PASS_LIMITED_CONTROLLED_TASK' and c['blind_zone_pass_proven'] and len(diff) and physicalbetter and comparison['minimum_shared_time_progress_delta_m']>=-.10 and comparison['strict_rendered_prefix_same'])
# Plot true trajectories, not candidate reference index.
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,6));theta=np.linspace(0,2*np.pi,200);ax.fill(.35*np.cos(theta),.35*np.sin(theta),color='gray',alpha=.5,label='physical cylinder');ax.plot(.75*np.cos(theta),.75*np.sin(theta),'k--',label='root collision boundary')
for o,(_,ss,_,_,_) in zip(outcomes,loaded):
 xy=np.array([s['position'][:2] for s in ss]);ax.plot(xy[:,0],xy[:,1],label=f"{o['arm']}: {o['termination']}");ax.scatter(*xy[-1],s=35)
ax.scatter([-2.2,2.2],[0,0],marker='*',s=100,c='green');ax.axis('equal');ax.grid();ax.legend();ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)');ax.set_title('Separate controlled task: known-pose binocular source / fixed roll-PID')
fig.savefig(H/'static_seed_belief_actual_v1.png',dpi=160);plt.close(fig)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=outcomes,comparison=comparison,memory_increment_accepted=increment,formal_triggered=False,original_strict_coldstart_unchanged=True,old_source_rejection_unchanged=True,whole_research_complete=False,all_labels_postclose_only=True)
out=H/'static_seed_belief_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(arms=[{k:v for k,v in r.items() if k not in ['source_scores','memory_usage','blind_zone_motion_frames']} for r in outcomes],comparison=comparison,memory_increment_accepted=increment),indent=2))
