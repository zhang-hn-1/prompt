from pathlib import Path
import sys,json,datetime,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent
for n in ['oracle_feasible_route_20260929_v9/hybrid','bilateral_temporal_geometry_navigation_20261003_v1','clean_visual_hard0_intervention_20261003_v1','visible_surface_connectivity_20261004_v1']:sys.path.insert(0,str(H.parent/n))
from negative_memory_validation import archives_equal,true_cylinder_clearance
from persistent_core_grid import PersistentCoreGrid
from route_geometry_audit import hazard_clearance
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
from bank_rock_observation_schedule import request
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'negative_ray_memory_stage_status_v2.json').read_text());assert status['stage']=='ACTUAL_PAIR_CLOSED_NEEDS_VERIFICATION';results=[];pairdata={}
for arm in ['R0','R1']:
 D=H/('negative_ray_memory_'+arm+'_run_v2');st=status['arms'][arm];assert st['complete'];seal=D/'artifact_manifest_post_close.json';assert sha(seal)==st['post_close_manifest_sha256'];man=json.loads(seal.read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in man.items())
 rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());contacts=json.loads((D/'contacts.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());es=json.loads((D/'execution_states.json').read_text());ph=np.load(D/'physics.npz');metrics=json.loads((D/'metrics.json').read_text());identity=json.loads((D/'identity.json').read_text());seed=dict(np.load(D/'frozen_memory_seed.npz'));poseaudit=json.loads((H/('negative_ray_memory_'+arm+'_sensor_verification_v2.json')).read_text());srcaudit=json.loads((H/('negative_ray_memory_'+arm+'_source_verification_v2.json')).read_text());N=len(rows)
 assert len(es)==N+1 and es[0] is None and len(clocks)==len(states)==N+1
 for i,r in enumerate(rows):
  if i<140:np.testing.assert_equal(r['request'],request(i));assert not r['planner_act_called']
  else:assert r['planner_act_called']
  np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],np.asarray(r['request']).reshape(5,2))
 clone=ExecutionClone.from_snapshot(es[1]);ticks=0
 for tick in range(5,5*N):
  applied,smooth=clone.step(ph['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]),aux_position_rad=ph['joint_position_rad'][tick,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][tick,1]);np.testing.assert_equal(applied,ph['applied_action'][tick+1]);np.testing.assert_equal(smooth,ph['smoothed_target'][tick+1]);ticks+=1
  if (tick+1)%5==0:assert clone.snapshot()==es[(tick+1)//5]
 init=np.load(D/'estimated_poses/frame_000.npz');g=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60);g.set_source(dict(time_s=float(init['time_s'])));g.update_points(np.empty((0,3)),init['root_position_world_m'],float(init['time_s']));g.seed(seed,float(seed['time_s']));mapfields=0;nominal=[];true_guards=[];guardcount=0;zero=[]
 for i,r in enumerate(rows):
  if i<140:continue
  l=r['local'];pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');t=float(pose['time_s']);sf=l['source_frame'];q=dict(np.load(D/'sensor_packets'/f'source_{sf:03d}.npz'));assert sf<=i and float(q['time_s'])<=t+1e-9;g.set_source(q);g.update_points(np.empty((0,3)),pose['root_position_world_m'],t);saved=np.load(D/'maps'/f'frame_{i:03d}.npz')
  for k,v in dict(labels=g.labels,provenance=g.provenance(),floor_time=g.memory.floor_time,hazard_time=g.memory.hazard_time,hypothesis_birth_time=g.hypothesis_birth_time,task_ground_prior=g.task_ground_prior,safe_start_prior=g.safe_start_prior).items():np.testing.assert_equal(v,saved[k]);mapfields+=1
  if np.all(np.asarray(r['request'])==0):zero.append(i)
  assert not np.isfinite(saved['floor_time']).any() and not np.isfinite(saved['hazard_time']).any();haz=saved['origin']+np.argwhere(saved['labels']==1)*float(saved['resolution'])
  dec=l['reference_decision'];guard=None if dec is None else dec.get('guard')
  if l['guard_measured_this_call'] and guard is not None:
   for key in ['suffix','bridge']:np.testing.assert_allclose(hazard_clearance(np.asarray(guard[key+'_xy']),haz),guard[key+'_clearance_m'],atol=1e-9)
   assert (dec['decision']=='retain')==guard['retention_necessary_screen_pass'];guardcount+=1
   true_guards.append(dict(frame=i,decision=dec['decision'],suffix_true_clearance_m=true_cylinder_clearance(guard['suffix_xy']),bridge_true_clearance_m=true_cylinder_clearance(guard['bridge_xy'])))
  if l['local_plan_created'] and l['plan'].get('status')=='success':
   plan=np.load(D/'local_plans'/f'frame_{i:03d}.npz');nominal.append(dict(frame=i,clearance_m=float(hazard_clearance(plan['pose'][:,:2],haz)),true_clearance_m=true_cylinder_clearance(plan['pose'][:,:2])))
 xy=np.array([s['position'][:2] for s in states]);xyz=np.array([s['position'] for s in states]);speed=np.array([np.linalg.norm(s['velocity'][:2]) for s in states]);nlow=0
 for v in speed[:141]:nlow=nlow+1 if v<.05 else 0
 videos={}
 for name in ['stereo_view','third_person']:
  c=cv2.VideoCapture(str(D/(name+'.mp4')));n=0
  while True:
   ok,_=c.read()
   if not ok:break
   n+=1
  c.release();videos[name]=n
 actual=dict(elapsed_s=metrics['elapsed_s'],termination_reason=metrics['termination_reason'],native_success=metrics['success'],final_XY_distance_m=float(np.linalg.norm(xy[-1]-[2.2,.15])),final_XYZ_distance_m=float(np.linalg.norm(xyz[-1]-[2.2,.15,.45])),axis_progress_m=float(xy[-1,0]-xy[0,0]),obstacle_passed=bool(xy[:,0].max()>.75),path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),body_proxy_clearance_m=float(hazard_clearance(xy,np.array([[0.,0.]]))-.75),contact_frames=sum(r['contact'] for r in contacts),collision_frames=sum(r['collision'] for r in contacts),prefix_max_XY_displacement_m=float(np.linalg.norm(xy[:141]-xy[0],axis=1).max()),prefix_final_true_XY_low_count=nlow,dispatch_ticks_exact=N*5,feedback_clone_ticks=ticks,map_fields_exact=mapfields,guard_calls_verified=guardcount,zero_nav_frames=len(zero),videos=videos,requests=N,source_wall_p50_s=float(np.median([c['rgb_source_wall_s'] for c in clocks if c['source_predicted']])),source_wall_p95_s=float(np.quantile([c['rgb_source_wall_s'] for c in clocks if c['source_predicted']],.95)),paused_physics_not_realtime=True)
 checks=dict(full_capture_and_videos=all(n==N+1 for n in videos.values()),pose_gate=poseaudit['pose_gate'],source_gate=srcaudit['decision']=='PASS_FRESH_LOCAL_SOURCE_ONLY',budget_original60=actual['elapsed_s']<=60+1e-5,goal_native_and_true=actual['native_success'] and actual['final_XYZ_distance_m']<=.2,axis_progress_ge4=actual['axis_progress_m']>=4.,actual_obstacle_passed=actual['obstacle_passed'],no_contact_or_collision=actual['contact_frames']==actual['collision_frames']==0,body_proxy_nonnegative=actual['body_proxy_clearance_m']>=0,safe_start=actual['prefix_max_XY_displacement_m']<=.7,actual_prefix_five_XY_low=nlow>=5,nominal_references=bool(nominal) and all(p['clearance_m']>=.8-1e-8 for p in nominal),true_nominal_references=bool(nominal) and all(p['true_clearance_m']>=.8-1e-8 for p in nominal),true_retained_guards=all(min(p['suffix_true_clearance_m'],p['bridge_true_clearance_m'])>=.6-1e-8 for p in true_guards if p['decision']=='retain'),same_eye_exposure=max(abs(c['eyes']['left']-c['eyes']['right']) for c in clocks)==0.)
 results.append(dict(arm=arm,decision='PASS_CONDITIONAL_MEMORY_NAVIGATION_ONLY' if all(checks.values()) else 'REJECT_CONDITIONAL_MEMORY_NAVIGATION',actual=actual,checks=checks,nominal=nominal,true_guards=true_guards,source=srcaudit,pose=poseaudit));pairdata[arm]=dict(states=states,rows=rows,es=es,ph=ph,xy=xy,D=D)
prefix=[];source_equal=[]
for i in range(141):
 a=pairdata['R0'];b=pairdata['R1'];equal=all(np.array_equal(a['states'][i][k],b['states'][i][k]) for k in ['position','quaternion','velocity','angular_velocity','joints','joint_velocity']) and a['es'][i]==b['es'][i]
 if i<140:equal=equal and np.array_equal(a['rows'][i]['request'],b['rows'][i]['request'])
 prefix.append(equal)
 if i%5==0:
  qa=np.load(a['D']/'possible_sets'/f'frame_{i:03d}.npz');qb=np.load(b['D']/'possible_sets'/f'frame_{i:03d}.npz');source_equal.append(set(qa.files)==set(qb.files) and all(np.array_equal(qa[k],qb[k],equal_nan=np.issubdtype(qa[k].dtype,np.inexact)) for k in qa.files))
input_groups={}
for directory,stem,stride in [('source_predictions','frame',5),('sensor_packets','source',5),('online_inputs','frame',1),('masks','frame',5),('estimated_poses','frame',1),('history','frame',1)]:
 equality=[archives_equal(pairdata['R0']['D']/directory/f'{stem}_{i:03d}.npz',pairdata['R1']['D']/directory/f'{stem}_{i:03d}.npz') for i in range(0,141,stride)];input_groups[directory]=dict(equal=sum(equality),total=len(equality),all_equal=all(equality))
full_physics_prefix=archives_equal(pairdata['R0']['D']/'physics.npz',pairdata['R1']['D']/'physics.npz',prefix_samples=701)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=results,strict_source_and_physical_prefix_equal=all(prefix) and all(source_equal) and full_physics_prefix and all(v['all_equal'] for v in input_groups.values()),full_physics_prefix_701_equal=full_physics_prefix,input_prefix_comparison=input_groups,physical_smoother_prefix_equal_frames=sum(prefix),possible_source_prefix_equal_updates=sum(source_equal),prefix_frames=141,prefix_source_updates=29,not_original35=True,not_formal_Oracle_B0=True,only_source_memory_component_changed=True,whole_research_complete=False,world_model_increment_not_executed=True,code_sha256=sha(__file__))
O=H/'negative_ray_memory_actual_assessment_v2.json';assert not O.exists();O.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='arms'},indent=2));print(json.dumps([{k:a[k] for k in ['arm','decision','actual','checks']} for a in results],indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(figsize=(9,6));theta=np.linspace(0,2*np.pi,100);ax.fill(.35*np.cos(theta),.35*np.sin(theta),color='grey',alpha=.5,label='post-close physical cylinder')
for arm in ['R0','R1']:xy=pairdata[arm]['xy'];ax.plot(xy[:,0],xy[:,1],label=arm+' actual');ax.scatter(xy[140,0],xy[140,1],s=50)
ax.scatter(2.2,.15,marker='*',s=120,c='green');ax.set_aspect('equal');ax.legend();ax.grid();ax.set_title('Fixed14 observation / independent60s memory consumer comparison');fig.savefig(H/'negative_ray_memory_actual_v2.png',dpi=160)
