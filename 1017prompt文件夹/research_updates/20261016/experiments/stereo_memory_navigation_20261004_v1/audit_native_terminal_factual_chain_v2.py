from pathlib import Path
import sys,json,hashlib,datetime,math,numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
import scripts.run_pointgoal_benchmark as runner
from benchmark.policies import PolicyObservation
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points as with_tracking_tube
from command_domain_cap import with_command_domain_cap
from persistent_core_grid import PersistentCoreGrid
from observation_clock_adapter import observe_clock_without_act
from guarded_reference import boundary_state
D=H/'native_flat_pose_navigation_R1_run_v1';O=H/'native_terminal_factual_chain_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
rows=json.loads((D/'policy_records.json').read_text());policy=with_command_domain_cap(with_tracking_tube(PriorRGBPolicy))(runner.load_action_protocol(P/'configs/data_contract.yaml'),memory_mode='current');policy.reset(None);init=np.load(D/'estimated_poses/frame_000.npz');policy.grid=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60)
for i in range(140):
 q=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');observe_clock_without_act(policy,dict(frame=i,time_s=float(q['time_s']),points_world=np.empty((0,3)),root_xyz=q['root_position_world_m']),float(q['time_s']))
seed=dict(np.load(D/'frozen_memory_seed.npz'));policy.grid.seed(seed,float(seed['time_s']));trace=[]
for i in range(140,600):
 q=np.load(D/'policy_inputs'/f'frame_{i:03d}.npz');pose=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');source=dict(np.load(D/'sensor_packets'/f'source_{i-i%5:03d}.npz'));source.update(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int));policy.set_source_packet(source);policy.set_current_packet(dict(frame=i,time_s=float(pose['time_s']),points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']))
 obs=PolicyObservation(**{k:q[k] for k in ['left_rgb','right_rgb']} ) if False else PolicyObservation(stereo_left_rgb=q['left_rgb'],stereo_right_rgb=q['right_rgb'],proprio_history=q['proprio_history'],action_history=q['action_history'],position_world_xy=q['position_world_xy'],yaw_world_rad=float(q['yaw_world_rad']),quaternion_world_wxyz_history=q['quaternion_history'])
 a=policy.act(obs,q['pointgoal_base_xy']);np.testing.assert_equal(a,rows[i]['request']);diag=policy.tracker.tracking_diagnostics;rec=dict(frame=i,request=a.tolist(),diagnostics=diag,geometry_counterfactual=None)
 rec['valid']=bool(diag.get('valid'));rec['request_diagnostics']=policy.tracker.request_diagnostics;rec['mapping_valid']=rec['valid'];rec['mapping']=policy.tracker.mapping_records[-1] if rec['mapping_valid'] and policy.tracker.mapping_records else None
 rec['boundary']=boundary_state(policy.tracker)
 rec['goal_distance_xy_m']=float(np.linalg.norm(q['pointgoal_base_xy']));rec['remaining_path_m']=None if policy.tracker._context is None else float(policy.tracker._remaining_path_length(obs.position_world_xy))
 rec['reference_endpoint_distance_to_known_goal_m']=None if policy.tracker._context is None else float(np.linalg.norm(policy.tracker._reference_xy[-1]-[2.2,.15]))
 trace.append(rec)
(O/'trace.json').write_text(json.dumps(trace,indent=2)+'\n')
inputs={str(f):sha(f) for f in [D/'policy_records.json',D/'frozen_memory_seed.npz',D/'artifact_manifest_post_close.json',H/'execution_domain_four_point_consumer_v2.py',H/'native_rolling_shadow.py',H.parent.parent/'docs/superpowers/plans/2026-10-08-native-terminal-factual-request-chain.md']}
(O/'actor_trace_pretruth_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),trace_sha256=sha(O/'trace.json'),input_sha256=inputs,code_sha256=sha(__file__),factual_requests_exact=len(trace),not_online=True,independent_cloned_policy=True,no_truth_actor_input=True),indent=2)+'\n')
# Only now measured physical/feedback labels; no alternative-plan labels.
ph=np.load(D/'physics.npz');dist=np.linalg.norm(ph['root_position_world_m']-[2.2,.15,.45],axis=1);nearest=int(dist.argmin());groups=[]
for lo,hi in [(140,400),(400,480),(480,600)]:
 group=[x for x in trace if lo<=x['frame']<hi];valid=[x for x in group if x['valid']];curves=[abs(x['diagnostics']['requested_curvature_1_m']) for x in valid]
 groups.append(dict(start_frame=lo,end_frame=hi,rows=len(group),valid_rows=len(valid),curvature_clamped_rows=sum(bool(x['mapping'].get('curvature_clamped')) for x in valid if x['mapping']),protocol_angle_bound_rows=sum(bool(x['diagnostics'].get('angle_target_at_bound')) for x in valid),absolute_curvature_median_1_m=None if not curves else float(np.median(curves)),absolute_curvature_peak_1_m=None if not curves else max(curves),braking_sources={str(k):sum(x['boundary']['braking_state']==k for x in group) for k in sorted(set(x['boundary']['braking_state'] for x in group))}))
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),factual_requests_exact=len(trace),groups=groups,nearest_physical_XYZ_distance_m=float(dist[nearest]),nearest_time_s=float(ph['simulation_time_s'][nearest]),no_future_or_truth_actor_inputs=True,not_new_navigation=True,no_new_candidate=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
