"""Same start and executed window: physical metrics; no path_index scoring."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;B=H/'original_E1_direct_memory_navigation_run_v1';C=H/'original_E1_static_sparse_memory_navigation_run_v1';O=H/'original_E1_static_sparse_AB_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for D in [B,C]:
 seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
 for p,d in seal['artifact_sha256'].items():assert sha(p)==d,p
b=np.load(B/'physics.npz');c=np.load(C/'physics.npz');br=np.load(B/'requests.npz')['requests'];cr=np.load(C/'requests.npz')['requests'];N=min(len(br),len(cr));same=np.all(br[:N]==cr[:N],axis=1);fork=int(np.flatnonzero(~same)[0]) if not same.all() else None
np.testing.assert_array_equal(b['root_position_world_m'][0],c['root_position_world_m'][0]);np.testing.assert_array_equal(b['root_quaternion_world_wxyz'][0],c['root_quaternion_world_wxyz'][0]);np.testing.assert_array_equal(b['joint_position_rad'][0],c['joint_position_rad'][0]);np.testing.assert_array_equal(b['joint_velocity_rad_s'][0],c['joint_velocity_rad_s'][0])
initial_b=json.loads((B/'initial_execution_after_reset.json').read_text());initial_c=json.loads((C/'initial_execution_after_reset.json').read_text());assert initial_b==initial_c
# Physics equality is bounded to the identical requested prefix, not all after fork.
limit=5*(N if fork is None else fork)+1
prefix_fields=[]
for key in ['root_position_world_m','root_quaternion_world_wxyz','position_world_xy','joint_position_rad','joint_velocity_rad_s','applied_action','smoothed_target','raw_command_target']:
 np.testing.assert_array_equal(b[key][:limit],c[key][:limit]);prefix_fields.append(key)
goal=np.array([3.300000005960465,5.960465010446114e-9]);start=np.array([-3.2999999940395353,5.960465010446114e-9]);direction=(goal-start)/np.linalg.norm(goal-start)
metrics=[]
for name,D,z,req in [('B0_TTL10_partial',B,b,br),('C_static_sparse_assumption',C,c,cr)]:
 p=z['root_position_world_m'][:,:2];contacts=json.loads((D/'physics_contacts.json').read_text());row=dict(name=name,executed_requests=len(req),sim_end_s=float(z['simulation_time_s'][-1]),goal_remaining_m=float(np.linalg.norm(p[-1]-goal)),axis_progress_m=float((p[-1]-start)@direction),physics_path_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),net_displacement_m=float(np.linalg.norm(p[-1]-start)),contact_samples=len(contacts),contacts=sum(bool(v['contact']) for v in contacts if v['available']),contacts_all_available=all(v['available'] for v in contacts));metrics.append(row)
geometry=json.loads((H/'original_E1_static_sparse_memory_navigation_assessment_v1/assessment.json').read_text());member=json.loads((H/'original_E1_static_sparse_membership_v1/assessment.json').read_text());runtime=json.loads((H/'original_E1_static_sparse_memory_navigation_runtime_verification_v1.json').read_text());memory=json.loads((H/'original_E1_static_memory_consumption_assessment_v1.json').read_text());causal=json.loads((H/'original_E1_static_policy_causal_assessment_v1.json').read_text());termination=json.loads((C/'termination.json').read_text());m=metrics[1]
prereq=all(geometry['checks'].values()) and member['semantic_subset95'] and m['contacts']==0 and m['contacts_all_available']
nav=prereq and termination['termination']=='success' and m['goal_remaining_m']<=.2
local=prereq and (len(cr)==546 or nav) and m['goal_remaining_m']<=4.054687998 and m['axis_progress_m']>=2.566441774
maps=[]
for i in range(len(cr)):
 z=np.load(C/'maps'/f'frame_{i:03d}.npz');time=float(z['time_s']);clock=z['hazard_acquisition_time'];assert not np.any(z['labels']==0);assert np.all(clock[np.isfinite(clock)]<=time)
 maps.append(dict(frame=i,time_s=time,hazard_cells=int((z['labels']==1).sum()),past_measured_older_than_TTL10_cells=int((np.isfinite(clock)&(time-clock>10)).sum()),unknown_cells=int((z['labels']==-1).sum()),no_3D_blind_clearance_certificate=True))
a=dict(decision='PASS_CONDITIONAL_TASKH_KNOWNPOSE_NAVIGATION' if nav else 'LOCAL_IMPROVEMENT_ONLY_NO_NAVIGATION_ADMISSION' if local else 'REJECT_STATIC_MEMORY_NAVIGATION_IMPROVEMENT',local_improvement=local,navigation_pass=nav,prerequisite_geometry_membership_contacts=prereq,metrics=metrics,first_request_fork=fork,identical_requested_prefix_physics_samples=limit,prefix_physics_fields_exact=prefix_fields,comparison='sameinitialstate_B0_completed54.6prefix_only_not_full60',static_scene_assumption=True,not_estimatedpose_or_pureRGB=True,original35_NOTRUN=True,learnedhead_REJECT=True,whole_research_complete=False,not_realtime_claim=True)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'map_age_rows.json').write_text(json.dumps(maps,indent=2)+'\n');print(json.dumps(a,indent=2))
