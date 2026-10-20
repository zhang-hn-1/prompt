from pathlib import Path
import sys,json,hashlib,datetime,numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
import scripts.run_pointgoal_benchmark as runner
from benchmark.policies import PolicyObservation
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from persistent_core_grid import PersistentCoreGrid
from observation_clock_adapter import observe_clock_without_act
from guarded_reference import boundary_state
from provided_historical_model import installable_model
D=H/'native_flat_pose_navigation_R1_run_v1';B=H/'native_terminal_bank_navigation_R1_run_v1';O=H/'provided_model_consumer_preflight_v2';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seedfile=D/'frozen_memory_seed.npz';manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];assert sha(seedfile)==manifest[str(seedfile)];raw=dict(np.load(seedfile));prior=installable_model(raw,float(raw['time_s']),prior_sequence=D.name,prior_sha256=sha(seedfile));np.savez_compressed(O/'provided_model_geometry.npz',possible_xy=prior['possible_xy'],raw_world_voxel_centres=prior['raw_world_voxel_centres']);(O/'provided_model_provenance.json').write_text(json.dumps(prior['provided_model_provenance'],indent=2)+'\n')
parents=json.loads((D/'policy_records.json').read_text());protocol=runner.load_action_protocol(P/'configs/data_contract.yaml');policies=[with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current') for _ in range(2)];init=np.load(D/'estimated_poses/frame_000.npz');inputs={str(seedfile):sha(seedfile)}
for policy in policies:
 policy.reset(None);policy.grid=PersistentCoreGrid(init['root_position_world_m'][:2]-20,60)
 for i in range(140):
  q=np.load(D/'estimated_poses'/f'frame_{i:03d}.npz');policy.grid.set_source(dict(time_s=float(q['time_s']),source='CURRENT_CLOCK_ONLY_NO_GEOMETRY'));policy.grid.update_points(np.empty((0,3)),q['root_position_world_m'],float(q['time_s']));observe_clock_without_act(policy,dict(frame=i,time_s=float(q['time_s']),points_world=np.empty((0,3)),root_xyz=q['root_position_world_m']),float(q['time_s']))
 policy.grid.seed(prior,float(raw['time_s']))
proof=[];different_RGB_frames=0
for i in range(140,600):
 files=[D/'policy_inputs'/f'frame_{i:03d}.npz',D/'estimated_poses'/f'frame_{i:03d}.npz',D/'sensor_packets'/f'source_{i-i%5:03d}.npz',B/'policy_inputs'/f'frame_{i:03d}.npz']
 for f in files:inputs[str(f)]=sha(f)
 q=np.load(files[0]);pose=np.load(files[1]);packet=dict(np.load(files[2]));packet.update(world=np.empty((0,3)),sigma_m=np.empty(0),query_pixel_id=np.empty(0,dtype=int),partner_pixel_id=np.empty(0,dtype=int));images=np.load(files[3]);different=not(np.array_equal(q['left_rgb'],images['left_rgb']) and np.array_equal(q['right_rgb'],images['right_rgb']));different_RGB_frames+=int(different);actions=[]
 for k,policy in enumerate(policies):
  policy.set_source_packet(packet);policy.set_current_packet(dict(frame=i,time_s=float(pose['time_s']),points_world=np.empty((0,3)),root_xyz=pose['root_position_world_m']));img=q if k==0 else images
  obs=PolicyObservation(stereo_left_rgb=img['left_rgb'],stereo_right_rgb=img['right_rgb'],proprio_history=q['proprio_history'],action_history=q['action_history'],position_world_xy=q['position_world_xy'],yaw_world_rad=float(q['yaw_world_rad']),quaternion_world_wxyz_history=q['quaternion_history']);action=policy.act(obs,q['pointgoal_base_xy']);np.testing.assert_array_equal(action,parents[i]['request'],err_msg='stop at any first action mismatch; no labels past divergence');actions.append(action)
 np.testing.assert_array_equal(actions[0],actions[1]);np.testing.assert_array_equal(policies[0].tracker._reference_xy,policies[1].tracker._reference_xy);np.testing.assert_array_equal(policies[0].tracker._reference_xy,parents[i]['local']['installed_reference_xy_after_act']);assert boundary_state(policies[0].tracker)==boundary_state(policies[1].tracker)
 for field in ['labels','codes','safe_start_prior','task_ground_prior','hypothesis_birth_time']:np.testing.assert_array_equal(getattr(policies[0].grid,field),getattr(policies[1].grid,field))
 saved=np.load(D/'maps'/f'frame_{i:03d}.npz');
 for field,key in [('labels','labels'),('codes','provenance'),('safe_start_prior','safe_start_prior'),('task_ground_prior','task_ground_prior'),('hypothesis_birth_time','hypothesis_birth_time')]:np.testing.assert_array_equal(getattr(policies[0].grid,field),saved[key])
 assert not np.isfinite(policies[0].grid.memory.floor_time).any() and not np.isfinite(policies[0].grid.memory.hazard_time).any();proof.append(dict(frame=i,current_RGB_altered=different,action=actions[0].tolist(),boundary=boundary_state(policies[0].tracker),reference_equal=True,map_equal=True,no_measured_free=True))
(O/'actor_rows.json').write_text(json.dumps(proof,indent=2)+'\n');r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_MODEL_CURRENT_CONSUMER_RGB_INDEPENDENCE_ONLY',two_independent_policies=True,all_prefix_mapper_updates_replayed=True,current_maps_five_fields_exact_to_parent=True,calls_per_policy=len(proof),factual_parent_requests_exact=len(proof),current_RGB_different_frames=different_RGB_frames,complete_current_installed_references_equal=True,same_provided_geometry=True,no_new_source_admission=True,no_physical_or_fixed_reference_AB=True,no_controller_candidate=True,no_future_or_truth_feedback=True,no_navigation_result=True,seen_development_not_fresh=True,whole_research_complete=False,provided_model_provenance=prior['provided_model_provenance'],input_sha256=inputs,source_sha256={str(f):sha(f) for f in [Path(__file__),H/'provided_historical_model.py',H.parent.parent/'docs/superpowers/plans/2026-10-08-provided-model-consumer-contract.md']},output_sha256={str(f):sha(f) for f in O.iterdir()});(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['input_sha256','output_sha256']},indent=2))
