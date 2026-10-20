"""Historical same-input consumer test, stops at first requested fork, not rollout."""
from pathlib import Path
import json,hashlib,copy,numpy as np
from scipy.spatial.transform import Rotation
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from readonly_boundary_shadow_v1 import ReadOnlyBoundaryShadow
from prior_rgb_navigation import PriorRGBPolicy
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition,CONFIG
from guarded_reference import boundary_state
from retained_reference_tube_viability_v1 import with_tube_viability_after_measured_reversal
from execution_domain_four_point_consumer_v2 import planning_clearance
from scipy.ndimage import distance_transform_edt
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from front_view_baseline_shadow_v1 import with_front_view_baseline_shadow
from observed_reference_tube_retention_v1 import with_observed_reference_tube_retention
from estimated_navigation_observation import EstimatedNavigationObservation
from tentative_point_memory import TentativePointMemory
H=Path(__file__).resolve().parent;D=H/'observed_reference_tube_navigation_run_v1';O=H/'observed_reference_tube_online_shadow_verification_v1';O.mkdir(exist_ok=False)
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));zero=np.zeros((480,640,3),np.uint8)
def make(grid,*,candidate=False):
 base=with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))
 if candidate:base=with_observed_reference_tube_retention(base)
 base=with_cusp_coast_acquisition(base)
 p=with_front_view_baseline_shadow(with_command_domain_cap(base))(protocol,memory_mode='current');p.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));p.grid=grid;return p
def run(stop,enabled=True,poison=False,stop_at=None):
 origin=np.array([-3.2999999940395353,5.960465010446114e-9])-20;p=make(RepeatedExposureMemoryGrid(origin,10,static_scene_assumption=True),candidate=True);shadow=ReadOnlyBoundaryShadow(make(RepeatedExposureMemoryGrid(origin,10,static_scene_assumption=True)),enabled=enabled);nav=EstimatedNavigationObservation();mem=TentativePointMemory();cursor=0;requests=[];refs=[];local=copy.deepcopy(states);rot=angular.copy()
 if poison:
  for s in local[stop+1:]:s.clear();s['FORBIDDEN_FUTURE']=float('nan')
  rot[stop*5+1:]=np.nan
 else:local=local[:stop+1];rot=rot[:stop*5+1]
 for i in range(stop+1):
  st=local[i];t=st['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(st['quaternion'])[[1,2,3,0]]).as_matrix();obs=PolicyObservation(zero,zero,hist['proprio_history'].copy(),hist['action_history'].copy(),np.asarray(st['position'])[:2],0.,hist['quaternion_world_wxyz_history'].copy());pose=dict(time_s=t,root_position_world_m=np.asarray(st['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(st['velocity']));obs,goal=nav.build(obs,pose,R.T@rot[i*5],[3.300000005960465,5.960465010446114e-9,.45])
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   s=sources[cursor];q=np.load(D/'partial_observations'/f"frame_{s['frame']:03d}.npz");mem.update(dict(frame=s['frame'],time_s=s['time_s'],world=q['world'],query_pixel_id=q['query_pixel_id'],partner_pixel_id=q['partner_pixel_id'],input_sha256=s['provenance']),s['time_s']);cursor+=1
  s=sources[cursor-1];source=dict(frame=s['frame'],time_s=s['time_s'],records=mem.at(t)['records']);current=dict(frame=i,time_s=t,root_xyz=np.asarray(st['position']),points_world=np.empty((0,3)));p.set_source_packet(source);p.set_current_packet(current);action=p.act(obs,goal);p.records[-1]['CPU_tracker_state_after_act']=boundary_state(p.tracker);ref=np.asarray(p.tracker._reference_xy).tolist();shadow_row=shadow.observe(action,obs,goal,source,current);requests.append(action);refs.append(ref)
  np.testing.assert_array_equal(action,actual[i])
  if shadow_row is not None:
   logged=json.loads((D/'B0_shadow_rows.ndjson').read_text().splitlines()[i])
   for key,value in shadow_row.items():assert logged[key]==value,(i,key)
  if (shadow_row is not None and shadow_row['first_request_fork']) or i==stop_at:break
 return np.asarray(requests),refs,shadow.rows,p.grid.diagnostics(),p.records,dict(labels=p.grid.labels.copy(),origin=p.grid.origin.copy(),resolution=p.grid.resolution,clock=p.grid.hazard_acquisition_time.copy())
on=run(len(actual)-1);found=bool(on[2] and on[2][-1]['first_request_fork']);fork=on[2][-1]['frame'] if found else len(actual)-1
off=run(fork,False,stop_at=fork);np.testing.assert_array_equal(on[0],off[0]);assert on[1]==off[1]
checks=[]
for cutoff in sorted(set([min(65,fork),min(155,fork),fork])):
 a=run(cutoff);load=np.load;opened=[]
 def guarded(file,*args,**kwargs):
  f=Path(file)
  if f.parent.name in ['history','partial_observations']:assert int(f.stem.split('_')[-1])<=cutoff;opened.append(str(f))
  return load(file,*args,**kwargs)
 try:np.load=guarded;b=run(cutoff,poison=True)
 finally:np.load=load
 np.testing.assert_array_equal(a[0],b[0]);np.testing.assert_array_equal(a[0],on[0][:cutoff+1]);assert a[1]==b[1] and a[2]==b[2];checks.append(dict(cutoff=cutoff,future_poison_truncation_exact=True,guarded_reads=len(opened)))
np.testing.assert_array_equal(on[0][:-1] if found else on[0],actual[:fork] if found else actual)
q=on[5];xy=np.asarray(states[fork]['position'])[:2];cells=np.rint((xy-q['origin'])/float(q['resolution'])).astype(int);clearance=distance_transform_edt(np.pad(q['labels']!=1,1))[1:-1,1:-1]*float(q['resolution']);current_clearance=float(clearance[tuple(cells)]);margin=planning_clearance(NonbindingSideHypothesisGrid(q['origin'],10,static_scene_assumption=True))
assessment=dict(decision='PASS_ACTUAL_ONLINE_SHADOW_CURRENT_PREFIX_ONLY',first_request_fork=fork if found else None,requests_evaluated=len(on[0]),actual_shadow_rows_and_candidate_requests_exact=True,shadow_on_off_candidate_requests_references_exact=True,checks=checks,current_start_clearance_m=current_clearance,nominal_margin_m=margin,not_start_replanning_acceptance=True,only_retention_consumer_changed=True,retrospective_reconstruction_of_actual_online_logs=True,actual_logged_online_interface_not_replay_only=True,no_alternate_future_physical_labels=True,no_new_physics_or_NN=True,known_GT_pose=True,not_navigation_or_safety_admission=True,whole_research_complete=False)
np.savez_compressed(O/'candidate_grid_at_fork.npz',**on[5]);(O/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n');(O/'candidate_records.json').write_text(json.dumps(on[4],indent=2)+'\n');(O/'shadow_rows.json').write_text(json.dumps(on[2],indent=2)+'\n');print(json.dumps(assessment,indent=2))
