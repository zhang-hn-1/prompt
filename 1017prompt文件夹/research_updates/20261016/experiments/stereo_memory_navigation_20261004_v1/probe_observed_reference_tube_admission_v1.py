"""Actual request reconstruction and past-only prefixes, not another physical run."""
from pathlib import Path
import json,copy,numpy as np
from scipy.spatial.transform import Rotation
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from nonbinding_side_hypothesis_v1 import NonbindingSideHypothesisGrid
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from prior_rgb_navigation import PriorRGBPolicy
from cusp_coast_acquisition_v1 import with_cusp_coast_acquisition
from retained_reference_tube_viability_v1 import with_tube_viability_after_measured_reversal
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from tentative_point_memory import TentativePointMemory
from front_view_acquisition_v1 import FrontViewAcquisition
from observation_clock_adapter import observe_clock_without_act
from observed_reference_tube_retention_v1 import with_observed_reference_tube_retention
H=Path(__file__).resolve().parent;D=H/'front_view_handoff_navigation_run_v1';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];zero=np.zeros((480,640,3),np.uint8);reconstructed_request_diagnostics=[]
def run(stop,poison=False):
 local=copy.deepcopy(states);a=angular.copy();comparisons=copy.deepcopy(rows)
 if poison:
  for v in local[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  a[stop*5+1:]=np.nan
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:local=local[:stop+1];a=a[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_command_domain_cap(with_cusp_coast_acquisition(with_observed_reference_tube_retention(with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)))))(protocol,memory_mode='current');policy.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));policy.grid=RepeatedExposureMemoryGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);nav=EstimatedNavigationObservation();mem=TentativePointMemory();acq=FrontViewAcquisition();cursor=0;out=[]
 for i in range(stop+1):
  state=local[i];t=state['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix();original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(state['position'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=np.asarray(state['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(state['velocity']));observation,goal=nav.build(original,pose,R.T@a[i*5],[3.300000005960465,5.960465010446114e-9,.45])
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   source=sources[cursor];assert source['time_s']<=t;p=np.load(D/'partial_observations'/f"frame_{source['frame']:03d}.npz");mem.update(dict(frame=source['frame'],time_s=source['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=source['provenance']),source['time_s']);cursor+=1
  snap=mem.at(t);source=sources[cursor-1];src=dict(frame=source['frame'],time_s=source['time_s'],records=snap['records']);policy.set_source_packet(src);packet=dict(frame=i,time_s=t,root_xyz=np.asarray(state['position']),points_world=np.empty((0,3)));action=acq.choose(t,state['position'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),src)
  if action is not None:
   policy.grid.set_source(src);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);observe_clock_without_act(policy,packet,t);policy.records.append(policy.grid.diagnostics())
  else:
   policy.set_current_packet(packet);action=policy.act(observation,goal)
  if not np.array_equal(action,actual[i]):
   out.append(action);return np.asarray(out),dict(frame=i,time_s=t,current_xy=state['position'][:2],screen=policy.last_tube_viability_screen,B0_request=actual[i].tolist(),candidate_request=action.tolist(),reference_version=policy.local_reference_version)
  np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy).tolist(),comparisons[i]['installed_reference_xy_after_act']);out.append(action)
  if stop==len(rows)-1 and not poison:reconstructed_request_diagnostics.append(dict(frame=i,time_s=t,request_diagnostics=policy.tracker.request_diagnostics,tracking_fields_valid=bool(policy.tracker.tracking_diagnostics.get('valid',False)),replay_exact_current_request=True))
 return np.asarray(out),None
full,fork=run(len(rows)-1);assert fork is not None
cutoff=fork['frame'];prefix,pfork=run(cutoff,True);np.testing.assert_array_equal(prefix,full);assert pfork==fork
result=dict(decision='PASS_NECESSARY_CURRENT_INPUT_FORK_ONLY',requests_until_fork=len(full),fork=fork,future_state_poison_exact=True,not_new_physics=True,not_alternate_future_truth=True,known_GT_pose=True,whole_research_complete=False)
(H/'observed_reference_tube_CPU_admission_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
