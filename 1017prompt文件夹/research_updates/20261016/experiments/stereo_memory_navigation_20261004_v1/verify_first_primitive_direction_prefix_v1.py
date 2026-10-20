from guarded_reference import boundary_state
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
from first_primitive_direction_consumer_v1 import with_first_primitive_direction
H=Path(__file__).resolve().parent;D=H/'observed_reference_tube_navigation_run_v1';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];zero=np.zeros((480,640,3),np.uint8);reconstructed_request_diagnostics=[]
def run(stop,poison=False):
 local=copy.deepcopy(states);a=angular.copy();comparisons=copy.deepcopy(rows)
 if poison:
  for v in local[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  a[stop*5+1:]=np.nan
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:local=local[:stop+1];a=a[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_first_primitive_direction(with_command_domain_cap(with_cusp_coast_acquisition(with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy)))))(protocol,memory_mode='current');policy.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));policy.grid=RepeatedExposureMemoryGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);nav=EstimatedNavigationObservation();mem=TentativePointMemory();acq=FrontViewAcquisition();cursor=0;out=[]
 for i in range(stop+1):
  state=local[i];t=state['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix();original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(state['position'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=np.asarray(state['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(state['velocity']));observation,goal=nav.build(original,pose,R.T@a[i*5],[3.300000005960465,5.960465010446114e-9,.45])
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   source=sources[cursor];assert source['time_s']<=t;p=np.load(D/'partial_observations'/f"frame_{source['frame']:03d}.npz");mem.update(dict(frame=source['frame'],time_s=source['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=source['provenance']),source['time_s']);cursor+=1
  snap=mem.at(t);source=sources[cursor-1];src=dict(frame=source['frame'],time_s=source['time_s'],records=snap['records']);policy.set_source_packet(src);packet=dict(frame=i,time_s=t,root_xyz=np.asarray(state['position']),points_world=np.empty((0,3)));action=acq.choose(t,state['position'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),src)
  if action is not None:
   policy.grid.set_source(src);policy.grid.update_points(np.empty((0,3)),pose['root_position_world_m'],t);observe_clock_without_act(policy,packet,t);policy.records.append(policy.grid.diagnostics())
  else:
   policy.set_current_packet(packet);action=policy.act(observation,goal)
  if i==211:
   return np.asarray(out+[action]),copy.deepcopy(policy.records[-1]),boundary_state(policy.tracker)
  np.testing.assert_array_equal(action,actual[i]);np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy).tolist(),comparisons[i]['installed_reference_xy_after_act']);assert policy.records[-1].get('cusp_coast_acquisition')==comparisons[i].get('cusp_coast_acquisition');assert policy.records[-1].get('tube_viability_armed')==comparisons[i].get('tube_viability_armed');assert policy.records[-1].get('tube_viability_event_armed')==comparisons[i].get('tube_viability_event_armed');assert policy.records[-1].get('route_side_commitment')==comparisons[i].get('route_side_commitment');assert policy.records[-1].get('route_side_commitment_released')==comparisons[i].get('route_side_commitment_released');out.append(action)
  if stop==len(rows)-1 and not poison:reconstructed_request_diagnostics.append(dict(frame=i,time_s=t,request_diagnostics=policy.tracker.request_diagnostics,tracking_fields_valid=bool(policy.tracker.tracking_diagnostics.get('valid',False)),replay_exact_current_request=True))
 return np.asarray(out)

a,row,state=run(211,True);b,row2,state2=run(211,False)
np.testing.assert_array_equal(a,b);assert row==row2 and state==state2
np.testing.assert_array_equal(a[:211],actual[:211]);assert not np.array_equal(a[211],actual[211])
r=dict(decision='PASS_NECESSARY_CURRENT_REQUEST_FORK_ONLY',prefix_exact=211,first_fork=211,B0_request=actual[211].tolist(),candidate_request=a[211].tolist(),current_query=row['first_primitive_direction_query'],future_poison_exact=True,old_latch_after_act=state,not_new_physics=True,not_navigation_admission=True)
(H/'first_primitive_direction_request_assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
