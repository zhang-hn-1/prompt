"""Actual request reconstruction and past-only prefixes, not another physical run."""
from pathlib import Path
import json,copy,numpy as np
from scipy.spatial.transform import Rotation
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from readonly_boundary_shadow_v1 import ReadOnlyBoundaryShadow
from guarded_reference import boundary_state
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from tentative_point_memory import TentativePointMemory
H=Path(__file__).resolve().parent;D=H/'original_E1_route_side_navigation_resumed_run_v2';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];zero=np.zeros((480,640,3),np.uint8);reconstructed_request_diagnostics=[]
saved_shadow=json.loads((D/"B0_shadow_until_fork.json").read_text());assert saved_shadow
assert len(saved_shadow)<=len(actual) and [r["frame"] for r in saved_shadow]==list(range(len(saved_shadow)))
assert all(r["same_request"] for r in saved_shadow[:-1])
if len(saved_shadow)<len(actual):assert saved_shadow[-1]["first_request_fork"]
def run(stop,poison=False):
 local=copy.deepcopy(states);a=angular.copy();comparisons=copy.deepcopy(rows)
 if poison:
  for v in local[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  a[stop*5+1:]=np.nan
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:local=local[:stop+1];a=a[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');policy.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));policy.grid=RouteSideCommitmentGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);baseline=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode="current");baseline.reset(EpisodeContext("golden_e1_0001",False,None,protocol.protocol_id));baseline.grid=StaticSparsePointRiskGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);shadow=ReadOnlyBoundaryShadow(baseline);nav=EstimatedNavigationObservation();mem=TentativePointMemory();cursor=0;out=[]
 for i in range(stop+1):
  state=local[i];t=state['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix();original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(state['position'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=np.asarray(state['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(state['velocity']));observation,goal=nav.build(original,pose,R.T@a[i*5],[3.300000005960465,5.960465010446114e-9,.45])
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   source=sources[cursor];assert source['time_s']<=t;p=np.load(D/'partial_observations'/f"frame_{source['frame']:03d}.npz");mem.update(dict(frame=source['frame'],time_s=source['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=source['provenance']),source['time_s']);cursor+=1
  snap=mem.at(t);source=sources[cursor-1];policy.set_source_packet(dict(frame=source['frame'],time_s=source['time_s'],records=snap['records']));policy.set_current_packet(dict(frame=i,time_s=t,root_xyz=np.asarray(state['position']),points_world=np.empty((0,3))));action=policy.act(observation,goal)
  record=shadow.observe(action,observation,goal,dict(frame=source['frame'],time_s=source['time_s'],records=snap['records']),dict(frame=i,time_s=t,root_xyz=np.asarray(state['position']),points_world=np.empty((0,3))))
  assert record is not None
  record['B0_state_after_act']=boundary_state(baseline.tracker)
  assert record==saved_shadow[i],'actual online shadow must reconstruct from current/past only'
  np.testing.assert_array_equal(record['candidate_request'],actual[i])
  np.testing.assert_array_equal(action,actual[i]);np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy).tolist(),comparisons[i]['installed_reference_xy_after_act']);assert policy.records[-1].get('route_side_commitment')==comparisons[i].get('route_side_commitment');assert policy.records[-1].get('route_side_commitment_released')==comparisons[i].get('route_side_commitment_released');out.append(action)
  if stop==len(rows)-1 and not poison:reconstructed_request_diagnostics.append(dict(frame=i,time_s=t,request_diagnostics=policy.tracker.request_diagnostics,tracking_fields_valid=bool(policy.tracker.tracking_diagnostics.get('valid',False)),replay_exact_current_request=True))
 return np.asarray(out)
end=len(saved_shadow)-1
prefix=run(end);checks=[]
for cutoff in sorted(set([min(65,end),min(155,end),end])):
 a=run(cutoff);loader=np.load;opened=[]
 def guarded(file,*args,**kwargs):
  p=Path(file)
  if p.parent.name in ['history','partial_observations']:
   assert int(p.stem.split('_')[-1])<=cutoff;opened.append(str(p))
  return loader(file,*args,**kwargs)
 try:np.load=guarded;b=run(cutoff,True)
 finally:np.load=loader
 np.testing.assert_array_equal(a,b);np.testing.assert_array_equal(a,prefix[:cutoff+1]);checks.append(dict(cutoff=cutoff,future_poison_truncation_exact=True,guarded_reads=len(opened)))
r=dict(decision='PASS_ACTUAL_ONLINE_READONLY_SHADOW_CURRENT_PAST_INTERFACE',actual_online_rows=len(saved_shadow),first_request_fork=end if saved_shadow[-1]['first_request_fork'] else None,stopped_at_first_request_fork=True,actual_shadow_rows_and_candidate_requests_exact=True,cutoffs=checks,B0_future_physical_labels_after_fork_NOTAPPLICABLE=True,not_new_online_replay=True,no_alternate_navigation_result=True,whole_research_complete=False)
(H/'original_E1_route_side_resumed_online_shadow_assessment_v2.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
