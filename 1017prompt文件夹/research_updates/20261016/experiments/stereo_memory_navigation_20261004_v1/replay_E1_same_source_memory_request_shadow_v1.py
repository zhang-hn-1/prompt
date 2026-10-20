"""Read-only retrospective same-source consumer diagnostic; STOP at request fork."""
from pathlib import Path
import json,hashlib,copy,numpy as np
from scipy.spatial.transform import Rotation
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from partial_point_risk_grid import PartialPointRiskGrid
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from prior_rgb_navigation import PriorRGBPolicy
from execution_domain_four_point_consumer_v2 import with_supported_four_points
from command_domain_cap import with_command_domain_cap
from estimated_navigation_observation import EstimatedNavigationObservation
from tentative_point_memory import TentativePointMemory
H=Path(__file__).resolve().parent;D=H/'original_E1_direct_memory_navigation_run_v1';O=H/'E1_same_source_memory_request_shadow_v1';O.mkdir(exist_ok=False)
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for f,s in seal['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));zero=np.zeros((480,640,3),np.uint8)
def run(stop,poison=False):
 local=copy.deepcopy(states);rot=angular.copy();expected=copy.deepcopy(rows)
 if poison:
  for v in local[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  rot[stop*5+1:]=np.nan
  for v in expected[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:local=local[:stop+1];rot=rot[:stop*5+1];expected=expected[:stop+1]
 policies=[];navs=[]
 for static in [False,True]:
  policy=with_command_domain_cap(with_supported_four_points(PriorRGBPolicy))(protocol,memory_mode='current');policy.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));origin=np.array([-3.2999999940395353,5.960465010446114e-9])-20;policy.grid=StaticSparsePointRiskGrid(origin,10,static_scene_assumption=True) if static else PartialPointRiskGrid(origin,10);policies.append(policy);navs.append(EstimatedNavigationObservation())
 mem=TentativePointMemory();cursor=0;records=[];first_ref=None;fork=None
 for i in range(stop+1):
  st=local[i];t=st['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(st['quaternion'])[[1,2,3,0]]).as_matrix()
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   src=sources[cursor];assert src['time_s']<=t;p=np.load(D/'partial_observations'/f"frame_{src['frame']:03d}.npz");mem.update(dict(frame=src['frame'],time_s=src['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=src['provenance']),src['time_s']);cursor+=1
  src=sources[cursor-1];snap=mem.at(t);actions=[];references=[]
  for policy,nav in zip(policies,navs):
   original=PolicyObservation(zero,zero,hist['proprio_history'].copy(),hist['action_history'].copy(),np.asarray(st['position'])[:2],0.,hist['quaternion_world_wxyz_history'].copy());pose=dict(time_s=t,root_position_world_m=np.asarray(st['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(st['velocity']));obs,goal=nav.build(original,pose,R.T@rot[i*5],[3.300000005960465,5.960465010446114e-9,.45]);policy.set_source_packet(dict(frame=src['frame'],time_s=src['time_s'],records=snap['records']));policy.set_current_packet(dict(frame=i,time_s=t,root_xyz=np.asarray(st['position']),points_world=np.empty((0,3))));actions.append(policy.act(obs,goal));references.append(np.asarray(policy.tracker._reference_xy).tolist())
  np.testing.assert_array_equal(actions[0],actual[i]);assert references[0]==expected[i]['installed_reference_xy_after_act']
  if first_ref is None and references[0]!=references[1]:first_ref=i
  same=np.array_equal(actions[0],actions[1]);records.append(dict(frame=i,time_s=t,source_frame=src['frame'],source_time_s=src['time_s'],same_requests=bool(same),B0_request=actions[0].tolist(),static_request=actions[1].tolist(),same_reference=references[0]==references[1],static_past_cells_older_than10=int((np.isfinite(policies[1].grid.hazard_acquisition_time)&(t-policies[1].grid.hazard_acquisition_time>10)).sum())))
  if not same:fork=i;break
 return dict(first_request_fork=fork,first_reference_fork=first_ref,rows=records)
full=run(len(rows)-1);last=full['rows'][-1]['frame'];checks=[]
for cutoff in sorted(set([min(65,last),min(155,last),last])):
 a=run(cutoff);load=np.load;opened=[]
 def guarded(file,*args,**kwargs):
  p=Path(file)
  if p.parent.name in ['history','partial_observations']:
   n=int(p.stem.split('_')[-1]);assert n<=cutoff;opened.append(str(p))
  return load(file,*args,**kwargs)
 try:np.load=guarded;b=run(cutoff,True)
 finally:np.load=load
 assert a==b and a['rows']==full['rows'][:cutoff+1];checks.append(dict(cutoff=cutoff,future_poison_truncation_exact=True,guarded_reads=len(opened)))
a=dict(decision='PASS_SAME_SOURCE_PAST_ONLY_REQUEST_MECHANISM_NOT_NAVIGATION',first_reference_fork=full['first_reference_fork'],first_request_fork=full['first_request_fork'],evaluated_frames=len(full['rows']),STOP_after_first_request_fork=True,checks=checks,B0_requests_reference_exact=True,source_input_identical_between_consumers=True,retrospective_not_online=True,no_alternate_future_trajectory_labels=True,known_GT_current_pose_upperbound=True,static_actual_trial_navigation_REJECT_preserved=True,actual_trial_common_source_equivalence_REJECT_preserved=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'rows.json').write_text(json.dumps(full['rows'],indent=2)+'\n');print(json.dumps(a,indent=2))
