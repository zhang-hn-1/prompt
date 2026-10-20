"""Historical same-input consumer test, stops at first requested fork, not rollout."""
from pathlib import Path
import json,hashlib,copy,numpy as np
from scipy.spatial.transform import Rotation
from benchmark.policies import PolicyObservation,EpisodeContext
from scripts.run_pointgoal_benchmark import load_action_protocol
from route_side_commitment_grid_v1 import RouteSideCommitmentGrid
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
from estimated_navigation_observation import EstimatedNavigationObservation
from tentative_point_memory import TentativePointMemory
H=Path(__file__).resolve().parent;D=H/'original_E1_tube_viability_navigation_run_v1';O=H/'E1_cusp_coast_acquisition_shadow_CPU_v1';O.mkdir(exist_ok=False)
for f,s in json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==s,f
states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));zero=np.zeros((480,640,3),np.uint8)
def make(grid,*,candidate=False):
 base=with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))
 if candidate:base=with_cusp_coast_acquisition(base)
 p=with_command_domain_cap(base)(protocol,memory_mode='current');p.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));p.grid=grid;return p
def run(stop,enabled=True,poison=False,stop_at=None):
 origin=np.array([-3.2999999940395353,5.960465010446114e-9])-20;p=make(RouteSideCommitmentGrid(origin,10,static_scene_assumption=True),candidate=True);shadow=ReadOnlyBoundaryShadow(make(RouteSideCommitmentGrid(origin,10,static_scene_assumption=True)),enabled=enabled);nav=EstimatedNavigationObservation();mem=TentativePointMemory();cursor=0;requests=[];refs=[];local=copy.deepcopy(states);rot=angular.copy()
 if poison:
  for s in local[stop+1:]:s.clear();s['FORBIDDEN_FUTURE']=float('nan')
  rot[stop*5+1:]=np.nan
 else:local=local[:stop+1];rot=rot[:stop*5+1]
 for i in range(stop+1):
  st=local[i];t=st['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(st['quaternion'])[[1,2,3,0]]).as_matrix();obs=PolicyObservation(zero,zero,hist['proprio_history'].copy(),hist['action_history'].copy(),np.asarray(st['position'])[:2],0.,hist['quaternion_world_wxyz_history'].copy());pose=dict(time_s=t,root_position_world_m=np.asarray(st['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(st['velocity']));obs,goal=nav.build(obs,pose,R.T@rot[i*5],[3.300000005960465,5.960465010446114e-9,.45])
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   s=sources[cursor];q=np.load(D/'partial_observations'/f"frame_{s['frame']:03d}.npz");mem.update(dict(frame=s['frame'],time_s=s['time_s'],world=q['world'],query_pixel_id=q['query_pixel_id'],partner_pixel_id=q['partner_pixel_id'],input_sha256=s['provenance']),s['time_s']);cursor+=1
  s=sources[cursor-1];source=dict(frame=s['frame'],time_s=s['time_s'],records=mem.at(t)['records']);current=dict(frame=i,time_s=t,root_xyz=np.asarray(st['position']),points_world=np.empty((0,3)));p.set_source_packet(source);p.set_current_packet(current);action=p.act(obs,goal);p.records[-1]['CPU_tracker_state_after_act']=boundary_state(p.tracker);ref=np.asarray(p.tracker._reference_xy).tolist();shadow_row=shadow.observe(action,obs,goal,source,current);requests.append(action);refs.append(ref)
  if shadow_row is not None:np.testing.assert_array_equal(shadow_row['B0_request'],actual[i])
  if (shadow_row is not None and shadow_row['first_request_fork']) or i==stop_at:break
 return np.asarray(requests),refs,shadow.rows,p.grid.diagnostics(),p.records
on=run(len(actual)-1);fork=on[2][-1]['frame'];assert on[2][-1]['first_request_fork'];off=run(fork,False,stop_at=fork);np.testing.assert_array_equal(on[0],off[0]);assert on[1]==off[1]
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
events=[r for r in on[4] if r.get('tube_viability_event_armed')];assert len(events)==1;event=events[0]['frame'];assert fork>event
acquisitions=[r for r in on[4] if r.get('cusp_coast_acquisition') is not None and r['cusp_coast_acquisition']['acquire_index'] is not None];assert len(acquisitions)==1;first=acquisitions[0];assert first['frame']==fork
original_rows=json.loads((D/'policy_records.json').read_text());assert on[1][-1]==original_rows[fork]['installed_reference_xy_after_act'];np.testing.assert_array_equal(on[0][:-1],actual[:fork]);np.testing.assert_array_equal(on[0][-1],np.zeros(10))
after=first['CPU_tracker_state_after_act'];assert after['active_direction']=='forward' and after['pending_direction']=='reverse'
q=np.load(D/'maps'/f"frame_{fork:03d}.npz");xy=np.asarray(states[fork]['position'])[:2];cells=np.rint((xy-q['origin'])/float(q['resolution'])).astype(int);clearance=distance_transform_edt(np.pad(q['labels']!=1,1))[1:-1,1:-1]*float(q['resolution']);current_clearance=float(clearance[tuple(cells)]);assert current_clearance>=first['plan']['planning_clearance_m']-1e-8
assessment=dict(decision='PASS_ONE_CUSP_COAST_ACQUISITION_CPU_NECESSARY_INTERFACE_ONLY',event_arm_frame=event,first_request_fork=fork,requests_evaluated=len(on[0]),first_decision=first['cusp_coast_acquisition'],current_start_clearance_m=current_clearance,existing_margin_m=first['plan']['planning_clearance_m'],active_direction_not_forced=True,existing_direction_handler_brakes=True,reference_geometry_exact_at_fork=True,B0_requests_exact_until_first_fork=True,shadow_on_off_candidate_requests_references_exact=True,checks=checks,retrospective_not_online=True,no_alternate_future_physical_labels=True,no_new_physics_or_NN=True,old_whole_sequences_DEVELOPMENT_not_fresh=True,heuristic_not_safe_stopping_bound=True,full_initial_state_coverage_not_proven=True,not_navigation_or_safety_admission=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n');(O/'candidate_records.json').write_text(json.dumps(on[4],indent=2)+'\n');(O/'shadow_rows.json').write_text(json.dumps(on[2],indent=2)+'\n');print(json.dumps(assessment,indent=2))
