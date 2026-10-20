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
H=Path(__file__).resolve().parent;D=H/'original_E1_repeated_exposure_memory_navigation_run_v1';protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());actual=np.load(D/'requests.npz')['requests'];angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s'];zero=np.zeros((480,640,3),np.uint8);reconstructed_request_diagnostics=[]
telemetry=[]
def run(stop,poison=False):
 local=copy.deepcopy(states);a=angular.copy();comparisons=copy.deepcopy(rows)
 if poison:
  for v in local[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
  a[stop*5+1:]=np.nan
  for v in comparisons[stop+1:]:v.clear();v['FORBIDDEN_FUTURE']=float('nan')
 else:local=local[:stop+1];a=a[:stop*5+1];comparisons=comparisons[:stop+1]
 policy=with_command_domain_cap(with_cusp_coast_acquisition(with_tube_viability_after_measured_reversal(with_supported_four_points(PriorRGBPolicy))))(protocol,memory_mode='current');policy.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));policy.grid=RepeatedExposureMemoryGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);nav=EstimatedNavigationObservation();mem=TentativePointMemory();cursor=0;out=[]
 for i in range(stop+1):
  state=local[i];t=state['time_s'];hist=np.load(D/'history'/f'frame_{i:03d}.npz');R=Rotation.from_quat(np.asarray(state['quaternion'])[[1,2,3,0]]).as_matrix();original=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],np.asarray(state['position'])[:2],0.,hist['quaternion_world_wxyz_history']);pose=dict(time_s=t,root_position_world_m=np.asarray(state['position']),root_rotation_world=R,velocity_world_m_s=np.asarray(state['velocity']));observation,goal=nav.build(original,pose,R.T@a[i*5],[3.300000005960465,5.960465010446114e-9,.45])
  while cursor<len(sources) and sources[cursor]['frame']<=i:
   source=sources[cursor];assert source['time_s']<=t;p=np.load(D/'partial_observations'/f"frame_{source['frame']:03d}.npz");mem.update(dict(frame=source['frame'],time_s=source['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=source['provenance']),source['time_s']);cursor+=1
  snap=mem.at(t);source=sources[cursor-1];policy.set_source_packet(dict(frame=source['frame'],time_s=source['time_s'],records=snap['records']));policy.set_current_packet(dict(frame=i,time_s=t,root_xyz=np.asarray(state['position']),points_world=np.empty((0,3))));previous_tracking_records=len(policy.tracker.tracking_records);action=policy.act(observation,goal);np.testing.assert_array_equal(action,actual[i]);np.testing.assert_array_equal(np.asarray(policy.tracker._reference_xy).tolist(),comparisons[i]['installed_reference_xy_after_act']);assert policy.records[-1].get('cusp_coast_acquisition')==comparisons[i].get('cusp_coast_acquisition');assert policy.records[-1].get('tube_viability_armed')==comparisons[i].get('tube_viability_armed');assert policy.records[-1].get('tube_viability_event_armed')==comparisons[i].get('tube_viability_event_armed');assert policy.records[-1].get('tube_viability_screen')==comparisons[i].get('tube_viability_screen');assert policy.records[-1].get('route_side_commitment')==comparisons[i].get('route_side_commitment');assert policy.records[-1].get('route_side_commitment_released')==comparisons[i].get('route_side_commitment_released');out.append(action)
  if stop==len(rows)-1 and not poison:
   called=len(policy.tracker.tracking_records)>previous_tracking_records
   audit=policy.tracker.tracking_diagnostics if called else {}
   telemetry.append(dict(frame=i,time_s=t,tracker_called=called,tracking=audit,request=action.tolist(),reference_version=comparisons[i]['local_reference_version'],tracker_state=comparisons[i]['tracker_state_after_act']))
  if stop==len(rows)-1 and not poison:reconstructed_request_diagnostics.append(dict(frame=i,time_s=t,request_diagnostics=policy.tracker.request_diagnostics,tracking_fields_valid=bool(policy.tracker.tracking_diagnostics.get('valid',False)),replay_exact_current_request=True))
 return np.asarray(out)
full=run(len(rows)-1)
O=H/'E1_execution_heading_audit_v1';O.mkdir(exist_ok=False)
(O/'single_call_telemetry.json').write_text(json.dumps(telemetry,indent=2)+'\n')
wrap=lambda x:(x+np.pi)%(2*np.pi)-np.pi
valid=[]
for row in telemetry:
 a=row['tracking']
 if not row['tracker_called'] or not a.get('valid') or a.get('source')!='tracking':continue
 speed=a['world_xy_speed_m_s'];course_valid=speed>=.05
 if not course_valid:continue
 heading=a['velocity_course_rad']-(np.pi if a['reference_drive_direction']=='reverse' else 0)
 delta=np.array([a['lookahead_x_m']-a['position_world_x_m'],a['lookahead_y_m']-a['position_world_y_m']])
 d2=float(delta@delta);kcourse=0. if d2<=1e-9 else 2*(-np.sin(heading)*delta[0]+np.cos(heading)*delta[1])/d2
 body=a['root_heading_rad'];kb=0. if d2<=1e-9 else 2*(-np.sin(body)*delta[0]+np.cos(body)*delta[1])/d2
 assert abs(kb-a['geometric_pure_pursuit_curvature_1_m'])<1e-10
 valid.append(dict(frame=row['frame'],time_s=row['time_s'],direction=a['reference_drive_direction'],version=row['reference_version'],body_drive_course_abs_deg=float(abs(np.degrees(wrap(heading-body)))),body_reference_tangent_abs_deg=float(abs(np.degrees(wrap(body-a['reference_body_heading_rad'])))),course_reference_tangent_abs_deg=float(abs(np.degrees(wrap(a['velocity_course_rad']-a['reference_motion_tangent_rad'])))),body_curvature_1_m=float(kb),virtual_course_curvature_1_m=float(kcourse),sign_changed=bool(kb*kcourse<0),protocol_clipped=bool(a['protocol_clipped']),angle_at_bound=bool(a['angle_target_at_bound']),mapping_curvature_clamped=bool(a.get('calibrated_bank_mapping',{}).get('curvature_clamped',False)),mapping_speed_clamped=bool(a.get('calibrated_bank_mapping',{}).get('speed_clamped',False)),not_alternate_action_or_truth=True))
def aggregate(samples):
 if not samples:return dict(n=0)
 return dict(n=len(samples),body_drive_course_p90_deg=float(np.percentile([v['body_drive_course_abs_deg'] for v in samples],90)),body_tangent_p90_deg=float(np.percentile([v['body_reference_tangent_abs_deg'] for v in samples],90)),course_tangent_p90_deg=float(np.percentile([v['course_reference_tangent_abs_deg'] for v in samples],90)),body_curvature_abs_p90=float(np.percentile([abs(v['body_curvature_1_m']) for v in samples],90)),virtual_course_curvature_abs_p90=float(np.percentile([abs(v['virtual_course_curvature_1_m']) for v in samples],90)),virtual_sign_changed=sum(v['sign_changed'] for v in samples),protocol_clipped=sum(v['protocol_clipped'] for v in samples),angle_at_bound=sum(v['angle_at_bound'] for v in samples),mapping_curvature_clamped=sum(v['mapping_curvature_clamped'] for v in samples),mapping_speed_clamped=sum(v['mapping_speed_clamped'] for v in samples))
references=[]
for path in sorted((D/'local_plans').glob('frame_*.json')):
 j=json.loads(path.read_text());wp=j['waypoints'];xy=np.array([[w['x_m'],w['y_m']] for w in wp]);length=np.linalg.norm(np.diff(xy,axis=0),axis=1);reverse=np.array([w['drive_direction']=='reverse' for w in wp[:-1]])
 references.append(dict(frame=j['frame'],version=j['reference_version'],target=j['plan_meta']['local_target'],reverse_length_m=float(length[reverse].sum()),forward_length_m=float(length[~reverse].sum()),cusp_count=j['direction_changes'],source='installed frozen nominal geometry',first_direction=wp[0]['drive_direction']))
a=dict(decision='FACTUAL_CURRENT_GEOMETRY_COMMAND_CHAIN_ONLY_NOT_CONTROL_ACCEPTANCE',actions_exact=len(full),tracker_called_frames=sum(r['tracker_called'] for r in telemetry),normal_current_course_valid=aggregate(valid),by_direction={d:aggregate([v for v in valid if v['direction']==d]) for d in ['forward','reverse']},by_reference={str(k):aggregate([v for v in valid if v['version']==k]) for k in sorted(set(v['version'] for v in valid))},installed_reference_geometry=references,not_online_interface=True,no_new_physics=True,no_alternate_plan_groundtruth=True,old_velocity_pursuit_REJECT_preserved=True,formal35_NOTRUN=True,whole_research_complete=False)
(O/'current_geometry_rows.json').write_text(json.dumps(valid,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
