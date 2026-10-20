from pathlib import Path
from types import SimpleNamespace
import json,sys,copy,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(P),str(H.parent/'oracle_feasible_route_20260929_v9/hybrid')]
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
from outgoing_lookahead import make_endpoint_oracle
from cusp_coast_acquisition_v1 import with_cusp_coast_tracker
from depth_local_curve import install_local_reference
from guarded_reference import boundary_state
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from benchmark.policies import PolicyObservation
from estimated_navigation_observation import EstimatedNavigationObservation
D=H/'E1_continuous_tube_navigation_run_v1';rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());ph=np.load(D/'physics.npz');episode=json.loads((D/'task_episode.json').read_text())['actual_episode'];worldgoal=episode['goal_position_world_m']
protocol=load_action_protocol(DEFAULT_CONTRACT.resolve())
def reconstruct(stop):
 tracker=make_endpoint_oracle(protocol);tracker.__class__=with_cusp_coast_tracker(type(tracker));tracker._coast_enabled=False;tracker._coast_decision=None;tracker._coast_current_observation=None
 adapter=EstimatedNavigationObservation();armed=False;checks=[]
 for i in range(stop+1):
  s=states[i];R=Rotation.from_quat(np.asarray(s['quaternion'])[[1,2,3,0]]).as_matrix();history=np.load(D/'history'/f'frame_{i:03d}.npz');original=PolicyObservation(position_world_xy=np.asarray(s['position'][:2]),yaw_world_rad=float(np.arctan2(R[1,0],R[0,0])),quaternion_world_wxyz_history=history['quaternion_world_wxyz_history'],proprio_history=history['proprio_history'],action_history=history['action_history'],stereo_left_rgb=np.zeros((480,640,3),np.uint8),stereo_right_rgb=np.zeros((480,640,3),np.uint8));pose=dict(time_s=s['time_s'],root_position_world_m=s['position'],root_rotation_world=R,velocity_world_m_s=s['velocity']);ob,goal=adapter.build(original,pose,R.T@ph['angular_velocity_world_rad_s'][i*5],worldgoal)
  if i==stop:
   clone=copy.deepcopy(tracker);before=boundary_state(tracker);before_dict=copy.deepcopy(tracker.__dict__)
  r=rows[i]
  if r['prefix_observer']:continue
  if r['local_plan_created']:
   plan=json.loads((D/'local_plans'/f'frame_{i:03d}.json').read_text());wps=tuple(KinodynamicWaypoint(**w) for w in plan['waypoints'])
   if len(wps)>=2:install_local_reference(tracker,SimpleNamespace(waypoints=wps))
  tracker._coast_enabled=armed;tracker._coast_decision=None
  if tracker._context is None:
   tracker._begin_tracking_audit(ob);request=tracker._zero_action('local_no_route')
  else:request=tracker.act(ob,goal)
  armed=bool(r['tube_viability_armed']);request_exact=np.array_equal(request,np.asarray(r['uncapped_request'],np.float32));state_exact=boundary_state(tracker)==r['tracker_state_after_act'];checks.append(dict(frame=i,request_exact=request_exact,state_exact=state_exact))
  if not request_exact or not state_exact:return dict(boundary=stop,passed=False,first_mismatch=checks[-1],got=request.tolist(),expected=r['uncapped_request'],got_state=boundary_state(tracker),expected_state=r['tracker_state_after_act'],prior_checks=checks)
  if i==stop:
   # Independent clone receives the same currently installed reference operation.
   if r['local_plan_created'] and len(wps)>=2:install_local_reference(clone,SimpleNamespace(waypoints=wps))
   clone._coast_enabled=bool(rows[i-1]['tube_viability_armed']);clone._coast_decision=None
   cr=clone.act(ob,goal);assert np.array_equal(cr,request);assert boundary_state(clone)==boundary_state(tracker)
 return dict(boundary=stop,passed=True,prefix_tracker_calls=len(checks),current_clone_request_exact=True,current_clone_state_exact=True,checks=checks,current_request=request.tolist(),clone_before_state=before,future_physical_feedback_available=False,future_B0_requests_available=False,causal_known_request_hold=dict(request=request.tolist(),scope='Explicit open-loop current-request hold only; not B0 future policy or physical prediction'),tracker_attribute_names=sorted(before_dict),tracker_MRO=[str(c) for c in type(tracker).__mro__])
result=dict(scope='POSTCLOSE_PAST_MEASURED_FEEDBACK_CONTROLLER_RECONSTRUCTION_NOT_ONLINE_NOT_PHYSICAL_PREDICTION',results=[reconstruct(i) for i in [221,371]],protocol_sha256=hashlib.sha256((H/'controller_clone_prefix_protocol_v1.json').read_bytes()).hexdigest(),no_actor_changed=True)
(D/'controller_prefix_clone_probe_v2.json').write_text(json.dumps(result,indent=2)+'\n');print([(r['boundary'],r['passed'],r.get('first_mismatch')) for r in result['results']])
