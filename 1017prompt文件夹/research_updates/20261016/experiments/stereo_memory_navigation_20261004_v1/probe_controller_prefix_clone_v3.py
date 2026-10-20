"""Past-prefix tracker clone, fixed already-installed reference; labels read after output seal."""
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
from benchmark.policies import PolicyObservation
from benchmark.kinodynamic_planning import KinodynamicWaypoint
from estimated_navigation_observation import EstimatedNavigationObservation
D=H/'E1_continuous_tube_navigation_run_v1';protocol=load_action_protocol(DEFAULT_CONTRACT.resolve())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def construct(stop,past_rows):
 assert len(past_rows)==stop and all(r['frame']<stop for r in past_rows)
 # The arrays may contain later samples on disk. Slice immediately; none can enter the constructor.
 states=json.loads((D/'states.json').read_text())[:stop+1]
 with np.load(D/'physics.npz') as ph:gyro_world=ph['angular_velocity_world_rad_s'][:stop*5+1].copy()
 worldgoal=json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m']
 tracker=make_endpoint_oracle(protocol);tracker.__class__=with_cusp_coast_tracker(type(tracker));tracker._coast_enabled=False;tracker._coast_decision=None;tracker._coast_current_observation=None
 adapter=EstimatedNavigationObservation();armed=False;checks=[];installed_time=None
 for i in range(stop+1):
  s=states[i];R=Rotation.from_quat(np.asarray(s['quaternion'])[[1,2,3,0]]).as_matrix();history=np.load(D/'history'/f'frame_{i:03d}.npz');original=PolicyObservation(position_world_xy=np.asarray(s['position'][:2]),yaw_world_rad=float(np.arctan2(R[1,0],R[0,0])),quaternion_world_wxyz_history=history['quaternion_world_wxyz_history'],proprio_history=history['proprio_history'],action_history=history['action_history'],stereo_left_rgb=np.zeros((480,640,3),np.uint8),stereo_right_rgb=np.zeros((480,640,3),np.uint8));pose=dict(time_s=s['time_s'],root_position_world_m=s['position'],root_rotation_world=R,velocity_world_m_s=s['velocity']);ob,goal=adapter.build(original,pose,R.T@gyro_world[i*5],worldgoal)
  if i==stop:
   before=boundary_state(tracker);clone=copy.deepcopy(tracker)
   # No current policy row/reference decision is read. This is explicitly fixed-known-reference tracker consumption.
   for obj in [tracker,clone]:obj._coast_enabled=armed;obj._coast_decision=None
   request=tracker.act(ob,goal);cr=clone.act(ob,goal)
   assert np.array_equal(cr,request) and boundary_state(clone)==boundary_state(tracker)
   # Verify original pre-act instance did not mutate through deepcopy references.
   independent=copy.deepcopy(clone);independent._reference_xy[0,0]+=100
   assert not np.array_equal(independent._reference_xy,clone._reference_xy)
   return dict(boundary=stop,prefix_checks=checks,before_state=before,after_state=boundary_state(clone),uncapped_request=request.tolist(),known_reference_installed_time_s=installed_time,current_policy_row_not_input=True,clone_same_current_request_and_state=True,clone_array_mutation_isolated=True,tracker_attribute_names=sorted(clone.__dict__),full_policy_planning_boundary_reconstruction=False,future_physical_feedback_available=False,future_B0_requests_available=False,known_open_loop_plan=dict(request=request.tolist(),duration_s=.1,description='Five known requests in current action block only; no extension to another block'),recursive_prediction_admitted=False)
  r=past_rows[i]
  if r['prefix_observer']:continue
  if r['local_plan_created']:
   plan=json.loads((D/'local_plans'/f'frame_{i:03d}.json').read_text());assert plan['time_s']<=s['time_s'];wps=tuple(KinodynamicWaypoint(**w) for w in plan['waypoints'])
   if len(wps)>=2:install_local_reference(tracker,SimpleNamespace(waypoints=wps));installed_time=plan['time_s']
  tracker._coast_enabled=armed;tracker._coast_decision=None
  if tracker._context is None:tracker._begin_tracking_audit(ob);request=tracker._zero_action('local_no_route')
  else:request=tracker.act(ob,goal)
  armed=bool(r['tube_viability_armed']);request_exact=np.array_equal(request,np.asarray(r['uncapped_request'],np.float32));state_exact=boundary_state(tracker)==r['tracker_state_after_act'];checks.append(dict(frame=i,request_exact=request_exact,state_exact=state_exact));assert request_exact and state_exact,i

all_rows=json.loads((D/'policy_records.json').read_text());outputs=[]
for stop in [221,371]:
 a=construct(stop,all_rows[:stop]);poison=all_rows[:stop]+[{'frame':9999,'request':'POISON'}]*(len(all_rows)-stop);b=construct(stop,poison[:stop]);assert a==b;outputs.append(a)
pred=D/'controller_prefix_clone_predictions_v3.json';pred.write_text(json.dumps(dict(scope='HISTORICAL_FIXED_KNOWN_REFERENCE_TRACKER_ONLY_NOT_FULL_POLICY_OR_ONLINE',records=outputs,future_policy_log_replacement_and_truncation_exact=True),indent=2)+'\n');seal=dict(file=str(pred),sha256=digest(pred),saved_before_current_label_comparison=True,producer_sha256=digest(Path(__file__)),protocol_sha256=digest(H/'controller_clone_prefix_protocol_v1.json'));(H/'controller_prefix_clone_prelabel_seal_v3.json').write_text(json.dumps(seal,indent=2)+'\n')
# Current act outputs are validation labels, never constructor inputs.
labels=[]
for a in outputs:
 r=all_rows[a['boundary']];labels.append(dict(boundary=a['boundary'],fixed_known_reference_assumption_matches_actual=not r['local_plan_created'] and r['reference_decision']['decision']=='retain',request_exact=np.array_equal(np.asarray(a['uncapped_request'],np.float32),np.asarray(r['uncapped_request'],np.float32)),state_exact=a['after_state']==r['tracker_state_after_act']))
assert all(all(v for k,v in row.items() if k!='boundary') for row in labels);assert digest(pred)==seal['sha256'];(D/'controller_prefix_clone_validation_v3.json').write_text(json.dumps(dict(labels=labels,prediction_sha256=seal['sha256'],no_new_physics=True,no_control_changed=True,no_future_feedback_prediction=True,formal35_NOTRUN=True),indent=2)+'\n');print(labels)
