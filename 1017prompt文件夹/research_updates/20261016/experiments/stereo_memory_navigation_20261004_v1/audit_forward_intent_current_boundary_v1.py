"""Frozen two-query current-boundary replay, not an online navigation result."""
from pathlib import Path
import json,hashlib,types,numpy as np
from scipy.spatial.transform import Rotation
from benchmark.policies import PolicyObservation
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from minimum_prefix_transition_consumer_v1 import measured_first_direction,local_curve_first_direction
from current_forward_intent_qualification_v1 import measured_forward_intent
H=Path(__file__).resolve().parent;D=H/'rolling_endpoint_coverage_known_e1_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
if __name__=='__main__':
 identity=json.loads((H/'forward_intent_current_boundary_protocol_v1.json').read_text())
 for p,h in identity['input_sha256'].items():assert sha(p)==h,p
 frame=120;loaded=[];original_load=np.load
 def guarded(p,*a,**kw):
  p=Path(p)
  if p.parent.name in ['frames','history','maps']:
   assert p.name==f'frame_{frame:03d}.npz','no future or other-frame array payload';loaded.append(str(p))
  return original_load(p,*a,**kw)
 np.load=guarded
 capture=dict(np.load(D/'frames'/f'frame_{frame:03d}.npz'));hist=dict(np.load(D/'history'/f'frame_{frame:03d}.npz'));m=dict(np.load(D/'maps'/f'frame_{frame:03d}.npz'));saved=json.loads((D/'local_plans'/f'frame_{frame:03d}.json').read_text())
 t=float(capture['time_s']);assert t==float(m['time_s'])==saved['time_s']
 R=Rotation.from_quat(capture['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));xy=capture['root_position_world_m'][:2];zero=np.zeros((480,640,3),np.uint8)
 observation=PolicyObservation(zero,zero,hist['proprio_history'],hist['action_history'],xy,yaw,hist['quaternion_world_wxyz_history'])
 row=next(json.loads(s)['row'] for s in (D/'policy_rows.ndjson').read_text().splitlines() if json.loads(s)['frame']==frame and json.loads(s)['phase']=='planned');state=row['first_primitive_direction_query']['measured_state'];old=measured_first_direction(observation,state);new=measured_forward_intent(observation,state)
 assert old==row['first_primitive_direction_query']['direction'] and new=='forward'
 grid=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);grid.labels=m['labels'].copy();grid.time=t;assert grid.resolution==float(m['resolution'])
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));drive=PriorRGBPolicy(protocol,memory_mode='current').tracker.drive_model;goal=json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]
 queries={}
 for label,first in [('original',old),('candidate',new)]:
  plan,meta=local_curve_first_direction(grid,xy,yaw,goal,drive,initial_direction=state['active_direction'],first_primitive_direction=first)
  waypoints=[] if plan is None else [vars(w) for w in plan.waypoints];queries[label]=dict(meta=meta,waypoints=waypoints)
 assert len(queries['original']['waypoints'])==len(saved['waypoints'])
 max_error=0.
 for got,want in zip(queries['original']['waypoints'],saved['waypoints']):
  assert got['drive_direction']==want['drive_direction']
  for k in ['x_m','y_m','yaw_rad','main_command_rad_s','auxiliary_angle_rad']:max_error=max(max_error,abs(got[k]-want[k]))
 assert max_error<=1e-8
 candidate=queries['candidate'];w=candidate['waypoints'];prefix=[a['drive_direction'] for a in w[:2]];passed=candidate['meta']['status']=='success' and len(w)>=3 and prefix==['forward','forward']
 out=H/'forward_intent_current_boundary_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(dict(frame=frame,time_s=t,original_reproduction_max_error=max_error,old_intent=old,new_intent=new,queries=queries,candidate_cpu_gate=passed,actual_current_map_unchanged=True,current_frame_array_reads=loaded,not_online=True,no_alternate_future_truth_labels=True,not_physical_admission=True,not_certified_stop_bound=True,whole_mission_complete=False),indent=2)+'\n');print(json.dumps(dict(original_error=max_error,new_intent=new,candidate_status=candidate['meta']['status'],candidate_cpu_gate=passed,old_changes=queries['original']['meta'].get('direction_changes'),new_changes=candidate['meta'].get('direction_changes'),old_length=queries['original']['meta'].get('local_path_length_m'),new_length=candidate['meta'].get('local_path_length_m'))))
