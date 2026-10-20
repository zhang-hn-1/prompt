"""One current stop-qualified reference query; no alternate future or physical labels."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;D=H/'reverse_main_navigation_e1_b0_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def select_stop_boundary(rows,speeds):
 if len(rows)!=len(speeds):raise ValueError('aligned speeds')
 for i in range(1,len(rows)):
  s=rows[i-1].get('tracker_state_after_act',{});speed=float(speeds[i])
  if not np.isfinite(speed) or speed<0:raise ValueError('finite XY norm')
  if s.get('braking_state')=='direction_change' and s.get('pending_direction') in ('forward','reverse') and s.get('pending_direction')!=s.get('active_direction') and s.get('stopped_cycles')==4 and speed<.05:return i
 return None

def main():
 p=H/'stop_boundary_reference_refresh_protocol_v1.json';cfg=json.loads(p.read_text())
 for table in ('source_sha256','input_sha256'):
  for n,h in cfg[table].items():assert sha(n)==h,n
 journal=H/'stop_boundary_reference_refresh_query_journal_v1.json';assert not journal.exists()
 rows=json.loads((D/'policy_records.json').read_text());speeds=[float(np.linalg.norm(np.load(D/'history'/f'frame_{i:03d}.npz')['proprio_history'][-1,:2])) for i in range(146)];F=select_stop_boundary(rows[:146],speeds);assert F==145
 before=rows[F-1]['tracker_state_after_act'];after=rows[F]['tracker_state_after_act'];assert before['stopped_cycles']==4 and after['active_direction']==before['pending_direction'] and after['stopped_cycles']==0
 assert all(speeds[i]<.05 for i in range(F-4,F+1));assert all(rows[i]['tracker_state_after_act']['stopped_cycles']==i-F+5 for i in range(F-4,F))
 created=[i for i,x in enumerate(rows[:F]) if x.get('planner_act_called') and x.get('local_plan_created')];rootframe=created[-1];assert rootframe==120
 data=np.load(D/'frames'/f'frame_{F:03d}.npz');m=np.load(D/'maps'/f'frame_{F:03d}.npz');old=json.loads((D/'local_plans'/f'frame_{F:03d}.json').read_text());start=np.load(D/'frames'/f'frame_{rootframe:03d}.npz');t=float(data['time_s']);assert t==float(m['time_s'])==rows[F]['time_s']==old['time_s'];R=Rotation.from_quat(data['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));xy=data['root_position_world_m'][:2];goal=np.asarray(json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);grid=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);grid.labels=m['labels'].copy();grid.time=t
 contract=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml');drive=PriorRGBPolicy(load_action_protocol(contract),memory_mode='current').tracker.drive_model
 journal.write_text(json.dumps(dict(attempts=1,no_retry=True,frame=F,utc=datetime.datetime.now(datetime.timezone.utc).isoformat()),indent=2)+'\n')
 plan,meta=local_curve_first_direction(grid,xy,yaw,goal,drive,initial_direction=before['active_direction'],first_primitive_direction=None);assert np.array_equal(grid.labels,m['labels']);w=[] if plan is None else [vars(p) for p in plan.waypoints]
 guard=reference_geometry_status(xy,np.array([[w['x_m'],w['y_m']] for w in old['waypoints']]),m['labels'],m['origin'],float(m['resolution']))
 result=dict(frame=F,root_reference_frame=rootframe,reference_age_s=t-float(start['time_s']),root_drift_m=float(np.linalg.norm(xy-start['root_position_world_m'][:2])),xy_speeds_five=speeds[F-4:F+1],before=before,after=after,retained=dict(archived_full_reference_nominal_arc_length_m=old['path_length_m'],archived_full_reference_cusps=old['direction_changes'],tracker_index_before=before['path_index'],geometric_projection_remaining_chord_arc_m=guard['remaining_arc_m'],geometric_projection_bridge_m=guard['projection_distance_m'],guard=guard,first_direction=old['waypoints'][0]['drive_direction'],comparison_warning='Old whole arc, continuous geometric suffix and fresh nominal arc are distinct; bridge is not certified executable travel; projection never alters tracker cursor.'),refreshed=dict(meta=meta,waypoints=w),labels_exact=True,scope='current known-pose geometric query only; no alternate-plan future labels, no actor/physics or model admission',whole_mission_complete=False,protocol_sha256=sha(p))
 out=H/'stop_boundary_reference_refresh_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');journal.write_text(json.dumps(dict(attempts=1,closed=True,no_retry=True,result=str(out)),indent=2)+'\n');print(json.dumps(dict(frame=F,drift_m=result['root_drift_m'],status=meta['status'],length=meta.get('local_path_length_m'),cusps=meta.get('direction_changes'),first_direction=None if not w else w[0]['drive_direction'])))
if __name__=='__main__':main()
