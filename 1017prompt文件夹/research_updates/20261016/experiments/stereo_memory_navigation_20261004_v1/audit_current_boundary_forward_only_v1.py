"""One new current-boundary geometry diagnostic, not a reused physics trial."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from supported_forward_point_consumer import local_curve_for_direction_tube
H=Path(__file__).resolve().parent;D=H/'reverse_main_navigation_e1_b0_run_v1';F=120
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def select_rebuild(rows):
 for x in rows:
  if x['planner_act_called']:assert 'local_plan_created' in x,'required valid planner diagnostic'
 created=[i for i,x in enumerate(rows) if x['planner_act_called'] and x.get('local_plan_created',False)]
 return created[1]

def main():
 p=H/'current_boundary_forward_only_protocol_v1.json';cfg=json.loads(p.read_text())
 for table in ('source_sha256','input_sha256'):
  for n,h in cfg[table].items():assert sha(n)==h,n
 journal=H/'current_boundary_forward_only_query_journal_v1.json';assert not journal.exists()
 rows=json.loads((D/'policy_records.json').read_text());selected=select_rebuild(rows);assert selected==F
 data=np.load(D/'frames'/f'frame_{F:03d}.npz');m=np.load(D/'maps'/f'frame_{F:03d}.npz');old=json.loads((D/'local_plans'/f'frame_{F:03d}.json').read_text());t=float(data['time_s']);assert t==float(m['time_s'])==rows[F]['time_s']==old['time_s'];R=Rotation.from_quat(data['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));xy=data['root_position_world_m'][:2];goal=np.asarray(json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);grid=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);grid.labels=m['labels'].copy();grid.time=t
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));drive=PriorRGBPolicy(protocol,memory_mode='current').tracker.drive_model
 journal.write_text(json.dumps(dict(attempts=1,no_retry=True,frame=F,utc=datetime.datetime.now(datetime.timezone.utc).isoformat()),indent=2)+'\n')
 plan,meta=local_curve_for_direction_tube(grid,xy,yaw,goal,drive,initial_direction=old['plan_meta']['initial_direction']);assert np.array_equal(grid.labels,m['labels']);assert meta['route_xy']==old['plan_meta']['route_xy'];w=[] if plan is None else [vars(p) for p in plan.waypoints];assert all(x['drive_direction']=='forward' for x in w)
 result=dict(query_count=1,current_frame=F,current_time_s=t,baseline=dict(path_length_m=old['path_length_m'],direction_changes=old['direction_changes'],waypoints=old['waypoints']),forward_only=dict(meta=meta,waypoints=w),global_route_exact=True,current_labels_exact=True,forward_only_feasible=plan is not None,old_R1_frame140_reject_unchanged=True,interpretation='Success only proves this discrete current-map geometric alternative. Failure is bounded solver/domain evidence, not impossibility of all physical forward motion. No actor, dynamics admission or alternative future truth.',no_new_physics=True,no_model_fit=True,whole_mission_complete=False,protocol_sha256=sha(p))
 out=H/'current_boundary_forward_only_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');journal.write_text(json.dumps(dict(attempts=1,no_retry=True,closed=True,result=str(out)),indent=2)+'\n');print(json.dumps(dict(feasible=plan is not None,status=meta['status'],expanded=meta.get('expanded_nodes'),length=meta.get('local_path_length_m'),cusps=meta.get('direction_changes'),no_new_physics=True)))
if __name__=='__main__':main()
