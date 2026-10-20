"""Exactly two candidate geometry queries; archived current comparator, no acts."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
from scripts.run_pointgoal_benchmark import load_action_protocol
from prior_rgb_navigation import PriorRGBPolicy
from soft_route_side_preference_v1 import SoftRouteSidePreferenceGrid
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 cfg=json.loads((H/'soft_route_side_preference_query_protocol_v1.json').read_text())
 for group in ('source_sha256','input_sha256'):
  for p,s in cfg[group].items():assert sha(p)==s,p
 journal=H/'soft_route_side_preference_query_journal_v1.json';assert not journal.exists();results=[];count=0
 protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'));drive=PriorRGBPolicy(protocol,memory_mode='current').tracker.drive_model
 for sel in cfg['selection']:
  D=H/sel['run'];F=sel['frame'];rows=json.loads((D/'policy_records.json').read_text());row=rows[F];assert row['local_plan_created']
  old=json.loads((D/'local_plans'/f'frame_{F:03d}.json').read_text());data=np.load(D/'frames'/f'frame_{F:03d}.npz');m=np.load(D/'maps'/f'frame_{F:03d}.npz');t=float(data['time_s']);assert t==float(m['time_s'])==row['time_s']==old['time_s'];xy=data['root_position_world_m'][:2];R=Rotation.from_quat(data['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));goal=np.asarray(json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);c=row['route_side_commitment'];assert not row['route_side_commitment_released']
  grid=SoftRouteSidePreferenceGrid(m['origin'],10,static_scene_assumption=True);grid.labels=m['labels'].copy();grid.time=t;grid.commitment=c;grid.commitment_released=False;grid._axis_start=np.asarray(c['axis_start']);grid._axis=np.asarray(c['axis']);grid._normal=np.array([-grid._axis[1],grid._axis[0]]);grid._goal=goal.copy();assert grid.resolution==float(m['resolution'])
  count+=1;assert count<=2;journal.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),attempts=count,started_selection=sel,no_retry=True,completed=results),indent=2)+'\n')
  plan,meta=local_curve_first_direction(grid,xy,yaw,goal,drive,initial_direction=old['plan_meta']['initial_direction'],first_primitive_direction=row['first_primitive_direction_query']['direction']);w=[] if plan is None else [vars(p) for p in plan.waypoints];assert np.array_equal(grid.labels,m['labels'])
  same=w==old['waypoints'];status=meta['status'];feasible=status=='success';length_pass=feasible and meta['local_path_length_m']<=old['path_length_m']+1e-8;cusps_pass=feasible and meta['direction_changes']<=old['direction_changes'];clear=distance_transform_edt(np.pad(m['labels']!=1,1))[1:-1,1:-1]*grid.resolution
  # Necessary nominal waypoint clearance only; formal curve checker is unchanged inside search.
  cells=grid.cells(np.asarray([[z['x_m'],z['y_m']] for z in w])) if w else np.empty((0,2),int);inside=grid.inside(cells);nominal_clearance=None if not w or not inside.all() else float(clear[tuple(cells.T)].min());risk_pass=feasible and nominal_clearance is not None and nominal_clearance>=meta['planning_clearance_m']-1e-8
  results.append(dict(selection=sel,current_time_s=t,prior=c,unchanged_current_map=True,baseline=dict(status=old['status'],path_length_m=old['path_length_m'],direction_changes=old['direction_changes'],waypoints=old['waypoints'],meta=old['plan_meta']),candidate=dict(meta=meta,waypoints=w),route_changed=not same,gates=dict(feasible=feasible,no_extra_length=length_pass,no_extra_cusps=cusps_pass,nominal_waypoint_clearance=risk_pass),nominal_waypoint_minimum_raster_clearance_m=nominal_clearance))
 passed=all(all(r['gates'].values()) for r in results) and any(r['route_changed'] for r in results)
 out=dict(results=results,query_count=count,decision='CPU_GEOMETRY_ONLY_PASS' if passed else 'CPU_GEOMETRY_CANDIDATE_REJECT',passed=passed,no_new_physics=True,no_alternate_future_labels=True,no_fresh_model_or_source_admission=True,no_physical_admission=True,whole_mission_complete=False)
 p=H/'soft_route_side_preference_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');journal.write_text(json.dumps(dict(attempts=count,closed=True,no_retry=True,assessment=str(p)),indent=2)+'\n');print(json.dumps(dict(passed=passed,query_count=count,results=[dict(selection=r['selection'],changed=r['route_changed'],gates=r['gates'],status=r['candidate']['meta']['status'],length=r['candidate']['meta'].get('local_path_length_m'),cusps=r['candidate']['meta'].get('direction_changes')) for r in results])))
if __name__=='__main__':main()
