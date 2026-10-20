import bootstrap
from bootstrap import D,W
import json,hashlib,numpy as np
from types import SimpleNamespace
from benchmark.drive_model import DriveModel
from scripts.run_tracking_acceptance import DEFAULT_DRIVE_PROFILE
from global_guidance import global_curve,original,search,supported_curvature,planning_clearance
from scipy.ndimage import distance_transform_edt
R=W/'experiments/source_confidence_calibration_20261012_v1/data_e1_run';records=json.loads((R/'policy_records.json').read_text());goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2];model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
frames=[i for i,r in enumerate(records) if r.get('local_plan_created') and r.get('local_reference_version',0)>0]
# Only installed-reference boundaries, deduplicated by reference version.
frames=list({r['local_reference_version']:i for i,r in reversed(list(enumerate(records))) if r.get('local_reference_version',0)>0}.values());frames.sort()
class FrozenGrid:
 def __init__(self,z,meta):
  self.labels=z['labels'].copy();self.origin=z['origin'].copy();self.resolution=float(z['resolution']);self.exploration_unknown_fraction=meta['unknown_fraction'];self.route=np.array(meta['route_xy'])
 def cells(self,xy):return np.floor((xy-self.origin)/self.resolution+.5).astype(int)
 def plan(self,xy,goal):return self.route.copy(),'unknown_exploration'
def stats(p):
 if p is None:return None
 dirs=[w.drive_direction for w in p.waypoints[:-1]];n=0
 for d in dirs:
  if d!='reverse':break
  n+=1
 return dict(length=p.path_length_m,cusps=p.direction_changes,initial_reverse_m=n*.3,waypoints=[vars(w) for w in p.waypoints],nodes=p.expanded_nodes)
result=[];identities={}
for i in frames:
 path=R/'local_plans'/f'frame_{i:03d}.json';mp=R/'maps'/f'frame_{i:03d}.npz';saved=json.loads(path.read_text());row=records[i];g=FrozenGrid(np.load(mp),saved['plan_meta']);before=g.labels.tobytes();start=saved['waypoints'][0];args=(g,[start['x_m'],start['y_m']],start['yaw_rad'],goal,model);kw=dict(initial_direction=saved['plan_meta']['initial_direction'],first_primitive_direction=row['first_primitive_direction_query']['direction'])
 b,bm=original(*args,**kw);assert stats(b)['waypoints']==saved['waypoints'] and b.expanded_nodes==saved['expanded_nodes']
 c,cm=global_curve(*args,**kw);detail=cm['global_kinematic'];assert before==g.labels.tobytes();safe=True;minclear=float('inf')
 if not detail['fallback']:
  clearance=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;occ=(clearance<planning_clearance(g)-1e-8).astype('uint8');wp=detail['full_waypoints']
  for a,z in zip(wp[:-1],wp[1:]):
   sample=search.sample_motion_primitive(start_pose=(a['x_m'],a['y_m'],a['yaw_rad']),signed_arc_length_m=.3*(1 if a['main_command_rad_s']>0 else -1),curvature_1_m=supported_curvature(a['main_command_rad_s'],a['auxiliary_angle_rad']),sample_spacing_m=.05)
   assert np.allclose(sample[-1],[z['x_m'],z['y_m'],z['yaw_rad']],atol=1e-10,rtol=0)
   assert search.primitive_is_collision_free(sample,occupancy_grid=occ,origin_xy=g.origin,resolution_m=g.resolution)
   cells=g.cells(sample[:,:2]);minclear=min(minclear,float(clearance[cells[:,0],cells[:,1]].min()))
 else:assert stats(b)==stats(c)
 result.append(dict(frame=i,baseline=stats(b),candidate=stats(c),global_result=detail,minimum_sample_clearance_m=minclear if np.isfinite(minclear) else None,map_unchanged=True,baseline_exact=True))
 for p in [path,mp]:identities[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
 print(i,detail['status'],detail['expanded_nodes'],'reverse',stats(b)['initial_reverse_m'],stats(c)['initial_reverse_m'],flush=True)
 (D/'replay_partial.json').write_text(json.dumps(result,indent=2)+'\n')
k=next(r for r in result if r['frame']==175);admitted=not k['global_result']['fallback'] and k['candidate']['initial_reverse_m']<k['baseline']['initial_reverse_m']-1e-6 and k['candidate']['cusps']<=k['baseline']['cusps']
(D/'replay.json').write_text(json.dumps(dict(pass_gate=admitted,queries=result,input_sha256=identities,scope='same current saved planner inputs, known development E1; no navigation'),indent=2)+'\n');print('PASS',admitted)
