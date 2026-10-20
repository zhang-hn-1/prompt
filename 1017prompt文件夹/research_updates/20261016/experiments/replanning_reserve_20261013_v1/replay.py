import bootstrap
from bootstrap import D,W,H
import json,hashlib,numpy as np
from benchmark.drive_model import DriveModel
from scripts.run_tracking_acceptance import DEFAULT_DRIVE_PROFILE
from minimum_prefix_transition_consumer_v1 import local_curve_first_direction
from reference_continuity_audit import reference_geometry_status
from reserve import reserve_curve,reserve_screen
from continuous_cell_tube_v1 import continuous_cell_tube_screen
from scipy.ndimage import distance_transform_edt
from execution_domain_four_point_consumer_v2 import supported_curvature
import minimum_prefix_transition_planning_v1 as search
R=W/'experiments/source_confidence_calibration_20261012_v1/data_e1_run';records=json.loads((R/'policy_records.json').read_text());goal=json.loads((R/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2];model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
class Grid:
 def __init__(self,z,meta):self.labels=z['labels'].copy();self.origin=z['origin'].copy();self.resolution=float(z['resolution']);self.exploration_unknown_fraction=meta.get('unknown_fraction',1);self.route=np.asarray(meta.get('route_xy',[]))
 def cells(self,p):return np.floor((np.asarray(p)-self.origin)/self.resolution+.5).astype(int)
 def inside(self,c):return ((c>=0)&(c<np.asarray(self.labels.shape))).all(1)
 def plan(self,xy,goal):return self.route.copy(),'unknown_exploration'
def accepted_check(p,g):
 if p is None:return None
 # Continuous primitives, not only their endpoints. Same old frozen .05 sampling.
 field=distance_transform_edt(np.pad(g.labels!=1,1))[1:-1,1:-1]*g.resolution;minimum=np.inf
 for a,b in zip(p.waypoints[:-1],p.waypoints[1:]):
  sample=search.sample_motion_primitive(start_pose=(a.x_m,a.y_m,a.yaw_rad),signed_arc_length_m=.3*(1 if a.main_command_rad_s>0 else -1),curvature_1_m=supported_curvature(a.main_command_rad_s,a.auxiliary_angle_rad),sample_spacing_m=.05)
  assert np.allclose(sample[-1],[b.x_m,b.y_m,b.yaw_rad],atol=1e-10,rtol=0)
  cells=g.cells(sample[:,:2]);assert g.inside(cells).all();minimum=min(minimum,float(field[tuple(cells.T)].min()))
 assert minimum>=1.0707106781186548-1e-8
 points=np.array([[w.x_m,w.y_m] for w in p.waypoints]);screen=reserve_screen(g,reference_geometry_status(points[0],points,g.labels,g.origin,g.resolution));assert screen['pass']
 return dict(minimum_sample_clearance_m=minimum,length=p.path_length_m,cusps=p.direction_changes,waypoints=[vars(w) for w in p.waypoints])
frames=list({r['local_reference_version']:i for i,r in reversed(list(enumerate(records))) if r.get('local_reference_version',0)>0}.values());frames.sort();installs=[];identities={}
for i in frames:
 sp=R/'local_plans'/f'frame_{i:03d}.json';saved=json.loads(sp.read_text());mp=R/'maps'/f'frame_{i:03d}.npz';g=Grid(np.load(mp),saved['plan_meta']);start=saved['waypoints'][0];args=(g,[start['x_m'],start['y_m']],start['yaw_rad'],goal,model);kw=dict(initial_direction=saved['plan_meta']['initial_direction'],first_primitive_direction=records[i]['first_primitive_direction_query']['direction'])
 b,bm=local_curve_first_direction(*args,**kw);assert [vars(w) for w in b.waypoints]==saved['waypoints'] and b.expanded_nodes==saved['expanded_nodes']
 before=g.labels.tobytes();c,cm=reserve_curve(*args,**kw);checks=accepted_check(c,g);assert g.labels.tobytes()==before
 installs.append(dict(frame=i,baseline_exact=True,status=cm['status'],candidate=checks));print('install',i,cm['status'],flush=True)
 for p in [sp,mp]:identities[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
boundaries=[]
for i in range(60,900,5):
 prev=records[i-1];a=records[i];decision=a.get('reference_decision') or {};guard=decision.get('guard')
 if guard is None:continue
 mp=R/'maps'/f'frame_{i:03d}.npz';posep=R/'known_poses'/f'frame_{i:03d}.npz';npz=np.load(mp);n=np.load(posep);xy=n['root_position_world_m'][:2]
 # Use native calibrated rig rotation only; no frame GT.
 T=n['left_camera_usd_world'];FLIP=np.diag([1,-1,-1]);ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
 rr=T[:3,:3]@FLIP@ROT.T;yaw=float(np.arctan2(rr[1,0],rr[0,0]));g=Grid(npz,a.get('plan') or {});path=np.asarray(prev['installed_reference_xy_after_act']);actualguard=reference_geometry_status(xy,path,g.labels,g.origin,g.resolution)
 # Original guard calculations must reproduce same source/current state.
 assert np.allclose(actualguard['suffix_xy'],guard['suffix_xy'],atol=1e-10,rtol=0)
 screen=reserve_screen(g,actualguard);oldkeep=decision['decision']=='retain';change=oldkeep and not screen['pass'];row=dict(frame=i,old_retain=oldkeep,reserve_screen=screen,additional_rebuild=change)
 if change:
  before=decision['state_before'];initial=before['active_direction'];fd=(a.get('first_primitive_direction_query') or {}).get('direction')
  # Compute measured first-direction eligibility exactly, independent of whether parent retained.
  from minimum_prefix_transition_consumer_v1 import measured_first_direction
  from types import SimpleNamespace
  hist=np.load(R/'history'/f'frame_{i:03d}.npz');fd=measured_first_direction(SimpleNamespace(proprio_history=hist['proprio_history'],yaw_world_rad=yaw,quaternion_world_wxyz_history=hist['quaternion_world_wxyz_history']),before)
  c,cm=reserve_curve(g,xy,yaw,goal,model,initial_direction=initial,first_primitive_direction=fd);row.update(status=cm['status'],candidate=accepted_check(c,g),direction=fd);print('reserve boundary',i,cm['status'],flush=True)
 boundaries.append(row)
 for p in [mp,posep]:identities[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
good=[r['frame'] for r in boundaries if r['additional_rebuild'] and r.get('candidate') is not None]
(D/'replay.json').write_text(json.dumps(dict(admitted=bool(good) and len(installs)==10,baseline_installs_exact=len(installs),installs=installs,boundaries=boundaries,effect_frames=good,input_sha256=identities,scope='Current saved development E1, not new navigation; .2 reserve assumption unc ertified'),indent=2)+'\n');print('ADMITTED',bool(good),good,flush=True)
