"""Two evaluator-only exact-geometry queries; hidden geometry never enters actor."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from risk_grid import astar_path
H=Path(__file__).resolve().parent;O=H/'E1_frozen_side_prior_fullgeometry_feasibility_v1';O.mkdir(exist_ok=False);D=H/'original_E1_tube_viability_navigation_run_v1';rows=json.loads((D/'policy_records.json').read_text());shapesfile=H/'original_E1_tube_viability_sparse_membership_v1/scene_collision_shapes.json';shapes=json.loads(shapesfile.read_text());commit=next(r['route_side_commitment'] for r in rows if r.get('route_side_commitment') is not None);m=np.load(D/'maps/frame_060.npz');origin=m['origin'];resolution=float(m['resolution']);shape=m['labels'].shape;start=np.array([-3.2999999940395353,5.960465010446114e-9]);goal=np.array([3.300000005960465,5.960465010446114e-9]);margin=.8707106781186548
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),question='Does fixed observed-side planning prior exclude all complete paths on exact originalE1 XY cylinder geometry at the alreadyfrozen margin?',maximum_queries=2,margin_m=margin,source='71 originalUSD scene collision cylinders; evaluatoronly',hidden_geometry_NOT_USED_BY_ACTOR=True,no_new_pose_or_depth_model=True,no_gate_or_action_changes=True,complete3D_safety_NOTPROVEN=True,old_T_development_not_fresh=True,inputs_sha256={str(shapesfile):hashlib.sha256(shapesfile.read_bytes()).hexdigest(),str(D/'artifact_manifest_post_close.json'):hashlib.sha256((D/'artifact_manifest_post_close.json').read_bytes()).hexdigest()},whole_research_complete=False)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
cells=np.indices(shape).reshape(2,-1).T;world=origin+cells*resolution;clear=np.full(len(world),np.inf)
for c in shapes:clear=np.minimum(clear,np.linalg.norm(world-np.array(c['position'][:2]),axis=1)-c['radius'])
base=(clear>=margin-1e-8).reshape(shape);axis=np.asarray(commit['axis']);normal=np.array([-axis[1],axis[0]]);axisstart=np.asarray(commit['axis_start']);long=(world-axisstart)@axis;side=(world-axisstart)@normal;lo,hi=commit['longitudinal_span_m'];prior=((long>=lo)&(long<=hi)&(commit['side']*side<-.5*resolution)).reshape(shape);sc=np.rint((start-origin)/resolution).astype(int);gc=np.rint((goal-origin)/resolution).astype(int);answers=[]
for useprior in [False,True]:
 free=base&(~prior if useprior else np.ones(shape,bool));path=None
 try:path=astar_path((~free).astype(np.uint8),start=tuple(sc),goal=tuple(gc))
 except ValueError:pass
 r=dict(side_prior_used=useprior,start_valid=bool(free[tuple(sc)]),goal_valid=bool(free[tuple(gc)]),full_geometry_route_exists=path is not None)
 if path is not None:
  xy=origin+path*resolution;r.update(path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),min_y_m=float(xy[:,1].min()),max_y_m=float(xy[:,1].max()));np.savez_compressed(O/('with_prior_route.npz' if useprior else 'without_prior_route.npz'),xy=xy)
 answers.append(r)
a=dict(decision='EVALUATOR_ONLY_FULL_GEOMETRY_PRIOR_FEASIBILITY_DIAGNOSTIC',answers=answers,prior=commit,margin_m=margin,collision_cylinders=len(shapes),no_controller_or_source_modification=True,no_new_physics=True,no_new_navigation_success=True,hidden_geometry_NOT_USED_BY_ACTOR=True,actual_running_candidate_NOTCHANGED=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
