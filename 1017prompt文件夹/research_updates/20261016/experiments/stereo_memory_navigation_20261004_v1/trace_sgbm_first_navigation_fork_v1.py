"""One sealed boundary/source cause audit; no controller intervention or gate."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.run_pointgoal_benchmark import load_action_protocol
from outgoing_lookahead import make_endpoint_oracle
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from execution_domain_four_point_consumer_v2 import local_curve_for_direction_tube
H=Path(__file__).resolve().parent;D=H/'sgbm_baseline35_e2_run_v1';B=H/'standby_handoff_e2_transfer_run_v1';O=H/'sgbm_first_navigation_fork_trace_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for directory in (D,B):
    seal=json.loads((directory/'artifact_manifest_post_close.json').read_text())
    assert all(sha(p)==h for p,h in seal['artifact_sha256'].items())
ar=json.loads((D/'policy_records.json').read_text());br=json.loads((B/'policy_records.json').read_text())
s=json.loads((D/'states.json').read_text())[60];ep=json.loads((D/'task_episode.json').read_text())['actual_episode']
q=np.asarray(s['quaternion']);R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]))
protocol=load_action_protocol(Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/configs/data_contract.yaml'))
model=make_endpoint_oracle(protocol).drive_model
with np.load(D/'maps/frame_060.npz') as m,np.load(B/'maps/frame_060.npz') as bm:
    origin=m['origin'].copy();labels=m['labels'].copy();oldlabels=bm['labels'].copy();assert np.array_equal(origin,bm['origin']);assert (oldlabels==-1).all()
cases={}
for name,lab in [('actual_SGBM',labels),('remove_only_past_frame5_risks_to_unknown',oldlabels)]:
    grid=RepeatedExposureMemoryGrid(origin,10,static_scene_assumption=True);grid.labels=lab.copy()
    plan,meta=local_curve_for_direction_tube(grid,np.asarray(s['position'])[:2],yaw,np.asarray(ep['goal_position_world_m'])[:2],model,initial_direction=None)
    assert plan is not None
    xy=np.array([[p.x_m,p.y_m] for p in plan.waypoints])
    if name=='actual_SGBM':np.testing.assert_array_equal(xy,ar[60]['installed_reference_xy_after_act']);assert meta['local_target']==ar[60]['plan']['local_target']
    cases[name]=dict(local_target=meta['local_target'],reference_xy=xy.tolist(),route_xy=meta['route_xy'],risk_cell_count=int((lab==1).sum()),changes_unknown_to_free=False)
with np.load(D/'partial_observations/frame_005.npz') as p,np.load(D/'source_inputs/frame_005.npz') as rgb:
    points=p['world'].copy();ids=p['query_pixel_id'].copy();partners=p['partner_pixel_id'].copy()
    colours={eye:rgb[eye+'_rgb'].reshape(-1,3)[query].tolist() for eye,query in [('left',ids),('right',partners)]}
    cells=np.rint((points[:,:2]-origin)/.1).astype(int);assert set(map(tuple,cells))==set(map(tuple,np.argwhere(labels==1)))
trace=dict(scope='offline sealed first boundary cause audit, not online or new control',first_request_fork_frame=60,first_request_fork_time_s=ar[60]['time_s'],past_source_frame=5,past_source_time_s=.5,source_points=len(points),source_world=points.tolist(),source_query_pixels=ids.tolist(),source_partner_pixels=partners.tolist(),source_pixel_colours=colours,cases=cases,actual_current_request_SGBM=ar[60]['request'],historical_Fast_request=br[60]['request'],actual_reference_reproduced_exact=True,no_policy_act_calls=True,no_new_physics_or_parameter_changes=True,no_source_acceptance_gate=True)
predp=O/'source_and_plan_trace_before_truth.json';predp.write_text(json.dumps(trace,indent=2)+'\n');(O/'prediction_seal_before_truth.json').write_text(json.dumps(dict(trace_sha256=sha(predp),input_sha256={str(p):sha(p) for p in [D/'maps/frame_060.npz',B/'maps/frame_060.npz',D/'partial_observations/frame_005.npz',D/'source_inputs/frame_005.npz']},teacher_depth_not_read_by_this_audit_yet=True),indent=2)+'\n')
# Labels now read for explanation only, never to remove points from the actor.
with np.load(D/'frames/frame_005.npz') as truth:
    depth=truth['left_depth_m'].ravel()[ids];valid=np.isfinite(depth)&(depth>0);x=ids%640;y=ids//640
    rays=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ids))]);pose=truth['left_camera_usd_world'];trueworld=pose[:3,3]+(rays*depth[:,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
    errors=np.linalg.norm(points-trueworld,axis=1)
label=dict(scope='postseal past exposure renderer-depth labels only; no source/control gate or filtering',count=len(ids),valid_depth_count=int(valid.sum()),invalid_depth_count=int((~valid).sum()),valid_world_error_median=float(np.median(errors[valid])) if valid.any() else None,valid_world_error_p90=float(np.quantile(errors[valid],.9)) if valid.any() else None,all_current_left_pixels_exact_black=int(np.sum(np.all(np.asarray(colours['left'])==0,axis=1))),all_current_right_pixels_exact_black=int(np.sum(np.all(np.asarray(colours['right'])==0,axis=1))),source_points_not_removed=True,depth_values_m=depth.tolist(),renderer_labels_not_full_environment_safety=True)
(O/'postseal_teacher_diagnostic.json').write_text(json.dumps(label,indent=2)+'\n')
(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()}),indent=2)+'\n');print(dict(cases={k:dict(target=v['local_target'],riskcells=v['risk_cell_count']) for k,v in cases.items()},labels=label))
