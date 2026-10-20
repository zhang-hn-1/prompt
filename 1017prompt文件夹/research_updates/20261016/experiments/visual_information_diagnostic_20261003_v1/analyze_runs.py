"""Common offline evaluator; no reference index from either tested controller."""
from pathlib import Path
import json,sys
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt
from information_policy import InformationPolicy
from local_map import BeliefGrid
from training.action_protocol import ActionProtocol
from benchmark.policies import PolicyObservation
H=Path(__file__).resolve().parent
FOCAL=16.45/18*640
y,x=np.mgrid[0:480:4,0:640:4]
RAYS=np.c_[(x.ravel()-319.5)/FOCAL,(y.ravel()-239.5)/FOCAL,np.ones(x.size)]
mask=np.load(H.parent/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/base_link_masks.npz')
def initial_belief(root):
    f=np.load(root/'offline_truth/frame_000.npz');xy=f['root_position_world_m'][:2]
    b=BeliefGrid(xy-20);points=[]
    for eye in ('left','right'):
        depth=f[eye+'_depth_m'][::4,::4].ravel();pose=f[eye+'_camera_usd_world']
        rot=np.c_[pose[:3,0],-pose[:3,1],-pose[:3,2]]
        valid=np.isfinite(depth)&(depth>=.45)&(depth<=8)&~mask[eye][::4,::4].ravel()
        points.append(pose[:3,3]+(RAYS[valid]*depth[valid,None])@rot.T)
    b.update_points(np.concatenate(points),f['root_position_world_m'],0);b.seed_body(xy)
    return b
def project(points,reference):
    starts=reference[:-1];delta=np.diff(reference,axis=0);length=np.linalg.norm(delta,axis=1)
    u=np.clip(((points[:,None,:]-starts)*delta).sum(-1)/np.maximum(length**2,1e-12),0,1)
    q=starts+u[...,None]*delta;error=np.linalg.norm(points[:,None,:]-q,axis=-1)
    index=error.argmin(1);rows=np.arange(len(points));arc=np.r_[0,np.cumsum(length)]
    return arc[index]+u[rows,index]*length[index],error[rows,index]
results={};pairs={};coverage={};references={}
for label in ('e0','e2','h3','h4'):
    roots={m:H/'runs'/m/label for m in ('map','visible')}
    maproot=roots['map'];f=np.load(maproot/'online_inputs/frame_000.npz')
    obs=PolicyObservation(f['left_rgb'],f['right_rgb'],f['proprio_history'],f['action_history'],f['position_world_xy'],float(f['yaw_world_rad']),f['quaternion_world_wxyz_history'])
    known=np.load(maproot/'truth_map.npz');p=InformationPolicy(ActionProtocol('ball_action_v2_aux30',(-2.,-np.pi/6),(2.,np.pi/6),5),'map')
    p.set_current_packet(dict(time_s=0.,points_world=np.empty((0,3)),known_grid={k:known[k] for k in known.files}))
    p.act(obs,f['pointgoal_base_xy']);ref=p._path_xy.copy();references[label]=ref
    np.savez_compressed(H/f'common_reference_{label}.npz',position_world_xy=ref,role='offline evaluation only; frozen initial full-map A*')
    a=np.load(roots['map']/'online_inputs/frame_000.npz');b=np.load(roots['visible']/'online_inputs/frame_000.npz')
    pairs[label]={k:float(np.max(np.abs(a[k]-b[k]))) for k in ('position_world_xy','yaw_world_rad','quaternion_world_wxyz_history','proprio_history','action_history','pointgoal_base_xy')}
    initial=initial_belief(roots['visible']);ii,jj=np.indices(initial.labels.shape);world=initial.origin+np.stack((ii,jj),axis=-1)*.1
    # Nominal first-meter path tube only. This is not a braking certification.
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(ref,axis=0),axis=1))]
    first=ref[arc<=1.1];distance=cKDTree(first).query(world.reshape(-1,2))[0].reshape(initial.labels.shape)
    tube=distance<=.6
    clearance=distance_transform_edt(np.pad(initial.labels==0,1))[1:-1,1:-1]*.1
    route,reason=initial.plan(obs.position_world_xy,ref[-1])
    start=tuple(initial.cells(obs.position_world_xy[None])[0])
    coverage[label]=dict(tube_cells=int(tube.sum()),known_free=int((tube&(initial.labels==0)).sum()),occupied=int((tube&(initial.labels==1)).sum()),unknown=int((tube&(initial.labels==-1)).sum()),known_free_fraction=float((initial.labels[tube]==0).mean()),start_observed_free_clearance_m=float(clearance[start]),route_reason=reason)
    for mode,root in roots.items():
        episode=next(root.glob('episode_*'));metrics=json.loads((episode/'metrics.json').read_text())
        state=np.load(episode/'states.npz');physics=np.load(root/'physics.npz');records=json.loads((root/'vision_records.json').read_text());points=state['position_world_xy']
        progress,error=project(points,ref)
        yaw=float(f['yaw_world_rad']);c,s=np.cos(yaw),np.sin(yaw)
        goal=obs.position_world_xy+np.array([[c,-s],[s,c]])@f['pointgoal_base_xy']
        manifest=json.loads((episode/'manifest.json').read_text())
        selected=next(e for e in manifest['episodes'] if e['episode_id']==episode.name.removeprefix('episode_'))
        exact_goal=np.asarray(selected['goal_position_world_m'])[:2]
        assert np.max(np.abs(goal-exact_goal))<1e-5
        goal=exact_goal
        contacts=json.loads((episode/'collision_audit.json').read_text());contact_times=[r['simulation_time_s'] for r in contacts['samples'] if r['contact']]
        clocks=json.loads((root/'camera_clocks.json').read_text());age=[abs(v['age_s']) for row in clocks for v in row['eyes'].values()]
        aux=np.array([r['request'][1] for r in records]);main=np.array([r['request'][0] for r in records])
        stop=np.flatnonzero((physics['simulation_time_s']>1)&np.all(physics['raw_command_target']==0,axis=1))
        last=physics['linear_velocity_world_m_s'][-1,:2]
        out=dict(mode=mode,label=label,termination=metrics['termination_reason'],success=bool(metrics['success']),elapsed_s=float(state['simulation_time_s'][-1]),path_length_m=float(metrics['path_length_m']),
            initial_goal_distance_m=float(np.linalg.norm(points[0]-goal)),final_goal_distance_m=float(np.linalg.norm(points[-1]-goal)),final_reference_progress_m=float(progress[-1]),reference_length_m=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum()),cross_track_rms_m=float(np.sqrt(np.mean(error**2))),cross_track_peak_m=float(error.max()),
            nonzero_request_blocks=int((main!=0).sum()),aux_at_bound_fraction=float(np.mean(np.abs(aux)>=np.pi/6-1e-6)),first_contact_s=None if not contact_times else float(min(contact_times)),terminal_speed_xy_m_s=float(np.linalg.norm(last)),camera_age_max_abs_s=float(max(age)),
            initial_free_cells=records[0]['known_free'],final_free_cells=records[-1]['known_free'],initial_occupied_cells=records[0]['known_occupied'],final_occupied_cells=records[-1]['known_occupied'],all_request_main_zero=bool(np.all(main==0)),
            final_raw_request=physics['raw_command_target'][-1].tolist(),final_applied=physics['applied_action'][-1].tolist(),first_zero_after_1s_s=None if not len(stop) else float(physics['simulation_time_s'][stop[0]]),
            total_camera_pairs=len(clocks),artifacts={k:str(episode/k) for k in ('third_person.mp4','stereo_view.mp4','trajectory_overlay.png')})
        results[mode+'_'+label]=out
(H/'run_assessment.json').write_text(json.dumps(dict(runs=results,initial_state_differences=pairs,initial_nominal_tube_coverage=coverage,reference_role='offline common evaluator; never online in visible'),indent=2)+'\n')
print(json.dumps(dict(runs={k:{s:r[s] for s in ('termination','elapsed_s','final_goal_distance_m','nonzero_request_blocks','first_contact_s','terminal_speed_xy_m_s','cross_track_peak_m')} for k,r in results.items()},pairs=pairs,coverage=coverage),indent=2))
