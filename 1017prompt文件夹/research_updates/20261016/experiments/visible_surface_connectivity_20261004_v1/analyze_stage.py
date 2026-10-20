"""Frozen-reference independent assessment, actual motion and map support."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt,label
H=Path(__file__).resolve().parent;OLD=H.parent/'visual_information_new_rig_20261004_v2'

def project(points,ref):
    delta=np.diff(ref,axis=0);length=np.linalg.norm(delta,axis=1)
    u=np.clip(((points[:,None]-ref[:-1])*delta).sum(-1)/np.maximum(length**2,1e-12),0,1)
    error=np.linalg.norm(points[:,None]-(ref[:-1]+u[:,:,None]*delta),axis=-1)
    index=error.argmin(1);row=np.arange(len(points));arc=np.r_[0,np.cumsum(length)]
    return arc[index]+u[row,index]*length[index],error[row,index]

def support(f):
    labels=f['labels'];origin=f['origin'];resolution=float(f['resolution']);clear=distance_transform_edt(np.pad(labels==0,1))[1:-1,1:-1]*resolution
    return labels,origin,resolution,clear

def compare_npz(a,b,keys):
    a=np.load(a);b=np.load(b)
    return {k:bool(np.array_equal(a[k],b[k])) for k in keys}

results={};pairs={};historical={};behavior={}
KEYS=('position_world_xy','yaw_world_rad','quaternion_world_wxyz_history','proprio_history','action_history','pointgoal_base_xy')
for case in ('e0','e2','h3','h4'):
    ref_file=OLD/f'common_reference_{case}.npz';ref=np.load(ref_file)['position_world_xy'];reference_hash=hashlib.sha256(ref_file.read_bytes()).hexdigest()
    roots={r:H/'runs'/r/case for r in ('points','surface')}
    pairs[case]=compare_npz(roots['points']/'online_inputs/frame_000.npz',roots['surface']/'online_inputs/frame_000.npz',KEYS)
    historical[case]=compare_npz(roots['points']/'online_inputs/frame_000.npz',OLD/'runs/visible'/case/'online_inputs/frame_000.npz',KEYS)
    for arm,root in roots.items():
        ep=next(root.glob('episode_*'));m=json.loads((ep/'metrics.json').read_text());s=np.load(ep/'states.npz');p=np.load(root/'physics.npz');records=json.loads((root/'vision_records.json').read_text());xy=s['position_world_xy'];progress,error=project(xy,ref)
        inp=np.load(root/'online_inputs/frame_000.npz');manifest=json.loads((ep/'manifest.json').read_text());entry=next(e for e in manifest['episodes'] if e['episode_id']==ep.name.removeprefix('episode_'));goal=np.array(entry['goal_position_world_m'])[:2]
        contact=json.loads((ep/'collision_audit.json').read_text());times=[r['simulation_time_s'] for r in contact['samples'] if r['contact']]
        requests=np.array([r['request'] for r in records]);main=requests[:,0];idx=np.flatnonzero(main!=0)
        clocks=json.loads((root/'camera_clocks.json').read_text());age=max(abs(v['age_s']) for c in clocks for v in c['eyes'].values())
        snapshots=[]
        for path in sorted((root/'belief_snapshots').glob('*.npz')):
            i=int(path.stem.split('_')[1]);f=np.load(path);labels,origin,res,clear=support(f);pos=np.array(records[i]['position_world_xy']);cell=np.rint((pos-origin)/res).astype(int)
            safe=(labels==0)&(clear>=.6-1e-8);safe[tuple(cell)]=True;cc,_=label(safe,np.ones((3,3)));choices=np.argwhere(cc==cc[tuple(cell)]);distance=np.linalg.norm(origin+choices*res-pos,axis=1)
            snapshots.append(dict(index=i,time_s=records[i]['time_s'],start_clearance_m=float(clear[tuple(cell)]),reachable_safe_cells=int(len(choices)),reachable_radius_m=float(distance.max()),known_free=int((labels==0).sum()),occupied=int((labels==1).sum()),route_reason=records[i]['route_reason']))
        out=dict(arm=arm,case=case,termination=m['termination_reason'],success=bool(m['success']),elapsed_s=float(s['simulation_time_s'][-1]),path_length_m=float(m['path_length_m']),net_displacement_m=float(np.linalg.norm(xy[-1]-xy[0])),initial_goal_distance_m=float(np.linalg.norm(xy[0]-goal)),final_goal_distance_m=float(np.linalg.norm(xy[-1]-goal)),goal_distance_reduction_m=float(np.linalg.norm(xy[0]-goal)-np.linalg.norm(xy[-1]-goal)),initial_reference_progress_m=float(progress[0]),final_reference_progress_m=float(progress[-1]),reference_progress_gain_m=float(progress[-1]-progress[0]),cross_track_rms_m=float(np.sqrt(np.mean(error**2))),cross_track_peak_m=float(error.max()),nonzero_main_request_blocks=int(len(idx)),first_nonzero_request_s=None if not len(idx) else records[int(idx[0])]['time_s'],first_contact_s=None if not times else min(times),terminal_speed_xy_m_s=float(np.linalg.norm(p['linear_velocity_world_m_s'][-1,:2])),initial_free_cells=records[0]['known_free'],final_free_cells=records[-1]['known_free'],initial_occupied_cells=records[0]['known_occupied'],final_occupied_cells=records[-1]['known_occupied'],all_request_zero=bool(np.all(requests==0)),camera_pair_count=len(clocks),max_camera_age_abs_s=float(age),no_route_blocks=sum(r['route_reason']=='no_known_free_route' for r in records),action_blocks=len(records),reference_file=str(ref_file),reference_sha256=reference_hash,support_over_time=snapshots,artifacts={name:str(ep/name) for name in ('third_person.mp4','stereo_view.mp4','trajectory_overlay.png')})
        results[arm+'_'+case]=out
    pa=np.load(roots['points']/'physics.npz');pb=np.load(roots['surface']/'physics.npz')
    rawsame=np.array_equal(pa['raw_command_target'],pb['raw_command_target']);appliedsame=np.array_equal(pa['applied_action'],pb['applied_action'])
    sa=np.load(next(roots['points'].glob('episode_*'))/'states.npz');sb=np.load(next(roots['surface'].glob('episode_*'))/'states.npz')
    behavior[case]=dict(raw_requests_bit_equal=bool(rawsame),applied_actions_bit_equal=bool(appliedsame),trajectory_bit_equal=bool(np.array_equal(sa['position_world_xy'],sb['position_world_xy'])),max_position_difference_m=float(np.max(np.abs(sa['position_world_xy']-sb['position_world_xy']))) if sa['position_world_xy'].shape==sb['position_world_xy'].shape else None)
    oldphys=np.load(OLD/'runs/visible'/case/'physics.npz');oldstate=np.load(next((OLD/'runs/visible'/case).glob('episode_*'))/'states.npz')
    historical[case]['entire_raw_request_equal']=bool(np.array_equal(oldphys['raw_command_target'],pa['raw_command_target']))
    historical[case]['entire_applied_equal']=bool(np.array_equal(oldphys['applied_action'],pa['applied_action']))
    historical[case]['entire_xy_state_equal']=bool(np.array_equal(oldstate['position_world_xy'],sa['position_world_xy']))
accept=all(results['surface_'+k]['success'] and results['surface_'+k]['first_contact_s'] is None for k in ('e0','e2'))
output=dict(runs=results,paired_initial_state_equal=pairs,historical_baseline_reproduction=historical,behavior_equivalence=behavior,candidate_accepted=accept,reference_scope='Previous frozen initial full-map reference used only for offline measurement; never consumed by visible policies.')
(H/'run_assessment.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps({k:{f:r[f] for f in ('termination','path_length_m','final_goal_distance_m','nonzero_main_request_blocks','reference_progress_gain_m')} for k,r in results.items()},indent=2))
