"""Only four CURRENT CAD masks/source sampling comparisons; no teacher or actor."""
from pathlib import Path
import collections,datetime,hashlib,json,sys
import numpy as np
from memory_availability_identity_v1 import verify_identity
from dense_current_source_packet_v1 import dense_point_packet
from dense_source_points import point_packet
from current_stereo_memory_ray_v2 import ray_evidence
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent
DEP=Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
sys.path.insert(0,str(DEP))
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio

if __name__=='__main__':
    verify_identity(H/'dense_current_stereo_ray_interface_identity_v1.json')
    output=H/'dense_current_stereo_ray_interface_diagnostic_v1.json'
    if output.exists():raise FileExistsError('fixed four-pair diagnostic already exists')
    D=H/'sgbm_baseline35_e1_coverage_run_v1';cad=TimedStaticBaseSelfMask();results=[];pairs=[]
    for c in (25,75,125,175):
        old=dict(np.load(D/'partial_observations'/f'frame_{c-5:03d}.npz'))
        raw=dict(np.load(D/'source_predictions'/f'frame_{c:03d}.npz'))
        pose=dict(np.load(D/'known_poses'/f'frame_{c:03d}.npz'))
        history=dict(np.load(D/'history'/f'frame_{c:03d}.npz'))
        original=dict(np.load(D/'sensor_packets'/f'source_{c:03d}.npz'))
        q=joints_from_proprio(history['proprio_history']);masks=cad.project(q)
        prediction=dict(depth_m=raw['depth_m'],confidence=raw['confidence'],disparity_native_px=restore_native(raw['disparity_px']),frame=int(raw['frame']),time_s=float(raw['time_s']))
        sparse=point_packet(prediction,pose,masks['left'],masks['right'])
        for key in ['world','sigma_m','query_pixel_id','partner_pixel_id']:
            if not np.array_equal(sparse[key],original[key]):raise ValueError('current sparse source reproduction mismatch: '+key)
        dense=dense_point_packet(prediction,pose,masks['left'],masks['right'])
        indices=np.searchsorted(dense['query_pixel_id'],sparse['query_pixel_id'])
        for key in ['world','sigma_m','query_pixel_id','partner_pixel_id']:
            if not np.array_equal(dense[key][indices],sparse[key]):raise ValueError('sparse source subset changed: '+key)
        rows=[ray_evidence(old_point=point,old_sigma=sigma,past_time_s=float(old['acquisition_time_s']),current_time_s=float(dense['time_s']),current_pose=pose['left_camera_usd_world'],pose_time_s=float(pose['time_s']),current_packet=dense) for point,sigma in zip(old['world'][:64],old['sigma_m'][:64])]
        results.extend(dict(past_frame=c-5,current_frame=c,source_index=i,**r) for i,r in enumerate(rows))
        pairs.append(dict(past_frame=c-5,current_frame=c,points=len(rows),sparse_endpoints=len(sparse['world']),dense_endpoints=len(dense['world']),sparse_reproduction_exact=True,sparse_subset_exact=True,current_measured_joints=q.tolist(),mask_sha256={k:hashlib.sha256(v.tobytes()).hexdigest() for k,v in masks.items()},counts=dict(collections.Counter(r['status'] for r in rows))))
    output.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),identity_sha256=hashlib.sha256((H/'dense_current_stereo_ray_interface_identity_v1.json').read_bytes()).hexdigest(),pairs=pairs,counts=dict(collections.Counter(r['status'] for r in results)),results=results,known_pose_privileged=True,new_CAD_projections=4,new_teacher=0,new_fit=0,new_physics=0,no_memory_clearing=True,future_availability_budget_stays_closed=True,whole_mission_complete=False),indent=2)+'\n')
    print(pairs)
