"""Current depth visibility and temporal evidence; no policy reads these labels."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from spatial_memory import SpatialMemory
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent
OLD=H.parent/'visible_surface_connectivity_20261004_v1'
import sys
sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(OLD))
from local_map import BeliefGrid
from surface_support import native_ground_mesh

def main():
    masks=np.load(OLD/'base_link_masks.npz');results={}
    y,x=np.mgrid[0:480:2,0:640:2]
    rays=np.stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)),axis=-1)
    for name in ('zero','bank','forward','reverse'):
        root=H/'observations'/name;states=json.loads((root/'states.json').read_text())
        positions=np.array([r['position'] for r in states]);v=np.array([r['velocity'] for r in states]);t=np.array([r['time_s'] for r in states])
        q=np.array([r['quaternion'] for r in states]);rotation=Rotation.from_quat(q[:,[1,2,3,0]])
        euler=rotation.as_euler('xyz',degrees=True)
        bank=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),-1,1)))
        speed=np.linalg.norm(v[:,:2],axis=1);count=0;qualified=[]
        for i,s in enumerate(speed):
            count=count+1 if s<.05 else 0
            if count>=5:qualified.append(float(t[i]))
        memory=SpatialMemory(positions[0,:2]-20,size=400,resolution=.1)
        b=BeliefGrid(positions[0,:2]-20)
        frames=[]
        for p in sorted((root/'frames').glob('*.npz')):
            z=np.load(p);now=float(z['time_s']);pointlist=[];mesh=[]
            for eye in ('left','right'):
                d=z[eye+'_depth_m'][::2,::2];pose=z[eye+'_camera_usd_world'];rot=pose[:3,:3]@np.diag([1,-1,-1])
                points=pose[:3,3]+(rays*d[...,None])@rot.T
                valid=np.isfinite(d)&(d>=.45)&(d<=8)&~masks[eye][::2,::2]&(np.linalg.norm(points-z['root_position_world_m'],axis=-1)>=.55)
                pointlist.append(points[valid]);mesh.append(native_ground_mesh(z[eye+'_depth_m'],pose,masks[eye],z['root_position_world_m']))
            points=np.concatenate(pointlist);memory.update(points,now,0.)
            b.update_points(points,z['root_position_world_m'],now,ground_meshes=mesh);b.seed_body(z['root_position_world_m'][:2])
            pose_rotation=Rotation.from_quat(z['root_quaternion_world_wxyz'][[1,2,3,0]]).as_matrix()
            body=(points-z['root_position_world_m'])@pose_rotation
            floor=(np.abs(points[:,2])<=.05)&(np.abs(body[:,1])<=.6)&(body[:,0]>0)
            nearest=float(body[floor,0].min()) if floor.any() else None
            current=memory.labels(now,current_only=True);hist=memory.labels(now)
            witness=memory.status(z['root_position_world_m'][:2]+pose_rotation[:2,0]*.5,now)
            path,reason=b.plan(z['root_position_world_m'][:2],z['root_position_world_m'][:2]+pose_rotation[:2,0]*2.)
            frames.append(dict(time_s=now,nearest_observed_floor_forward_m=nearest,current_floor_cells=int((current==0).sum()),memory_floor_cells=int((hist==0).sum()),
                occupied_cells=int((hist==1).sum()),witness_0p5m_status=witness,route_reason=reason,route_exists=path is not None,
                known_floor_is_not_full_3d_clearance=True))
        after=t>=6.-1e-5;delta=np.linalg.norm(np.diff(positions[:,:2],axis=0),axis=1)
        post_stop=t>= (10 if name=='bank' else 9 if name in ('forward','reverse') else 6)-1e-5
        results[name]=dict(duration_s=float(t[-1]),captures=len(states),saved_frames=len(frames),path_m=float(delta.sum()),
            path_after_settle_m=float(delta[59:].sum()),net_displacement_after_settle_m=float(np.linalg.norm(positions[-1,:2]-positions[60,:2])),
            speed_peak_m_s=float(speed.max()),speed_final_m_s=float(speed[-1]),bank_range_deg=[float(bank.min()),float(bank.max())],
            euler_pitch_range_deg=[float(euler[:,1].min()),float(euler[:,1].max())],
            post_request_stop_path_m=float(delta[np.nonzero(post_stop)[0][0]:].sum()),
            stop_qualified_times_s=qualified,qualified_after_braking_s=next((v for v in qualified if v>=float(t[post_stop][0])+.4-1e-5),None),
            frames=frames,any_forward_route_after_observation=any(f['route_exists'] for f in frames if f['time_s']>=6),
            role='open-plane depth/pose privileged observability diagnostic')
    (H/'observation_assessment.json').write_text(json.dumps(results,indent=2)+'\n')
    for name,r in results.items():print(name,{k:v for k,v in r.items() if k not in ('frames','stop_qualified_times_s')},flush=True)
if __name__=='__main__':main()
