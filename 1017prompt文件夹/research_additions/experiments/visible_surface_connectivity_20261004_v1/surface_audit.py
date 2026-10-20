"""Causal packet reconstruction and independent current-view support audit."""
import json
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt,label
from local_map import BeliefGrid
from surface_support import native_ground_mesh
H=Path(__file__).resolve().parent
OLD=H.parent/'visual_information_new_rig_20261004_v2'
F=16.45/18*640
MASK=np.load(H/'base_link_masks.npz')

def meshes_from_truth(f,stride=4):
    y,x=np.mgrid[0:480:stride,0:640:stride]
    rays=np.c_[(x.ravel()-319.5)/F,(y.ravel()-239.5)/F,np.ones(x.size)]
    meshes=[]
    for eye in ('left','right'):
        d=f[eye+'_depth_m'][::stride,::stride].ravel();pose=f[eye+'_camera_usd_world'];rot=np.c_[pose[:3,0],-pose[:3,1],-pose[:3,2]]
        valid=np.isfinite(d)&(d>=.45)&(d<=8)&~MASK[eye][::stride,::stride].ravel()
        pts=np.full((len(d),3),np.nan);pts[valid]=pose[:3,3]+(rays[valid]*d[valid,None])@rot.T
        meshes.append(pts.reshape(y.shape+(3,)))
    return meshes

def belief(f,representation,stride=4):
    root=f['root_position_world_m'];b=BeliefGrid(root[:2]-20)
    meshes=meshes_from_truth(f,stride);pts=np.concatenate([m.reshape(-1,3) for m in meshes])
    ground=[native_ground_mesh(f[eye+'_depth_m'],f[eye+'_camera_usd_world'],MASK[eye],root) for eye in ('left','right')] if representation=='surface' else ()
    b.update_points(pts,root,0.,ground_meshes=ground)
    b.seed_body(root[:2]);return b,meshes

def geometry(b,goal):
    clear=distance_transform_edt(np.pad(b.labels==0,1))[1:-1,1:-1]*b.resolution
    safe=(b.labels==0)&(clear>=.6-1e-8);safe[200,200]=True
    cc,_=label(safe,np.ones((3,3)));reachable=cc==cc[200,200]
    cells=np.argwhere(reachable);dist=np.linalg.norm(cells-[200,200],axis=1)*b.resolution
    route,reason=b.plan(b.origin+np.array([200,200])*.1,goal)
    return dict(free_cells=int((b.labels==0).sum()),occupied_cells=int((b.labels==1).sum()),start_clearance_m=float(clear[200,200]),reachable_safe_cells=int(len(cells)),max_reachable_radius_m=float(dist.max()),route_reason=reason,route_exists=route is not None)

def current_visibility(f,points):
    """Volume probe only: line of sight never converted into free floor cells."""
    views=[]
    for eye in ('left','right'):
        pose=f[eye+'_camera_usd_world'];rot=np.c_[pose[:3,0],-pose[:3,1],-pose[:3,2]]
        optical=(points-pose[:3,3])@rot
        z=optical[:,2];uv=optical[:,:2]/np.maximum(z[:,None],1e-12)*F+np.array([319.5,239.5])
        inside=(z>0)&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
        indices=np.rint(np.clip(uv,[0,0],[639,479])).astype(int)
        d=f[eye+'_depth_m'][indices[:,1],indices[:,0]]
        mask=MASK[eye][indices[:,1],indices[:,0]]
        valid=inside&~mask&np.isfinite(d)&(d>=.45)&(d<=8)
        los=valid&(z<=d+.01)
        views.append(dict(inside=inside,unmasked=inside&~mask,valid=valid,los=los))
    result={}
    for k in views[0]:
        a=views[0][k]|views[1][k];result[k+'_count']=int(a.sum());result[k+'_fraction']=float(a.mean())
    return result

def audit_case(case):
    f=np.load(OLD/'runs/visible'/case/'offline_truth/frame_000.npz')
    inp=np.load(OLD/'runs/visible'/case/'online_inputs/frame_000.npz');root=f['root_position_world_m'];yaw=float(inp['yaw_world_rad']);rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    goal=root[:2]+rot@inp['pointgoal_base_xy'];out={'representations':{}}
    for representation,stride in [('points',4),('dense_points',1),('surface',4)]:
        b,meshes=belief(f,'points' if representation=='dense_points' else representation,stride)
        out['representations'][representation]=geometry(b,goal)
        np.savez_compressed(H/f'initial_belief_{case}_{representation}.npz',labels=b.labels,origin=b.origin,resolution=b.resolution)
        if representation=='surface':
            pts=np.concatenate([m.reshape(-1,3) for m in meshes]);valid=np.isfinite(pts).all(1)&(abs(pts[:,2])<=.05)&(np.linalg.norm(pts-root,axis=1)>=.55);p=pts[valid];local=(p[:,:2]-root[:2])@rot;band=(local[:,0]>0)&(abs(local[:,1])<=.6)
            out['nearest_observed_floor_forward_in_1p2m_band_m']=float(local[band,0].min()) if band.any() else None
            out['near_field_unknown_witness_world_xy']=(b.origin+np.array([200,200])*.1+rot@np.array([.5,0.])).tolist()
            witness=b.cells(np.array(out['near_field_unknown_witness_world_xy'])[None])[0];out['witness_label']=int(b.labels[tuple(witness)])
            center=b.origin+np.array([200,200])*.1;ii,jj=np.indices(b.labels.shape);world=b.origin+np.stack((ii,jj),axis=-1)*.1;delta=(world-center)@rot
            tube=(delta[...,0]>=-.6)&(delta[...,0]<=1.6)&(abs(delta[...,1])<=.6)
            out['representations'][representation]['first_meter_body_tube_cells']=int(tube.sum())
            out['representations'][representation]['tube_known_free_fraction']=float((b.labels[tube]==0).mean())
    # Current line-of-sight at low/center/top height; formal volume certification not claimed.
    grid=np.array([(s,l) for s in np.arange(.4,1.11,.1) for l in np.arange(-.6,.61,.1)])
    xy=root[:2]+grid@rot.T
    out['visibility_probes']={str(z):current_visibility(f,np.c_[xy,np.full(len(xy),z)]) for z in (.05,.4,.8)}
    out['visibility_probe_points_per_height']=len(xy)
    return out

if __name__=='__main__':
    result={k:audit_case(k) for k in ('e0','e2','h3','h4')}
    (H/'initial_support_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
