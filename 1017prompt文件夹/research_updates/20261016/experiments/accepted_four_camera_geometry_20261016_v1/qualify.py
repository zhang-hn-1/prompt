"""All eight saved exposures: CAD eligibility and GT-only visible BEV geometry.

No model, fitting, new physics, online truth, or free-space certification.
"""
from pathlib import Path
import hashlib, json, sys, time
import numpy as np
from scipy.spatial import cKDTree

D = Path(__file__).resolve().parent
W = D.parents[1]
INPUT = W / 'experiments/user_fov_position_only_20261015_v1'
sys.path[:0] = [str(INPUT), str(W/'experiments/stereo_memory_navigation_20261004_v1')]
from four_camera import NAMES, validate_complete
from dynamic_self_mask import RobotSelfMask
from self_mask import mask_triangles

sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
AX = np.diag([1., -1., -1.])

def triangles_body(cad, q):
    bodies = cad.body_transforms(q)
    return np.concatenate([v @ bodies[n][:3,:3].T + bodies[n][:3,3]
                           for n,v in cad.triangles.items()])

def main():
    start = time.monotonic()
    cfg = json.loads((D/'protocol.json').read_text())
    source_cfg = json.loads((INPUT/'protocol.json').read_text())
    R = INPUT/'recovery_run'
    seal = validate_complete(json.loads((INPUT/'recovery_run_status.json').read_text()), R)
    cad = RobotSelfMask()
    local = [np.array(v['local_matrix']) for v in seal['initial_local_identity']]
    focal_mm, ap_w, ap_h, near, far = seal['initial_local_identity'][0]['optics']
    F = focal_mm/ap_w*640
    for eye in seal['initial_local_identity']:
        assert np.allclose(eye['optics'], [focal_mm,ap_w,ap_h,near,far], atol=1e-7)
    assert abs(F-source_cfg['focal_px']) < 1e-4
    occ = np.load(source_cfg['occupancy'])
    origin = occ['global_origin_xy']; resolution = float(occ['resolution_m'])
    assert abs(resolution-.1)<1e-7 and set(np.unique(occ['grid'])) <= {0,1}
    reference = np.argwhere(occ['grid']>0)*resolution+origin
    tree = cKDTree(reference)
    y,x = np.indices((480,640)); uv = np.stack([x,y],-1)
    rays = np.stack([(x-319.5)/F, (y-239.5)/F, np.ones_like(x)],-1)
    cell = lambda w: np.unique(np.floor((w[:,:2]-origin)/.1+.5).astype(np.int64), axis=0)
    aset = lambda c: set(map(tuple,c.tolist()))
    array = lambda c: np.array(sorted(c),np.int64).reshape(-1,2)
    theta = np.arange(3600)*2*np.pi/3600
    ring_body = np.c_[2*np.cos(theta),2*np.sin(theta),np.full(3600,.05)]
    rows=[]; max_inverse=0.; max_relative=0.; all_dist=[]
    inputs={str(p):sha(p) for p in [INPUT/'protocol.json', INPUT/'four_camera.py',
            INPUT/'recovery_run_status.json', R/'capture_seal.json', Path(source_cfg['occupancy'])]}
    for rr in seal['rows']:
        path=R/'frames'/rr['frame'];inputs[str(path)]=sha(path)
        assert inputs[str(path)]==rr['sha256']
        with np.load(path) as a:
            tri = triangles_body(cad,a['joints_current'])
            masks={};sets=[];arms=[];body_matrices=[];save={};raw_ring=[];masked_ring=[]
            for j,name in enumerate(NAMES):
                T=a[name+'_camera_usd_world_label'];B=T@np.linalg.inv(local[j]);body_matrices.append(B)
                optical=(tri-local[j][:3,3])@(local[j][:3,:3]@AX)
                mask=mask_triangles(optical,640,480,F,319.5,239.5);masks[name]=mask
                z=a[name+'_depth_label'];finite=np.isfinite(z)&(z>0)
                domain=finite&(z>=.5)&(z<=3);eligible=domain&~mask
                world=T[:3,3]+(rays[eligible]*z[eligible,None])@(T[:3,:3]@AX).T
                inverse=(world-T[:3,3])@(T[:3,:3]@AX)
                roundtrip=inverse[:,:2]/inverse[:,2,None]*F+[319.5,239.5]
                max_inverse=max(max_inverse,float(np.max(abs(roundtrip-uv[eligible]))) if len(world) else 0.)
                outside=np.linalg.norm(world-a['root_position_physics_label'],axis=1)>=.55
                world=world[outside]
                band=(world[:,2]>.05)&(world[:,2]<=1.5);cells=cell(world[band]);sets.append(aset(cells))
                distance=tree.query(cells*.1+origin)[0] if len(cells) else np.array([])
                arms.append(dict(name=name,all_pixels=z.size,CAD_pixels=int(mask.sum()),
                    finite_pixels=int(finite.sum()),range_pixels=int(domain.sum()),
                    range_nonCAD_pixels=int(eligible.sum()),root_excluded_points=int((~outside).sum()),
                    bodyband_points=int(band.sum()),
                    ground_height_band_points=int(((world[:,2]>=-.05)&(world[:,2]<=.05)).sum()),
                    visible_bodyband_cells=len(cells),near_reference_cells=int((distance<=.15).sum()),
                    far_reference_cells=int((distance>.15).sum()),
                    reference_distance_p90_m=float(np.quantile(distance,.9)) if len(cells) else None))
                save[name+'_CAD']=mask;save[name+'_cells']=cells
                # Ring has exact body coordinates, with current calibrated camera and CAD only.
                world_ring=ring_body@B[:3,:3].T+B[:3,3]
                c=(world_ring-T[:3,3])@(T[:3,:3]@AX)
                u=F*c[:,0]/np.maximum(c[:,2],1e-20)+319.5
                v=F*c[:,1]/np.maximum(c[:,2],1e-20)+239.5
                k=(c[:,2]>=.5)&(c[:,2]<=3)&(u>=-.5)&(u<639.5)&(v>=-.5)&(v<479.5)
                raw_ring.append(k.copy());ids=np.flatnonzero(k)
                k[ids]&=~mask[np.clip(np.rint(v[ids]).astype(int),0,479),np.clip(np.rint(u[ids]).astype(int),0,639)]
                masked_ring.append(k)
            difference=max(float(np.max(abs(B-body_matrices[0]))) for B in body_matrices)
            max_relative=max(max_relative,difference)
            front=sets[0]|sets[1];rear=sets[2]|sets[3];union=front|rear
            dist=tree.query(array(union)*.1+origin)[0] if union else np.array([]);all_dist.extend(dist.tolist())
            raw=np.any(raw_ring,axis=0);vis=np.any(masked_ring,axis=0)
            row=dict(frame=rr['frame'],time_s=float(a['source_time_s']),arms=arms,
                front_union=len(front),rear_union=len(rear),overlap=len(front&rear),all_union=len(union),
                rear_new=len(rear-front),union_near_reference=int((dist<=.15).sum()),
                union_far_reference=int((dist>.15).sum()),body_transform_agreement=difference,
                ring_FOV_range=int(raw.sum()),ring_after_CAD=int(vis.sum()),ring_total=3600)
            rows.append(row)
            save.update(front_union=array(front),rear_union=array(rear),all_union=array(union),
                origin=origin,reference_xy=reference,root=a['root_position_physics_label'],
                ring_body=ring_body,ring_raw=raw,ring_visible=vis)
            np.savez_compressed(D/(Path(rr['frame']).stem+'_audit.npz'),**save)
            print(rr['frame'],[arm['CAD_pixels'] for arm in arms],len(union),int(vis.sum()),flush=True)
    keys=['front_union','rear_union','overlap','all_union','rear_new','union_near_reference','union_far_reference']
    result=dict(complete=True,frames=rows,aggregate_grid_instances={k:sum(r[k] for r in rows) for k in keys},
        CAD_fraction_ranges={name:[min(r['arms'][i]['CAD_pixels'] for r in rows)/307200,
                                  max(r['arms'][i]['CAD_pixels'] for r in rows)/307200] for i,name in enumerate(NAMES)},
        union_reference_distance_p90_m=float(np.quantile(all_dist,.9)) if all_dist else None,
        maximum_inverse_pixel_error=max_inverse,maximum_body_matrix_difference=max_relative,
        wall_s=time.monotonic()-start,input_sha256=inputs,
        cad_asset_identity={k:cad.identity[k] for k in ['asset','source_layers_sha256']},
        cad_projection=dict(parts=list(cad.triangles),joint_names=list(cad.joint_names),
            transforms_source='Actual sealed initial_local_identity matrices',
            camera_names=list(NAMES),local_matrices=[T.tolist() for T in local],
            focal_px=F,principal_point=[319.5,239.5],resolution=[640,480],
            near_clip_m=.01,dilation_pixels=0,
            method='All visible CAD triangles from all four articulated bodies; exact current measured joints; conservative clipped raster silhouette'),
        physical=0,model=0,fit=0,navigation=0,
        scope='GT-only geometry over eight saved poses. Reference proximity is not model precision or scene recall. CAD mask is conservative raster silhouette. No predicted ground error or long-term map exists.',
        source_sha256={str(p):sha(p) for p in [Path(__file__),D/'protocol.json',
              W/'experiments/stereo_memory_navigation_20261004_v1/dynamic_self_mask.py',
              W/'experiments/fixed_rig_self_mask_and_curve_qualification_20261003_v1/self_mask.py']})
    (D/'geometry_result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(result['aggregate_grid_instances'],flush=True)

if __name__=='__main__': main()
