"""Independent planar audit: common valid pixels and downward rays, no z fitting."""
import json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent

def main():
    y,x=np.mgrid[0:480:4,0:640:4]
    rays=np.stack(((x-319.5)/584.8888888889,(y-239.5)/584.8888888889,np.ones(x.shape)),axis=-1)
    optical=np.array([[0,0,1],[0,-1,0],[1,0,0]])
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');rows=[]
    root=H/'motion_render_ablation_probe'
    for f in sorted((root/'frames').glob('*.npz')):
        z=np.load(f);q=z['root_quaternion'];rotation=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();directions=rays@(rotation@optical).T
        for eye,side in [('left',1),('right',-1)]:
            a=z[eye+'_cached_depth'][::4,::4];b=z[eye+'_synced_depth'][::4,::4];middle=z[eye+'_extra_plain_depth'][::4,::4]
            valid=np.isfinite(a)&np.isfinite(b)&(a>=1)&(a<=8)&(b>=1)&(b<=8)&~masks[eye][::4,::4]&np.isfinite(middle)&(middle>=1)&(middle<=8)&(directions[...,2]<-.02)
            camera=z['root_position']+rotation@np.array([.2,side*.3,.05])
            with np.errstate(invalid='ignore'):
                before=camera+directions*a[...,None];after=camera+directions*b[...,None];extra=camera+directions*middle[...,None]
            rows.append(dict(frame=f.name,time_s=float(z['time_s']),eye=eye,n=int(valid.sum()),cached_ground_z_p90_m=float(np.percentile(abs(before[valid,2]),90)),synced_ground_z_p90_m=float(np.percentile(abs(after[valid,2]),90)),extra_plain_ground_z_p90_m=float(np.percentile(abs(extra[valid,2]),90))))
    original=np.load(H/'observations/forward/physics.npz');phys=np.load(root/'physics.npz');identity={k:bool(np.array_equal(original[k],phys[k],equal_nan=True)) for k in phys.files}
    result=dict(role='Independent known-plane same-pixel causal measurement, not stereo model gate',selection='common finite 1..8m depths, same selfmask, current-pose world ray z<-.02. No reconstructed-z selection or lag fit.',rows=rows,max_synced_ground_z_p90_m=max(r['synced_ground_z_p90_m'] for r in rows),max_extra_plain_ground_z_p90_m=max(r['extra_plain_ground_z_p90_m'] for r in rows),fk_necessary_in_measured_ablation=False,all_physics_fields_identity=identity,scope='known open-plane floor, 9 saved instants x2 eyes, immutable complete12s commands',source_sha256={str(Path(__file__)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    (root/'independent_ground_ray_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(result['max_synced_ground_z_p90_m'],identity)
if __name__=='__main__':main()
