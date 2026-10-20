"""Offline known-plane causal check. Truth never becomes a stereo/model input."""
from pathlib import Path
import json,hashlib
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
Y,X=np.mgrid[0:480:4,0:640:4]
RAYS=np.stack(((X-319.5)/584.8888888889,(Y-239.5)/584.8888888889,np.ones(X.shape)),axis=-1)
OPTICAL=np.array([[0,0,1],[0,-1,0],[1,0,0]])
MASK=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')['left'][::4,::4]

def main():
    allresults={}
    for folder,types in [('motion_acquisition_probe',('cached','direct')),('motion_kinematic_sync_probe',('cached','synced'))]:
        root=H/folder;phys=np.load(root/'physics.npz');t=phys['simulation_time_s'];pos=phys['root_position_world_m'];q=phys['root_quaternion_world_wxyz'];rotation=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix();rows=[]
        for path in sorted((root/'frames').glob('*.npz')):
            z=np.load(path);now=float(z['time_s']);pose=z['left_pose'];row=dict(frame=path.name,time_s=now,measurements={});selected={};current_points={}
            for typ in types:
                d=z['left_'+typ+'_depth'][::4,::4]
                with np.errstate(invalid='ignore'):
                    points=pose[:3,3]+(RAYS*d[...,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
                valid=np.isfinite(d)&(d>=1)&(d<=8)&~MASK&(points[...,2]>-.3)&(points[...,2]<.3)&(np.linalg.norm(points-z['root_position'],axis=-1)>=.55)
                selected[typ]=valid;current_points[typ]=points
                measurements=[]
                for lag in (0,.1):
                    index=int(np.argmin(abs(t-(now-lag))))
                    with np.errstate(invalid='ignore'):
                        world=pos[index]+rotation[index]@np.array([.2,.3,.05])+(RAYS*d[...,None])@(rotation[index]@OPTICAL).T
                    residual=np.abs(world[valid,2])
                    measurements.append(dict(lag_s=lag,n=len(residual),z_absolute_median_m=float(np.median(residual)),z_absolute_p90_m=float(np.percentile(residual,90))))
                row['measurements'][typ]=measurements
            common=selected[types[0]]&selected[types[1]]
            row['common_pixel_current_pose_check']=dict(n=int(common.sum()),p90_abs_z_m={typ:float(np.percentile(abs(current_points[typ][common,2]),90)) for typ in types})
            rows.append(row)
        original=np.load(H/'observations/forward/physics.npz')
        keys=('raw_command_target','smoothed_target','applied_action','root_position_world_m','root_quaternion_world_wxyz','joint_position_rad','joint_velocity_rad_s','simulation_time_s')
        identity={k:bool(np.array_equal(original[k],phys[k],equal_nan=True)) for k in keys}
        passed=types[-1]=='synced' and all(r['measurements']['synced'][0]['z_absolute_p90_m']<=1e-5 for r in rows)
        result=dict(role='One actual complete12s open-plane acquisition diagnostic; not fresh stereo/network validation',pose_lags_are_offline_geometry_comparison=True,selection='depth1..8m, valid self-mask, current reconstructed z in[-.3,.3], distance to body>=.55',captures=len(json.loads((root/'records.json').read_text())),physical_identity=identity,physical_unchanged=all(identity.values()),selected_geometry_consistent=passed,scope='nine selected timestamps in this known open plane; not proof for arbitrary scenes/sensors',rows=rows,source_sha256={str(Path(__file__)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
        (root/'assessment_v2.json').write_text(json.dumps(result,indent=2)+'\n');allresults[folder]=result
    result=allresults['motion_kinematic_sync_probe'];rows=result['rows'];time=[r['time_s'] for r in rows]
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for typ,label in [('cached','Render-only barrier'),('synced','FK refresh + render barrier')]:
        axes[0].semilogy(time,[r['measurements'][typ][0]['z_absolute_p90_m'] for r in rows],'o-',label=label)
    axes[0].set_xlabel('Actual time (s)');axes[0].set_ylabel('Ground reprojection |z| p90 (m)');axes[0].set_title('Same current pose and physical state');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2)
    z=np.load(H/'motion_kinematic_sync_probe/frames/frame_004.npz');axes[1].imshow(np.concatenate((z['left_cached_rgb'],z['left_synced_rgb']),axis=1));axes[1].set_title('Same 0.4s: before FK refresh | after FK refresh');axes[1].axis('off')
    fig.tight_layout();fig.savefig(H/'figures/motion_acquisition_causal_check.png',dpi=160)
    print({k:(v['selected_geometry_consistent'],v['physical_unchanged']) for k,v in allresults.items()})
if __name__=='__main__':main()
