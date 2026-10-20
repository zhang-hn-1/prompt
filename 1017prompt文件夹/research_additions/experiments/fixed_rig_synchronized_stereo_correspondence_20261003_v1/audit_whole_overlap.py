"""Whole-sequence stereo observability and independent geometry validation."""
from pathlib import Path
import sys,json
import numpy as np,cv2
from correspondence import stereo_estimate,FOCAL_LENGTH_PX,BASELINE_M
H=Path(__file__).parent;E=H.parent;DATA=E/'clean_visual_hard0_intervention_20261003_v1/synchronized_v2'
sys.path.insert(0,str(E/'bilateral_temporal_geometry_navigation_20261003_v1'))
from geometry import camera_pose,project
masks=np.load(E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/base_link_masks.npz')
yy,xx=np.mgrid[0:480:4,0:640:4];uv=np.column_stack((xx.ravel(),yy.ravel())).astype(float);rays=np.column_stack(((uv[:,0]-319.5)/FOCAL_LENGTH_PX,(uv[:,1]-239.5)/FOCAL_LENGTH_PX,np.ones(len(uv))))
cv2.setNumThreads(2)

def actual_points(frame,eye):
    pose=frame[eye+'_camera_usd_world'];rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]));depth=frame[eye+'_depth_m'][::4,::4].reshape(-1)
    valid=np.isfinite(depth)&(depth>=.45)&(depth<=8)
    return pose[:3,3]+(rays*np.where(valid,depth,0)[:,None])@rotation.T,valid,depth

def body_coordinates(points,xy,yaw):
    c,s=np.cos(yaw),np.sin(yaw);return (points[:,:2]-xy)@np.array([[c,-s],[s,c]])

def overlap(frame,eye):
    other='right' if eye=='left' else 'left';points,valid,depth=actual_points(frame,eye);root=frame['root_position_world_m'];q=frame['root_quaternion_world_wxyz'];w,x,y,z=q;yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    body=body_coordinates(points,root[:2],yaw);scene=valid&(np.linalg.norm(points-root,axis=1)>=.55)&~masks[eye][::4,::4].reshape(-1)
    band=scene&(points[:,2]>=.25)&(points[:,2]<=1.5)&(body[:,0]>=.35)&(body[:,0]<=4.5)
    front=band&(np.abs(body[:,1])<=.6)&(body[:,0]<=3.5)
    other_pose=frame[other+'_camera_usd_world'];other_r=np.column_stack((other_pose[:3,0],-other_pose[:3,1],-other_pose[:3,2]));mapped,expected=project(points,(other_pose[:3,3],other_r));u=np.rint(np.nan_to_num(mapped[:,0],nan=-1,posinf=-1,neginf=-1)).astype(int);v=np.rint(np.nan_to_num(mapped[:,1],nan=-1,posinf=-1,neginf=-1)).astype(int)
    inside=(expected>.45)&(u>=0)&(u<640)&(v>=0)&(v<480);uclip=np.clip(u,0,639);vclip=np.clip(v,0,479)
    observed=frame[other+'_depth_m'][vclip,uclip];cad=inside&~masks[other][vclip,uclip]
    same=cad&np.isfinite(observed)&(observed>.45)&(np.abs(observed-expected)<=np.maximum(.1,.05*expected))
    return {name:{'source':int(chosen.sum()),'projected_inside':int((chosen&inside).sum()),'other_cad_visible':int((chosen&cad).sum()),'same_surface':int((chosen&same).sum())} for name,chosen in [('all_scene',scene),('bodyband',band),('front',front)]}


def error_stats(values):
    v=np.asarray(values);finite=v[np.isfinite(v)]
    return {'n':len(v),'invalid_truth_depth':int((~np.isfinite(v)).sum()),'within_0p3m':float(np.mean(v<=.3)) if len(v) else None,'finite_p50_m':float(np.median(finite)) if len(finite) else None,'finite_p90_m':float(np.percentile(finite,90)) if len(finite) else None}

summary={}
roots=[(label,DATA/label) for label in ('e0','e1','e2','hard_0_e1_0003','hard_0_e1_0004')]+[('direct_e1',H/'direct_e1')]
for label,root in roots:
    records=[];totals={};late={}
    for path in sorted((root/'offline_truth').glob('frame_*.npz')):
        with np.load(path) as frame:
            row={'time_s':float(frame['simulation_time_s']),'eyes':{}}
            for eye in ('left','right'):
                ov=overlap(frame,eye);row['eyes'][eye]=ov
                if eye not in totals:totals[eye]={a:{b:0 for b in v} for a,v in ov.items()}
                if eye not in late:late[eye]={a:{b:0 for b in v} for a,v in ov.items()}
                for region,counts in ov.items():
                    for key,value in counts.items():
                        totals[eye][region][key]+=value
                        if row['time_s']>=4-1e-8:late[eye][region][key]+=value
            records.append(row)
    summary[label]={'captures':len(records),'totals':totals,'last_part_from_4s':late,'records':records}
(H/'overlap_whole_sequences.json').write_text(json.dumps(summary,indent=2)+'\n')
print({label:{'captures':x['captures'],'totals':x['totals']} for label,x in summary.items()})
