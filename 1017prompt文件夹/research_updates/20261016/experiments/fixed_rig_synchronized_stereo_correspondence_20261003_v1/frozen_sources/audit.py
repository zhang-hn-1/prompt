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
for label in ('e0','e1','e2','hard_0_e1_0003','hard_0_e1_0004'):
    root=DATA/label;records=[];errors=[];raw_errors=[];all_errors=[];consecutive=0;first=None;totals={};output=H/'replay'/label;output.mkdir(parents=True,exist_ok=True)
    for i,path in enumerate(sorted((root/'online_inputs').glob('frame_*.npz'))):
        with np.load(path) as f,np.load(root/'offline_truth'/path.name) as truth:
            xy=f['position_world_xy'];q=f['quaternion_world_wxyz_history'][-1];yaw=float(f['yaw_world_rad']);estimate=stereo_estimate(f['left_rgb'],f['right_rgb'],masks)
            eyes={};front=[];save={}
            for eye,item in estimate.items():
                candidate_depth=item['depth'][::4,::4].reshape(-1);accepted=item['valid'][::4,::4].reshape(-1);raw=item['raw_valid'][::4,::4].reshape(-1)&~masks[eye][::4,::4].reshape(-1)
                center,rotation=camera_pose(xy,q,eye);predicted=center+(rays*np.where(np.isfinite(candidate_depth),candidate_depth,0)[:,None])@rotation.T
                actual,true_valid,_=actual_points(truth,eye);error=np.linalg.norm(predicted-actual,axis=1);error[~true_valid]=np.inf
                body=body_coordinates(predicted,xy,yaw);band=(predicted[:,2]>=.25)&(predicted[:,2]<=1.5)&(np.linalg.norm(predicted-np.r_[xy,.4],axis=1)>=.55)
                frontal=accepted&band&(body[:,0]>=.55)&(body[:,0]<=3.5)&(np.abs(body[:,1])<=.6)
                errors.extend(error[accepted&band].tolist());raw_errors.extend(error[raw&band].tolist());all_errors.extend(error[accepted].tolist());front.append(body[frontal]);ov=overlap(truth,eye)
                if eye not in totals:totals[eye]={a:{b:0 for b in v} for a,v in ov.items()}
                for region,counts in ov.items():
                    for key,value in counts.items():totals[eye][region][key]+=value
                eyes[eye]={'raw_sampled_valid':int(raw.sum()),'accepted_sampled':int(accepted.sum()),'accepted_bodyband':int((accepted&band).sum()),'front_points':int(frontal.sum()),'overlap':ov}
                save[eye+'_uv']=uv[accepted];save[eye+'_points_world']=predicted[accepted];save[eye+'_truth_error_m']=error[accepted];save[eye+'_bodyband']=band[accepted]
            cells=np.unique(np.floor(np.concatenate(front)/.1).astype(int),axis=0) if any(len(a) for a in front) else np.empty((0,2),int)
            consecutive=consecutive+1 if len(cells)>=6 else 0
            if consecutive>=2 and first is None:first=i*.1
            records.append({'time_s':i*.1,'front_cells':len(cells),'front_event':consecutive>=2,'eyes':eyes});np.savez_compressed(output/path.name,**save)
    summary[label]={'frames':len(records),'raw_bodyband_error':error_stats(raw_errors),'validated_bodyband_error':error_stats(errors),'all_validated_error':error_stats(all_errors),'first_front_event_s':first,'max_front_cells':max(r['front_cells'] for r in records),'overlap_totals':totals,'records':records}
    (H/'audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(label,{key:value for key,value in summary[label].items() if key not in ('records','overlap_totals')},flush=True)
