"""Offline check: does ignoring the conservative target-eye mask restore surfaces?"""
import json
import numpy as np
from measurement import H,DATA,actual_points,body_coordinates,project,masks

summary={}
roots=[(label,DATA/label) for label in ('e0','e1','e2','hard_0_e1_0003','hard_0_e1_0004')]+[('direct_e1',H/'direct_e1')]
for label,root in roots:
    counts={eye:{'source_front':0,'same_without_target_mask':0,'same_with_target_mask':0} for eye in ('left','right')}
    late={eye:{'source_front':0,'same_without_target_mask':0,'same_with_target_mask':0} for eye in ('left','right')}
    for path in sorted((root/'offline_truth').glob('frame_*.npz')):
        with np.load(path) as frame:
            root_position=frame['root_position_world_m'];w,x,y,z=frame['root_quaternion_world_wxyz']
            yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
            for eye in ('left','right'):
                other='right' if eye=='left' else 'left'
                points,valid,_=actual_points(frame,eye);body=body_coordinates(points,root_position[:2],yaw)
                front=valid&(np.linalg.norm(points-root_position,axis=1)>=.55)&~masks[eye][::4,::4].ravel()&(points[:,2]>=.25)&(points[:,2]<=1.5)&(body[:,0]>=.35)&(body[:,0]<=3.5)&(np.abs(body[:,1])<=.6)
                pose=frame[other+'_camera_usd_world'];rotation=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]))
                mapped,expected=project(points,(pose[:3,3],rotation))
                u,v=np.rint(np.nan_to_num(mapped,nan=-1,posinf=-1,neginf=-1)).astype(int).T
                inside=(expected>.45)&(u>=0)&(u<640)&(v>=0)&(v<480)
                u=np.clip(u,0,639);v=np.clip(v,0,479);observed=frame[other+'_depth_m'][v,u]
                same=front&inside&np.isfinite(observed)&(observed>.45)&(np.abs(observed-expected)<=np.maximum(.1,.05*expected))
                row={'source_front':int(front.sum()),'same_without_target_mask':int(same.sum()),'same_with_target_mask':int((same&~masks[other][v,u]).sum())}
                for key,value in row.items():
                    counts[eye][key]+=value
                    if float(frame['simulation_time_s'])>=4-1e-8:late[eye][key]+=value
    summary[label]={'counts':counts,'from_4s':late}
(H/'mask_sensitivity.json').write_text(json.dumps(summary,indent=2)+'\n')
print(summary)
