"""Whole historical clean sequences; all labels are scoring-only."""
import json
from pathlib import Path
import numpy as np
from geometry import MultiViewGeometry,FOCAL,CX,CY
import sys
sys.path.append(str(Path(__file__).parent.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from control import observed_obstacles,front_blocked

HERE=Path(__file__).parent
OLD=HERE.parent/'bilateral_temporal_geometry_navigation_20261003_v1/clean_render_v2'


def main():
    with np.load(HERE/'base_link_masks.npz') as f:masks={k:f[k].copy() for k in ('left','right')}
    output={}
    for label in ('e2','e1'):
        root=OLD/f'candidate_{label}';model=MultiViewGeometry(masks=masks)
        errors=[];scene_errors=[];origins=dict(near_body=0,floor=0,body_height=0,other=0);eye_counts={k:0 for k in masks}
        rows=[];cache=[];masked_scene=masked_known=0;quality=[]
        for i,path in enumerate(sorted((root/'online_inputs').glob('frame_*.npz'))):
            with np.load(path) as f:
                xy=f['position_world_xy'];q=f['quaternion_world_wxyz_history'][-1];yaw=float(f['yaw_world_rad'])
                goal=xy+np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])@f['pointgoal_base_xy']
                result=model.update(f['left_rgb'],f['right_rgb'],xy,q,i/10)
            fresh=[]
            with np.load(root/'offline_truth'/path.name) as truth:
                for eye in masks:
                    pred=result[eye];eye_counts[eye]+=len(pred['depth_m']);uv=pred['uv'];coords=np.rint(uv).astype(int)
                    matrix=truth[f'{eye}_camera_usd_world'];rotation=np.c_[matrix[:3,0],-matrix[:3,1],-matrix[:3,2]]
                    z=truth[f'{eye}_depth_m'][coords[:,1],coords[:,0]];valid=np.isfinite(z)&(z>0)&(z<=10)
                    err=np.where(valid&(z>=.45),np.abs(pred['depth_m']-z),np.inf);errors.extend(err.tolist())
                    rays=np.c_[(uv[:,0]-CX)/FOCAL,(uv[:,1]-CY)/FOCAL,np.ones(len(uv))]
                    world=matrix[:3,3]+(rays*np.where(valid,z,0)[:,None])@rotation.T
                    near=valid&(np.linalg.norm(world-truth['root_position_world_m'],axis=1)<=.55)
                    scene_errors.extend(err[valid&(z>=.45)&~near].tolist())
                    floor=valid&~near&(abs(world[:,2])<=.12)
                    body=valid&~near&(world[:,2]>=.25)&(world[:,2]<=1.5)
                    pt=pred['points_world'];picked=(pt[:,2]>=.25)&(pt[:,2]<=1.5)&(np.linalg.norm(pt-np.r_[xy,.4],axis=1)>.55)
                    for name,mask in [('near_body',near),('floor',floor),('body_height',body),('other',~(near|floor|body))]:origins[name]+=int((picked&mask).sum())
                    fresh.append(observed_obstacles(pt,xy))
                    yy,xx=np.mgrid[0:480:4,0:640:4]
                    depth=truth[f'{eye}_depth_m'][yy,xx];good=np.isfinite(depth)&(depth>0)&(depth<=10)
                    ray=np.stack(((xx-CX)/FOCAL,(yy-CY)/FOCAL,np.ones_like(xx)),axis=-1)
                    w=matrix[:3,3]+(ray*np.where(good,depth,0)[:,:,None])@rotation.T
                    own=good&(np.linalg.norm(w-truth['root_position_world_m'],axis=2)<=.55)
                    mask=masks[eye][yy,xx];known=(mask&good).sum();wrong=(mask&good&~own).sum()
                    masked_known+=int(known);masked_scene+=int(wrong)
                    quality.append(dict(frame=i,eye=eye,mask_known_pixels=int(known),mask_non_near_pixels=int(wrong)))
            cache=[(t,points) for t,points in cache if t>=i/10-3]
            cache.append((i/10,np.vstack(fresh)));cells=np.unique(np.vstack([x[1] for x in cache]),axis=0)
            rows.append(dict(block=i,time_s=i/10,central_blocked=front_blocked(cells,xy,goal),cells=len(cells)))
        def stats(values):
            values=np.asarray(values);p90=np.sort(values)[int(np.ceil(.9*len(values)))-1] if len(values) else np.inf
            return dict(n=len(values),fraction_le_0_5_m=float(np.mean(values<=.5)) if len(values) else None,
              p90_m=float(p90) if np.isfinite(p90) else None,p90_unmeasurable=bool(len(values) and not np.isfinite(p90)))
        event=next((r['time_s'] for j,r in enumerate(rows) if j and r['central_blocked'] and rows[j-1]['central_blocked']),None)
        prefix=[r for r in rows if r['time_s']<2.1] if label=='e2' else rows
        output[label]=dict(frames=len(rows),all_accepted=stats(errors),scene=stats(scene_errors),eyes=eye_counts,
            inferred_obstacle_origins=origins,mask_known_pixel_observations=masked_known,mask_scene_pixel_observations=masked_scene,
            mask_non_near_fraction=float(masked_scene/max(masked_known,1)),first_two_frame_event_s=event,
            blocked_prefix_fraction=float(np.mean([r['central_blocked'] for r in prefix])),rows=rows,mask_quality=quality)
        print(label,{k:v for k,v in output[label].items() if k not in ('rows','mask_quality')},flush=True)
    gate=dict(support=all(v['scene']['n']>=30 for v in output.values()),
      accuracy=all(v['all_accepted']['fraction_le_0_5_m'] is not None and v['all_accepted']['fraction_le_0_5_m']>=.8
       and v['all_accepted']['p90_m'] is not None and v['all_accepted']['p90_m']<=1 for v in output.values()),
      e1_event=output['e1']['first_two_frame_event_s'] is not None and output['e1']['first_two_frame_event_s']<4,
      e2_false_blocking=output['e2']['blocked_prefix_fraction']<=.1)
    (HERE/'mask_regression.json').write_text(json.dumps(dict(results=output,gate=gate,accepted=all(gate.values()),
        historical_development_not_fresh=True),indent=2)+'\n')
    print('gate',gate,'accepted',all(gate.values()),flush=True)


if __name__=='__main__':main()
