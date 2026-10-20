"""Classify retained RGB geometry using offline renderer measurements only."""
import json
from pathlib import Path
import numpy as np
from geometry import MultiViewGeometry
from control import observed_obstacles

HERE=Path(__file__).resolve().parent
PILOT=HERE.parent/'fixed_rig_visual_pilot_20261001'


def main():
    results={}
    for label,folder in [('e0','integrated_fill_e0b'),('e1','integrated_fill_e1'),('e2','integrated_fill_e2')]:
        model=MultiViewGeometry();counts={eye:{k:0 for k in ('predictions','truth_near_self','truth_floor','truth_body_height_surface','truth_other_or_unknown',
                         'predicted_body_height_obstacle','obstacle_from_true_self','obstacle_from_true_floor','obstacle_from_true_body_height','obstacle_from_other_or_unknown')}
                         for eye in ('left','right')}
        for path in sorted((PILOT/folder/'depth_truth').glob('frame_*.npz')):
            with np.load(path) as f:
                root=f['root_position_world_m'];q=f['root_quaternion_world_wxyz'];t=float(f['simulation_time_s'])
                outputs=model.update(f['left_rgb'],f['right_rgb'],root[:2],q,t)
                for eye,p in outputs.items():
                    uv=np.rint(p['uv']).astype(int);n=len(uv)
                    truth=f[f'{eye}_depth_m'][uv[:,1],uv[:,0]]
                    valid=np.isfinite(truth)&(truth>0)&(truth<=10)
                    matrix=f[f'{eye}_camera_usd_world'];R=np.column_stack((matrix[:3,0],-matrix[:3,1],-matrix[:3,2]))
                    ray=np.column_stack(((p['uv'][:,0]-319.5)/(16.45/18*640),
                                         (p['uv'][:,1]-239.5)/(16.45/18*640),np.ones(n)))
                    world=matrix[:3,3]+(ray*np.where(valid,truth,0)[:,None])@R.T
                    own=valid&(np.linalg.norm(world-root,axis=1)<=.55)
                    floor=valid&~own&(abs(world[:,2])<=.12)
                    obstacle=valid&~own&(world[:,2]>=.25)&(world[:,2]<=1.5)
                    other=~(own|floor|obstacle)
                    predicted=p['points_world'];picked=(predicted[:,2]>=.25)&(predicted[:,2]<=1.5)
                    picked&=np.linalg.norm(predicted-np.r_[root[:2],.4],axis=1)>.55
                    c=counts[eye];c['predictions']+=n;c['predicted_body_height_obstacle']+=int(picked.sum())
                    for category,mask in [('self',own),('floor',floor),('body_height',obstacle),('other_or_unknown',other)]:
                        key={'self':'truth_near_self','floor':'truth_floor','body_height':'truth_body_height_surface','other_or_unknown':'truth_other_or_unknown'}[category]
                        c[key]+=int(mask.sum());c[f'obstacle_from_true_{category}' if category!='other_or_unknown' else 'obstacle_from_other_or_unknown']+=int((picked&mask).sum())
        results[label]=counts
        print(label,counts,flush=True)
    (HERE/'error_origin_audit.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
