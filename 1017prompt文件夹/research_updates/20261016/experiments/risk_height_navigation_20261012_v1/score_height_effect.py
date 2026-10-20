"""Same-frame renderer labels score source qualification only, never feed navigation."""
from pathlib import Path
import json,numpy as np
D=Path(__file__).resolve().parent;R=D/'candidate_e2_run'
c=dict(before_points=0,after_points=0,truth_valid=0,before_floor=0,after_floor=0,before_obstacle=0,after_obstacle=0)
for p in sorted((R/'baseline_partial_observations').glob('*.npz')):
 b=np.load(p);a=np.load(R/'partial_observations'/p.name);f=np.load(R/'frames'/p.name);q=b['query_pixel_id'];xx=q%640;yy=q//640;z=f['left_depth_m'][yy,xx];valid=np.isfinite(z)&(z>0);rays=np.column_stack([(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]);T=f['left_camera_usd_world'];world=T[:3,3]+(rays*np.where(valid,z,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 floor=valid&(abs(world[:,2])<=.03);obstacle=valid&(world[:,2]>.05)&(world[:,2]<=1.5);keep=np.isin(q,a['query_pixel_id'])
 c['before_points']+=len(q);c['after_points']+=len(a['world']);c['truth_valid']+=int(valid.sum())
 for k,mask in [('floor',floor),('obstacle',obstacle)]:c['before_'+k]+=int(mask.sum());c['after_'+k]+=int((mask&keep).sum())
x=dict(counts=c,scope='Same actual candidate source packets with unchanged baseline shadow; renderer same-frame only offline scoring. Ground bandabsZ<=.03 scoring tolerance. Counts correlated across frames/pixels, not independent observations; uncertainty not calibrated; source effect does not establish navigation causality.')
(D/'height_effect_assessment.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps(x))
