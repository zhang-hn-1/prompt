"""Post-hoc explanation only: does not modify frozen ownership gate or outputs."""
from pathlib import Path
import json,numpy as np
D=Path(__file__).resolve().parent;rows=json.loads((D/'prediction_seal.json').read_text())['rows'];result=[]
for row in rows:
 R=Path(row['directory']);name=row['name'];sums={k:dict(n=0,bad030=0,accepted=0,accepted_bad030=0) for k in ['ground_absz_le03','positive_obstacle_05_15','other_finite']}
 for p in sorted((R/'source_predictions').glob('*.npz')):
  a=np.load(p);v=np.load(D/'predictions'/f'{name}_{p.name}');q=v['query'];k=v['keep'];f=np.load(R/'frames'/p.name);gt=f['left_depth_m'].ravel()[q];pred=a['depth_m'].ravel()[q];ok=np.isfinite(gt)&(gt>0);x=q%640;y=q//640;ray=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1);T=f['left_camera_usd_world'];world=T[:3,3]+(ray*np.where(ok,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;ground=ok&(abs(world[:,2])<=.03);ob=ok&(world[:,2]>.05)&(world[:,2]<=1.5);other=ok&~ground&~ob;bad=abs(pred-gt)>.3;accepted=np.isin(q,np.load(R/'partial_observations'/p.name)['query_pixel_id'])
  for label,mask in zip(sums,[ground,ob,other]):
   m=mask&k;z=sums[label];z['n']+=int(m.sum());z['bad030']+=int((m&bad).sum());z['accepted']+=int((m&accepted).sum());z['accepted_bad030']+=int((m&accepted&bad).sum())
 result.append(dict(sequence=name,partitions=sums))
(D/'error_partition.json').write_text(json.dumps(dict(scope='post-hoc GT-only explanation of retained packet measurements; categories not online labels; no old gate rescue',results=result),indent=2)+'\n');print(json.dumps(result,indent=2))
