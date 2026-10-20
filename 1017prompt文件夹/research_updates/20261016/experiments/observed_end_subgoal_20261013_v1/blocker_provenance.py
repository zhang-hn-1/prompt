"""One post-run blocking cell: trace source pixels without editing its history."""
import json, numpy as np
from pathlib import Path
D=Path(__file__).resolve().parent;R=D/'candidate_e1_run';m=np.load(R/'maps/frame_545.npz')
target=np.array([-1.1,-.1]);cell=np.floor((target-m['origin'])/float(m['resolution'])+.5).astype(int)
rows=[]
for p in sorted((R/'partial_observations').glob('*.npz')):
    a=np.load(p);ids=np.flatnonzero(np.all(np.floor((a['world'][:,:2]-m['origin'])/float(m['resolution'])+.5).astype(int)==cell,axis=1))
    if not len(ids):continue
    f=np.load(R/'frames'/p.name);q=a['query_pixel_id'][ids].astype(int);xx=q%640;yy=q//640;gt=f['left_depth_m'][yy,xx];ok=np.isfinite(gt)&(gt>0)
    pred=np.load(R/'source_predictions'/p.name)['depth_m'][yy,xx]
    rays=np.stack([(xx[ok]-319.5)/584.8888888888889,(yy[ok]-239.5)/584.8888888888889,np.ones(ok.sum())],1)
    T=f['left_camera_usd_world'];world=T[:3,3]+(rays*gt[ok,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
    rows.append(dict(frame=p.stem,point_instances=len(ids),finite_truth=int(ok.sum()),ground_truth_instances=int((np.abs(world[:,2])<=.03).sum()),
        pixel_ids=q.tolist(),predicted_world=a['world'][ids].tolist(),truth_world_for_valid=world.tolist(),
        finite_truth_depth_m=gt[ok].tolist(),predicted_depth_m=pred.tolist(),absolute_depth_errors_m=np.abs(pred[ok]-gt[ok]).tolist(),position_errors_m=np.linalg.norm(a['world'][ids][ok]-world,axis=1).tolist()))
result=dict(cell_xy=(m['origin']+cell*float(m['resolution'])).tolist(),source_epochs=rows,
    total_point_instances=sum(r['point_instances'] for r in rows),finite_truth_point_instances=sum(r['finite_truth'] for r in rows),ground_truth_instances=sum(r['ground_truth_instances'] for r in rows),
    scope='Only sources that quantize to the one actual blocked-start cell, all181 saved source pairs scanned. Renderer truth post-run only; invalid labels not free; no source or history changed.')
(D/'blocker_provenance.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
