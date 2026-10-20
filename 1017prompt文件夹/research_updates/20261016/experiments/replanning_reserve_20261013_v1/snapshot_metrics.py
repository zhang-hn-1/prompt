from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;results={}
for arm in ['baseline_e1','candidate_e1']:
    R=D/(arm+'_run')
    if not (R/'termination.json').exists():continue
    paths=sorted((R/'source_predictions').glob('*.npz'));p=paths[len(paths)//2];a=np.load(p);f=np.load(R/'frames'/p.name);z=a['depth_m'];gt=f['left_depth_m']
    keep=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3);ok=keep&np.isfinite(gt)&(gt>0);err=np.abs(z[ok]-gt[ok])
    config=json.loads((D/(arm+'_protocol.json')).read_text());ref=np.load(config['occupancy']);truth=ref['global_origin_xy']+np.argwhere(ref['grid']>0)*float(ref['resolution_m']);tree=cKDTree(truth)
    def map_stats(path):
        m=np.load(path);cells=np.argwhere(m['labels']==1);xy=m['origin']+cells*float(m['resolution']);far=tree.query(xy)[0]>.15;old=float(m['time_s'])-m['hazard_acquisition_time'][tuple(cells.T)]>10
        return dict(time_s=float(m['time_s']),risk_cells=len(cells),farther015m_cells=int(far.sum()),old10s_cells=int(old.sum()),old10s_farther015m_cells=int((old&far).sum()))
    pt=np.load(R/'partial_observations'/p.name);ids=pt['query_pixel_id'].astype(int);xx=ids%640;yy=ids//640;tz=gt[yy,xx];valid=np.isfinite(tz)&(tz>0);T=f['left_camera_usd_world'];rays=np.stack([(xx[valid]-319.5)/584.8888888888889,(yy[valid]-239.5)/584.8888888888889,np.ones(valid.sum())],1);world=T[:3,3]+(rays*tz[valid,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
    results[arm]=dict(selection='temporal midpoint of complete saved source sequence, not worst frame',time_s=float(a['time_s']),scored_pixels=int(ok.sum()),predicted_confident_pixels=int(keep.sum()),full_image_pixels=z.size,p50_p90_p95_m=np.quantile(err,[.5,.9,.95]).tolist() if len(err) else None,accepted_point_instances=len(ids),truth_valid_point_instances=int(valid.sum()),ground_false_point_instances=int((np.abs(world[:,2])<=.03).sum()),snapshot_map=map_stats(R/'maps'/p.name),final_map=map_stats(sorted((R/'maps').glob('*.npz'))[-1]))
(D/'snapshot_metrics.json').write_text(json.dumps(results,indent=2)+'\n');print(results)
