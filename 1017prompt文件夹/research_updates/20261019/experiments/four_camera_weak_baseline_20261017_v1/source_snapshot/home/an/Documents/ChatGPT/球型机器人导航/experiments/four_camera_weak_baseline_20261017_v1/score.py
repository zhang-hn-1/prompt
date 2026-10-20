"""Only sealed predictions; post hoc GT never enters baseline decisions."""
from paths import *
from source import NAMES,F,RAYS,AX
import numpy as np,json,hashlib
from scipy.spatial import cKDTree
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def metric(pred,target):
    d=cKDTree(target*.1).query(pred*.1)[0] if len(pred) and len(target) else np.full(len(pred),np.inf)
    td=cKDTree(pred*.1).query(target*.1)[0] if len(pred) and len(target) else np.full(len(target),np.inf)
    return dict(predicted=len(pred),target=len(target),correct=int((d<=.15).sum()),covered=int((td<=.15).sum()),precision=float(np.mean(d<=.15)) if len(d) else None,recall=float(np.mean(td<=.15)) if len(td) else None,grid_P90_m=float(np.quantile(d,.9)) if len(d) and np.isfinite(d).all() else None)
def main():
    ev=json.loads((D/'evaluation_protocol.json').read_text())
    for f,h in ev['input_sha256'].items():assert sha(f)==h,f
    assert sha(Path(__file__))==ev['score_sha256']
    cfg=json.loads((D/'protocol.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text());assert seal['complete']
    rows=[];errors={n:[] for n in NAMES};total={n:dict(target_pixels=0,target_valid=0,floor_pixels=0,floor_stride2_pixels=0,floor_risk_proxy=0) for n in NAMES}
    for i,r in enumerate(seal['rows']):
        pf=D/'e0_predictions'/r['frame'];assert sha(pf)==r['prediction_sha256']
        ip=Path(cfg['data_root'])/'frames'/r['frame'];assert sha(ip)==r['input_sha256']
        with np.load(pf) as p,np.load(ip) as a,np.load(W/'experiments/contact_boundary_source_20261016_v1/e0_recovery_qualification'/r['frame']) as cache:
            origin=cache['origin'];target=cache['target_union'];per={};worlds=[]
            for n in NAMES:
                z=a[n+'_depth_label'];T=a[n+'_camera_usd_world_label'];mask=p[n+'_CAD'];pd=p[n+'_depth'];ok=p[n+'_valid']&np.isfinite(pd)&(pd>=.5)&(pd<=3)
                finite=np.isfinite(z)&(z>=.5)&(z<=3)&~mask
                xyz=RAYS*z[...,None];world=xyz@(T[:3,:3]@AX).T+T[:3,3]
                out=np.linalg.norm(world-a['root_position_physics_label'],axis=2)>=.55
                targetpix=finite&out&(world[...,2]>.05)&(world[...,2]<=1.5)
                floor=finite&out&(abs(world[...,2])<=.03)
                ee=abs(pd[targetpix&ok]-z[targetpix&ok]);errors[n].extend(ee.tolist())
                # Compare projected risk at corresponding native pixels; semantic proxy remains disclosed.
                ids=p[n+'_query_pixel'];false=int(floor.reshape(-1)[ids].sum());worlds.append(p[n+'_world'])
                c=total[n];c['target_pixels']+=int(targetpix.sum());c['target_valid']+=int((targetpix&ok).sum());c['floor_pixels']+=int(floor.sum());c['floor_stride2_pixels']+=int(floor[::2,::2].sum());c['floor_risk_proxy']+=false
                per[n]=dict(target_pixels=int(targetpix.sum()),target_valid=int((targetpix&ok).sum()),risk_points=len(ids),conditional_depth_P90_m=float(np.quantile(ee,.9)) if len(ee) else None)
            world=np.concatenate(worlds);cells=np.unique(np.floor((world[:,:2]-origin)/.1+.5).astype(np.int64),axis=0)
            row=dict(frame=r['frame'],time_s=r['time_s'],generation_s=r['generation_s'],eyes=per,union=metric(cells,target),suspended=metric(cells,cache['suspended_cells']),ground_intersecting=metric(cells,cache['floor_intersecting_cells']))
            rows.append(row)
    summary={k:sum(r['union'][k] for r in rows) for k in ('predicted','target','correct','covered')}
    summary.update(precision=summary['correct']/summary['predicted'] if summary['predicted'] else None,recall=summary['covered']/summary['target'] if summary['target'] else None)
    nonempty=sum(any(e['risk_points'] for e in r['eyes'].values()) for r in rows);points=sum(e['risk_points'] for r in rows for e in r['eyes'].values())
    sufficient=nonempty>=11 and points>=100
    result=dict(complete=True,rows=rows,aggregate=summary,per_eye={n:dict(**total[n],conditional_depth_P50_P90_P95_m=np.quantile(errors[n],[.5,.9,.95]).tolist() if errors[n] else None) for n in NAMES},generation_P50_P90_max_s=np.quantile([r['generation_s'] for r in rows],[.5,.9,1]).tolist(),nonempty_risk_frames=nonempty,total_risk_points=points,minimum_observation_support_for_conditional_baseline=sufficient,old_strict_source_admitted=False,scope='Actual RGB SGBM current four camera E0 DEVELOPMENT. Height proxy not RGB semantics; missing depth unknown; pair geometry does not imply all targets overlap; source generation excludes planning/map/control.',physics=0,fit=0,evaluation_protocol_sha256=sha(D/'evaluation_protocol.json'))
    (D/'e0_result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('RESULT',summary,nonempty,points,sufficient,flush=True)
if __name__=='__main__':main()
