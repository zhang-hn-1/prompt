"""Additional source-supported blind-forgetting denominator; postseal teacher only.
Frozen P0 original metric (all historical visible targets) stays a coverage gap metric.
"""
from paths import *
import json,numpy as np,hashlib
from scipy.spatial import cKDTree
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
out=D/'e1_run';manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())
for f,h in manifest['artifact_sha256'].items():assert sha(f)==h,f
r=json.loads((D/'e1_result.json').read_text());cap=json.loads((out/'capture_seal.json').read_text());cfg=json.loads((D/'protocol.json').read_text());occ=np.load(cfg['occupancy']);origin=occ['global_origin_xy']
policy=json.loads((out/'policy_records.json').read_text());ever=set();visible=set();idx=0;den=0;missing=0;rows=[]
for i,row in enumerate(policy):
    while idx<len(r['source_rows']) and r['source_rows'][idx]['time_s']<=row['time_s']+1e-8:
        source=r['source_rows'][idx];visible=set(map(tuple,source['target_cells_array']));T=np.array(source['target_cells_array'],int).reshape(-1,2)*.1+origin
        p=np.load(out/'predictions'/cap['rows'][idx]['frame']);world=np.concatenate([p[n+'_world'] for n in ['front_left','front_right','rear_left','rear_right']])
        matched=cKDTree(world[:,:2]).query(T)[0]<=.15 if len(world) else np.zeros(len(T),bool)
        ever.update(tuple(source['target_cells_array'][j]) for j in np.flatnonzero(matched));idx+=1
    blind=np.array(sorted(ever-visible),int).reshape(-1,2)*.1+origin
    with np.load(out/'maps'/f'frame_{i:03d}.npz') as m:
        xy=np.argwhere(m['labels']==1)*float(m['resolution'])+m['origin'];distance=cKDTree(xy).query(blind)[0] if len(xy) else np.full(len(blind),np.inf)
        miss=int((distance>.15).sum());den+=len(blind);missing+=miss;rows.append(dict(time_s=row['time_s'],blind_previously_source_supported_targets=len(blind),missing=miss))
nonzero=np.sign(np.load(out/'requests.npz')['requests'][:,::2].mean(axis=1));nonzero=nonzero[nonzero!=0]
result=dict(complete=True,blind_previously_source_supported_target_queries=den,missing_queries=missing,missing_fraction=missing/den if den else None,observed_request_direction_switches_ignoring_zero=int((np.diff(nonzero)!=0).sum()),tracker_pending_direction_time_s=sum(p['tracker_state']['pending_direction'] is not None for p in policy)*.1,rows=rows,scope='All E1 actual B policy epochs; blind denominator is GT target cells previously supported by actual current SGBM risk points within.15m, absent current GT visible target union. Teacher only; not complete blindvolume. Do not compare frozen original all-visible-gap metric with this source-supported forgetting metric.',manifest_sha256=sha(out/'artifact_manifest_post_close.json'))
(D/'supported_memory_result.json').write_text(json.dumps(result,indent=2)+'\n');print('SUPPORTED_MEMORY',den,missing,result['missing_fraction'],flush=True)
