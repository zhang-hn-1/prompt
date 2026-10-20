from pathlib import Path
import sys,json,hashlib,numpy as np
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;W=D.parents[1];S=W/'experiments/four_camera_weak_baseline_20261017_v1';H=W/'experiments/contact_boundary_source_20261016_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    cfg=json.loads((D/'protocol.json').read_text());ev=json.loads((D/'evaluation_protocol.json').read_text())
    assert sha(Path(__file__))==ev['score_sha256']
    for f,h in ev['input_sha256'].items():assert sha(f)==h,f
    seal=json.loads((D/'replay_seal.json').read_text());assert seal['complete']
    occ=np.load(ev['occupancy']);reference=np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy'];truth=cKDTree(reference)
    rows=[];ever_true_observed=set();metrics={k:dict(false_cell_seconds=0.,blind_historical_target_queries=0,blind_missing_queries=0,active_cell_seconds=0.) for k in 'ABC'}
    maps_differ=0;current_discarded=0
    for i,rr in enumerate(seal['rows']):
        f=D/'maps'/f'frame_{i:03d}.npz';assert sha(f)==rr['sha256']
        with np.load(f) as a,np.load(H/'e0_recovery_qualification'/f'frame_{i:03d}.npz') as target:
            origin=a['origin'];T=target['target_union']*.1+target['origin'];last=i==len(seal['rows'])-1;dt=0. if last else seal['rows'][i+1]['time_s']-rr['time_s']
            xy={k:np.argwhere(a[k+'_labels']==1)*.1+origin for k in 'ABC'}
            ts={k:cKDTree(v) if len(v) else None for k,v in xy.items()}
            observed=(ts['A'].query(T)[0]<=.15) if ts['A'] is not None else np.zeros(len(T),bool)
            # Global grid physical coordinates keyed at original GT grid indices.
            original_indices=target['target_union'];ever_true_observed.update(map(tuple,original_indices[observed].tolist()))
            current_visible=set(map(tuple,original_indices.tolist()));blind=np.array(sorted(ever_true_observed-current_visible),int).reshape(-1,2)*.1+target['origin']
            row=dict(frame=i,time_s=rr['time_s'],blind_supported_targets=len(blind))
            for k in 'ABC':
                bad=int((truth.query(xy[k])[0]>.15).sum()) if len(xy[k]) else 0
                missed=int((ts[k].query(blind)[0]>.15).sum()) if ts[k] is not None else len(blind)
                metrics[k]['false_cell_seconds']+=bad*dt;metrics[k]['active_cell_seconds']+=len(xy[k])*dt;metrics[k]['blind_historical_target_queries']+=len(blind);metrics[k]['blind_missing_queries']+=missed
                row[k]=dict(active=len(xy[k]),false=bad,blind_missing=missed,confirmed=rr[k].get('confirmed_persistent_hazard_cells',0))
            maps_differ+=int(not np.array_equal(a['B_labels'],a['C_labels']))
            current_discarded+=int(((a['A_labels']==1)&(a['C_labels']!=1)).sum())
            assert all(not (a[k+'_labels']==0).any() for k in 'ABC')
            rows.append(row)
    for m in metrics.values():m['blind_missing_fraction']=m['blind_missing_queries']/m['blind_historical_target_queries'] if m['blind_historical_target_queries'] else None
    b=metrics['B'];c=metrics['C'];reduction=1-c['false_cell_seconds']/b['false_cell_seconds'] if b['false_cell_seconds'] else 0.
    drop=c['blind_missing_fraction']-b['blind_missing_fraction'] if b['blind_missing_fraction'] is not None else None
    gates=dict(false_risk_reduction_ge20pct=reduction>=.2,blind_missing_increase_le5pp=drop is not None and drop<=.05,map_difference=maps_differ>=1,no_current_positive_discarded=current_discarded==0,all_logical_checks=all(seal['checks'].values()))
    result=dict(complete=True,rows=rows,aggregate=metrics,false_cell_seconds_reduction_C_vs_B=reduction,blind_missing_fraction_increase_C_vs_B=drop,maps_changed_frames=maps_differ,current_positive_discarded=current_discarded,gates=gates,pass_predefined_offline_performance=all(gates.values()),decision='PROCEED_TO_FROZEN_P2_PROTOCOL' if all(gates.values()) else 'REJECT_C_OFFLINE_NO_PHYSICAL_SWEEP',physics=0,fit=0,scope='Same sealed E0 inputs replay only, no navigation improvement or generalization. False cells vs entire static bodyband occupancy within.15m. Blind denominator only GT targets ever covered by A, absent currentGTtargetunion. Source/pose errors are inherited; outputs absent=unknown not free. Raw poses have no certified error bound.',evaluation_protocol_sha256=sha(D/'evaluation_protocol.json'))
    (D/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n');print('P1_REPLAY_RESULT',metrics,reduction,drop,gates,flush=True)
if __name__=='__main__':main()
