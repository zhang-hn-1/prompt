"""Postseal teacher scoring. Reuses existing visible-target arrays, not new geometry oracle."""
from replay import *
from scipy.spatial import cKDTree
NAMES=['front_left','front_right','rear_left','rear_right']
def main():
    cfg=load(D/'protocol.json');seal=load(D/'replay_seal.json');assert seal['complete'] and seal['GT_used_in_oracle'];fs=load(cfg['feature_seal']);original=load(cfg['label_result']);results={};allrows=[];oldBmatches=0
    for seq in cfg['sequences']:
        with np.load(seq['occupancy']) as occ:
            origin=occ['global_origin_xy'];res=float(occ['resolution_m']);truth=cKDTree(np.argwhere(occ['grid']>0)*res+origin)
        exposures=[r for r in seal['rows'] if r['sequence']==seq['name']];features=[r for r in fs['rows'] if r['sequence']==seq['name']]
        times=[dict(time_s=r['time_s']) for r in exposures] if seq['name']=='E0' else load(seq['policy_records'])
        prior=load(seq['prior_result']);targetrows=None if seq['name']=='E0' else prior['source_rows'];latest=set();ever=set();index=0;metrics={k:dict(false_cell_seconds=0.,active_cell_seconds=0.,blind_supported_queries=0,blind_missing=0) for k in ['B','O']};frames=[]
        for pi,pol in enumerate(times):
            t=float(pol['time_s'])
            while index<len(exposures) and exposures[index]['time_s']<=t+1e-8:
                rr=exposures[index];assert sha(rr['path'])==rr['sha256']
                with np.load(rr['path']) as a:
                    baseorigin=a['origin'].copy();persistent={k:a[k+'_persistent'].copy() for k in ['B','O']};last=a['B_last'].copy();assert np.array_equal(last,a['O_last'])
                if seq['name']=='E0':
                    targetfile=Path(seq['target_root'])/f'frame_{index:03d}.npz'
                    with np.load(targetfile) as a:Tcells=a['target_union'];T=Tcells*.1+a['origin'];targetorigin=a['origin'].copy()
                    with np.load(features[index]['feature_path']) as f:xy=f['cells']*.1+f['origin']
                else:
                    tr=targetrows[index];Tcells=np.array(tr['target_cells_array'],int).reshape(-1,2);T=Tcells*.1+origin;targetorigin=origin
                    with np.load(features[index]['prediction_path']) as p:xy=np.concatenate([p[n+'_world'][:,:2] for n in NAMES])
                supported=cKDTree(xy).query(T)[0]<=.15 if len(xy) else np.zeros(len(T),bool)
                latest=set(map(tuple,Tcells.tolist()));ever.update(map(tuple,Tcells[supported].tolist()));index+=1
            assert index>0
            blind=np.array(sorted(ever-latest),int).reshape(-1,2)*.1+targetorigin
            dt=(times[pi+1]['time_s']-t if pi+1<len(times) else 0.) if seq['name']=='E0' else .1
            fr=dict(policy=pi,time_s=t,blind_supported_queries=len(blind))
            for k in ['B','O']:
                active=(np.isfinite(last)&(t-last<=10.))|persistent[k];labels=np.full((400,400),-1,np.int8);labels[active]=1
                if k=='B' and seq['old_B_maps']:
                    oldf=Path(seq['old_B_maps'])/f'frame_{pi:03d}.npz'
                    with np.load(oldf) as a:assert np.array_equal(labels,a['B_labels'] if seq['name']=='E0' else a['labels'])
                    oldBmatches+=1
                xy=np.argwhere(active)*.1+baseorigin;false=int((truth.query(xy)[0]>.15).sum()) if len(xy) else 0;missing=int((cKDTree(xy).query(blind)[0]>.15).sum()) if len(xy) else len(blind)
                v=metrics[k];v['false_cell_seconds']+=false*dt;v['active_cell_seconds']+=len(xy)*dt;v['blind_supported_queries']+=len(blind);v['blind_missing']+=missing
                fr[k]=dict(active=len(xy),false=false,blind_missing=missing)
            frames.append(fr)
        for v in metrics.values():v['blind_missing_fraction']=v['blind_missing']/v['blind_supported_queries'] if v['blind_supported_queries'] else None
        b,o=metrics['B'],metrics['O'];reduction=1-o['false_cell_seconds']/b['false_cell_seconds'] if b['false_cell_seconds'] else None;drop=o['blind_missing_fraction']-b['blind_missing_fraction'] if b['blind_missing_fraction'] is not None else None
        if seq['name'] in ['E0','E1B']:
            expected=prior['aggregate']['B']['false_cell_seconds'] if seq['name']=='E0' else prior['memory']['false_map_cell_seconds'];assert abs(b['false_cell_seconds']-expected)<1e-6
            supportedprior=load(seq['supported_result']) if seq['name']=='E1B' else prior['aggregate']['B'];den=supportedprior['blind_previously_source_supported_target_queries'] if seq['name']=='E1B' else supportedprior['blind_historical_target_queries'];miss=supportedprior['missing_queries'] if seq['name']=='E1B' else supportedprior['blind_missing_queries'];assert (b['blind_supported_queries'],b['blind_missing'])==(den,miss)
        gates=dict(false_reduction_ge20pct=reduction is not None and reduction>=.2,blind_missing_increase_le5pp=drop is not None and drop<=.05,no_current_recent_discarded=seal['current_risk_lost']==seal['recent_risk_lost']==0)
        results[seq['name']]=dict(metrics=metrics,false_reduction=reduction,blind_missing_increase=drop,gates=gates,pass_necessary=all(gates.values()),rows=frames);print('SCORED',seq['name'],reduction,drop,gates,flush=True)
    assert oldBmatches==1010
    passed=all(r['pass_necessary'] for r in results.values())
    put(D/'result.json',dict(complete=True,decision='NECESSARY_IDEAL_TEACHER_CONSUMER_PASSED_DATA_DESIGN_ONLY' if passed else 'STOP_THREEWAY_ENTRY_BEFORE_DATA_OR_FIT',necessary_passed=passed,sequences=results,B_original_policy_maps_exact=oldBmatches,scope='Privileged same-sealed-input diagnostic only; unknown andnegative both withhold LONGTERM admission, never current/recent risk. Not deployable, fitted performance, navigation, fresh holdout or a formal best-possible upper bound.',original_binary_data_qualification_unchanged=True,teacher_bands_unchanged=True,fit=0,learned_inference=0,source_regeneration=0,physics=0,navigation=0,replay_seal_sha256=sha(D/'replay_seal.json'),protocol_sha256=sha(D/'protocol.json')))
if __name__=='__main__':main()
