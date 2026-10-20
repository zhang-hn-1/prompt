"""Postseal teacher scoring of fixed advisory flags, never fed back to probe."""
from common import *
from scipy.spatial import cKDTree

def ratio(a,b):return a/b if b else None

def main():
    cfg=load(D/'protocol.json');seal=load(D/'prediction_seal.json');assert seal['complete'] and not seal['GT_read']
    for p,h in cfg['teacher_input_sha256'].items():assert sha(p)==h,p
    for p,h in seal['source_sha256'].items():assert sha(p)==h,p
    with np.load(cfg['teacher_occupancy']) as occ:
        xy=np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy']
    truth=cKDTree(xy);rows=[];totals={};event=None
    for row in seal['rows']:
        f=Path(row['advisory']);assert sha(f)==row['sha256']
        with np.load(f) as a:
            cells=a['cells'];bad=truth.query(cells*.1+a['origin'])[0]>.15;hist=a['historical'];real=~bad
            counts=dict(risk_cells=len(cells),false_cells=int(bad.sum()),current_cells=int(a['current_positive'].sum()),historical_cells=int(hist.sum()),historical_false=int((hist&bad).sum()),historical_real=int((hist&real).sum()),historical_without_anchor=int((hist&(a['anchor_count']==0)).sum()),anchor_queries=len(a['anchor_world']))
            for flag in ['any_anchor_dispute','all_anchor_dispute']:
                q=a[flag];assert not np.any(q&a['current_positive']);counts.update({flag+'_false':int((q&bad).sum()),flag+'_real':int((q&real).sum())})
            for code,name in enumerate(['unavailable','closer_occluded','compatible','farther','mixed_partial']):counts['anchor_pair_'+name]=int((a['current_pair_classes']==code).sum())
            for key,value in counts.items():totals[key]=totals.get(key,0)+value
            rows.append(dict(frame=row['frame'],time_s=row['time_s'],**counts))
            if row['frame']==12:
                # Predetermined first sustained no-route event at policy61 uses the identical source12.
                with np.load(RUN/'maps/frame_061.npz') as m:
                    assert np.array_equal(cells,np.argwhere(m['labels']==1))
                event=dict(policy_frame=61,time_s=6.099999863654375,source_frame=12,source_time_s=row['time_s'],counts=counts)
    summary={}
    for flag in ['any_anchor_dispute','all_anchor_dispute']:
        false=totals[flag+'_false'];real=totals[flag+'_real']
        summary[flag]=dict(false_historical_dispute_recall=ratio(false,totals['historical_false']),real_historical_false_dispute_rate=ratio(real,totals['historical_real']),dispute_teacher_precision=ratio(false,false+real),false_disputes=real,correct_false_risk_disputes=false)
    primary=summary['all_anchor_dispute'];gates=dict(false_historical_coverage=primary['false_historical_dispute_recall'] is not None and primary['false_historical_dispute_recall']>=.2,real_historical_false_dispute=primary['real_historical_false_dispute_rate'] is not None and primary['real_historical_false_dispute_rate']<=.01,current_positive_protected=True,causal=True)
    result=dict(complete=True,domain='All180 unique active source exposures; cell-epoch queries, not independent samples or 900 repeated policy frames. Full B risk cells then historical/current split. Representatives latest per eye, not complete obstacle volume.',totals=totals,summary=summary,primary_gates=gates,necessary_input_gates_pass=all(gates.values()),event=event,rows=rows,decision='INPUT_AUDIT_ONLY_NO_MAP_CHANGE_NO_NEW_POLICY_ADMISSION',GT_scope='Static full E1 bodyband occupancy within.15m in XY, coarse teacher diagnostic not volumetric clearance/safety. Original initial-gauge/height/rolling/ideal feedback limitations retained.',fit=0,source_regeneration=0,physics=0,navigation=0,old_budgets_reopened=False,strict_flags_not_sufficient_clearance_proof=True,source_seal_sha256=sha(D/'prediction_seal.json'),protocol_sha256=sha(D/'protocol.json'))
    write(D/'result.json',result);print('RESULT',json.dumps(dict(totals=totals,summary=summary,gates=gates,event=event)),flush=True)
if __name__=='__main__':main()
