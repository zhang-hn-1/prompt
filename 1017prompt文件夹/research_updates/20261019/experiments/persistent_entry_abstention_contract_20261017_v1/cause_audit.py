"""One read-only failure-layer attribution: mandatory recent-risk error floor."""
from replay import *
from scipy.spatial import cKDTree

def main():
    cfg=load(D/'protocol.json');seal=load(D/'replay_seal.json');result=load(D/'result.json');assert result['complete'];answer={};rows=[]
    for seq in cfg['sequences']:
        with np.load(seq['occupancy']) as a:truth=cKDTree(np.argwhere(a['grid']>0)*float(a['resolution_m'])+a['global_origin_xy'])
        ex=[r for r in seal['rows'] if r['sequence']==seq['name']];times=[dict(time_s=r['time_s']) for r in ex] if seq['name']=='E0' else load(seq['policy_records']);idx=0;recentfloor=0.;extrab=0.;extrao=0.;maxrecent=0
        for i,row in enumerate(times):
            t=row['time_s']
            while idx<len(ex) and ex[idx]['time_s']<=t+1e-8:
                with np.load(ex[idx]['path']) as a:last=a['B_last'].copy();pB=a['B_persistent'].copy();pO=a['O_persistent'].copy();origin=a['origin'].copy()
                idx+=1
            recent=np.isfinite(last)&(t-last<=10.);union=recent|pB|pO;cells=np.argwhere(union);bad=truth.query(cells*.1+origin)[0]>.15
            badcells=cells[bad];ix=tuple(badcells.T);protected=int(recent[ix].sum());onlyb=int((pB[ix]&~recent[ix]).sum());onlyo=int((pO[ix]&~recent[ix]).sum())
            dt=(times[i+1]['time_s']-t if i+1<len(times) else 0.) if seq['name']=='E0' else .1
            recentfloor+=protected*dt;extrab+=onlyb*dt;extrao+=onlyo*dt;maxrecent=max(maxrecent,protected);rows.append(dict(sequence=seq['name'],index=i,time_s=t,recent_false=protected,B_false_outside_recent=onlyb,O_false_outside_recent=onlyo,dt_s=dt))
        prior=result['sequences'][seq['name']]['metrics'];assert abs(recentfloor+extrab-prior['B']['false_cell_seconds'])<1e-6;assert abs(recentfloor+extrao-prior['O']['false_cell_seconds'])<1e-6;assert abs(extrao)<1e-12
        answer[seq['name']]=dict(mandatory_recent_false_cell_seconds=recentfloor,B_additional_persistent_false_cell_seconds=extrab,O_additional_persistent_false_cell_seconds=extrao,max_recent_false_cells=maxrecent,upper_bound_fractional_false_reduction_for_any_consumer_preserving_all_recent=1-recentfloor/prior['B']['false_cell_seconds'],scope='Exact fixed-input/static-teacher bound: any risk map containing every current/recent positive has at least these false cell-seconds. Does not bound other consumers, different source streams, or navigation success.')
    # E1A source-supported denominator is independently retained even though its B maps are counterfactual.
    assert result['sequences']['E1A']['metrics']['B']['blind_supported_queries']==66215
    put(D/'cause_assessment.json',dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),attribution=answer,rows=rows,result_sha256=sha(D/'result.json'),replay_seal_sha256=sha(D/'replay_seal.json'),source_regeneration=0,fit=0,physics=0,decision='E0 current/recent error floor makes frozen20pct gain impossible for any longterm-only rule retaining this recent layer; do not buy data or train this proposal. E1 headroom exists but cannot posthocdropE0 to rescue frozenallsequence gate.'))
    print(json.dumps(answer,indent=2))
if __name__=='__main__':main()
