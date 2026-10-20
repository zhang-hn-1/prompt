"""Prediction-only advisory audit. Does not open frame RGB/depth labels or teacher files.
Does not change source, memory, planner, or command. All advisory arrays sealed before score.
"""
from common import *
import os,datetime

def main():
    cfg=load(D/'protocol.json'); assert not (D/'prediction_seal.json').exists()
    for p,h in cfg['generation_input_sha256'].items():assert sha(p)==h,p
    seal=load(RUN/'source_seal.json');policy=load(RUN/'policy_records.json')
    # Necessary branch checks: absence/closer never evidence of clearing; both-far required.
    fixture=np.array([[0,0,0,0],[1,3,0,0],[2,2,0,0],[3,3,0,0],[3,0,0,0]],np.int8)
    assert pair_classes(fixture)[:,0].tolist()==[0,1,2,3,4]
    bank={};rows=[];out=D/'advisories';out.mkdir()
    shape=None;origin=None;resolution=.1;max_world_error=0.
    for i,s in enumerate(seal['rows'][:180]):
        assert s['frame']==i and abs(policy[5*i]['source_time_s']-s['time_s'])<1e-5
        pf=RUN/'predictions'/f'frame_{i:03d}.npz';mf=RUN/'maps'/f'frame_{5*i:03d}.npz'
        assert sha(pf)==s['prediction_sha256'];assert sha(mf)==cfg['generation_input_sha256'][str(mf)]
        with np.load(mf) as m,np.load(pf) as p:
            if shape is None:shape=m['labels'].shape;origin=m['origin'].copy()
            assert tuple(m['labels'].shape)==tuple(shape) and np.array_equal(origin,m['origin']) and abs(float(m['resolution'])-.1)<1e-8
            assert abs(float(p['time_s'])-s['time_s'])<1e-8
            body=p['body_estimated'];assert body.shape==(4,4) and np.isfinite(body).all()
            hazard=np.argwhere(m['labels']==1);hzids=hazard[:,0]*shape[1]+hazard[:,1];current=set()
            for e,n in enumerate(NAMES):
                world=p[n+'_world'];uids=p[n+'_query_pixel'];partner=p[n+'_partner_pixel']
                assert len(world)==len(uids)==len(partner)
                # Exhaustive endpoint consistency, no GT; keep same original quantization convention.
                T=body@LOCAL[e];u=uids%640;v=uids//640;z=p[n+'_depth'].reshape(-1)[uids]
                expected=np.stack([(u-319.5)/F,(v-239.5)/F,np.ones(len(u))],1)*z[:,None]
                expected=expected@(T[:3,:3]@AX).T+T[:3,3]
                if len(world):max_world_error=max(max_world_error,float(abs(world-expected).max()))
                assert np.allclose(world,expected,atol=1e-6,rtol=1e-6)
                cells=np.floor((world[:,:2]-origin)/resolution+.5).astype(int)
                keep=(cells>=0).all(1)&(cells<np.array(shape)).all(1)
                cells=cells[keep];world=world[keep];uids=uids[keep];partner=partner[keep]
                ids=cells[:,0]*shape[1]+cells[:,1];current.update(map(int,ids))
                # Latest exposure, lowest query UID per cell/eye; deterministic representative, not full cell volume.
                order=np.argsort(uids,kind='stable');_,first=np.unique(ids[order],return_index=True)
                for j in order[first]:bank[(int(ids[j]),e)]=(world[j].copy(),i,int(uids[j]),int(partner[j]))
            current_mask=np.isin(hzids,np.array(sorted(current),int));history=~current_mask
            acell=[];aeye=[];aw=[];af=[];aq=[];ap=[];cell_index={int(c):j for j,c in enumerate(hzids)}
            for (c,e),(w,f,uid,puid) in sorted(bank.items()):
                if c in cell_index and f<i and history[cell_index[c]]:
                    acell.append(cell_index[c]);aeye.append(e);aw.append(w);af.append(f);aq.append(uid);ap.append(puid)
            aw=np.asarray(aw).reshape(-1,3);votes=[];pixels=[];az=[];observed=[]
            for e,n in enumerate(NAMES):
                vv,px,zz,oo=camera_vote(aw,body@LOCAL[e],p[n+'_depth'],p[n+'_valid'],p[n+'_CAD'])
                votes.append(vv);pixels.append(px);az.append(zz);observed.append(oo)
            votes=np.stack(votes,1);classes=pair_classes(votes)
            anchor_far=(classes==3).any(1);anchor_blocked=np.isin(classes,[1,2]).any(1)
            anchor_dispute=anchor_far&~anchor_blocked
            acell=np.array(acell,int);count=np.bincount(acell,minlength=len(hazard));farcount=np.bincount(acell,weights=anchor_dispute.astype(int),minlength=len(hazard))
            anyflag=(farcount>0)&history;strict=(count>0)&(farcount==count)&history
            assert not np.any(strict&current_mask) and np.all(np.array(af)<i)
            dest=out/f'frame_{i:03d}.npz'
            np.savez_compressed(dest,cells=hazard,origin=origin,resolution=resolution,time_s=s['time_s'],current_positive=current_mask,historical=history,anchor_count=count,any_anchor_dispute=anyflag,all_anchor_dispute=strict,anchor_cell_index=acell,anchor_eye=np.array(aeye,int),anchor_world=aw,anchor_frame=np.array(af,int),anchor_query_uid=np.array(aq,int),anchor_partner_uid=np.array(ap,int),current_votes=votes,current_pair_classes=classes,current_query_uid=np.stack(pixels,1),anchor_current_z=np.stack(az,1),current_observed_z=np.stack(observed,1),anchor_dispute=anchor_dispute)
            rows.append(dict(frame=i,policy_frame=5*i,time_s=s['time_s'],input_prediction_sha256=sha(pf),input_map_sha256=sha(mf),advisory=str(dest),sha256=sha(dest),risk_cells=len(hazard),historical_cells=int(history.sum()),current_cells=int(current_mask.sum()),anchors=len(aw),any_anchor_dispute=int(anyflag.sum()),all_anchor_dispute=int(strict.sum())))
        if i%30==0:print('AUDIT',i,rows[-1],flush=True)
    write(D/'prediction_seal.json',dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,source_last_active_frame=179,terminal_frame180_not_a_decision=True,GT_read=False,map_changed=False,source_regenerated=False,max_endpoint_consistency_error_m=max_world_error,protocol_sha256=sha(D/'protocol.json'),source_sha256={str(D/x):sha(D/x) for x in ['common.py','probe.py','score.py','supervise.py']}))
    print('SEALED',len(rows),flush=True)
if __name__=='__main__':main()
