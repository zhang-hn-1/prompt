"""Privileged necessary-consumer diagnostic only. No learned or source predictions."""
from pathlib import Path
import json,hashlib,importlib.util,datetime
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
load=lambda p:json.loads(Path(p).read_text())
def put(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def main():
    cfg=load(D/'protocol.json')
    for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
    for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
    spec=importlib.util.spec_from_file_location('immutable_entry',cfg['consumer']);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    fs=load(cfg['feature_seal']);lr=load(cfg['label_result']);out=D/'maps';out.mkdir(exist_ok=False);rows=[];checks=m.interface_checks();bmatches=0;currentlost=0;recentlost=0
    for seq in cfg['sequences']:
        frames=[(f,l) for f,l in zip(fs['rows'],lr['rows']) if f['sequence']==seq['name']];g={k:m.EntryState() for k in ['B','O']};previous=-np.inf;lastrows={};lastsource=-np.inf
        for f,l in frames:
            assert sha(f['feature_path'])==f['feature_sha256'] and sha(l['label_file'])==l['sha256']
            with np.load(f['feature_path']) as a,np.load(l['label_file']) as teacher:
                cells=a['cells'];t=f['time_s'];assert t>previous;previous=t;labels=teacher['label'];scores=np.where(labels==1,1.,np.nan)
                # Negative and UNKNOWN remain DISTINCT saved labels. Both decline promotion only.
                assert np.array_equal(a['promotion_eligible'],teacher['promotion_eligible'])
                arrays={};metric={}
                for k in ['B','O']:
                    y,clock=g[k].update(f['frame'],t,t,cells,np.ones(len(cells)) if k=='B' else scores,f['frame'],t)
                    arrays[k+'_labels']=y;arrays[k+'_clock']=clock;arrays[k+'_persistent']=np.isfinite(g[k].persistent);arrays[k+'_last']=g[k].last.copy()
                    recent=np.isfinite(g[k].last)&(t-g[k].last<=10.);recentlost+=int((recent&(y!=1)).sum())
                    currentlost+=int((y[tuple(cells.T)]!=1).sum())
                assert not np.any(arrays['O_labels']==0) and not np.any(arrays['B_labels']==0)
                if seq['old_B_maps']:
                    mf=Path(seq['old_B_maps'])/f'frame_{f["frame"] if seq["name"]=="E0" else 5*f["frame"]:03d}.npz'
                    with np.load(mf) as old:assert np.array_equal(arrays['B_labels'],old['B_labels'] if seq['name']=='E0' else old['labels'])
                    bmatches+=1
                dest=out/f'{seq["name"]}_{f["frame"]:03d}.npz';np.savez_compressed(dest,**arrays,cells=cells,teacher_class=labels,origin=a['origin'],resolution=.1,time_s=t)
                rows.append(dict(sequence=seq['name'],frame=f['frame'],time_s=t,path=str(dest),sha256=sha(dest),feature_sha256=f['feature_sha256'],label_sha256=l['sha256']))
        print('ORACLE_MAPS',seq['name'],len(frames),flush=True)
    assert len(rows)==470 and bmatches==290 and currentlost==recentlost==0
    put(D/'replay_seal.json',dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,checks=checks,all_admit_original_B_exposure_maps_exact=bmatches,current_risk_lost=currentlost,recent_risk_lost=recentlost,GT_used_in_oracle=True,oracle_not_online=True,learned_inference=0,fit=0,source_regeneration=0,physics=0,protocol_sha256=sha(D/'protocol.json')))
    print('PRIVILEGED_ORACLE_SEALED',len(rows),flush=True)
if __name__=='__main__':main()
