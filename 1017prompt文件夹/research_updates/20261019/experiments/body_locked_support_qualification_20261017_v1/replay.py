"""Frozen untrained geometry advisories ONLY; no teacher action/physical class read in map updates."""
from common import *
import importlib.util
def main():
    cfg=load(D/'protocol.json');seal=load(D/'prediction_seal.json');assert seal['complete']
    spec=importlib.util.spec_from_file_location('frozen_capture_action_consumer',cfg['consumer']);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    fs=load(cfg['feature_seal']);out=D/'maps';out.mkdir();rows=[];checks=m.interface_checks();bmatches=0
    for seq in cfg['sequences']:
        pairs=[(f,p) for f,p in zip(fs['rows'],seal['rows']) if f['sequence']==seq['name']]
        b=m.CaptureAdmission();q=m.CaptureAdmission()
        for f,p in pairs:
            assert (f['sequence'],f['frame'])==(p['sequence'],p['frame'])
            assert sha(f['feature_path'])==f['feature_sha256'] and sha(p['path'])==p['sha256']
            with np.load(f['feature_path']) as a,np.load(p['path']) as pred:
                t=f['time_s'];cells=a['cells'];assert np.array_equal(cells,pred['cells']) and t==float(pred['time_s'])
                by,_=b.update(f['frame'],t,t,cells,np.zeros(len(cells)),f['frame'],t)
                qy,_=q.update(f['frame'],t,t,cells,pred['scores'],f['frame'],t)
                assert not np.any((qy==1)&(by!=1))
                if seq['old_B_maps']:
                    path=Path(seq['old_B_maps'])/f'frame_{f["frame"] if seq["name"]=="E0" else 5*f["frame"]:03d}.npz'
                    with np.load(path) as z:assert np.array_equal(by,z['B_labels'] if seq['name']=='E0' else z['labels'])
                    bmatches+=1
                path=out/f'{seq["name"]}_{f["frame"]:03d}.npz'
                np.savez_compressed(path,B_labels=by,Q_labels=qy,B_last=b.engine.last,Q_last=q.engine.last,
                    B_persistent=np.isfinite(b.engine.persistent),Q_persistent=np.isfinite(q.engine.persistent),
                    B_epochs=b.engine.epochs,Q_epochs=q.engine.epochs,cells=cells,advisory_scores=pred['scores'],origin=a['origin'],time_s=t)
                rows.append(dict(sequence=f['sequence'],frame=f['frame'],time_s=t,path=str(path),sha256=sha(path),feature_sha256=f['feature_sha256'],prediction_sha256=p['sha256']))
        print('GEOMETRY_ADVISORY_REPLAY',seq['name'],len(pairs),flush=True)
    assert len(rows)==470 and bmatches==290
    put(D/'replay_seal.json',dict(complete=True,utc=now(),rows=rows,fixtures=checks,B_exposure_maps_exact=bmatches,teacher_read_in_map_update=False,
        prediction_seal_sha256=sha(D/'prediction_seal.json'),fit=0,source_calls=0,physics=0,navigation=0))
if __name__=='__main__':main()
