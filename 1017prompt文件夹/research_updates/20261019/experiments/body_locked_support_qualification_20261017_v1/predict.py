"""Sourcepoint/estimatedpose only. Full advisory seal precedes all teacher reads."""
from common import *
from selector import MotionCompatibility,joint_actions
import time
NAMES=('front_left','front_right','rear_left','rear_right')
def main():
    cfg=load(D/'protocol.json');assert not (D/'prediction_seal.json').exists()
    for p,h in cfg['generation_input_sha256'].items():assert sha(p)==h,p
    checks=load(D/'fixture_result.json')['checks'];assert all(checks.values());fs=load(cfg['feature_seal']);rows=[];trackers={};times={};totalpoints=0;totalcells=0;out=D/'predictions';out.mkdir()
    for f in fs['rows']:
        assert sha(f['feature_path'])==f['feature_sha256'] and sha(f['prediction_path'])==f['prediction_sha256']
        with np.load(f['prediction_path']) as p,np.load(f['feature_path']) as a:
            clouds=[p[n+'_world'] for n in NAMES];body=p['body_estimated'];cells=a['cells'];inverse=a['source_cell_row']
            assert len(inverse)==sum(len(x) for x in clouds) and len(cells)==f['cells']
            eye=np.concatenate([np.full(len(x),j,int) for j,x in enumerate(clouds)]);assert np.array_equal(eye,a['source_eye'])
            assert np.array_equal(np.concatenate([p[n+'_query_pixel'] for n in NAMES]),a['source_query_uid'])
            assert np.array_equal(np.concatenate([p[n+'_partner_pixel'] for n in NAMES]),a['source_partner_uid'])
            tracker=trackers.setdefault(f['sequence'],MotionCompatibility());begin=time.perf_counter()
            r=tracker.update(f['frame'],f['time_s'],f['time_s'],body,clouds);scores=joint_actions(inverse,r['point_witness'],len(cells));elapsed=time.perf_counter()-begin
            times.setdefault(f['sequence'],[]).append(elapsed)
            assert np.array_equal(r['point_eye'],eye) and np.isin(scores,[0.,1.]).all()
            if r['anchor_frame']>=0:assert r['anchor_frame']<f['frame'] and 0<f['time_s']-r['anchor_time_s']<=10. and r['anchor_translation_m']>=.2
            path=out/f'{f["sequence"]}_frame_{f["frame"]:03d}.npz'
            np.savez_compressed(path,cells=cells,scores=scores,origin=a['origin'],time_s=f['time_s'],
                distance_world=r['distance_world'],distance_body=r['distance_body'],point_witness=r['point_witness'],point_eye=r['point_eye'],anchor_frame=r['anchor_frame'])
            rows.append(dict(sequence=f['sequence'],frame=f['frame'],time_s=f['time_s'],path=str(path),sha256=sha(path),
                source_prediction_sha256=f['prediction_sha256'],feature_path=f['feature_path'],feature_sha256=f['feature_sha256'],source_points=len(inverse),cells=len(cells),
                anchor_frame=r['anchor_frame'],anchor_time_s=r['anchor_time_s'],anchor_translation_m=r['anchor_translation_m'],
                witnessed_points=int(r['point_witness'].sum()),withheld_cells=int(scores.sum()),generation_s=elapsed))
            totalpoints+=len(inverse);totalcells+=len(cells)
        if len(rows)%50==0:print('CAUSAL_DUALFRAME_ADVISORY',len(rows),rows[-1]['withheld_cells'],flush=True)
    assert len(rows)==470 and totalpoints==2985832 and totalcells==40052
    timing={seq:dict(P50_s=float(np.quantile(v,.5)),P90_s=float(np.quantile(v,.9)),max_s=float(max(v))) for seq,v in times.items()}
    put(D/'prediction_seal.json',dict(complete=True,utc=now(),rows=rows,full_source_points=totalpoints,full_cell_actions=totalcells,fixtures=checks,
        sequence_timing=timing,teacher_labels_read_in_prediction=False,model_calls=0,fit=0,source_calls=0,physics=0,navigation=0,protocol_sha256=sha(D/'protocol.json')))
    print('ALL470_GEOMETRY_ADVISORIES_SEALED',totalpoints,totalcells,timing,flush=True)
if __name__=='__main__':main()
