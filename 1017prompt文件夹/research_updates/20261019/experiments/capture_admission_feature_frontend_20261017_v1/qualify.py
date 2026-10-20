from pathlib import Path
import json,hashlib,datetime,time
import numpy as np
from frontend import FeatureTracker,NAMES,interface_checks
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
load=lambda p:json.loads(Path(p).read_text())
def put(p,x): Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def main():
    cfg=load(D/'protocol.json'); assert not (D/'result.json').exists()
    for p,h in cfg['metadata_sha256'].items(): assert sha(p)==h,p
    for p,h in cfg['code_sha256'].items(): assert sha(p)==h,p
    prior=load(cfg['prerequisite_result']); assert prior['necessary_passed']
    fs=load(cfg['feature_seal']); trackers={}; rows=[]; total=0; points=0; outside=0; times=[]
    checks=interface_checks()
    for i,r in enumerate(fs['rows']):
        for p,h in [(r['input_rgb_frame'],r['input_sha256']),(r['prediction_path'],r['prediction_sha256']),(r['feature_path'],r['feature_sha256'])]: assert sha(p)==h,p
        with np.load(r['input_rgb_frame']) as raw,np.load(r['prediction_path']) as pred,np.load(r['feature_path']) as original:
            payload=dict(rgb={n:raw[n+'_rgb'] for n in NAMES},body_estimated=pred['body_estimated'],
                eyes={n:{k:pred[n+'_'+k] for k in ['world','query_pixel','partner_pixel','depth','disparity','CAD']} for n in NAMES})
            tracker=trackers.setdefault(r['sequence'],FeatureTracker());begin=time.perf_counter()
            out=tracker.update(r['frame'],r['time_s'],r['time_s'],payload);elapsed=time.perf_counter()-begin;times.append(elapsed)
            keys=['features','cells','origin','source_eye','source_query_uid','source_partner_uid','source_cell_row']
            for k in keys: assert out[k].dtype==original[k].dtype and out[k].shape==original[k].shape and out[k].tobytes()==original[k].tobytes(),(r['sequence'],r['frame'],k)
            assert out['time_s']==float(original['time_s']) and out['original_points']==r['source_points']
            total+=len(out['cells']);points+=out['original_points'];outside+=out['outside_points']
            # Reused exposure is rejected before payload; feature history is not double-counted.
            class Poison:
                def __iter__(self): raise AssertionError('duplicate payload read')
            try: tracker.update(r['frame'],r['time_s'],r['time_s'],Poison())
            except ValueError: pass
            else: raise AssertionError('duplicate feature update')
            rows.append(dict(sequence=r['sequence'],frame=r['frame'],time_s=r['time_s'],
                input_path=r['input_rgb_frame'],input_sha256=r['input_sha256'],prediction_path=r['prediction_path'],prediction_sha256=r['prediction_sha256'],
                reference_path=r['feature_path'],reference_sha256=r['feature_sha256'],cells=len(out['cells']),points=out['original_points'],
                arrays_byte_exact=len(keys),feature_generation_s=elapsed,GT_read=False,duplicate_before_payload=True))
        if i%50==0: print('FEATURE_FRONTEND_EXACT',i+1,r['sequence'],flush=True)
    assert len(rows)==470 and total==40052 and points==2985832 and outside==0
    result=dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        decision='FULL470_CAUSAL_FEATURE_INTERFACE_BYTE_EQUIVALENT_NOT_FIT_ADMISSION',
        frames=len(rows),cell_rows=total,feature_channels=24,feature_scalars=total*24,
        arrays_byte_exact=7*len(rows),source_points=points,outside_points=outside,checks=checks,rows=rows,
        feature_generation_P50_P90_max_s=np.quantile(times,[.5,.9,1]).tolist(),
        scope=cfg['scope'],fit=0,learned_inference=0,source_calls=0,physics=0,navigation=0,
        protocol_sha256=sha(D/'protocol.json'))
    put(D/'result.json',result);print('ALL_FEATURES_EXACT',total,total*24,flush=True)
if __name__=='__main__':main()
