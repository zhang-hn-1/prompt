from pathlib import Path
import sys, json, hashlib, time, datetime, gc
import numpy as np
from reference import build_cost, aggregate
from evidence_decode import evidence_decode
from self_support import self_support
D = Path(__file__).resolve().parent; W = D.parents[1]
sys.path.insert(0,str(W/'experiments/four_camera_weak_baseline_20261017_v1'))
import source as old
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def guard(cfg):
    for sec in ['input_sha256','code_sha256']:
        for p,h in cfg[sec].items(): assert sha(p)==h,p
def main():
    cfg=json.loads((D/'protocol.json').read_text());guard(cfg)
    assert all(json.loads((D/'interfaces_result.json').read_text())['checks'].values())
    old.cv2.setNumThreads(1); rows=[]
    for row in cfg['rows']:
        with np.load(row['input_rgb_frame']) as f: rgb={n:f[n+'_rgb'] for n in old.NAMES}
        with np.load(row['prediction_path']) as f: masks={n:f[n+'_CAD'] for n in old.NAMES};body=f['body_estimated'];timestamp=float(f['time_s'])
        with np.load(row['reference_baseline']) as f: origin=f['origin']
        disparities={};times=[];started=time.monotonic()
        for q,p in old.PAIRS:
            for name,partner,ds in [(q,p,np.arange(448)),(p,q,np.arange(-448,0))]:
                a,b=[np.ascontiguousarray(np.rot90(rgb[n])) for n in [name,partner]]
                m,n=[np.ascontiguousarray(np.rot90(masks[k])) for k in [name,partner]]
                (D/'progress.json').write_text(json.dumps(dict(sequence=row['sequence'],frame=row['frame'],eye=name,phase='cost'))+'\n')
                tick=time.monotonic();cost,support=build_cost(a,b,ds);before=int(support.sum());available=self_support(support,m,n,ds);after=int(available.sum());total,_=aggregate(cost,available)
                disparities[name]=evidence_decode(total,cost,available,ds)
                yy,xx=np.nonzero(np.isfinite(disparities[name]));ii=(disparities[name][yy,xx]-ds[0]).astype(int);assert available[yy,xx,ii].all()
                times.append(dict(eye=name,supported_before=before,supported_after=after,seconds=time.monotonic()-tick))
                del cost,support,available,total;gc.collect()
        pred={}
        for q,p in old.PAIRS:
            lm,rm=[np.ascontiguousarray(np.rot90(masks[n])) for n in [q,p]]
            for n,pn,mm,om,sign in [(q,p,lm,rm,1),(p,q,rm,lm,-1)]:
                z,v,d=old.depths(disparities[n],disparities[pn],mm,om,sign);pred[n]=dict(depth=z,valid=v,disparity=d,CAD=masks[n],partner=pn,sign=sign)
        rec,arr=old.points(pred,body,row['frame'],timestamp)
        for n in old.NAMES:
            for k in ['depth','valid','disparity','CAD']:arr[n+'_'+k]=pred[n][k]
        arr.update(body_estimated=body,time_s=np.asarray(timestamp),origin=origin)
        path=D/(row['sequence']+'_'+str(row['frame']).zfill(3)+'_self_supported.npz');np.savez_compressed(path,**arr)
        rows.extend([dict(sequence=row['sequence'],frame=row['frame'],arm='baseline',path=row['reference_baseline'],sha256=sha(row['reference_baseline']),reused_closed_reference=True),dict(sequence=row['sequence'],frame=row['frame'],arm='self_supported',path=str(path),sha256=sha(path),seconds=time.monotonic()-started,components=times,GT_read=False)])
        print('SEALED_FRAME',row['sequence'],row['frame'],time.monotonic()-started,flush=True)
    guard(cfg)
    (D/'prediction_seal.json').write_text(json.dumps(dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,protocol_sha256=sha(D/'protocol.json'),GT_read=False,new_reference_predictions=4,reused_reference_predictions=4,actual_RGB_cost_volumes=16,full_aggregations=16),indent=2)+'\n')
if __name__=='__main__':main()
