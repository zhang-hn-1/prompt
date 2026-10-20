from pathlib import Path
import sys,json,hashlib,time,datetime,gc
import numpy as np
from reference import build_cost,aggregate,decode
from evidence_decode import evidence_decode
D=Path(__file__).resolve().parent;W=D.parents[1];sys.path.insert(0,str(W/'experiments/four_camera_weak_baseline_20261017_v1'))
import source as old
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def put(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def guard(cfg):
 for sec in ['code_sha256','input_sha256']:
  for p,h in cfg[sec].items():assert sha(p)==h,p

def main():
 cfg=json.loads((D/'protocol.json').read_text());guard(cfg);assert all(json.loads((D/'interfaces_result.json').read_text())['checks'].values());old.cv2.setNumThreads(1);outrows=[]
 for row in cfg['rows']:
  with np.load(row['input_rgb_frame']) as a:rgb={n:a[n+'_rgb'] for n in old.NAMES}
  with np.load(row['prediction_path']) as a:masks={n:a[n+'_CAD'] for n in old.NAMES};body=a['body_estimated'];t=float(a['time_s'])
  with np.load(row['feature_path']) as a:origin=a['origin']
  maps={arm:{} for arm in ['unmasked','masked']};timing=[];raw_before={n:hashlib.sha256(rgb[n].tobytes()).hexdigest() for n in old.NAMES};begin=time.monotonic()
  for pair,(q,p) in enumerate(old.PAIRS):
   for name,partner,ds in [(q,p,np.arange(448)),(p,q,np.arange(-448,0))]:
    put(D/'progress.json',dict(utc=now(),sequence=row['sequence'],frame=row['frame'],eye=name,phase='RGBcost',completed_outputs=len(outrows)))
    a,b=[np.ascontiguousarray(np.rot90(rgb[n])) for n in [name,partner]];tic=time.monotonic();cost,support=build_cost(a,b,ds);cost_s=time.monotonic()-tic;sig=hashlib.sha256(memoryview(cost)).hexdigest();validhyp=int(support.sum())
    for arm in (['unmasked','masked'] if row['frame']%2==0 else ['masked','unmasked']):
     put(D/'progress.json',dict(utc=now(),sequence=row['sequence'],frame=row['frame'],eye=name,phase=arm+'_aggregate',completed_outputs=len(outrows)))
     tic=time.monotonic();total,_=aggregate(cost,support if arm=='masked' else np.ones_like(support));dd=evidence_decode(total,cost,support,ds);seconds=time.monotonic()-tic
     # Masked decoding must never emit an unavailable candidate.
     if arm=='masked':
      yy,xx=np.nonzero(np.isfinite(dd));ii=(dd[yy,xx]-ds[0]).astype(int);assert support[yy,xx,ii].all()
     maps[arm][name]=dd;timing.append(dict(eye=name,arm=arm,cost_shared_s=cost_s,aggregate_decode_s=seconds,cost_sha256=sig,supported_hypotheses=validhyp,total_hypotheses=int(support.size)))
     del total;gc.collect()
    del cost,support;gc.collect()
  compute_both_s=time.monotonic()-begin
  for n in old.NAMES:assert hashlib.sha256(rgb[n].tobytes()).hexdigest()==raw_before[n]
  post_seconds=0.
  for arm in ['unmasked','masked']:
   post_begin=time.monotonic();pred={}
   for q,p in old.PAIRS:
    lm,rm=[np.ascontiguousarray(np.rot90(masks[n])) for n in [q,p]]
    for n,pn,mm,om,sign in [(q,p,lm,rm,1),(p,q,rm,lm,-1)]:
     z,v,dd=old.depths(maps[arm][n],maps[arm][pn],mm,om,sign);pred[n]=dict(depth=z,valid=v,disparity=dd,CAD=masks[n],partner=pn,sign=sign)
   records,arr=old.points(pred,body,row['frame'],t)
   for n in old.NAMES:
    for k in ['depth','valid','disparity','CAD']:arr[n+'_'+k]=pred[n][k]
   arr.update(body_estimated=body,time_s=np.asarray(t),origin=origin,source_identity=np.array('RGB_BT_float64_3path_integer_'+arm))
   post_elapsed=time.monotonic()-post_begin;post_seconds+=post_elapsed
   f=D/(row['sequence']+'_'+str(row['frame']).zfill(3)+'_'+arm+'.npz');np.savez_compressed(f,**arr);outrows.append(dict(sequence=row['sequence'],frame=row['frame'],time_s=row['time_s'],arm=arm,path=str(f),sha256=sha(f),risk_points=sum(len(arr[n+'_query_pixel']) for n in old.NAMES),combined_twoarm_cost_aggregate_decode_s=compute_both_s,post_LR_projection_s=post_elapsed,component_timings=timing,GT_read=False))
  for rr in outrows[-2:]:
   rr['combined_twoarm_LR_projection_s']=post_seconds;rr['combined_twoarm_total_source_without_IO_s']=compute_both_s+post_seconds;rr['combined_twoarm_including_IO_s']=time.monotonic()-begin
  print('FRAME_DONE',row['sequence'],row['frame'],compute_both_s,'s',flush=True)
 guard(cfg);assert len(outrows)==8
 put(D/'prediction_seal.json',dict(complete=True,utc=now(),rows=outrows,RGB_frames=4,RGB_cost_volumes=16,full_aggregations=32,four_eye_predictions=8,raw_RGB_bytes_unchanged=True,GT_read=False,protocol_sha256=sha(D/'protocol.json'),timing_scope='Measured combinedtwoarm four-eye cost/aggregate/decode plus LR/depth/projection; masks/pose preloaded/shared, excludes currentCADgeneration/render/semantics/map/planner/controller. Source component sum excludes outputIO; includingIO reported separately. Not actualonline endtoend.'))
 put(D/'progress.json',dict(utc=now(),status='all_predictions_sealed',completed_outputs=8))
if __name__=='__main__':main()
