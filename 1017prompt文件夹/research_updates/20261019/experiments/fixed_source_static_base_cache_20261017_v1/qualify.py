from pathlib import Path
import json,hashlib,time,datetime,sys
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];load=lambda p:json.loads(Path(p).read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def put(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def main():
 cfg=load(D/'protocol.json')
 for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
 for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
 receipt=load(D/'prerequisite_receipt.json');assert sha(cfg['prerequisite_result'])==receipt['sha256'];assert load(cfg['prerequisite_result'])['output_arrays_byte_exact']==7520
 from cached_source import CachedBaseParallelSGBM,ParallelFourSGBM,legacy
 rows=load(cfg['feature_seal'])['rows'];selection=[next(r for r in rows if r['sequence']==s and r['frame']==f) for s,f in cfg['latency_frames']]
 before=time.monotonic();old=ParallelFourSGBM();oldinit=time.monotonic()-before;new=CachedBaseParallelSGBM();bench=[];full=[]
 try:
  for i,row in enumerate(selection):
   assert sha(row['input_rgb_frame'])==row['input_sha256'] and sha(row['prediction_path'])==row['prediction_sha256']
   with np.load(row['input_rgb_frame']) as a:rgb={n:a[n+'_rgb'] for n in legacy.NAMES};j=a['joints_current']
   times={}
   for kind,m in ([('uncached',old),('cached',new)] if i%2==0 else [('cached',new),('uncached',old)]):
    pred,seconds=m.predict(rgb,j);times[kind]=seconds
    with np.load(row['prediction_path']) as ref:
     for n in legacy.NAMES:
      for k in ['depth','valid','disparity','CAD']:assert pred[n][k].dtype==ref[n+'_'+k].dtype and pred[n][k].shape==ref[n+'_'+k].shape and pred[n][k].tobytes()==ref[n+'_'+k].tobytes(),(row['sequence'],row['frame'],n,k)
   bench.append(dict(sequence=row['sequence'],frame=row['frame'],**times,ratio=times['cached']/times['uncached'],all16_arrays_byte_exact=True));print('CACHE_BENCH',bench[-1],flush=True)
  gates=dict(median_cached_ratio_le80pct=float(np.median([r['ratio'] for r in bench]))<=.8,each_cached_ratio_le110pct=max(r['ratio'] for r in bench)<=1.1,all4_source_outputs_exact=True)
  put(D/'latency_result.json',dict(complete=True,rows=bench,gates=gates,uncached_initialization_s=oldinit,cached_initialization_s=new.initialization_wall_s,full_source_invocations=8,scope='Four declared knownframes onceperimplementation, alternatingorder, no statisticalendtoendclaim; coldinitialization separately reported.'))
  if not all(gates.values()):put(D/'result.json',dict(complete=True,decision='REJECT_LOCAL_CACHE_PERFORMANCE',gates=gates,fit=0,physics=0));return
  for i,row in enumerate(rows):
   assert sha(row['input_rgb_frame'])==row['input_sha256'] and sha(row['prediction_path'])==row['prediction_sha256']
   with np.load(row['input_rgb_frame']) as a:j=a['joints_current']
   start=time.monotonic();mask=new.masks(j);seconds=time.monotonic()-start
   with np.load(row['prediction_path']) as ref:
    for n in legacy.NAMES:assert mask[n].dtype==ref[n+'_CAD'].dtype and mask[n].shape==ref[n+'_CAD'].shape and mask[n].tobytes()==ref[n+'_CAD'].tobytes(),(row['sequence'],row['frame'],n)
   full.append(dict(sequence=row['sequence'],frame=row['frame'],masks_byte_exact=4,mask_wall_s=seconds,input_sha256=row['input_sha256'],prediction_sha256=row['prediction_sha256']))
   if (i+1)%20==0:put(D/'progress.json',dict(processed=i+1,total=470,last_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
  assert len(full)==470
  put(D/'result.json',dict(complete=True,decision='FULL470_MASK_EQUIVALENCE_AND4FRAME_SOURCE_EQUIVALENCE_PASSED',rows=full,gates=gates,full_mask_invocations=470,mask_arrays_byte_exact=1880,full_source_invocations=8,fit=0,learned_inference=0,physics=0,navigation=0,scope='All470 masks byteexact plus8fullsourceforwards on4fixedframes. Sourcecomposition equivalence uses prior7520arrays exact and unchanged downstream functions; not claiming to have empiricallyrerun470fullsources with cache. Not onlinechain, navigation or new sourceaccuracy.',known_CAD_not_GT_scene=True,mechanism='Existingstatic-base memoization engineering adaptation, not new measuringmechanism or learning.'))
 finally:old.close();new.close()
if __name__=='__main__':main()
