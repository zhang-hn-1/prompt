"""One fixed padding counterfactual. Diagnostic only, no alternative source admission."""
from pathlib import Path
import sys,json,hashlib,time,datetime
import numpy as np
D=Path(__file__).resolve().parent; W=D.parents[1]
sys.path.insert(0,str(W/'experiments/four_camera_weak_baseline_20261017_v1'))
import source as old
cv2=old.cv2
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def put(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def guard(cfg):
 for section in ['input_sha256','code_sha256']:
  for p,h in cfg[section].items():assert sha(p)==h,p

def predict(rgb,masks,border):
 out={};begin=time.monotonic();calls=0
 ml=cv2.StereoSGBM_create(minDisparity=0,**old.CONFIG)
 mr=cv2.StereoSGBM_create(minDisparity=-448,**old.CONFIG)
 for q,p in old.PAIRS:
  a,b=[np.ascontiguousarray(np.rot90(rgb[n])) for n in (q,p)]
  lm,rm=[np.ascontiguousarray(np.rot90(masks[n])) for n in (q,p)]
  ap,bp=[cv2.copyMakeBorder(v,0,0,448,448,border,value=0) for v in (a,b)]
  assert np.array_equal(ap[:,448:-448],a) and np.array_equal(bp[:,448:-448],b)
  dl=ml.compute(ap,bp).astype(np.float32)[:,448:-448]/16.; calls+=1
  dr=mr.compute(bp,ap).astype(np.float32)[:,448:-448]/16.; calls+=1
  for eye,partner,dd,od,mm,om,sign in [(q,p,dl,dr,lm,rm,1),(p,q,dr,dl,rm,lm,-1)]:
   z,v,d=old.depths(dd,od,mm,om,sign)
   out[eye]=dict(depth=z,valid=v,disparity=d,CAD=masks[eye],partner=partner,sign=sign)
 return out,time.monotonic()-begin,calls

def main():
 cfg=json.loads((D/'protocol.json').read_text());guard(cfg);cv2.setNumThreads(1)
 assert cv2.__version__=='4.6.0'; allrows=[]; output=[];exact=0;point_exact=0
 for i,row in enumerate(cfg['rows']):
  with np.load(row['input_rgb_frame']) as a:rgb={n:a[n+'_rgb'] for n in old.NAMES}
  with np.load(row['prediction_path']) as a:
   masks={n:a[n+'_CAD'] for n in old.NAMES};body=a['body_estimated'];t=float(a['time_s'])
  preds={};timing={}
  for kind,border in [('constant',cv2.BORDER_CONSTANT),('replicate',cv2.BORDER_REPLICATE)]:
   pred,seconds,calls=predict(rgb,masks,border);assert calls==4
   records,arr=old.points(pred,body,row['frame'],t)
   for n in old.NAMES:
    for k in ['depth','valid','disparity','CAD']:arr[n+'_'+k]=pred[n][k]
   arr.update(body_estimated=body,time_s=np.asarray(t))
   if kind=='constant':
    with np.load(row['prediction_path']) as a:
     for n in old.NAMES:
      for k in ['depth','valid','disparity','CAD']:
       x=arr[n+'_'+k];y=a[n+'_'+k];assert x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes(),(row['sequence'],row['frame'],n,k);exact+=1
      for k in ['world','query_pixel','partner_pixel']:
       x=arr[n+'_'+k];y=a[n+'_'+k];assert x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes(),(row['sequence'],row['frame'],n,k);point_exact+=1
   path=D/(row['sequence']+'_'+str(row['frame']).zfill(3)+'_'+kind+'.npz');np.savez_compressed(path,**arr)
   output.append(dict(path=str(path),sha256=sha(path),arm=kind,sequence=row['sequence'],frame=row['frame']))
   preds[kind]=arr;timing[kind]=seconds
  allrows.append((row,preds,timing));print('DONE',row['sequence'],row['frame'],flush=True)
 guard(cfg);put(D/'prediction_seal.json',dict(utc=now(),outputs=output,baseline_16arrays_exact=exact,baseline_sourcepoint_arrays_exact=point_exact,GT_read=False,matcher_calls=32,point_projection_calls=8))
 assert exact==64 and point_exact==48
 results=[]
 for row,arms,timing in allrows:
  a,b=arms['constant'],arms['replicate'];eyes=[]
  for n in old.NAMES:
   va=a[n+'_valid']; vb=b[n+'_valid']; za=a[n+'_depth'];zb=b[n+'_depth'];both=va&vb
   dif=np.zeros_like(za,dtype=np.float64);dif[both]=abs(za[both].astype(float)-zb[both].astype(float))
   q=a[n+'_query_pixel'];yy=q//640;xx=q%640;pv=a[n+'_partner_pixel']//640
   interior=(yy>=3)&(yy<477)&(pv>=3)&(pv<477)&(xx>=3)&(xx<637)
   lost=~vb.ravel()[q];changed=lost|(dif.ravel()[q]>.15)
   common=dif[both]
   eyes.append(dict(eye=n,full_pixel_denominator=int(va.size),constant_valid=int(va.sum()),replicate_valid=int(vb.sum()),validity_changed=int((va!=vb).sum()),both_valid=int(both.sum()),depth_delta_gt15cm_both=int((dif>.15).sum()),depth_delta_both_P90_m=float(np.percentile(common,90)) if len(common) else None,old_risk_point_denominator=int(len(q)),old_risk_points_lost_or_shift_gt15cm=int(changed.sum()),old_risk_points_interior_denominator=int(interior.sum()),old_risk_points_interior_lost_or_shift_gt15cm=int((changed&interior).sum()),constant_risk_points=int(len(q)),replicate_risk_points=int(len(b[n+'_query_pixel']))))
  results.append(dict(sequence=row['sequence'],frame=row['frame'],time_s=row['time_s'],eyes=eyes,matcher_only_wall_s=timing))
 total=lambda k:sum(x[k] for r in results for x in r['eyes'])
 nums={k:total(k) for k in ['full_pixel_denominator','constant_valid','replicate_valid','validity_changed','both_valid','depth_delta_gt15cm_both','old_risk_point_denominator','old_risk_points_lost_or_shift_gt15cm','old_risk_points_interior_denominator','old_risk_points_interior_lost_or_shift_gt15cm','replicate_risk_points']}
 dependency=nums['old_risk_points_interior_lost_or_shift_gt15cm']>0
 guard(cfg)
 put(D/'result.json',dict(utc=now(),complete=True,rows=results,totals=nums,nonlocal_padding_dependency_witnessed=dependency,decision='PADDING_DEPENDENCY_WITNESSED_DIAGNOSTIC_ONLY' if dependency else 'NO_INTERIOR_RISK_DEPENDENCY_WITNESS_ON_FOUR_FIXED_FRAMES',baseline_arrays_exact=64,baseline_sourcepoint_arrays_exact=48,scope='Four seen development frames. Only off-image fabricated values change; interior margin3 is local block/derivative support, NOT a proof all SGM costs use only measured pixels. Causal output dependency, not metric error, accuracy benefit, uncertainty interval or software specification defect. Replicate is not a deployable repair; no GT/labels read or map/nav run.',source_compute_replays=8,matcher_calls=32,source_point_projections=8,new_CAD=0,fit=0,physics=0,navigation=0,GT_scoring=0))
if __name__=='__main__':main()
