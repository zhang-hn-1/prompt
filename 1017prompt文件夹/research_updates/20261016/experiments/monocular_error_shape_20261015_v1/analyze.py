from pathlib import Path
import sys,json,hashlib,time
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];S=W/'experiments/dual_monocular_semantic_bev_20261015_v1'
sys.path.insert(0,str(S));from common import GROUPS
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((S/'prediction_seal.json').read_text()); prior=json.loads((S/'result.json').read_text()); hashes={};groups={};start=time.time()
for name,(root,cp) in GROUPS.items():
 cfg=json.loads(cp.read_text());masks=np.load(cfg['static_masks']);rows=[]
 for p in sorted((root/'source_inputs').glob('*.npz')):
  pred=S/'predictions'/name/p.name;gtpath=root/'frames'/p.name
  assert sha(pred)==seal['outputs'][str(pred)]
  assert sha(gtpath)==prior['truth_hashes'][str(gtpath)]
  hashes[str(pred)]=sha(pred);hashes[str(gtpath)]=sha(gtpath)
  with np.load(pred) as a,np.load(gtpath) as f:
   eyes=[]
   for eye in ['left','right']:
    g=f[eye+'_depth_m'].astype(np.float64);z=a[eye+'_depth_m'].astype(np.float64)
    valid=np.isfinite(g)&(g>=.5)&(g<=3)&~masks[eye].astype(bool);ok=valid&np.isfinite(z);err=z[ok]-g[ok];positive=ok&(z>0);ratio=z[positive]/g[positive]
    eyes.append(dict(valid=int(valid.sum()),finite=int(ok.sum()),positive=int(positive.sum()),over015=int((err>.15).sum()),under015=int((err<-.15).sum()),within015=int((np.abs(err)<=.15).sum()),signed_q=np.quantile(err,[.1,.5,.9]).tolist(),abs_p90=float(np.quantile(np.abs(err),.9)),ratio_q=np.quantile(ratio,[.1,.5,.9]).tolist()))
   rows.append(dict(frame=p.name,eyes=eyes))
 out=[]
 for j in range(2):
  ee=[r['eyes'][j] for r in rows];tot={k:sum(e[k] for e in ee) for k in ['valid','finite','positive','over015','under015','within015']}
  tot.update(median_ratio_frame_min_median_max=np.quantile([e['ratio_q'][1] for e in ee],[0,.5,1]).tolist(),within_frame_ratio_p90_minus_p10_quantiles=np.quantile([e['ratio_q'][2]-e['ratio_q'][0] for e in ee],[.1,.5,.9]).tolist(),frames_both_over_and_under_gt015_atleast10pct=sum(e['over015']>=.1*e['finite'] and e['under015']>=.1*e['finite'] for e in ee))
  out.append(tot)
 groups[name]=dict(frames=rows,eyes=out,lr_image_median_ratio_absdiff_quantiles=np.quantile([abs(r['eyes'][0]['ratio_q'][1]-r['eyes'][1]['ratio_q'][1]) for r in rows],[.1,.5,.9]).tolist())
 print(name,json.dumps(out),flush=True)
(D/'result.json').write_text(json.dumps(dict(groups=groups,input_hashes=hashes,wall_s=time.time()-start,fit=0,physical=0,model_inference=0),indent=2)+'\n')
print('COMPLETE',time.time()-start,flush=True)
