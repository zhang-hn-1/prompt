"""Separate causal feature cache from offline training labels/ranges."""
from common import *
import numpy as np
from causal_input import features
verify_oracle()
inputs={};outputs={};rows=[];sums=np.zeros(10);sq=np.zeros(10);count=0;classcount=np.zeros(3,np.int64)
for name,R in GROUPS.items():
 (D/'features'/name).mkdir(parents=True,exist_ok=False);(D/'supervision'/name).mkdir(parents=True,exist_ok=False)
 paths=sorted((R/'source_inputs').glob('*.npz'));assert [int(p.stem.split('_')[1]) for p in paths]==list(range(0,901,5))
 cc=np.zeros(3,np.int64);eligible=labelled=0
 for p in paths:
  npath=R/'known_poses'/p.name;lpath=Q/'labels'/name/p.name;fpath=R/'frames'/p.name;hpath=R/'history'/p.name
  f=np.load(p);n=np.load(npath);l=np.load(lpath);gt=np.load(fpath);h=np.load(hpath)
  assert float(f['time_s'])==float(n['time_s'])==float(l['time_s'])==float(gt['time_s'])
  x=features(f['left_rgb'],f['right_rgb'],n['left_camera_usd_world'],n['right_camera_usd_world'])
  labels=np.stack([l[e+'_category'][::2,::2] for e in ['left','right']]);ranges=np.stack([gt[e+'_depth_m'][::2,::2] for e in ['left','right']]).astype(np.float32)
  ev=np.stack([l[e+'_eligible'][::2,::2] for e in ['left','right']]);eligible+=int(ev.sum());labelled+=int((labels>0).sum());cc+=np.bincount(labels.ravel(),minlength=3)
  fp=D/'features'/name/p.name;sp=D/'supervision'/name/p.name
  np.savez_compressed(fp,x=x,time_s=f['time_s']);np.savez_compressed(sp,category=labels,range_m=ranges,time_s=f['time_s'])
  if name in ['H','E0']:
   a=x.astype(np.float64).reshape(10,-1);sums+=a.sum(1);sq+=(a*a).sum(1);count+=a.shape[1];classcount+=np.bincount(labels.ravel(),minlength=3)
  for pp in [p,npath,lpath,fpath,hpath]:inputs[str(pp)]=sha(pp)
  for pp in [fp,sp]:outputs[str(pp)]=sha(pp)
 rows.append(dict(group=name,pairs=len(paths),eligible=eligible,labelled=labelled,label_coverage=labelled/eligible,class_pixels=cc.tolist(),role='train' if name in ['H','E0'] else 'fit_excluded_seen_development'))
 print(name,'complete',flush=True)
mean=sums/count;std=np.sqrt(np.maximum(sq/count-mean*mean,1e-6));weights=1/np.sqrt(classcount);weights/=weights.mean()
assert all(r['pairs']==181 and r['label_coverage']>=.95 for r in rows) and (classcount>1000).all()
save('coverage.json',dict(rows=rows,qualified=True,mean=mean.tolist(),std=std.tolist(),class_weights=weights.tolist(),train_class_pixels=classcount.tolist(),input_sha256=inputs,output_sha256=outputs,scope='Whole-sequence split H/E0 train; E1/B54/C54 seen development, no independent holdout; no frame shuffle split. Native orientation ideal IMU privilege; GT only supervision.'))
